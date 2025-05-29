#include "common.h"
#include <cjson/cJSON.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#define UDP_PORT 5005
#define BUFFER_SIZE 4096

volatile sig_atomic_t stop_flag = 0;

void handle_sigint(int sig) {
    printf("\n[수신] SIGINT (%d) 발생! 프로그램을 정리하고 종료합니다.\n", sig);
    stop_flag = 1;
}

// 1. 공유 메모리 연결
int attach_shm(const char *shm_name, CameraQueue **shm_q) {
    int fd = shm_open(shm_name, O_RDWR, 0666);
    if (fd == -1) {
        perror("[Camera] shm_open failed");
        return -1;
    }

    void *ptr = mmap(0, sizeof(CameraQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (ptr == MAP_FAILED) {
        perror("[Camera] mmap failed");
        return -1;
    }

    *shm_q = (CameraQueue *)ptr;
    return 0;
}

// 2. UDP 서버 초기화
int init_udp_server(int port) {
    int sockfd;
    struct sockaddr_in servaddr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("[Camera] Socket creation failed");
        exit(EXIT_FAILURE);
    }

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("[Camera] Bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("[Camera] UDP Listening on %d...\n", port);
    return sockfd;
}

// 3. JSON 파싱 및 데이터 출력 로직
int parse_json_to_item(const char *json_str, CameraItem *item, int *out_cam_id) {
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) return -1;

    // Camera ID
    cJSON *cam = cJSON_GetObjectItem(root, "cam_id");
    if (cJSON_IsNumber(cam)) *out_cam_id = cam->valueint;

    // Status (0: LOST, 1: OK)
    cJSON *status = cJSON_GetObjectItem(root, "status");
    int status_val = cJSON_IsNumber(status) ? status->valueint : 0;

    // Timestamp (float)
    cJSON *ts = cJSON_GetObjectItem(root, "timestamp");
    if (cJSON_IsNumber(ts)) item->timestamp = (float)ts->valuedouble;

    // 터미널 출력 헤더
    printf("\n[Cam: %d | Status: %s | TS: %f]", 
           *out_cam_id, 
           (status_val == 1) ? "\033[0;32mOK\033[0m" : "\033[0;31mLOST\033[0m", 
           item->timestamp);

    // Detections 파싱
    cJSON *detections = cJSON_GetObjectItem(root, "detections");
    int count = 0;
    if (cJSON_IsArray(detections)) {
        cJSON *obj = NULL;
        cJSON_ArrayForEach(obj, detections) {
            if (count >= MAX_BBOX_OBJS) break;
            cJSON *id = cJSON_GetObjectItem(obj, "id");
            cJSON *x  = cJSON_GetObjectItem(obj, "x");
            cJSON *y  = cJSON_GetObjectItem(obj, "y");
            cJSON *w  = cJSON_GetObjectItem(obj, "w");
            cJSON *h  = cJSON_GetObjectItem(obj, "h");

            if (cJSON_IsNumber(id)) item->objects[count].class_id = id->valueint;
            if (cJSON_IsNumber(x))  item->objects[count].x = (float)x->valuedouble;
            if (cJSON_IsNumber(y))  item->objects[count].y = (float)y->valuedouble;
            if (cJSON_IsNumber(w))  item->objects[count].w = (float)w->valuedouble;
            if (cJSON_IsNumber(h))  item->objects[count].h = (float)h->valuedouble;
            
            // 좌표 전체 출력
            printf(" [ID:%d, x:%.1f, y:%.1f, w:%.1f, h:%.1f]", 
                   item->objects[count].class_id, 
                   item->objects[count].x, item->objects[count].y,
                   item->objects[count].w, item->objects[count].h);
            count++;
        }
    }
    
    item->obj_count = count;
    cJSON_Delete(root);
    return 0;
}

// 4. Main 함수
int main(int argc, char *argv[]) {
    CameraQueue *front_q = NULL;
    CameraQueue *back_q = NULL;
    int port = UDP_PORT;

    signal(SIGINT, handle_sigint);

    // 1. 공유 메모리 연결
    if (attach_shm(SHM_NAME_FRONT_CAMERA, &front_q) < 0) {
        fprintf(stderr, "[Camera] Front SHM not ready.\n");
        return -1;
    }

    if (attach_shm(SHM_NAME_BACK_CAMERA, &back_q) < 0) {
        fprintf(stderr, "[Camera] Back SHM not ready.\n");
        return -1;
    }

    // 2. UDP 소켓 오픈
    int sockfd = init_udp_server(port);
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);
    char buffer[BUFFER_SIZE];

    // 3. 생산자 루프 (Receive -> Parse -> Produce)
    while (!stop_flag) {
        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE - 1, 0, (struct sockaddr *)&cliaddr, &len);
        if (n > 0) {
            buffer[n] = '\0';
            CameraItem temp_item;
            int received_cam_id = -1;

            if (parse_json_to_item(buffer, &temp_item, &received_cam_id) == 0) {
                // cam_id에 따라 타겟 큐 결정 (0: Front, 1: Back)
                CameraQueue *target_q = (received_cam_id == 0) ? front_q : back_q;

                if (target_q) {
                    // 소비자가 데이터를 가져갈 때까지 대기
                    sem_wait(&target_q->sem_empty);
                    pthread_mutex_lock(&target_q->mutex);

                    int idx = target_q->tail;
                    memcpy(&target_q->buffer[idx], &temp_item, sizeof(CameraItem));
                    target_q->tail = (target_q->tail + 1) % QUEUE_SIZE;

                    pthread_mutex_unlock(&target_q->mutex);
                    sem_post(&target_q->sem_full);
                }
            }
            fflush(stdout);
        }
    }
    
    munmap(front_q, sizeof(CameraQueue));
    munmap(back_q, sizeof(CameraQueue));
    close(sockfd);
    return 0;
}