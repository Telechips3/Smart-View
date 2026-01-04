#include "common.h"
#include <cjson/cJSON.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#define UDP_PORT 5005
#define BUFFER_SIZE 4096

// 1. 공유 메모리 연결 (이미 생성된 메모리에 붙기만 함)
int attach_shm(const char *shm_name, CameraQueue* shm_q) {
    // O_CREAT 없음 -> 이미 있는 것을 염
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

    shm_q = (CameraQueue *)ptr;
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

int parse_json_to_item(const char *json_str, CameraItem *item, int *out_cam_id) {
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            fprintf(stderr, "[Camera] JSON Parse Error before: %s\n", error_ptr);
        }
        return -1;
    }

    cJSON *cam = cJSON_GetObjectItem(root, "cam_id");
    if (cJSON_IsNumber(cam))
    {
        *out_cam_id = cam->valueint;
        printf("cam_id: %d ", *out_cam_id);
    }
    else { cJSON_Delete(root); return -1; }

    // 1. Timestamp (Python의 time.time()은 double임)
    cJSON *ts = cJSON_GetObjectItem(root, "timestamp");
    if (cJSON_IsNumber(ts)) {
        item->timestamp = (float)ts->valuedouble;
        printf("timestamp: %f ", item->timestamp);
    } else {
        // 에러 처리 시 cJSON_Delete 잊지 말 것
        perror("[Camera] timestamp is not in upd socket");
        cJSON_Delete(root);
        return -1;
    }

    // 2. Detections Array
    cJSON *detections = cJSON_GetObjectItem(root, "detections");
    int count = 0;

    if (cJSON_IsArray(detections)) {
        cJSON *obj = NULL;
        cJSON_ArrayForEach(obj, detections) {
            if (count >= MAX_BBOX_OBJS) break;
            
            // 각 필드를 개별 객체로 가져옴
            cJSON *id = cJSON_GetObjectItem(obj, "id");
            cJSON *x  = cJSON_GetObjectItem(obj, "x");
            cJSON *y  = cJSON_GetObjectItem(obj, "y");
            cJSON *w  = cJSON_GetObjectItem(obj, "w");
            cJSON *h  = cJSON_GetObjectItem(obj, "h");

            // 데이터 타입 검증 및 할당
            if (cJSON_IsNumber(id)) item->objects[count].class_id = id->valueint;
            if (cJSON_IsNumber(x))  item->objects[count].x = (float)x->valuedouble;
            if (cJSON_IsNumber(y))  item->objects[count].y = (float)y->valuedouble;
            if (cJSON_IsNumber(w))  item->objects[count].w = (float)w->valuedouble;
            if (cJSON_IsNumber(h))  item->objects[count].h = (float)h->valuedouble;
            
            printf("[ID: %d, x: %f, y: %f, w: %f, h: %f\n]", 
                item->objects[count].class_id,
                item->objects[count].x,
                item->objects[count].y,
                item->objects[count].h,
                item->objects[count].w
            );
            count++;
        }
    }
    
    item->obj_count = count;
    cJSON_Delete(root); // 메모리 해제 필수
    return 0;
}

int main(int argc, char *argv[]) {
    // 기본값: Front Camera
    CameraQueue *front_q = NULL;
    CameraQueue *back_q = NULL;

    int port = UDP_PORT;

    // 1. 공유 메모리 연결 (init 프로세스가 이미 만들어 뒀다고 가정)
    if (attach_shm(SHM_NAME_FRONT_CAMERA, front_q) < 0) {
        fprintf(stderr, "[Camera] Front SHM not ready. Is 'init' process running?\n");
        return -1;
    }

    if (attach_shm(SHM_NAME_BACK_CAMERA, back_q) < 0) {
        fprintf(stderr, "[Camera] back SHM not ready. Is 'init' process running?\n");
        return -1;
    }

    // 2. UDP 소켓 오픈
    int sockfd = init_udp_server(port);
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);
    char buffer[BUFFER_SIZE];

    // 3. 생산자 루프 (Receive -> Parse -> Produce)
    while (1) {
        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE - 1, 0, NULL, NULL);
        if (n > 0) {
            buffer[n] = '\0';
            CameraItem temp_item;
            int received_cam_id = -1;

            if (parse_json_to_item(buffer, &temp_item, &received_cam_id) == 0) {
                // [핵심] cam_id에 따라 타겟 큐 결정
                CameraQueue *target_q = (received_cam_id == 0) ? front_q : back_q;

                if (target_q) {
                    sem_wait(&target_q->sem_empty);
                    pthread_mutex_lock(&target_q->mutex);

                    int idx = target_q->head;
                    memcpy(&target_q->buffer[idx], &temp_item, sizeof(CameraItem));
                    target_q->head = (target_q->head + 1) % QUEUE_SIZE;

                    pthread_mutex_unlock(&target_q->mutex);
                    sem_post(&target_q->sem_full);
                }
            }
        }
    }
    
    munmap(front_q, sizeof(CameraQueue));
    munmap(back_q, sizeof(CameraQueue));
    close(sockfd);
    return 0;
}