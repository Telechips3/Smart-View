#include "common.h"
#include <cjson/cJSON.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#define UDP_PORT 5005
#define BUFFER_SIZE 4096

// 전역 공유 메모리 포인터
static CameraQueue *shm_q = NULL;

// 1. 공유 메모리 연결 (이미 생성된 메모리에 붙기만 함)
int attach_shm(const char *shm_name) {
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

// 3. JSON 파싱 (문자열 -> 구조체)
int parse_json_to_item(const char *json_str, CameraItem *item) {
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) return -1;

    // Timestamp
    cJSON *ts = cJSON_GetObjectItem(root, "timestamp");
    if (cJSON_IsNumber(ts)) item->timestamp = (float)ts->valuedouble;
    else {
        perror("[Camera] timestamp is not in upd socket");
        exit(EXIT_FAILURE);
    }

    // Objects
    cJSON *objects = cJSON_GetObjectItem(root, "detections");
    cJSON *obj = NULL;
    int count = 0;

    if (cJSON_IsArray(objects)) {
        cJSON_ArrayForEach(obj, objects) {
            if (count >= MAX_BBOX_OBJS) break;
            
            //이 부분 읽어드리는 json정확하게 수정 필요
            cJSON *id = cJSON_GetObjectItem(obj, "id");
            cJSON *x  = cJSON_GetObjectItem(obj, "x");
            cJSON *y  = cJSON_GetObjectItem(obj, "y");
            cJSON *w  = cJSON_GetObjectItem(obj, "w");
            cJSON *h  = cJSON_GetObjectItem(obj, "h");

            if (cJSON_IsNumber(id)) item->objects[count].track_id = id->valueint;
            if (cJSON_IsNumber(x))  item->objects[count].x = (float)x->valuedouble;
            if (cJSON_IsNumber(y))  item->objects[count].y = (float)y->valuedouble;
            if (cJSON_IsNumber(w))  item->objects[count].w = (float)w->valuedouble;
            if (cJSON_IsNumber(h))  item->objects[count].h = (float)h->valuedouble;

            count++;
        }
    }
    item->obj_count = count;
    cJSON_Delete(root);
    return 0;
}

int main(int argc, char *argv[]) {
    // 기본값: Front Camera
    const char *target_shm = SHM_NAME_FRONT_CAMERA;
    int port = UDP_PORT;

    if (argc > 1 && strcmp(argv[1], "back") == 0) {
        target_shm = SHM_NAME_BACK_CAMERA;
        port = UDP_PORT + 1;
    }

    // 1. 공유 메모리 연결 (init 프로세스가 이미 만들어 뒀다고 가정)
    if (attach_shm(target_shm) < 0) {
        fprintf(stderr, "[Camera] SHM not ready. Is 'init' process running?\n");
        return -1;
    }

    // 2. UDP 소켓 오픈
    int sockfd = init_udp_server(port);
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);
    char buffer[BUFFER_SIZE];

    // 3. 생산자 루프 (Receive -> Parse -> Produce)
    while (1) {
        // (A) UDP 수신
        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE - 1, 0, (struct sockaddr *)&cliaddr, &len);
        if (n > 0) {
            buffer[n] = '\0';
            CameraItem temp_item;
            memset(&temp_item, 0, sizeof(CameraItem));

            // (B) 파싱 성공 시 공유 메모리에 씀
            if (parse_json_to_item(buffer, &temp_item) == 0) {
                
                // --- [Producer Critical Section] ---
                sem_wait(&shm_q->sem_empty);      // 1. 빈 방 기다림
                pthread_mutex_lock(&shm_q->mutex); // 2. 잠금

                // 3. 데이터 복사 (Enqueue)
                int idx = shm_q->head;
                memcpy(&shm_q->buffer[idx], &temp_item, sizeof(CameraItem));
                shm_q->head = (shm_q->head + 1) % QUEUE_SIZE;

                pthread_mutex_unlock(&shm_q->mutex); // 4. 잠금 해제
                sem_post(&shm_q->sem_full);        // 5. 내용물 있다고 알림
                // -----------------------------------
            }
        }
    }
    
    munmap(shm_q, sizeof(CameraQueue));
    close(sockfd);
    return 0;
}