#include <cjson/cJSON.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include "../common.h"

#define BUFFER_SIZE (1<<16)

volatile sig_atomic_t stop_flag = 0;

void handle_sigint(int sig)
{
    printf("\n[수신] SIGINT (%d) 발생! 프로그램을 정리하고 종료합니다.\n", sig);
    stop_flag = 1;
}

int64_t get_timestamp_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    // 초를 밀리초로 변환 + 나노초를 밀리초로 변환
    return (int64_t)ts.tv_sec * 1000 + (ts.tv_nsec / 1000000);
}

// 1. 공유 메모리 연결
int attach_shm(const char *shm_name, CameraQueue **shm_q)
{
    int fd = shm_open(shm_name, O_RDWR, 0666);
    if (fd == -1)
    {
        perror("[Camera] shm_open failed");
        return -1;
    }

    void *ptr = mmap(0, sizeof(CameraQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (ptr == MAP_FAILED)
    {
        perror("[Camera] mmap failed");
        return -1;
    }

    *shm_q = (CameraQueue *)ptr;
    return 0;
}

// 2. UDP 서버 초기화
int init_udp_server(int port)
{
    int sockfd;
    struct sockaddr_in servaddr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("[Camera] Socket creation failed");
        exit(EXIT_FAILURE);
    }

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        perror("[Camera] Bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("[Camera] UDP Listening on %d...\n", port);
    return sockfd;
}

// 3. JSON 파싱 및 데이터 출력 로직
int parse_json_to_item(const char *json_str, CameraItem *item, int *out_cam_id)
{
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL)
        return -1;

    // Camera ID
    cJSON *cam = cJSON_GetObjectItem(root, "cam_id");
    if (cJSON_IsNumber(cam))
        *out_cam_id = cam->valueint;

    // Status (0: LOST, 1: OK)
    cJSON *status = cJSON_GetObjectItem(root, "status");
    int status_val = cJSON_IsNumber(status) ? status->valueint : 0;

    // Timestamp (float)
    cJSON *ts = cJSON_GetObjectItem(root, "timestamp");
    if (cJSON_IsNumber(ts))
        item->timestamp = ts->valuedouble;

    uint64_t current_time = get_timestamp_ms() * 1000ll;
    if ((int64_t)current_time - (int64_t)(item->timestamp) < 0)
    {
        printf(" Timestamp Error!\n");
    }
    // 터미널 출력 헤더
    // printf("\n[Cam: %d | Status: %s | TS: %ld]",
    //        *out_cam_id,
    //        (status_val == 1) ? "\033[0;32mOK\033[0m" : "\033[0;31mLOST\033[0m",
    //        item->timestamp);

    // Detections 파싱
    cJSON *detections = cJSON_GetObjectItem(root, "detections");
    int count = 0;
    if (cJSON_IsArray(detections))
    {
        cJSON *obj = NULL;
        cJSON_ArrayForEach(obj, detections)
        {
            if (count >= MAX_BBOX_OBJS)
                break;
            cJSON *id = cJSON_GetObjectItem(obj, "id");
            cJSON *x = cJSON_GetObjectItem(obj, "x");
            cJSON *y = cJSON_GetObjectItem(obj, "y");
            cJSON *w = cJSON_GetObjectItem(obj, "w");
            cJSON *h = cJSON_GetObjectItem(obj, "h");
            cJSON *d = cJSON_GetObjectItem(obj, "d");

            if (cJSON_IsNumber(id))
            {
                if (id->valueint < 0 || id->valueint >= 4)
                {
                    printf(" Invalid class ID: %d\n", id->valueint);
                    continue; // 잘못된 클래스 ID는 무시
                }
                item->objects[count].class_id = id->valueint;
            }
            if (cJSON_IsNumber(x))
                item->objects[count].x = (float)x->valuedouble;
            if (cJSON_IsNumber(y))
                item->objects[count].y = (float)y->valuedouble;
            if (cJSON_IsNumber(w))
                item->objects[count].w = (float)w->valuedouble;
            if (cJSON_IsNumber(h))
                item->objects[count].h = (float)h->valuedouble;
            if (cJSON_IsNumber(d))
                item->objects[count].distance = (float)d->valuedouble;

            // 전방 카메라
            // 좌표 전체 출력
            //   printf(" [ID:%d, x:%.1f, y:%.1f, w:%.1f, h:%.1f\n",
            //          item->objects[count].class_id,
            //          item->objects[count].x, item->objects[count].y,
            //          item->objects[count].w, item->objects[count].h
            //          );
            count++;
        }
    }

    // base image data 파싱
    cJSON *data = cJSON_GetObjectItem(root, "image_base64");
    if (cJSON_IsString(data) && (data->valuestring != NULL))
    {
        size_t source_len = strlen(data->valuestring);
        // 2. 버퍼가 수용 가능한 최대 길이 (null 문자 제외)
        size_t max_safe_len = sizeof(item->data) - 1;
        // 3. 둘 중 더 작은 값만큼만 복사 (버퍼 오버플로우 방지)
        size_t copy_len = (source_len < max_safe_len) ? source_len : max_safe_len;
        // 4. memcpy 실행
        memcpy(item->data, data->valuestring, copy_len);

        // 5. 마지막에 반드시 null 문자 추가
        item->data[copy_len] = '\0';
    }
    else
    {
        item->data[0] = '\0'; // 빈 문자열로 초기화
    }

    item->obj_count = count;
    cJSON_Delete(root);
    return 0;
}

// 4. Main 함수
int main(int argc, char *argv[])
{
    CameraQueue *front_q = NULL;
    CameraQueue *back_q = NULL;
    int port = UDP_PORT;

    signal(SIGINT, handle_sigint);

    // 1. 공유 메모리 연결
    if (attach_shm(SHM_NAME_FRONT_CAMERA, &front_q) < 0)
    {
        fprintf(stderr, "[Camera] Front SHM not ready.\n");
        return -1;
    }

    if (attach_shm(SHM_NAME_BACK_CAMERA, &back_q) < 0)
    {
        fprintf(stderr, "[Camera] Back SHM not ready.\n");
        return -1;
    }

    // 2. UDP 소켓 오픈
    int sockfd = init_udp_server(port);
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);
    char buffer[BUFFER_SIZE];

    // 3. 생산자 루프 (Receive -> Parse -> Produce)
    while (!stop_flag)
    {
        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE - 1, 0, (struct sockaddr *)&cliaddr, &len);
        //printf("Received packet from %s:%d\n", inet_ntoa(cliaddr.sin_addr), ntohs(cliaddr.sin_port));
        if (n > 0)
        {
            buffer[n] = '\0';
            CameraItem temp_item;
            int received_cam_id = -1;
            if (parse_json_to_item(buffer, &temp_item, &received_cam_id) == 0)
            {
                // cam_id에 따라 타겟 큐 결정 (0: Front, 1: Back)
                CameraQueue *target_q = (received_cam_id == 0) ? front_q : back_q;

                if (target_q)
                {
                    // 소비자가 데이터를 가져갈 때까지 대기
                    int val;
                    sem_getvalue(&target_q->sem_empty, &val);
                    //printf("[Debug] Before wait - sem_empty: %d, tail: %d\n", val, target_q->tail);

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
