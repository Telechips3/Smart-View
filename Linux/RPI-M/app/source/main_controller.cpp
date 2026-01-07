#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <arpa/inet.h>
#include <opencv2/opencv.hpp>
#include "ydlidar_sdk.h"
#include "../common.h"
#include "../spi/common/protocol.h"

using namespace std;
using namespace cv;

float angF = -0.023f, yF = -0.150f, sF = 0.76f;
float angR = -0.023f, yR = -0.150f, sR = 0.79f;
volatile sig_atomic_t stop_flag = 0; // 시그널 핸들러와 공유할 인자 역할

// [차량 규격 설정] (단위: 미터)
// 차량 폭 15cm -> 좌우 반폭 7.5cm (0.075m)
const float VEH_HALF_WIDTH = 0.075f;

// 차량 길이 24cm -> 라이다가 중앙에 있다면 앞뒤로 12cm (0.12m)
const float VEH_FRONT_LEN = 0.12f;
const float VEH_REAR_LEN = 0.12f;

void handle_sigint(int sig)
{
    printf("\n[수신] SIGINT (%d) 발생! 프로그램을 정리하고 종료합니다.\n", sig);
    stop_flag = 1;
}

/**
 * [차량 외곽선 보정 함수]
 * 라이다 측정 각도에 따라 센서 중심에서 해당 방향의 범퍼 끝까지의 거리를 계산합니다.
 */
float getVehicleOffset(float angle_rad, bool isFront)
{
    // 1. 각도 정규화 (전/후방 구분 없이 중심점 기준 상대 각도로 변환)
    float adjusted_angle = isFront ? angle_rad : (angle_rad - M_PI);
    while (adjusted_angle > M_PI)
        adjusted_angle -= 2 * M_PI;
    while (adjusted_angle < -M_PI)
        adjusted_angle += 2 * M_PI;

    float abs_angle = std::abs(adjusted_angle);
    float L = isFront ? VEH_FRONT_LEN : VEH_REAR_LEN; // 전/후방 길이 선택
    float W = VEH_HALF_WIDTH;                         // 차량 폭

    // 코사인 값이 너무 작아지는 것 방지
    float cos_v = std::max(std::cos(abs_angle), 0.1f);
    float sin_v = std::max(std::sin(abs_angle), 0.1f);

    float dist_to_front = L / cos_v;
    float dist_to_side = W / sin_v;

    // 핵심: 오프셋이 차체 크기보다 커지지 않도록 제한
    float offset = std::min(dist_to_front, dist_to_side);
    return std::min(offset, 0.25f); // 차가 작으므로 최대 보정치를 25cm로 제한
}

void drawContour(Mat &img, const vector<LidarPoint> &pts, Scalar color)
{
    if (pts.size() < 2)
        return;

    for (size_t i = 1; i < pts.size(); i++)
    {
        // 인접한 포인트 사이의 거리 차이가 작을 때만 선을 연결
        if (std::abs(pts[i].dist - pts[i - 1].dist) < 0.5f)
        {
            // .pt 대신 .u, .v를 사용하여 Point 객체를 생성합니다.
            Point p1((int)pts[i - 1].u, (int)pts[i - 1].v);
            Point p2((int)pts[i].u, (int)pts[i].v);

            // 이미지 범위 안인지 확인 후 그리기
            if (p1.x >= 0 && p1.x < img.cols && p2.x >= 0 && p2.x < img.cols)
            {
                line(img, p1, p2, color, 2, LINE_AA);
            }
        }
    }
}

int fd;
void calibrateAndMatch(CameraQueue *cur_q, const vector<LidarPoint> &currentPts, Mat &targetView, bool isfront, int64_t timestamp)
{
    CameraItem target_item; // 복사본을 저장할 로컬 변수
    const int64_t THRESHOLD = 150000;
    bool found_item = false;
    // printf("[Main] Searching Camera Queue (isfront=%d) for Timestamp: %lu\n", isfront ? 1 : 0, timestamp);

    while (!found_item)
    {
        // 큐에서 타임스탬프가 일치하는 항목을 찾음
        if (sem_trywait(&cur_q->sem_full) != 0)
        {
            // 카메라 큐가 비어있으면 일단 탈출 (다음 라이다 프레임에서 다시 시도)
            break;
        }

        pthread_mutex_lock(&cur_q->mutex);
        CameraItem *c_item = &cur_q->buffer[cur_q->head];
        int64_t diff = (int64_t)timestamp - (int64_t)c_item->timestamp;
        // printf("[Main] Comparing Camera Timestamp: %lu, Diff: %ld\n", c_item->timestamp, diff);

        // 2. 타임스탬프 비교 로직
        if (std::abs(diff) <= THRESHOLD)
        {
            // [매칭 성공] 오차 범위 내에 있음
            memcpy(&target_item, c_item, sizeof(CameraItem));
            found_item = true;

            // 매칭된 데이터는 소모했으므로 head 이동
            cur_q->head = (cur_q->head + 1) % QUEUE_SIZE;
            pthread_mutex_unlock(&cur_q->mutex);
            sem_post(&cur_q->sem_empty);
            break;
        }
        else if (diff > THRESHOLD)
        {
            // [과거 데이터] 카메라 데이터가 라이다보다 너무 오래됨 -> 버리고 다음 것 확인
            cur_q->head = (cur_q->head + 1) % QUEUE_SIZE;
            pthread_mutex_unlock(&cur_q->mutex);
            sem_post(&cur_q->sem_empty);
            // continue; 다음 카메라 아이템 확인
        }
        else
        {
            // [미래 데이터] 카메라 데이터가 라이다보다 너무 최신임
            // 현재 라이다 프레임은 버리거나, 이 카메라 데이터를 나중에 써야 함
            // 여기서는 일단 뮤텍스만 풀고 세마포어 원복 후 종료
            pthread_mutex_unlock(&cur_q->mutex);
            sem_post(&cur_q->sem_full); // 꺼내려 했던 것을 다시 원복
            break;
        }
    }

    if (found_item)
    {
        float targetA = 0.0f;
        float minD = 1000000.0f;
        for (int i = 0; i < target_item.obj_count; i++)
        {
            if (i == 1)
                break;
            BBox &b = target_item.objects[i];
            Rect box((int)b.x, (int)b.y, (int)b.w, (int)b.h);
            Rect searchBox(box.x - 10, box.y - 10, box.width + 20, box.height + 20);

            // [추가됨 1] 파란색 박스 그리기: AI가 인식한 영역이 어디인지 눈으로 확인
            rectangle(targetView, searchBox, Scalar(255, 0, 0), 2);

            targetA = 0.0f;
            minD = 1000000.0f;
            // bool found = false;

            // Lidar 포인트와 BBox 매칭. 여기에 거리 코드가 만들어져야함.

            for (const auto &lp : currentPts)
            {
                // [추가됨 2] 노란색 점 그리기: 박스 안에 들어가든 말든 일단 모든 라이다 점을 화면에 찍어봄
                // 이 점들이 파란 박스 근처에 있는지, 엉뚱한 데 있는지 확인해야 함
                if (lp.u >= 0 && lp.u < 640 && lp.v >= 0 && lp.v < 480)
                {
                    // circle(targetView, Point((int)lp.u, (int)lp.v), 2, Scalar(0, 255, 255), -1);
                }

                float getOffset = lp.dist - getVehicleOffset(lp.angle, isfront);
                // printf("Lidar Point Angle: %.2f rad, Dist: %.2f m, Vehicle Offset: %.2f m\n",
                //        lp.angle, lp.dist,
                //        getVehicleOffset(lp.angle, isfront));

                if (getOffset < minD)
                {
                    minD = getOffset;
                    targetA = lp.angle;
                }

                if (searchBox.contains(cv::Point2f(lp.u, lp.v)))
                {
                    // [추가됨 3] 빨간색 점 그리기: 박스 안에 성공적으로 들어온 점만 빨간색으로 덧칠
                    // circle(targetView, Point((int)lp.u, (int)lp.v), 3, Scalar(0, 0, 255), -1);
                }
            }

            // UART_Packet_t p = {0};
            // p.header = 0xAA;
            // p.distance = minD * 100.0f;
            // p.class_ID = target_item.objects[i].class_id;
            // p.detected = isfront ? 0 : 1;
            // p.timestamp = timestamp;
            // p.bbox_x = b.x;
            // p.bbox_y = b.y;
            // p.bbox_h = b.h;
            // p.bbox_w = b.w;

            // uint8_t crc = 0;
            // uint8_t *ptr = (uint8_t *)&p;
            // for (int i = 0; i < (int)PACKET_SIZE - 1; i++)
            //     crc ^= ptr[i];
            // p.checksum = crc;
            // write(fd, &p, PACKET_SIZE);

            if (isfront)
            {
                printf("Front is 0: %d, TimeStamp: %lu BBox %d: Distance = %.2f m\n", isfront ? 0 : 1, timestamp, target_item.objects[i].class_id, minD);
            }
        }
    }
    // 마지막에 SPI 필요
}

int main()
{
    int fd_l = shm_open(SHM_NAME_LIDAR, O_RDWR, 0666);
    int fd_f = shm_open(SHM_NAME_FRONT_CAMERA, O_RDWR, 0666);
    int fd_b = shm_open(SHM_NAME_BACK_CAMERA, O_RDWR, 0666);

    signal(SIGINT, handle_sigint);

    LidarQueue *q_l = (LidarQueue *)mmap(NULL, sizeof(LidarQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd_l, 0);
    CameraQueue *q_f = (CameraQueue *)mmap(NULL, sizeof(CameraQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd_f, 0);
    CameraQueue *q_b = (CameraQueue *)mmap(NULL, sizeof(CameraQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd_b, 0);

    vector<LidarPoint> ptsF, ptsR;
    ptsF.reserve(MAX_LIDAR_POINTS);
    ptsR.reserve(MAX_LIDAR_POINTS);

    // int fd = open("/dev/stm32_spi", O_WRONLY);
    // if (fd < 0)
    // {
    //     perror("장치 열기 실패");
    //     return -1;
    // }

    int64_t timestamp = 0;
    while (!stop_flag)
    {
        Mat viewF = Mat::zeros(480, 640, CV_8UC3);
        Mat viewR = Mat::zeros(480, 640, CV_8UC3);

        // --- [Step 1] Lidar 데이터 가져오기 ---
        // sem_trywait을 사용하면 데이터가 없을 때 기다리지 않고 넘어갑니다.
        // 실시간성을 위해 최신 데이터를 기다리려면 sem_wait을 사용하세요.
        if (sem_wait(&q_l->sem_full) == 0)
        {
            pthread_mutex_lock(&q_l->mutex);
            LidarItem *item = &q_l->buffer[q_l->head];

            // 새 데이터가 왔을 때만 기존 리스트를 지우고 새로 채움
            ptsF.clear();
            ptsR.clear();

            // [확인용 로그]
            // printf("[Main] Lidar Data Received! Count: %d, First Dist: %.2f\n", item->count, item->points[0].dist);

            for (int i = 0; i < item->count; i++)
            {
                // 각도에 따라 전방/후방 분류 (생산자 로직과 매칭)
                // 생산자에서 이미 u, v를 계산해서 넘겨주므로 적절한 view에 할당
                if (abs(item->points[i].angle) < 1.57f)
                    ptsR.push_back(item->points[i]);
                else
                    ptsF.push_back(item->points[i]); // 대략 전방 180도
            }

            timestamp = item->timestamp;
            q_l->head = (q_l->head + 1) % QUEUE_SIZE;
            pthread_mutex_unlock(&q_l->mutex);
            sem_post(&q_l->sem_empty);
        }

        // --- [Step 2] 카메라 데이터(BBox) 처리 및 매칭 ---
        // 전방(Front)과 후방(Back) 각각 처리
        calibrateAndMatch(q_f, ptsF, viewF, true, timestamp);
        calibrateAndMatch(q_b, ptsR, viewR, false, timestamp);

        // [화면 띄우기 코드 활성화]
        // 전방(Front)과 후방(Rear) 화면을 하나로 합쳐서 보여줍니다.
        Mat combined;
        // hconcat(viewF, viewR, combined); // 두 화면을 가로로 붙임

        // imshow("Main Controller", combined); // "Main Controller"라는 창에 띄움
        //  키 입력을 1ms 대기 (이게 없으면 창이 안 뜹니다!)
        if (waitKey(1) == 'q')
            break; // 'q'를 누르면 종료
    }

    munmap(q_l, sizeof(LidarQueue));
    munmap(q_f, sizeof(CameraQueue));
    munmap(q_b, sizeof(CameraQueue));
    close(fd_l);
    close(fd_f);
    close(fd_b);
    return 0;
}
