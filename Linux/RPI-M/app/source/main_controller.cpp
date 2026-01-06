#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <arpa/inet.h>
#include <opencv2/opencv.hpp>
#include "ydlidar_sdk.h"
#include "../common.h"

using namespace std;
using namespace cv;

float angF = -0.023f, yF = -0.150f, sF = 0.76f;
float angR = -0.023f, yR = -0.150f, sR = 0.79f;
volatile sig_atomic_t stop_flag = 0; // 시그널 핸들러와 공유할 인자 역할

/**
 * [차량 규격 설정]
 * 라이다 센서의 위치를 중심으로 차량의 실제 범퍼 외곽까지의 거리를 설정합니다.
 * 이 값들을 기준으로 측정된 라이다 거리에서 차량 크기를 차감합니다.
 */
const float VEH_HALF_WIDTH = 0.15f;  // 차량 폭의 절반 (중심에서 좌/우 거리)
const float VEH_FRONT_LEN = 0.20f;   // 라이다 중심에서 전방 범퍼 끝까지의 거리
const float VEH_REAR_LEN = 0.10f;    // 라이다 중심에서 후방 범퍼 끝까지의 거리


void handle_sigint(int sig) {
    printf("\n[수신] SIGINT (%d) 발생! 프로그램을 정리하고 종료합니다.\n", sig);
    stop_flag = 1;
}

/**
 * [차량 외곽선 보정 함수]
 * 라이다 측정 각도에 따라 센서 중심에서 해당 방향의 범퍼 끝까지의 거리를 계산합니다.
 */
float getVehicleOffset(float angle_rad, bool isFront) {
    float abs_angle = abs(angle_rad);
    float L = isFront ? VEH_FRONT_LEN : VEH_REAR_LEN; // 전/후방 길이 선택
    float W = VEH_HALF_WIDTH;                         // 차량 폭

    if (abs_angle < 1e-6) return L; // 정면일 경우 직진 거리 반환

    // 삼각함수를 이용하여 직사각형 차량의 경계면까지의 거리를 계산
    float boundary_dist = L / cos(abs_angle); // 앞/뒤 면까지의 거리
    float side_dist = W / sin(abs_angle);     // 좌/우 측면까지의 거리
    
    // 두 값 중 짧은 값이 실제 차량의 외곽선까지의 거리임
    return abs(min(boundary_dist, side_dist));
}

void drawContour(Mat &img, const vector<LidarPoint> &pts, Scalar color)
{
    if (pts.size() < 2) return;

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

void calibrateAndMatch(CameraQueue *cur_q, const vector<LidarPoint> &currentPts, Mat &targetView, bool isfront, int64_t timestamp)
{
    CameraItem target_item; // 복사본을 저장할 로컬 변수
    bool found_item = false;
    //printf("[Main] Searching Camera Queue (isfront=%d) for Timestamp: %lu\n", isfront ? 1 : 0, timestamp);
    while(!found_item)
    {
        // 큐에서 타임스탬프가 일치하는 항목을 찾음
        if (sem_wait(&cur_q->sem_full) == 0)
        {
            pthread_mutex_lock(&cur_q->mutex);
            CameraItem *c_item = &cur_q->buffer[cur_q->head];
            //printf("[Main] Checking Camera Item Timestamp: %lu\n", c_item->timestamp);

            if ( abs(c_item->timestamp - timestamp) <= 5000000 )
            {
                // 타임스탬프가 일치하면 로컬 변수에 복사
                memcpy(&target_item, c_item, sizeof(CameraItem));
                found_item = true;
            }
            cur_q->head = (cur_q->head + 1) % QUEUE_SIZE;
            pthread_mutex_unlock(&cur_q->mutex);
            sem_post(&cur_q->sem_empty);
        }
    }
    
    if (found_item)
    {
        for (int i = 0; i < target_item.obj_count; i++)
        {
            BBox &b = target_item.objects[i];
            Rect box((int)b.x, (int)b.y, (int)b.w, (int)b.h);
            Rect searchBox(box.x - 10, box.y - 10, box.width + 20, box.height + 20);

            float minD = 1000000.0f, targetA = 0.0f;
            bool found = false;

            // Lidar 포인트와 BBox 매칭. 여기에 거리 코드가 만들어져야함.
            for (const auto &lp : currentPts)
            {
                if (searchBox.contains(cv::Point2f(lp.u, lp.v)))
                {
                    float getOffset = lp.dist - getVehicleOffset(lp.angle, isfront); // 화면 상단이 전방
                    // printf("Lidar Point Angle: %.2f rad, Dist: %.2f m, Vehicle Offset: %.2f m, Adjusted Dist: %.2f m\n",
                    //        lp.angle, lp.dist,
                    //        getVehicleOffset(lp.angle, isfront),
                    //        getOffset);
                    if (getOffset < minD)
                    {
                        minD = getOffset;
                        targetA = lp.angle;
                        found = true;
                    }
                }
            }

            printf("Front is 0: %d, TimeStamp: %lu BBox %d: Distance = %.2f m\n", isfront ? 0: 1, timestamp, target_item.objects[i].class_id, minD);
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
            //printf("[Main] Lidar Data Received! Count: %d, First Dist: %.2f\n", item->count, item->points[0].dist);

            for (int i = 0; i < item->count; i++)
            {
                // 각도에 따라 전방/후방 분류 (생산자 로직과 매칭)
                // 생산자에서 이미 u, v를 계산해서 넘겨주므로 적절한 view에 할당
                if (abs(item->points[i].angle) < 1.57f)
                    ptsF.push_back(item->points[i]); // 대략 전방 180도
                else
                    ptsR.push_back(item->points[i]);
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
    }

    munmap(q_l, sizeof(LidarQueue));
    munmap(q_f, sizeof(CameraQueue));
    munmap(q_b, sizeof(CameraQueue));
    close(fd_l);
    close(fd_f);
    close(fd_b);
    return 0;
}