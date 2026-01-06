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


// [차량 규격 설정] (단위: 미터)
// 차량 폭 15cm -> 좌우 반폭 7.5cm (0.075m)
const float VEH_HALF_WIDTH = 0.075f; 

// 차량 길이 24cm -> 라이다가 중앙에 있다면 앞뒤로 12cm (0.12m)
const float VEH_FRONT_LEN  = 0.12f; 
const float VEH_REAR_LEN   = 0.12f;

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

void calibrateAndMatch(CameraQueue *cur_q, const vector<LidarPoint> &currentPts, Mat &targetView, bool isfront, int64_t timestamp)
{
    CameraItem target_item; // 복사본을 저장할 로컬 변수
    bool found_item = false;
    // printf("[Main] Searching Camera Queue (isfront=%d) for Timestamp: %lu\n", isfront ? 1 : 0, timestamp);

    while (!found_item)
    {
        // 큐에서 타임스탬프가 일치하는 항목을 찾음
        if (sem_wait(&cur_q->sem_full) == 0)
        {
            pthread_mutex_lock(&cur_q->mutex);
            CameraItem *c_item = &cur_q->buffer[cur_q->head];
            // printf("[Main] Checking Camera Item Timestamp: %lu\n", c_item->timestamp);

            // 50ms 이내의 오차를 허용하여 매칭
            if ((int64_t)timestamp - (int64_t)c_item->timestamp <= 100000)
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

    // if (found_item)
    // {
    //     for (int i = 0; i < target_item.obj_count; i++)
    //     {
    //         BBox &b = target_item.objects[i];

    //         // [핵심 변경 1] 박스 영역 축소 (Core Box)
    //         // AI 박스 전체를 쓰면 배경(벽)이 포함될 수 있으므로, 박스 중앙 50% 영역만 신뢰합니다.
    //         int center_x = (int)(b.x + b.w / 2);
    //         int center_y = (int)(b.y + b.h / 2);
    //         int core_w = (int)(b.w * 0.5);
    //         int core_h = (int)(b.h * 0.5);

    //         // 화면 밖으로 나가지 않도록 좌표 검사
    //         Rect coreBox(center_x - core_w / 2, center_y - core_h / 2, core_w, core_h);

    //         // 시각적 디버깅용: Core Box 그리기 (파란색 얇은 선)
    //         // rectangle(targetView, coreBox, Scalar(255, 0, 0), 1);

    //         // 거리값과 각도를 저장할 후보 리스트
    //         vector<pair<float, float>> candidates;

    //         // Lidar 포인트 전수 조사
    //         for (const auto &lp : currentPts)
    //         {
    //             // 투영된 좌표(u,v)가 Core Box 안에 들어오는지 확인
    //             // printf("Lidar Point U: %.2f, V: %.2f D: %.2f | CoreBox [x:%d, y:%d, w:%d, h:%d]\n",
    //             //        lp.u, lp.v, lp.dist,
    //             //        coreBox.x, coreBox.y, coreBox.width, coreBox.height);
    //             float real_dist = lp.dist - getVehicleOffset(lp.angle, isfront);

    //             // 유효한 거리(0m 이상)인 경우에만 후보군에 등록
    //             if (real_dist > 0.0f)
    //             {
    //                 candidates.push_back({real_dist, lp.angle});
    //             }

    //             if (coreBox.contains(cv::Point2f(lp.u, lp.v)))
    //             {
    //                 // 차량 범퍼 기준 보정 거리 계산
    //             }
    //         }

    //         float finalDist = 0.0f;
    //         float finalAng = 0.0f;
    //         bool valid_obj = false;

    //         // [핵심 변경 2] Robust Filtering (노이즈 제거)
    //         if (!candidates.empty())
    //         {
    //             // 거리를 기준으로 오름차순 정렬 (가까운 순서)
    //             std::sort(candidates.begin(), candidates.end());

    //             // 후보 점이 적을 때는(3개 미만) 그냥 가장 가까운 값 사용
    //             if (candidates.size() < 3)
    //             {
    //                 finalDist = candidates[0].first;
    //                 finalAng = candidates[0].second;
    //             }
    //             else
    //             {
    //                 // 후보 점이 많으면 하위 10~20% 지점의 값을 사용 (튀는 노이즈 무시)
    //                 // 예: 점이 100개면 10번째로 가까운 점 선택 -> 아주 가까운 먼지(0~9번째) 무시됨
    //                 int safe_idx = (int)(candidates.size() * 0.15); // 상위 15% 지점
    //                 finalDist = candidates[safe_idx].first;
    //                 finalAng = candidates[safe_idx].second;
    //             }
    //             valid_obj = true;
    //         }
    //         // [결과 출력] 여기에 printf를 추가했습니다.
    //         if (valid_obj)
    //         {
    //             // 1. 화면에 박스 그리기
    //             //Rect drawBox((int)b.x, (int)b.y, (int)b.w, (int)b.h);
    //             //rectangle(targetView, drawBox, Scalar(0, 255, 0), 2);

    //             // 2. 화면에 거리 텍스트 쓰기
    //             // char text[50];
    //             // sprintf(text, "%.2fm", finalDist);
    //             // putText(targetView, text, Point(drawBox.x, drawBox.y - 5), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);

    //             // 3. [추가됨] 터미널에 거리값 출력
    //             printf("⚠️ [%s] ID:%d | 거리: %.2f m (포인트:%lu개)\n",
    //                    isfront ? "전방" : "후방",
    //                    target_item.objects[i].class_id,
    //                    finalDist,
    //                    candidates.size());
    //         }
    //     }

    if (found_item)
    {
        for (int i = 0; i < target_item.obj_count; i++)
        {
            BBox &b = target_item.objects[i];
            Rect box((int)b.x, (int)b.y, (int)b.w, (int)b.h);
            Rect searchBox(box.x - 10, box.y - 10, box.width + 20, box.height + 20);

            // [추가됨 1] 파란색 박스 그리기: AI가 인식한 영역이 어디인지 눈으로 확인
            rectangle(targetView, searchBox, Scalar(255, 0, 0), 2);

            float targetA = 0.0f;
            float minD = 1000000.0f;
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

            printf("Front is 0: %d, TimeStamp: %lu BBox %d: Distance = %.2f m\n", isfront ? 0 : 1, timestamp, target_item.objects[i].class_id, minD);
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
            // printf("[Main] Lidar Data Received! Count: %d, First Dist: %.2f\n", item->count, item->points[0].dist);

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