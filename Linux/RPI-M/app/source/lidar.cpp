#include <iostream>
#include <vector>
#include <string>
#include <sys/socket.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "../common.h"
#include "ydlidar_sdk.h"

using namespace std;
using namespace cv;

// 카메라 파라미터(보정값)
Mat mtxF = (Mat_<double>(3, 3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat mtxR = (Mat_<double>(3, 3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);

// 라이다-카메라 외부 파라미터 보정
float angF = -0.023f, yF = -0.150f, sF = 0.76f;
float angR = -0.023f, yR = -0.150f, sR = 0.79f;

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

int main()
{
    int fd = shm_open(SHM_NAME_LIDAR, O_RDWR, 0666);
    if (fd == -1)
    {
        perror("Producer: shm_open failed");
        return -1;
    }

    LidarQueue *q = (LidarQueue *)mmap(NULL, sizeof(LidarQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (q == MAP_FAILED)
    {
        perror("Producer: mmap failed");
        return -1;
    }

    signal(SIGINT, handle_sigint);

    // --- 1. 라이다 하드웨어 초기화 및 구동 섹션 ---
    os_init(); // YDLidar SDK 내부 운영체제 자원 초기화

    // 라이다 제어를 위한 객체 생성 (동적 할당)
    YDLidar *laser = lidarCreate();

    // 라이다가 연결된 시리얼 포트 경로와 통신 속도 설정
    const char *port = "/dev/ttyUSB0";
    int baud = 230400; // 초당 230,400 비트 전송 (YDLidar 표준 속도)

    // 생성한 라이다 객체에 포트와 보레이트 옵션을 적용
    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baud, sizeof(int));

    // 라이다 장치 초기화 시도 (성공 시 통신 준비 완료)
    if (!initialize(laser))
    {
        fprintf(stderr, "Lidar Initialize failed\n");
        return -1;
    }

    // 라이다 모터를 가동하고 실제로 레이저 스캔 시작
    turnOn(laser);

    // 라이다 한 바퀴 스캔 데이터를 담을 구조체 선언
    LaserFan scan;
    // 최대 2048개의 점을 담을 수 있는 메모리 공간을 미리 할당 (성능 최적화)
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * MAX_LIDAR_POINTS);

    printf("[Lidar Producer] Ready to produce data...\n");
    // [시각화 설정] 탑뷰 화면 크기 및 축척
    const int winSize = 600;
    const int cx = winSize / 2;
    const int cy = winSize / 2;
    const float scale = 50.0f; // 1미터 = 50픽셀 (화면 반경 약 6m 표현)

    while (!stop_flag)
    {
        if (doProcessSimple(laser, &scan))
        {

            // 1. 공유 메모리 잠금 (데이터 쓰기 시작)
            // 주의: for문 밖에서 한 번만 잠가야 합니다!
            LidarItem item;
            item.count = 0;
            item.timestamp = get_timestamp_ms() * 1000ll;

            // [시각화] 매 프레임마다 빈 화면(검은색) 생성
            Mat topView = Mat::zeros(winSize, winSize, CV_8UC3);

            // [시각화] 기준 원 그리기 (1m, 2m, 3m)
            for (int r = 1; r <= 4; r++)
            {
                circle(topView, Point(cx, cy), (int)(r * scale), Scalar(50, 50, 50), 1);
                putText(topView, to_string(r) + "m", Point(cx + 5, cy - (int)(r * scale)), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(100, 100, 100));
            }
            // 십자선
            line(topView, Point(0, cy), Point(winSize, cy), Scalar(50, 50, 50), 1);
            line(topView, Point(cx, 0), Point(cx, winSize), Scalar(50, 50, 50), 1);

            // 2. 포인트 처리 루프
            for (int i = 0; i < scan.npoints; i++)
            {
                if (item.count >= MAX_LIDAR_POINTS)
                    break;

                float dist = scan.points[i].range;
                float angle = scan.points[i].angle;

                // [거리 필터]
                if (dist < 0.1f || dist > 6.0f)
                    continue;

                // [영역 판별] 전방인지 후방인지 확인
                // 전방 확인
                float aF = angle + angF;
                bool is_front = abs(aF) < (50.0f * M_PI / 180.0f);

                // 후방 확인 (각도 회전 및 정규화)
                float aR = angle - M_PI + angR;
                while (aR > M_PI)
                    aR -= 2 * M_PI;
                while (aR < -M_PI)
                    aR += 2 * M_PI;
                bool is_rear = abs(aR) < (50.0f * M_PI / 180.0f);

                // [시각화] 탑뷰 좌표 계산 (화면 그리기용)
                // 라이다 데이터를 평면 지도(Top-View) 좌표로 변환
                int px = cx + (int)(dist * sin(angle) * scale);
                int py = cy - (int)(dist * cos(angle) * scale);

                // [시각화] 점 찍기 (색상 구분)
                Scalar ptColor = Scalar(80, 80, 80); // 기본 회색 (측면/무시됨)
                if (is_front)
                    ptColor = Scalar(0, 255, 0); // 전방: 초록
                if (is_rear)
                    ptColor = Scalar(0, 0, 255); // 후방: 빨강

                if (px >= 0 && px < winSize && py >= 0 && py < winSize)
                {
                    circle(topView, Point(px, py), 2, ptColor, -1);
                }

                // [로직 필터] 전방도 아니고 후방도 아니면(측면이면) 여기서 중단 -> 큐에 안 들어감!
                if (!is_front && !is_rear)
                    continue;

                // --- 여기부터는 유효한(전/후방) 데이터만 진입 ---
                LidarPoint lp;
                lp.dist = dist;
                lp.angle = angle;
                lp.x = dist * sin(angle);
                lp.y = dist * cos(angle);

                bool is_valid_proj = false;

                // [전방 투영]
                if (is_front)
                {
                    float Z = dist * cos(aF);
                    float X = dist * -sin(aF);
                    float Y = -0.05f; // 라이다가 5cm 위 (마이너스 부호 확인)

                    if (Z > 0.1f)
                    {
                        lp.u = (float)((mtxF.at<double>(0, 0) * X / Z) + mtxF.at<double>(0, 2));
                        lp.v = (float)((mtxF.at<double>(1, 1) * Y / Z) + mtxF.at<double>(1, 2));
                        is_valid_proj = true;
                    }
                }
                // [후방 투영]
                else if (is_rear)
                {
                    float Z = dist * cos(aR);
                    float X = dist * -sin(aR);
                    float Y = -0.05f; // 라이다가 5cm 위 (마이너스 부호 확인)

                    if (Z > 0.1f)
                    {
                        lp.u = (float)((mtxR.at<double>(0, 0) * X / Z) + mtxR.at<double>(0, 2));
                        lp.v = (float)((mtxR.at<double>(1, 1) * Y / Z) + mtxR.at<double>(1, 2));
                        is_valid_proj = true;
                    }
                }

                if (is_valid_proj)
                {
                    if (lp.u >= 0 && lp.u < 640 && lp.v >= 0 && lp.v < 480)
                    {
                        item.points[item.count++] = lp;
                    }
                }
            } // for문 끝
            
            sem_wait(&q->sem_empty);
            pthread_mutex_lock(&q->mutex);

            //복사 코드 필요
            memcpy(&q->buffer[q->tail], &item, sizeof(LidarItem));
           
            q->tail = (q->tail + 1) % QUEUE_SIZE;
            pthread_mutex_unlock(&q->mutex);
            sem_post(&q->sem_full);

            // [시각화] 화면 업데이트
            //imshow("Lidar Top View", topView);
            // 'ESC' 누르면 종료
            if (waitKey(1) == 27)
                stop_flag = 1;
        }

        //     while (!stop_flag)
        //     {
        //         // 전방(F)과 후방(R) 모니터링을 위한 빈 검은색 화면(640x480)을 매 프레임 생성
        //         Mat viewF = Mat::zeros(480, 640, CV_8UC3);
        //         Mat viewR = Mat::zeros(480, 640, CV_8UC3);

        //         // 라이다 센서로부터 최신 스캔 데이터(한 바퀴)를 가져옴
        //         if (doProcessSimple(laser, &scan))
        //         {
        //             // --- [생산자 영역 시작] ---
        //             // 빈 슬롯이 생길 때까지 대기
        //             // int val;
        //             // sem_getvalue(&q->sem_empty, &val);
        //             // printf("[Debug] Before wait - sem_empty: %d, tail: %d\n", val, q->tail);

        //             sem_wait(&q->sem_empty);
        //             pthread_mutex_lock(&q->mutex);

        //             // 큐의 tail 위치에 데이터 쓰기
        //             LidarItem *item = &q->buffer[q->tail];
        //             item->count = 0;
        //             item->timestamp = get_timestamp_ms() * 1000ll;

        //             // [시각화] 매 프레임마다 빈 화면(검은색) 생성
        //             Mat topView = Mat::zeros(winSize, winSize, CV_8UC3);

        //             // [시각화] 기준 원 그리기 (1m, 2m, 3m)
        //             for(int r=1; r<=3; r++) {
        //                 circle(topView, Point(cx, cy), (int)(r*scale), Scalar(50,50,50), 1);
        //                 putText(topView, to_string(r)+"m", Point(cx+5, cy-(int)(r*scale)), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(100,100,100));
        //             }
        //             // 십자선
        //             line(topView, Point(0, cy), Point(winSize, cy), Scalar(50, 50, 50), 1);
        //             line(topView, Point(cx, 0), Point(cx, winSize), Scalar(50, 50, 50), 1);

        //             for (int i = 0; i < scan.npoints; i++)
        //             {
        //                 if (item->count >= MAX_LIDAR_POINTS)
        //                     break;

        //                 float dist = scan.points[i].range;
        //                 float angle = scan.points[i].angle;

        //                 if (dist < 0.1f || dist > 6.0f)
        //                     continue;

        //                 // 공통 구조체에 맞게 데이터 가공
        //                 LidarPoint lp;
        //                 lp.dist = dist;
        //                 lp.angle = angle;
        //                 lp.x = dist * sin(angle); // Lidar 기준 X
        //                 lp.y = dist * cos(angle); // Lidar 기준 Y (Depth)

        //                 bool is_valid_proj = false;

        //                 // 전방 투영
        //                 float aF = angle + angF;
        //                 // 시야각 내에 있는지 확인
        //                 if (abs(aF) < 50.0f * M_PI / 180.0f)
        //                 {
        //                     // 3. 카메라 좌표계 변환 (Z: 전방, X: 우측, Y: 하방)
        //                     // aF가 0도(전방)일 때 cos(0)=1 이므로 Z가 거리값이 됨 -> 맞음.
        //                     // aF가 +값(좌측?)일 때 sin은 + -> 카메라는 우측이 X+이므로 부호 확인 필요
        //                     // 보통 Lidar 각도가 반시계(+)면 좌측이 +인데, 카메라는 우측이 +라 -를 붙여야 할 수 있음.
        //                     printf("Lidar Point %d: Dist %.2f m, Angle %.3f rad\n", i, dist, angle);

        //                     float Z_cam = dist * cos(aF);
        //                     float X_cam = dist * -sin(aF); // 카메라 X축(오른쪽)과 라이다 Y축(왼쪽) 반대 고려

        //                     // 라이다가 카메라보다 '아래'에 있다면 y_pos는 양수여야 함 (단위: 미터)
        //                     // 예: 카메라보다 15cm 아래라면 0.15f
        //                     float Y_cam = -0.05f;

        //                     if (Z_cam > 0.1f)
        //                     {
        //                         // OpenCV 핀홀 모델 공식: u = fx * (X/Z) + cx
        //                         lp.u = (float)((mtxF.at<double>(0, 0) * X_cam / Z_cam) + mtxF.at<double>(0, 2));

        //                         // OpenCV 핀홀 모델 공식: v = fy * (Y/Z) + cy
        //                         lp.v = (float)((mtxF.at<double>(1, 1) * Y_cam / Z_cam) + mtxF.at<double>(1, 2));

        //                         is_valid_proj = true;
        //                     }
        //                 }

        //                 // 후방 투영
        //                 else
        //                 {
        //                     // 1. 후방 기준 각도 변환
        //                     // 라이다의 180도(PI) 지점이 후방 카메라의 정면(0도)이 되도록 회전
        //                     float aR = angle - M_PI + angR;

        //                     // 2. 각도 정규화 (범위를 -PI ~ +PI 로 맞춤)
        //                     while (aR > M_PI)
        //                         aR -= 2 * M_PI;
        //                     while (aR < -M_PI)
        //                         aR += 2 * M_PI;

        //                     // 3. 후방 카메라 시야각(FOV) 체크 (좌우 50도 이내만 처리)
        //                     if (std::abs(aR) < 50.0f * M_PI / 180.0f)
        //                     {
        //                         // 4. 후방 카메라 기준 3D 좌표계 계산
        //                         // Z_cam: 카메라 렌즈 앞쪽으로 뻗어나가는 거리
        //                         float Z_cam = dist * cos(aR);

        //                         // X_cam: 카메라의 가로축 (OpenCV는 오른쪽이 +, 라이다 각도는 왼쪽(반시계)이 +)
        //                         // 따라서 sin 값에 마이너스를 붙여 방향을 맞춰줍니다.
        //                         float X_cam = dist * -sin(aR);

        //                         // Y_cam: 카메라와 라이다의 높이 차이 (OpenCV는 아래쪽이 +)
        //                         // 라이다가 카메라보다 '아래'에 설치되어 있다면 양수(+) 값이어야 합니다.
        //                         // 예: 15cm 아래라면 0.15f. (기존 yR 변수 사용 시 부호 확인 필수)
        //                         float Y_cam = 0.15f;

        //                         // 0으로 나누기 방지 및 최소 거리 필터
        //                         if (Z_cam > 0.1f)
        //                         {
        //                             // 5. 핀홀 카메라 모델 투영 공식 (sR 제거됨)
        //                             // u = fx * (X / Z) + cx
        //                             lp.u = (float)((mtxR.at<double>(0, 0) * X_cam / Z_cam) + mtxR.at<double>(0, 2));

        //                             // v = fy * (Y / Z) + cy
        //                             lp.v = (float)((mtxR.at<double>(1, 1) * Y_cam / Z_cam) + mtxR.at<double>(1, 2));

        //                             is_valid_proj = true;
        //                         }
        //                     }
        //                 }

        //                 // 큐 인덱스 이동
        //                 q->tail = (q->tail + 1) % QUEUE_SIZE;

        //                 pthread_mutex_unlock(&q->mutex);
        //                 sem_post(&q->sem_full);
        //             }
        //         }
        //     }
    }
    printf("라이다 종료\n");
    // 1. 동적 할당된 메모리 해제: malloc으로 빌렸던 라이다 점 저장용 메모리를 OS에 반납
    free(scan.points);

    // 2. 라이다 하드웨어 정지: 회전 중인 모터를 멈추고 레이저 발사를 중단하여 장치 수명 보호
    turnOff(laser);
    lidarDestroy(&laser);
    destroyAllWindows(); // 창 닫기

    return 0;
}