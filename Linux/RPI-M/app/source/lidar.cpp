#include <iostream>
#include <vector>
#include <string>
#include <sys/socket.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "../common.h"
#include "ydlidar_sdk.h"

using namespace std;
using namespace cv;

// 카메라 보정값(내부 파라미터)
Mat mtxF = (Mat_<double>(3, 3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat mtxR = (Mat_<double>(3, 3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);

// 라이다-카메라 보정값(외부 파라미터)
float angF = -0.05f, yF = -0.05f, sF = 0.76f;
float angR = -0.05f, yR = -0.05f, sR = 0.79f;

volatile sig_atomic_t stop_flag = 0;

void handle_sigint(int sig)
{
    printf("\n[수신] SIGINT (%d) 발생! 프로그램을 정리하고 종료합니다.\n", sig);
    stop_flag = 1;
}

// 밀리초 단위 타임스탬프 생성 (데이터 동기화용)
int64_t get_timestamp_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
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

    // G4 YDLidar 하드웨어 설정
    os_init();
    YDLidar *laser = lidarCreate();
    const char *port = "/dev/ttyUSB0";
    int baud = 230400;

    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baud, sizeof(int));

    if (!initialize(laser))
    {
        fprintf(stderr, "Lidar Initialize failed\n");
        return -1;
    }

    turnOn(laser);

    LaserFan scan;
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * MAX_LIDAR_POINTS);

    printf("[Lidar Producer] Ready to produce data...\n");

    // PCL 필터 설정 (노이즈 제거용)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    while (!stop_flag)
    {
        cloud_raw->clear();
        cloud_filtered->clear();
    
        if (doProcessSimple(laser, &scan))
        {
            for (int i = 0; i < scan.npoints; i++)
            {
                if (i >= MAX_LIDAR_POINTS) break;

                float dist = scan.points[i].range;
                float angle = scan.points[i].angle;

                if (dist < 0.1f || dist > 6.0f) continue;

                // Point Cloud에 원시 데이터 추가
                pcl::PointXYZ pt;
                pt.x = dist * sin(angle);
                pt.y = dist * cos(angle);
                pt.z = 0.0f;
                cloud_raw->points.push_back(pt);
            }

            // printf("[Debug] Before SOR: %zu points\n", cloud_raw->points.size());

            // 통계적 아웃라이어 제거 (공중에 뜬 노이즈 점들 제거)
            if (cloud_raw->points.size() > 10)
            { // 최소 점 개수가 확보될 때만 실행
                sor.setInputCloud(cloud_raw);
                sor.setMeanK(10);            // 분석할 주변 점 개수
                sor.setStddevMulThresh(0.5); // 1.0 표준편차 이상 떨어지면 제거
                sor.filter(*cloud_filtered);
            }
            else
            {
                *cloud_filtered = *cloud_raw; // 점이 너무 적으면 필터 스킵
            }

            // printf("[Debug] After SOR: %zu points (Removed: %zu)\n",
            //        cloud_filtered->points.size(),
            //        cloud_raw->points.size() - cloud_filtered->points.size());

            // 공유 메모리 기록 시작 (Mutex 잠금)
            sem_wait(&q->sem_empty);
            pthread_mutex_lock(&q->mutex);

            LidarItem *item = &q->buffer[q->tail];
            item->count = 0;
            item->timestamp = get_timestamp_ms() * 1000ll;


            // // 필터링된 점들을 카메라 화면 좌표(u, v)로 투영??
            // for (const auto &pt : cloud_filtered->points)
            // {
            //     if (item->count >= MAX_LIDAR_POINTS)
            //         break;

            //     float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            //     float angle = std::atan2(pt.x, pt.y); // 원본 x, y를 다시 각도로

            //     // 공통 구조체에 맞게 데이터 가공
            //     LidarPoint lp;
            //     lp.dist = dist;
            //     lp.angle = angle;
            //     lp.x = pt.x; // Lidar 기준 X
            //     lp.y = pt.y; // Lidar 기준 Y (Depth)

            //     bool is_valid_proj = false;

            //     // 람다 함수 정의: 3D 좌표를 카메라 2D 좌표로 투영
            //     auto projectToCamera = [&](float dist, float angle_corr, float y_offset, const Mat &mtx, LidarPoint &lp) -> bool
            //     {
            //         float Z_cam = dist * cos(angle_corr);
            //         // [수정] 전방이 반대로 찍힌다면 여기서 sin 앞의 부호를 반전시킵니다.
            //         // 기존에 -sin 이었다면 sin으로, sin 이었다면 -sin으로 변경
            //         float X_cam = dist * sin(angle_corr);
            //         float Y_cam = y_offset;

            //         if (Z_cam > 0.1f)
            //         {
            //             lp.u = (float)((mtx.at<double>(0, 0) * X_cam / Z_cam) + mtx.at<double>(0, 2));
            //             lp.v = (float)((mtx.at<double>(1, 1) * Y_cam / Z_cam) + mtx.at<double>(1, 2));

            //             // 화면 범위 내에 있는지 확인 (640x480 기준)
            //             return (lp.u >= 0 && lp.u < 640 && lp.v >= 0 && lp.v < 480);
            //         }
            //         return false;
            //     };

                // 메인 루프 내 처리 부분 ?? 이거 왜 두개임?
                for (const auto &pt : cloud_filtered->points)
                {
                    if (item->count >= MAX_LIDAR_POINTS)
                        break;

                    float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
                    float angle = std::atan2(pt.x, pt.y);

                    LidarPoint lp;
                    lp.dist = dist;
                    lp.angle = angle;
                    lp.x = pt.x;
                    lp.y = pt.y;

                    bool is_valid = false;

                    // 1. 전방 확인
                    float aF = angle + angF;
                    if (std::abs(aF) < 50.0f * M_PI / 180.0f)
                    {
                        is_valid = projectToCamera(dist, aF, yF, mtxF, lp);
                    }
                    // 2. 후방 확인 (전방이 아닐 때만)
                    else
                    {
                        float aR = angle - M_PI + angR;
                        while (aR > M_PI)
                            aR -= 2.0f * M_PI;
                        while (aR < -M_PI)
                            aR += 2.0f * M_PI;

                        if (std::abs(aR) < 50.0f * M_PI / 180.0f)
                        {
                            // 후방은 이미 해결하셨으므로, 후방 전용 X_cam 부호를 위해
                            // 별도의 계산이 필요하다면 아래처럼 직접 호출하거나
                            // 람다 내부에서 부호를 조건부로 처리할 수 있습니다.

                            // 후방이 잘 나왔던 로직이 X_cam = dist * -sin(aR) 이었다면
                            // 아래 호출 시 각도에 마이너스를 붙여 전달하는 식으로 처리 가능합니다.
                            is_valid = projectToCamera(dist, aR, yR, mtxR, lp);

                            // 만약 후방과 전방의 좌우 방향이 서로 다르다면 (한쪽은 sin, 한쪽은 -sin)
                            // 람다 함수를 쓰지 않고 아래처럼 명시적으로 나누는 것이 디버깅에 유리합니다.
                        }
                    }

                    if (is_valid)
                    {
                        item->points[item->count++] = lp;
                    }
                }


                if (is_valid_proj)
                {
                    item->points[item->count++] = lp;
                }
            
            // printf("[Lidar Producer] Produced %d points\n", item->count);
            //  큐 인덱스 이동
            q->tail = (q->tail + 1) % QUEUE_SIZE;

            pthread_mutex_unlock(&q->mutex);
            sem_post(&q->sem_full);
        }
    }

    printf("라이다 종료\n");
    free(scan.points);
    turnOff(laser);
    lidarDestroy(&laser);
    return 0;
}
