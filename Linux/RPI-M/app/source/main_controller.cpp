#include <iostream>
#include <vector>
#include <string>
#include <arpa/inet.h>
#include <opencv2/opencv.hpp>
#include "ydlidar_sdk.h"
#include "common.h"

using namespace std;
using namespace cv;

Mat mtxF = (Mat_<double>(3, 3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat mtxR = (Mat_<double>(3, 3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);

float angF = -0.023f, yF = -0.150f, sF = 0.76f;
float angR = -0.023f, yR = -0.150f, sR = 0.79f;

// void drawContour(Mat &img, const vector<LidarPoint> &pts, Scalar color)
// {
//     if (pts.size() < 2)
//         return;
//     for (size_t i = 1; i < pts.size(); i++)
//     {
//         if (abs(pts[i].dist - pts[i - 1].dist) < 0.5f)
//         {
//             line(img, pts[i - 1].pt, pts[i].pt, color, 2, LINE_AA);
//         }
//     }
// }

void calibrateAndMatch(CameraQueue *cur_q, LidarQueue *lidar_q, const vector<LidarPoint> &currentPts, Mat &targetView)
{
    if (sem_trywait(&cur_q->sem_full) == 0)
    {
        pthread_mutex_lock(&cur_q->mutex);
        CameraItem *c_item = &cur_q->buffer[cur_q->head];

        Scalar color = Scalar(0, 0, 255);

        for (int i = 0; i < c_item->obj_count; i++)
        {
            BBox &b = c_item->objects[i];
            Rect box((int)b.x, (int)b.y, (int)b.w, (int)b.h);
            Rect searchBox(box.x - 10, box.y - 10, box.width + 20, box.height + 20);

            float minD = 99.0f, targetA = 0.0f;
            bool found = false;

            // Lidar 포인트와 BBox 매칭. 여기에 거리 코드가 만들어져야함.
            for (const auto &lp : currentPts)
            {
                if (searchBox.contains(cv::Point2f(lp.x, lp.y)))
                {
                    if (lp.dist < minD)
                    {
                        minD = lp.dist;
                        targetA = lp.angle;
                        found = true;
                    }
                }
            }

            // 드로잉
            rectangle(targetView, box, color, 2);
            if (found)
            {
                char label[50];
                sprintf(label, "%.2fm, %.1fdeg", minD, targetA * 180.0f / M_PI);
                putText(targetView, label, Point(box.x, box.y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
            }
        }
        cur_q->head = (cur_q->head + 1) % QUEUE_SIZE;
        pthread_mutex_unlock(&cur_q->mutex);
        sem_post(&cur_q->sem_empty);
    }
    // 마지막에 SPI 필요
}

int main()
{
    int fd_l = shm_open(SHM_NAME_LIDAR, O_RDWR, 0666);
    int fd_f = shm_open(SHM_NAME_FRONT_CAMERA, O_RDWR, 0666);
    int fd_b = shm_open(SHM_NAME_BACK_CAMERA, O_RDWR, 0666);

    LidarQueue *q_l = (LidarQueue *)mmap(NULL, sizeof(LidarQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd_l, 0);
    CameraQueue *q_f = (CameraQueue *)mmap(NULL, sizeof(CameraQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd_f, 0);
    CameraQueue *q_b = (CameraQueue *)mmap(NULL, sizeof(CameraQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd_b, 0);

    vector<LidarPoint> ptsF, ptsR;
    ptsF.reserve(MAX_LIDAR_POINTS);
    ptsR.reserve(MAX_LIDAR_POINTS);
    while (true)
    {
        Mat viewF = Mat::zeros(480, 640, CV_8UC3);
        Mat viewR = Mat::zeros(480, 640, CV_8UC3);

        ptsF.clear();
        ptsR.clear();

        // --- [Step 1] Lidar 데이터 가져오기 ---
        // sem_trywait을 사용하면 데이터가 없을 때 기다리지 않고 넘어갑니다.
        // 실시간성을 위해 최신 데이터를 기다리려면 sem_wait을 사용하세요.
        if (sem_wait(&q_l->sem_full) == 0)
        {
            pthread_mutex_lock(&q_l->mutex);
            LidarItem *item = &q_l->buffer[q_l->head];

            for (int i = 0; i < item->count; i++)
            {
                // 각도에 따라 전방/후방 분류 (생산자 로직과 매칭)
                // 생산자에서 이미 u, v를 계산해서 넘겨주므로 적절한 view에 할당
                if (abs(item->points[i].angle) < 1.57f)
                    ptsF.push_back(item->points[i]); // 대략 전방 180도
                else
                    ptsR.push_back(item->points[i]);
            }

            q_l->head = (q_l->head + 1) % QUEUE_SIZE;
            pthread_mutex_unlock(&q_l->mutex);
            sem_post(&q_l->sem_empty);
        }

        // --- [Step 2] 카메라 데이터(BBox) 처리 및 매칭 ---
        // 전방(Front)과 후방(Back) 각각 처리
        calibrateAndMatch(q_f, q_l, ptsF, viewF);
        calibrateAndMatch(q_b, q_l, ptsR, viewR);

        // --- [Step 3] 최종 출력 ---
        // drawContour(viewF, ptsF, Scalar(0, 255, 0));
        // drawContour(viewR, ptsR, Scalar(0, 0, 255));

        Mat combined;
        hconcat(viewF, viewR, combined);
        imshow("Lidar-Camera Fusion Monitoring", combined);

        if (waitKey(10) == 'q')
            break;
    }

    munmap(q_l, sizeof(LidarQueue));
    munmap(q_f, sizeof(CameraQueue));
    munmap(q_b, sizeof(CameraQueue));
    close(fd_l);
    close(fd_f);
    close(fd_b);
    return 0;
}