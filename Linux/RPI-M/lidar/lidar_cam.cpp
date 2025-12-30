#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "ydlidar_sdk.h"

using namespace cv;
using namespace std;

const int CAM_WIDTH = 640;
const int CAM_HEIGHT = 480;
const int MAP_SIZE = 600;

// [튜닝 파라미터]
float FOCAL_LENGTH = 600.0f; 
const float Y_DIST = -0.05f; 
const float FOV_DEG = 60.0f; // 요청에 따라 탐지 범위를 60도(좌우 30도)로 변경

int main() {
    // 1. 카메라 설정
    VideoCapture capFront(0, CAP_V4L2);
    VideoCapture capRear(2, CAP_V4L2);

    if (!capFront.isOpened() || !capRear.isOpened()) {
        printf("카메라 연결 실패!\n");
        return -1;
    }

    capFront.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capRear.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capFront.set(CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
    capFront.set(CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);
    capRear.set(CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
    capRear.set(CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);

    // 2. YDLidar 설정
    os_init();
    YDLidar *laser = lidarCreate();
    const char *port = "/dev/ttyUSB0";
    int baudrate = 230400;
    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baudrate, sizeof(int));
    
    if (!initialize(laser) || !turnOn(laser)) return -1;

    LaserFan scan;
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * 2048);

    float fov_rad = FOV_DEG * M_PI / 180.0f;

    while (true) {
        Mat frameF, frameR, lidar_map;
        capFront >> frameF;
        capRear >> frameR;

        if (frameF.empty() || frameR.empty()) continue;

        // 탑뷰 초기화
        lidar_map = Mat::zeros(MAP_SIZE, MAP_SIZE, CV_8UC3);
        int cx = MAP_SIZE / 2;
        int cy = MAP_SIZE / 2;
        circle(lidar_map, Point(cx, cy), 3, Scalar(255, 255, 255), -1);

        if (doProcessSimple(laser, &scan)) {
            for (int i = 0; i < scan.npoints; i++) {
                float dist = scan.points[i].range;
                float angle = scan.points[i].angle;
                if (dist <= 0.1f) continue;

                // 기본 탑뷰 데이터 (회색)
                int mx = cx + (int)(dist * 60 * sin(angle));
                int my = cy - (int)(dist * 60 * cos(angle));
                if (mx >= 0 && mx < MAP_SIZE && my >= 0 && my < MAP_SIZE) {
                    lidar_map.at<Vec3b>(my, mx) = Vec3b(80, 80, 80); 
                }

                // --- 전방 범위 처리 (-30 ~ 30도) ---
                float rel_front = angle; 
                while (rel_front > M_PI) rel_front -= 2.0f * M_PI;
                while (rel_front < -M_PI) rel_front += 2.0f * M_PI;

                if (abs(rel_front) <= fov_rad / 2.0f) {
                    if (mx >= 0 && mx < MAP_SIZE && my >= 0 && my < MAP_SIZE) {
                        circle(lidar_map, Point(mx, my), 2, Scalar(0, 255, 0), -1); // 전방 초록색
                    }
                    float X = dist * sin(rel_front);
                    float Z = dist * cos(rel_front);
                    int u = (int)((FOCAL_LENGTH * X / Z) + (CAM_WIDTH / 2));
                    int v = (int)((FOCAL_LENGTH * Y_DIST / Z) + (CAM_HEIGHT / 2));
                    if (u >= 0 && u < CAM_WIDTH && v >= 0 && v < CAM_HEIGHT && Z > 0.1f) {
                        circle(frameF, Point(u, v), 3, Scalar(0, 255, 0), -1);
                    }
                }

                // --- 후방 범위 처리 (150 ~ 210도) ---
                // 중심이 180도(M_PI)이므로 180도 기준 좌우 30도 필터링
                float rel_rear = angle - M_PI;
                while (rel_rear > M_PI) rel_rear -= 2.0f * M_PI;
                while (rel_rear < -M_PI) rel_rear += 2.0f * M_PI;

                if (abs(rel_rear) <= fov_rad / 2.0f) {
                    if (mx >= 0 && mx < MAP_SIZE && my >= 0 && my < MAP_SIZE) {
                        circle(lidar_map, Point(mx, my), 2, Scalar(0, 0, 255), -1); // 후방 빨간색
                    }
                    float X = dist * sin(rel_rear);
                    float Z = dist * cos(rel_rear);
                    int u = (int)((FOCAL_LENGTH * X / Z) + (CAM_WIDTH / 2));
                    int v = (int)((FOCAL_LENGTH * Y_DIST / Z) + (CAM_HEIGHT / 2));
                    if (u >= 0 && u < CAM_WIDTH && v >= 0 && v < CAM_HEIGHT && Z > 0.1f) {
                        circle(frameR, Point(u, v), 3, Scalar(0, 0, 255), -1);
                    }
                }
            }
        }

        // 텍스트 안내 업데이트
        putText(frameF, "FRONT: -30 to 30 deg", Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1);
        putText(frameR, "REAR: 150 to 210 deg", Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1);

        imshow("Front Video", frameF);
        imshow("Rear Video", frameR);
        imshow("LiDAR Top-view", lidar_map);

        if (waitKey(1) == 'q') break;
    }

    free(scan.points);
    turnOff(laser);
    lidarDestroy(&laser);
    return 0;
}