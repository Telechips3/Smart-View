#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include "ydlidar_sdk.h"

using namespace cv;
using namespace std;

// [상수 설정]
const int CAM_WIDTH = 320;
const int CAM_HEIGHT = 320;
const int MAP_SIZE = 600; 

// [튜닝 파라미터]
float FOCAL_LENGTH = 800.0f; 
float ANGLE_OFFSET_FRONT = 0.0f; // 전방 카메라 각도 미세조정
float ANGLE_OFFSET_BACK = 0.0f;  // 후방 카메라 각도 미세조정
float V_OFFSET = 0.0f;           // 수직 오프셋
const float Y_DIST = -0.03f;     // 카메라와 라이다 간의 수직 거리 (m)

// 투영 함수: 거리와 각도 데이터를 카메라 좌표계로 변환하여 점을 그립니다.
void projectToCamera(Mat& frame, float dist, float angle_rad, Scalar color) {
    // 카메라 좌표계: Z(앞방향), X(좌우), Y(상하)
    float X = dist * sin(angle_rad); 
    float Z = dist * cos(angle_rad);
    float Y = Y_DIST;

    // 카메라 전방(Z > 0)에 있는 점들만 투영
    if (Z > 0.1f) {
        int u = (int)((FOCAL_LENGTH * X / Z) + (CAM_WIDTH / 2));
        int v = (int)((FOCAL_LENGTH * Y / Z) + (CAM_HEIGHT / 2) + V_OFFSET);

        // 이미지 경계 내에 있는 점만 그리기
        if (u >= 0 && u < CAM_WIDTH && v >= 0 && v < CAM_HEIGHT) {
            circle(frame, Point(u, v), 2, color, -1);
        }
    }
}

int main() {
    // 1. 카메라 설정 (0: 전방, 2: 후방 - 연결 상태에 따라 번호 확인 필요)
    VideoCapture cap_front(0, CAP_V4L2);
    VideoCapture cap_back(2, CAP_V4L2); 

    if (!cap_front.isOpened() || !cap_back.isOpened()) {
        printf("카메라를 열 수 없습니다! 인덱스를 확인하세요.\n");
        return -1;
    }

    cap_front.set(CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
    cap_front.set(CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);
    cap_back.set(CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
    cap_back.set(CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);

    // 2. 라이다 설정
    os_init();
    YDLidar *laser = lidarCreate();
    const char *port = "/dev/ttyUSB0";
    int baudrate = 230400;
    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baudrate, sizeof(int));

    if (!initialize(laser) || !turnOn(laser)) {
        printf("라이다를 시작할 수 없습니다.\n");
        return -1;
    }

    LaserFan scan;
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * 2000);

    // 3. FPS 계산용 변수
    double t_start = (double)getTickCount();
    int frame_count = 0;
    float fps = 0.0;

    while (true) {
        Mat frame_f, frame_b, lidar_map;
        cap_front >> frame_f;
        cap_back >> frame_b;
        if (frame_f.empty() || frame_b.empty()) continue;

        // FPS 계산 (1초마다 갱신)
        frame_count++;
        double t_now = (double)getTickCount();
        double elapsed = (t_now - t_start) / getTickFrequency();
        if (elapsed >= 1.0) {
            fps = (float)(frame_count / elapsed);
            t_start = t_now;
            frame_count = 0;
        }

        // 라이다 맵 초기화 및 중앙점 그리기
        lidar_map = Mat::zeros(MAP_SIZE, MAP_SIZE, CV_8UC3);
        circle(lidar_map, Point(MAP_SIZE/2, MAP_SIZE/2), 5, Scalar(0, 0, 255), -1);

        if (doProcessSimple(laser, &scan)) {
            for (int i = 0; i < scan.npoints; i++) {
                float dist = scan.points[i].range;
                if (dist <= 0.1f) continue;

                float raw_angle = scan.points[i].angle;

                // [탑뷰 지도 작성]
                int map_x = MAP_SIZE / 2 + (int)(dist * 50 * sin(raw_angle));
                int map_y = MAP_SIZE / 2 - (int)(dist * 50 * cos(raw_angle));
                if (map_x >= 0 && map_x < MAP_SIZE && map_y >= 0 && map_y < MAP_SIZE) {
                    lidar_map.at<Vec3b>(map_y, map_x) = Vec3b(255, 255, 255);
                }

                // [카메라 퓨전 투영]
                // 전방 카메라: 라이다의 180도(M_PI) 방향을 매칭
                float angle_f = raw_angle + M_PI + (ANGLE_OFFSET_FRONT * M_PI / 180.0f);
                projectToCamera(frame_f, dist, angle_f, Scalar(0, 255, 0));

                // 후방 카메라: 라이다의 0도 방향을 매칭
                float angle_b = raw_angle + (ANGLE_OFFSET_BACK * M_PI / 180.0f);
                projectToCamera(frame_b, dist, angle_b, Scalar(0, 255, 255));
            }
        }

        // [정보 출력] FPS 텍스트 생성
        char fps_text[20];
        sprintf(fps_text, "FPS: %.1f", fps);
        
        // 전방 영상에 FPS 및 라벨 표시
        putText(frame_f, "FRONT (180 deg)", Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
        putText(frame_f, fps_text, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

        // 후방 영상에 FPS 및 라벨 표시
        putText(frame_b, "BACK (0 deg)", Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
        putText(frame_b, fps_text, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

        // 윈도우 표시
        imshow("Front Fusion", frame_f);
        imshow("Back Fusion", frame_b);
        imshow("LiDAR Map", lidar_map);

        // 키 제어
        int key = waitKey(1);
        if (key == 27) break; // ESC 종료
        else if (key == 'a') ANGLE_OFFSET_FRONT -= 1.0f;
        else if (key == 'd') ANGLE_OFFSET_FRONT += 1.0f;
        else if (key == 'j') ANGLE_OFFSET_BACK -= 1.0f;
        else if (key == 'l') ANGLE_OFFSET_BACK += 1.0f;
    }

    // 종료 및 자원 해제
    free(scan.points);
    turnOff(laser);
    lidarDestroy(&laser);
    return 0;
}