#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "ydlidar_sdk.h"

using namespace cv;
using namespace std;

// --- [1단계: 분석된 내부 파라미터 적용] ---
Mat mtxF = (Mat_<double>(3,3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat distF = (Mat_<double>(1,5) << -0.01189, 0.91172, -0.00182, -0.00026, -2.6566);

Mat mtxR = (Mat_<double>(3,3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);
Mat distR = (Mat_<double>(1,5) << -0.03829, 1.44263, -0.00224, -0.00417, -4.1455);

// --- [2단계: 외부 파라미터 튜닝 변수] ---
float angF = 0.0f, yF = -0.05f, sF = 1.0f; // 전방 (초록)
float angR = 0.0f, yR = -0.05f, sR = 1.0f; // 후방 (빨강)

// 굴곡(선)을 그리는 함수
void drawContour(Mat& img, const vector<Point>& pts, Scalar color) {
    for (size_t i = 1; i < pts.size(); i++) {
        // 점 사이의 거리가 급격히 변하면 다른 물체이므로 선을 끊음
        if (norm(pts[i] - pts[i-1]) < 40) {
            line(img, pts[i-1], pts[i], color, 2);
        }
        circle(img, pts[i], 2, color, -1); // 점도 함께 표시
    }
}

int main() {
    // 카메라 설정 (대역폭 절약을 위해 MJPEG 및 저주사율 설정)
    VideoCapture capF(0, CAP_V4L2);
    VideoCapture capR(2, CAP_V4L2);
    capF.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    capR.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    capF.set(CAP_PROP_FPS, 20);
    capR.set(CAP_PROP_FPS, 20);

    if(!capF.isOpened() || !capR.isOpened()) {
        cout << "카메라를 찾을 수 없습니다! (0번, 2번 확인)" << endl;
        return -1;
    }

    // 라이다 초기화
    os_init();
    YDLidar *laser = lidarCreate();
    const char *port = "/dev/ttyUSB0";
    int baud = 230400;
    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baud, sizeof(int));
    if (!initialize(laser) || !turnOn(laser)) return -1;

    LaserFan scan;
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * 2048);

    cout << "=== 최종 통합 보정 시스템 시작 ===" << endl;
    cout << "W/S/A/D/I/K : 전방 조절 | T/G/J/L/U/O : 후방 조절 | Q : 종료" << endl;

    while (true) {
        Mat frameF, frameR, undF, undR;
        if (!capF.read(frameF) || !capR.read(frameR)) continue;

        // 왜곡 보정 적용
        undistort(frameF, undF, mtxF, distF);
        undistort(frameR, undR, mtxR, distR);

        vector<Point> ptsF, ptsR;

        if (doProcessSimple(laser, &scan)) {
            for (int i = 0; i < scan.npoints; i++) {
                float d = scan.points[i].range;
                float a_raw = scan.points[i].angle;
                if (d < 0.1f || d > 6.0f) continue;

                // --- 전방 투영 계산 ---
                float aF = a_raw + angF;
                if (abs(aF) < 45.0f * M_PI / 180.0f) {
                    float Z = d * cos(aF);
                    int u = (int)((mtxF.at<double>(0,0)*sF * (d*sin(aF)) / Z) + mtxF.at<double>(0,2));
                    int v = (int)((mtxF.at<double>(1,1)*sF * yF / Z) + mtxF.at<double>(1,2));
                    if (u>=0 && u<640 && v>=0 && v<480) ptsF.push_back(Point(u, v));
                }

                // --- 후방 투영 계산 ---
                float aR = a_raw - M_PI + angR;
                while(aR > M_PI) aR -= 2*M_PI; while(aR < -M_PI) aR += 2*M_PI;
                if (abs(aR) < 45.0f * M_PI / 180.0f) {
                    float Z = d * cos(aR);
                    int u = (int)((mtxR.at<double>(0,0)*sR * (d*sin(aR)) / Z) + mtxR.at<double>(0,2));
                    int v = (int)((mtxR.at<double>(1,1)*sR * yR / Z) + mtxR.at<double>(1,2));
                    if (u>=0 && u<640 && v>=0 && v<480) ptsR.push_back(Point(u, v));
                }
            }
        }

        // 굴곡 그리기
        drawContour(undF, ptsF, Scalar(0, 255, 0)); // 전방: 초록색
        drawContour(undR, ptsR, Scalar(0, 0, 255)); // 후방: 빨간색

        // 화면 수직 병합 (VNC 출력용)
        Mat combined;
        vconcat(undF, undR, combined);
        imshow("LiDAR-Camera Fusion System", combined);

        int key = waitKey(30);
        if (key == 'q') break;
        // 전방 조작
        else if (key == 'a') angF -= 0.005f; else if (key == 'd') angF += 0.005f;
        else if (key == 'i') yF += 0.005f;   else if (key == 'k') yF -= 0.005f;
        else if (key == 'w') sF += 0.01f;    else if (key == 's') sF -= 0.01f;
        // 후방 조작
        else if (key == 'j') angR -= 0.005f; else if (key == 'l') angR += 0.005f;
        else if (key == 'u') yR += 0.005f;   else if (key == 'o') yR -= 0.005f;
        else if (key == 't') sR += 0.01f;    else if (key == 'g') sR -= 0.01f;
    }

    free(scan.points);
    turnOff(laser);
    lidarDestroy(&laser);
    return 0;
}