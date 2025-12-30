#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "ydlidar_sdk.h"

using namespace cv;
using namespace std;

// --- [내부 파라미터 고정] ---
Mat mtxF = (Mat_<double>(3,3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat distF = (Mat_<double>(1,5) << -0.01189, 0.91172, -0.00182, -0.00026, -2.6566);
Mat mtxR = (Mat_<double>(3,3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);
Mat distR = (Mat_<double>(1,5) << -0.03829, 1.44263, -0.00224, -0.00417, -4.1455);

// --- [외부 파라미터: 알려주신 값을 초기값으로 설정] ---
float angF = -0.023f, yF = -0.050f, sF = 0.76f; // 전방 초기값
float angR = -0.023f, yR = -0.040f, sR = 0.79f; // 후방 초기값

void drawEnhancedContour(Mat& img, const vector<Point>& pts, Scalar color) {
    if (pts.size() < 2) return;
    for (size_t i = 1; i < pts.size(); i++) {
        if (norm(pts[i] - pts[i-1]) < 70) line(img, pts[i-1], pts[i], color, 2, LINE_AA);
        circle(img, pts[i], 2, color, -1);
    }
}

int main() {
    VideoCapture capF(0, CAP_V4L2);
    VideoCapture capR(2, CAP_V4L2);
    
    capF.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    capF.set(CAP_PROP_FRAME_WIDTH, 640);
    capF.set(CAP_PROP_FRAME_HEIGHT, 480);
    capR.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    capR.set(CAP_PROP_FRAME_WIDTH, 640);
    capR.set(CAP_PROP_FRAME_HEIGHT, 480);

    os_init();
    YDLidar *laser = lidarCreate();
    const char *port = "/dev/ttyUSB0";
    int baud = 230400;
    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baud, sizeof(int));
    initialize(laser); turnOn(laser);

    LaserFan scan;
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * 2048);

    cout << "보정 시스템 시작 (Q: 종료)" << endl;
    cout << "전방 조절: W/S(배율), A/D(각도), I/K(높이)" << endl;
    cout << "후방 조절: T/G(배율), J/L(각도), U/O(높이)" << endl;

    while (true) {
        Mat frameF, frameR, undF, undR;
        if (!capF.read(frameF) || !capR.read(frameR)) continue;

        undistort(frameF, undF, mtxF, distF);
        undistort(frameR, undR, mtxR, distR);

        vector<Point> ptsF, ptsR;
        if (doProcessSimple(laser, &scan)) {
            for (int i = 0; i < scan.npoints; i++) {
                float dist = scan.points[i].range;
                float angle = scan.points[i].angle;
                if (dist < 0.1f || dist > 6.0f) continue;

                // 전방 투영
                float aF = angle + angF;
                if (abs(aF) < 50.0f * M_PI / 180.0f) {
                    float Z = dist * cos(aF);
                    int u = (int)((mtxF.at<double>(0,0)*sF * (dist*sin(aF)) / Z) + mtxF.at<double>(0,2));
                    int v = (int)((mtxF.at<double>(1,1)*sF * yF / Z) + mtxF.at<double>(1,2));
                    if (u>=0 && u<640 && v>=0 && v<480) ptsF.push_back(Point(u, v));
                }

                // 후방 투영
                float aR = angle - M_PI + angR;
                while(aR > M_PI) aR -= 2*M_PI; while(aR < -M_PI) aR += 2*M_PI;
                if (abs(aR) < 50.0f * M_PI / 180.0f) {
                    float Z = dist * cos(aR);
                    int u = (int)((mtxR.at<double>(0,0)*sR * (dist*sin(aR)) / Z) + mtxR.at<double>(0,2));
                    int v = (int)((mtxR.at<double>(1,1)*sR * yR / Z) + mtxR.at<double>(1,2));
                    if (u>=0 && u<640 && v>=0 && v<480) ptsR.push_back(Point(u, v));
                }
            }
        }

        drawEnhancedContour(undF, ptsF, Scalar(0, 255, 0));
        drawEnhancedContour(undR, ptsR, Scalar(0, 0, 255));

        // 실시간 수치 표시
        char txtF[60], txtR[60];
        sprintf(txtF, "F [A:%.3f Y:%.3f S:%.2f]", angF, yF, sF);
        sprintf(txtR, "R [A:%.3f Y:%.3f S:%.2f]", angR, yR, sR);
        
        rectangle(undF, Rect(5, 445, 300, 30), Scalar(0,0,0), -1);
        rectangle(undR, Rect(5, 445, 300, 30), Scalar(0,0,0), -1);
        putText(undF, txtF, Point(10, 467), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,255), 2);
        putText(undR, txtR, Point(10, 467), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,255), 2);

        Mat combined;
        hconcat(undF, undR, combined);
        imshow("LiDAR-Camera Tuning System", combined);

        int key = waitKey(10);
        if (key == 'q') break;
        // 전방 조절 (Left)
        else if (key == 'a') angF -= 0.001f; else if (key == 'd') angF += 0.001f;
        else if (key == 'i') yF += 0.001f;   else if (key == 'k') yF -= 0.001f;
        else if (key == 'w') sF += 0.005f;   else if (key == 's') sF -= 0.005f;
        // 후방 조절 (Right)
        else if (key == 'j') angR -= 0.001f; else if (key == 'l') angR += 0.001f;
        else if (key == 'u') yR += 0.001f;   else if (key == 'o') yR -= 0.001f;
        else if (key == 't') sR += 0.005f;   else if (key == 'g') sR -= 0.005f;
    }

    free(scan.points); turnOff(laser); lidarDestroy(&laser);
    return 0;
}