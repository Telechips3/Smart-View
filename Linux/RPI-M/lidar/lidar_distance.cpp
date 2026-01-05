#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "ydlidar_sdk.h"

using namespace cv;
using namespace std;

// 내부 파라미터 (기존 유지)
Mat mtxF = (Mat_<double>(3,3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat distF = (Mat_<double>(1,5) << -0.01189, 0.91172, -0.00182, -0.00026, -2.6566);
Mat mtxR = (Mat_<double>(3,3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);
Mat distR = (Mat_<double>(1,5) << -0.03829, 1.44263, -0.00224, -0.00417, -4.1455);

// 초기값 설정
float angF = -0.023f, yF = -0.050f, sF = 0.76f;
float angR = -0.023f, yR = -0.040f, sR = 0.79f;

void drawEnhancedContour(Mat& img, const vector<Point>& pts, Scalar color) {
    if (pts.size() < 2) return;
    for (size_t i = 1; i < pts.size(); i++) {
        if (norm(pts[i] - pts[i-1]) < 70) line(img, pts[i-1], pts[i], color, 2, LINE_AA);
        circle(img, pts[i], 2, color, -1);
    }
}

int main() {
    VideoCapture capF(0, CAP_V4L2); VideoCapture capR(2, CAP_V4L2);
    capF.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    capR.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));

    os_init(); YDLidar *laser = lidarCreate();
    const char *port = "/dev/ttyUSB0"; int baud = 230400;
    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baud, sizeof(int));
    initialize(laser); turnOn(laser);

    LaserFan scan;
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * 2048);

    while (true) {
        Mat frameF, frameR, undF, undR;
        if (!capF.read(frameF) || !capR.read(frameR)) continue;
        undistort(frameF, undF, mtxF, distF);
        undistort(frameR, undR, mtxR, distR);

        vector<Point> ptsF, ptsR;
        float minCDistF = 99.0f, minCAngleF = 0.0f; // 전방 정면 최소 거리
        float minCDistR = 99.0f, minCAngleR = 0.0f; // 후방 정면 최소 거리

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
                    if (u>=0 && u<640 && v>=0 && v<480) {
                        ptsF.push_back(Point(u, v));
                        // 정면 10도 이내 물체 중 가장 가까운 것 찾기
                        if (abs(aF) < 10.0f * M_PI / 180.0f && dist < minCDistF) {
                            minCDistF = dist; minCAngleF = aF * 180.0f / M_PI;
                        }
                    }
                }

                // 후방 투영
                float aR = angle - M_PI + angR;
                while(aR > M_PI) aR -= 2*M_PI; while(aR < -M_PI) aR += 2*M_PI;
                if (abs(aR) < 50.0f * M_PI / 180.0f) {
                    float Z = dist * cos(aR);
                    int u = (int)((mtxR.at<double>(0,0)*sR * (dist*sin(aR)) / Z) + mtxR.at<double>(0,2));
                    int v = (int)((mtxR.at<double>(1,1)*sR * yR / Z) + mtxR.at<double>(1,2));
                    if (u>=0 && u<640 && v>=0 && v<480) {
                        ptsR.push_back(Point(u, v));
                        // 후방 정면 10도 이내
                        if (abs(aR) < 10.0f * M_PI / 180.0f && dist < minCDistR) {
                            minCDistR = dist; minCAngleR = aR * 180.0f / M_PI;
                        }
                    }
                }
            }
        }

        drawEnhancedContour(undF, ptsF, Scalar(0, 255, 0));
        drawEnhancedContour(undR, ptsR, Scalar(0, 0, 255));

        // 화면에 거리/각도 텍스트 오버레이
        char infoF[100], infoR[100];
        if(minCDistF < 10.0f) sprintf(infoF, "FRONT OBJ: %.2fm / %.1f deg", minCDistF, minCAngleF);
        else sprintf(infoF, "FRONT OBJ: NONE");
        
        if(minCDistR < 10.0f) sprintf(infoR, "REAR OBJ: %.2fm / %.1f deg", minCDistR, minCAngleR);
        else sprintf(infoR, "REAR OBJ: NONE");

        // 시각적 강조를 위해 텍스트 상자 추가
        rectangle(undF, Rect(10, 10, 350, 40), Scalar(0,0,0), -1);
        putText(undF, infoF, Point(20, 35), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,0), 2);
        
        rectangle(undR, Rect(10, 10, 350, 40), Scalar(0,0,0), -1);
        putText(undR, infoR, Point(20, 35), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);

        Mat combined; hconcat(undF, undR, combined);
        imshow("Lidar Distance Monitoring", combined);

        if (waitKey(10) == 'q') break;
    }
    free(scan.points); turnOff(laser); lidarDestroy(&laser);
    return 0;
}