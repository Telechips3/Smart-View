#include <iostream>
#include <vector>
#include <string>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "ydlidar_sdk.h"
#include "common.h"

using json = nlohmann::json;
using namespace std;
using namespace cv;

Mat mtxF = (Mat_<double>(3,3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat mtxR = (Mat_<double>(3,3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);

float angF = -0.023f, yF = -0.150f, sF = 0.76f;
float angR = -0.023f, yR = -0.150f, sR = 0.79f;

void drawContour(Mat& img, const vector<LidarProj>& pts, Scalar color) {
    if (pts.size() < 2) return;
    for (size_t i = 1; i < pts.size(); i++) {
        if (abs(pts[i].dist - pts[i-1].dist) < 0.5f) {
            line(img, pts[i-1].pt, pts[i].pt, color, 2, LINE_AA);
        }
    }
}

int main()
{
    while (true) {
        Mat viewF = Mat::zeros(480, 640, CV_8UC3);
        Mat viewR = Mat::zeros(480, 640, CV_8UC3);
        vector<LidarProj> ptsF, ptsR;

        //여기서 lidar 읽는 값 필요
        if (lidar_ok) {
            for (int i = 0; i < scan.npoints; i++) {
                float dist = scan.points[i].range;
                float angle = scan.points[i].angle;
                if (dist < 0.1f || dist > 6.0f) continue;

                float aF = angle + angF;
                if (abs(aF) < 50.0f * M_PI / 180.0f) {
                    float Z = dist * cos(aF);
                    int u = (int)((mtxF.at<double>(0,0)*sF * (dist*sin(aF)) / Z) + mtxF.at<double>(0,2));
                    int v = (int)((mtxF.at<double>(1,1)*sF * yF / Z) + mtxF.at<double>(1,2));
                    if (u>=0 && u<640 && v>=0 && v<480) ptsF.push_back({Point(u, v), dist, angle});
                }

                float aR = angle - M_PI + angR;
                while(aR > M_PI) aR -= 2*M_PI; while(aR < -M_PI) aR += 2*M_PI;
                if (abs(aR) < 50.0f * M_PI / 180.0f) {
                    float Z = dist * cos(aR);
                    int u = (int)((mtxR.at<double>(0,0)*sR * (dist*sin(aR)) / Z) + mtxR.at<double>(0,2));
                    int v = (int)((mtxR.at<double>(1,1)*sR * yR / Z) + mtxR.at<double>(1,2));
                    if (u>=0 && u<640 && v>=0 && v<480) ptsR.push_back({Point(u, v), dist, angle});
                }
            }
        }

        drawContour(viewF, ptsF, Scalar(0, 255, 0));
        drawContour(viewR, ptsR, Scalar(0, 0, 255));

        //여기서 camera 읽는 값 필요
        if (n > 0) {
            buffer[n] = '\0';
            try {
                auto j = json::parse(buffer);
                for (auto& det : j["detections"]) {
                    int cam_id = det[0];
                    float rx = det[2].get<float>(), ry = det[3].get<float>();
                    float rw = det[4].get<float>(), rh = det[5].get<float>();

                    // 1. 비정상적인 박스 필터링 (너무 큰 박스는 무시)
                    if (rw > 600 || rh > 450) continue; 

                    // 2. 화면 범위 내로 좌표 제한 (Clip)
                    int bx = max(0, (int)rx);
                    int by = max(0, (int)ry);
                    int bw = min(640 - bx, (int)rw);
                    int bh = min(480 - by, (int)rh);

                    if (bw <= 0 || bh <= 0) continue;

                    Rect box(bx, by, bw, bh);
                    Rect searchBox(bx - 5, by - 5, bw + 10, bh + 10);

                    float minD = 99.0f, targetA = 0.0f;
                    bool found = false;

                    const vector<LidarProj>& currentPts = (cam_id == 0) ? ptsF : ptsR;
                    for (const auto& p : currentPts) {
                        if (searchBox.contains(p.pt)) {
                            if (p.dist < minD) { minD = p.dist; targetA = p.ang; found = true; }
                        }
                    }

                    Mat& targetView = (cam_id == 0) ? viewF : viewR;
                    Scalar color = (cam_id == 0) ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
                    rectangle(targetView, box, color, 2);

                    if (found) {
                        char label[50];
                        sprintf(label, "%.2fm, %.1fdeg", minD, targetA * 180.0f / M_PI);
                        putText(targetView, label, Point(bx, by - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 3);
                        putText(targetView, label, Point(bx, by - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
                    }
                }
            } catch (...) {}
        }

        Mat combined; hconcat(viewF, viewR, combined);
        imshow("Lidar Monitoring", combined);
        if (waitKey(10) == 'q') break;
    }
    return 0;
}