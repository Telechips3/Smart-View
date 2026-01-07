#include <iostream>
#include <vector>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include "ydlidar_sdk.h"

using json = nlohmann::json;
using namespace std;
using namespace cv;

#define UDP_PORT 5005

// --- [Base64 디코딩 함수 (제공해주신 코드 적용)] ---
static const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::vector<unsigned char> base64_decode(const std::string& in) {
    std::vector<unsigned char> out;
    std::vector<int> T(256, -1);
    for (int i = 0; i < 64; i++) T[base64_chars[i]] = i;

    int val = 0, valb = -8;
    for (unsigned char c : in) {
        if (T[c] == -1) break;
        val = (val << 6) + T[c];
        valb += 6;
        if (valb >= 0) {
            out.push_back(char((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    return out;
}
// ---------------------------------------------------

// 카메라 파라미터
Mat mtxF = (Mat_<double>(3,3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat mtxR = (Mat_<double>(3,3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);
float angF = -0.023f, yF = -0.150f, sF = 0.76f;
float angR = -0.023f, yR = -0.150f, sR = 0.79f;

struct LidarProj { Point pt; float dist; float ang; };

// 라이다 점 그리기 함수
void drawContour(Mat& img, const vector<LidarProj>& pts, Scalar color) {
    if (pts.size() < 2) return;
    for (size_t i = 1; i < pts.size(); i++) {
        if (abs(pts[i].dist - pts[i-1].dist) < 0.5f) {
            line(img, pts[i-1].pt, pts[i].pt, color, 2, LINE_AA);
        }
    }
}

int main() {
    // 1. LiDAR 초기화
    os_init(); YDLidar *laser = lidarCreate();
    const char *port = "/dev/ttyUSB0"; int baud = 230400;
    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baud, sizeof(int));
    initialize(laser); turnOn(laser);

    // 2. UDP 소켓 설정
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(UDP_PORT);
    bind(sock, (const struct sockaddr *)&servaddr, sizeof(servaddr));

    // Non-blocking 수신을 위한 타임아웃 (1ms)
    struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    LaserFan scan;
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * 2048);
    
    // UDP 버퍼 (65KB)
    char buffer[65535];

    // 화면 버퍼 (수신 전에는 검은색)
    Mat viewF = Mat::zeros(480, 640, CV_8UC3);
    Mat viewR = Mat::zeros(480, 640, CV_8UC3);

    while (true) {
        // [A] LiDAR 데이터 처리
        vector<LidarProj> ptsF, ptsR;
        bool lidar_ok = doProcessSimple(laser, &scan);
        if (lidar_ok) {
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
                    if (u>=0 && u<640 && v>=0 && v<480) ptsF.push_back({Point(u, v), dist, angle});
                }
                // 후방 투영
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

        // [B] UDP 데이터 수신 (JSON with Base64 Image)
        while (true) {
            ssize_t n = recv(sock, buffer, sizeof(buffer) - 1, 0);
            if (n <= 0) break; // 데이터 없으면 루프 탈출
            buffer[n] = '\0';

            try {
                // 1. JSON 파싱
                auto j = json::parse(buffer);
                int cam_id = 0;
                if (j.contains("cam_id")) cam_id = j["cam_id"].get<int>();

                // 2. Base64 이미지 디코딩 및 배경 업데이트
                if (j.contains("image_base64") && !j["image_base64"].is_null()) {
                    string b64_str = j["image_base64"];
                    if (!b64_str.empty()) {
                        vector<unsigned char> img_data = base64_decode(b64_str);
                        Mat decoded = imdecode(img_data, IMREAD_COLOR);
                        
                        if (!decoded.empty()) {
                            if (cam_id == 0) viewF = decoded;
                            else if (cam_id == 1) viewR = decoded;
                        }
                    }
                }

                // 3. 탐지 박스(Detections) 처리 및 거리 측정
                if (j.contains("detections")) {
                    for (auto& det : j["detections"]) {
                        // Python에서 보내는 키값에 맞춰 수정 ("x", "y", "w", "h")
                        float rx = det["x"].get<float>();
                        float ry = det["y"].get<float>();
                        float rw = det["w"].get<float>();
                        float rh = det["h"].get<float>();

                        // 박스 예외 처리
                        if (rw > 600 || rh > 450) continue; 
                        int bx = max(0, (int)rx);
                        int by = max(0, (int)ry);
                        int bw = min(640 - bx, (int)rw);
                        int bh = min(480 - by, (int)rh);
                        if (bw <= 0 || bh <= 0) continue;

                        Rect box(bx, by, bw, bh);
                        Rect searchBox(bx - 5, by - 5, bw + 10, bh + 10);

                        // 라이다 포인트와 매칭
                        float minD = 99.0f;
                        bool found = false;
                        const vector<LidarProj>& currentPts = (cam_id == 0) ? ptsF : ptsR;

                        for (const auto& p : currentPts) {
                            if (searchBox.contains(p.pt)) {
                                if (p.dist < minD) { minD = p.dist; found = true; }
                            }
                        }

                        // 결과 그리기
                        Mat& targetView = (cam_id == 0) ? viewF : viewR;
                        Scalar color = (cam_id == 0) ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
                        rectangle(targetView, box, color, 2);

                        if (found) {
                            char label[50];
                            sprintf(label, "%.2fm", minD);
                            putText(targetView, label, Point(bx, by - 10), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);
                        }
                    }
                }

            } catch (const exception& e) {
                // 패킷 깨짐 등 에러 발생 시 무시
                // cout << "Parsing Error: " << e.what() << endl;
            }
        }

        // [C] 라이다 스캔 라인 그리기 (이미지 위에 덮어쓰기)
        drawContour(viewF, ptsF, Scalar(0, 255, 0));
        drawContour(viewR, ptsR, Scalar(0, 0, 255));

        // [D] 화면 표시
        Mat combined;
        hconcat(viewF, viewR, combined);
        imshow("Smart-View Fusion Monitor", combined);

        if (waitKey(1) == 'q') break;
    }

    // 종료 처리
    free(scan.points);
    turnOff(laser);
    lidarDestroy(&laser);
    close(sock);
    return 0;
}