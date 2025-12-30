#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>
#include <chrono> // 타임스탬프 측정을 위한 헤더
#include "nlohmann/json.hpp"
#include "ydlidar_sdk.h"

using namespace cv;
using namespace std;
using json = nlohmann::json;

// --- [환경 설정 및 상수] ---
#define UDP_PORT 5005           // RPi 5로부터 BBox를 받을 포트
#define MAX_TIME_DIFF_MS 100    // 허용 가능한 최대 시간차 (데이터 동기화 임계값)

// --- [차량 규격 설정 (단위: m)] ---
const float VEH_HALF_WIDTH = 0.15f;  // 차량 폭의 절반
const float VEH_FRONT_LEN = 0.20f;   // 라이다 중심 ~ 전방 범퍼 끝
const float VEH_REAR_LEN = 0.10f;    // 라이다 중심 ~ 후방 범퍼 끝

// --- [카메라 캘리브레이션 데이터] ---
// RPi 5에 달린 카메라의 내부 파라미터 (투영 연산에 필요)
Mat mtxF = (Mat_<double>(3,3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat mtxR = (Mat_<double>(3,3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);

// 라이다-카메라 간의 설치 위치 보정값 (외부 파라미터)
float angF = -0.023f, yF = -0.050f, sF = 0.76f;
float angR = -0.023f, yR = -0.040f, sR = 0.79f;

// 객체 정보를 담을 구조체
struct Detection {
    int cam_id;         // 어느 카메라에서 감지되었는가 (0:전, 1:후)
    Rect box;           // 바운딩 박스 좌표
    float minClearance = 99.0f; // 박스 내 물체까지의 최소 여유 거리
};

/**
 * @brief 현재 시스템의 절대 시간(ms)을 반환
 */
long long getCurrentTimeMs() {
    return chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now().time_since_epoch()).count();
}

/**
 * @brief 라이다 측정 각도에 따른 차량 외곽선(범퍼)까지의 보정 거리 계산
 */
float getVehicleOffset(float angle_rad, bool isFront) {
    float abs_angle = abs(angle_rad);
    float L = isFront ? VEH_FRONT_LEN : VEH_REAR_LEN;
    float W = VEH_HALF_WIDTH;
    if (abs_angle == 0) return L;
    // 직사각형 형태의 차량 경계면까지의 거리 (기하학적 계산)
    float boundary_dist = L / cos(abs_angle);
    float side_dist = W / sin(abs_angle);
    return min(boundary_dist, side_dist);
}

int main() {
    // 1. UDP 소켓 설정 (RPi 5로부터 데이터를 받기 위함)
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(sock, (const struct sockaddr*)&addr, sizeof(addr));
    fcntl(sock, F_SETFL, O_NONBLOCK); // 데이터가 없어도 대기하지 않고 루프 진행 (Non-blocking)
    char buf[8192];

    // 2. YDLidar SDK 초기화 및 작동
    os_init(); YDLidar *laser = lidarCreate();
    const char *port = "/dev/ttyUSB0"; int baud = 230400;
    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baud, sizeof(int));
    initialize(laser); turnOn(laser);

    LaserFan scan;
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * 2048);

    vector<Detection> last_dets; // 마지막으로 수신된 박스 목록
    long long last_bbox_ts = 0;   // 마지막으로 수신된 박스의 시간표

    cout << "RPi 4 Fusion Node Started. Waiting for RPi 5 data..." << endl;

    while (true) {
        // 시각화를 위한 가상 캔버스 생성 (검은 배경)
        Mat canvasF = Mat::zeros(480, 640, CV_8UC3);
        Mat canvasR = Mat::zeros(480, 640, CV_8UC3);

        // 3. UDP 통신으로 RPi 5의 BBox 데이터 수신
        int len = recv(sock, buf, sizeof(buf)-1, 0);
        if (len > 0) {
            buf[len] = '\0';
            try {
                auto j = json::parse(buf);
                last_bbox_ts = j["timestamp"].get<long long>(); // RPi 5가 데이터를 보낸 시각
                last_dets.clear();
                for (auto& d : j["detections"]) {
                    last_dets.push_back({d[0].get<int>(), 
                                        Rect((int)d[2].get<float>(), (int)d[3].get<float>(), 
                                             (int)d[4].get<float>(), (int)d[5].get<float>()), 99.0f});
                }
            } catch (...) { /* 파싱 에러 무시 */ }
        }

        // 4. 타임스탬프 동기화 확인
        long long current_ts = getCurrentTimeMs();
        long long diff = abs(current_ts - last_bbox_ts);

        // [데이터 퓨전 조건] 
        // 1. 라이다 스캔 데이터가 정상적으로 들어옴
        // 2. RPi 5의 박스 데이터와 RPi 4의 라이다 데이터 시간 차이가 100ms 이내임
        if (diff <= MAX_TIME_DIFF_MS && doProcessSimple(laser, &scan)) {
            for (int i = 0; i < scan.npoints; i++) {
                float dist = scan.points[i].range;
                float angle = scan.points[i].angle;
                if (dist < 0.1f || dist > 6.0f) continue;

                // --- 전방 카메라 영역 투영 ---
                float aF = angle + angF;
                if (abs(aF) < 55.0f * M_PI / 180.0f) { // 전방 시야각 내부인 경우
                    float clr = dist - getVehicleOffset(aF, true); // 범퍼 기준 여유 거리 계산
                    float Z = dist * cos(aF);
                    int u = (int)((mtxF.at<double>(0,0)*sF * (dist*sin(aF)) / Z) + mtxF.at<double>(0,2));
                    int v = (int)((mtxF.at<double>(1,1)*sF * yF / Z) + mtxF.at<double>(1,2));

                    if (u>=0 && u<640 && v>=0 && v<480) {
                        circle(canvasF, Point(u, v), 2, Scalar(0, 255, 0), -1); // 라이다 점 표시
                        for (auto& d : last_dets) {
                            // 점이 수신된 YOLO 박스 안에 있는지 판정
                            if (d.cam_id == 0 && d.box.contains(Point(u, v))) {
                                if (clr < d.minClearance) d.minClearance = max(0.0f, clr);
                            }
                        }
                    }
                }

                // --- 후방 카메라 영역 투영 ---
                float aR = angle - M_PI + angR;
                while(aR > M_PI) aR -= 2*M_PI; while(aR < -M_PI) aR += 2*M_PI;
                if (abs(aR) < 55.0f * M_PI / 180.0f) {
                    float clr = dist - getVehicleOffset(aR, false);
                    float Z = dist * cos(aR);
                    int u = (int)((mtxR.at<double>(0,0)*sR * (dist*sin(aR)) / Z) + mtxR.at<double>(0,2));
                    int v = (int)((mtxR.at<double>(1,1)*sR * yR / Z) + mtxR.at<double>(1,2));

                    if (u>=0 && u<640 && v>=0 && v<480) {
                        circle(canvasR, Point(u, v), 2, Scalar(0, 0, 255), -1); // 라이다 점 표시
                        for (auto& d : last_dets) {
                            if (d.cam_id == 1 && d.box.contains(Point(u, v))) {
                                if (clr < d.minClearance) d.minClearance = max(0.0f, clr);
                            }
                        }
                    }
                }
            }
        }

        // 5. 시각화 렌더링
        for (const auto& d : last_dets) {
            Mat& img = (d.cam_id == 0) ? canvasF : canvasR;
            rectangle(img, d.box, Scalar(255, 255, 0), 2); // 박스 그리기
            if (d.minClearance < 10.0f) {
                string label = format("CLR: %.2fm", d.minClearance);
                // 30cm 이내면 빨간색 경고
                Scalar color = (d.minClearance < 0.3f) ? Scalar(0, 0, 255) : Scalar(255, 255, 255);
                putText(img, label, Point(d.box.x, d.box.y - 10), FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            }
        }

        // 결과 합치기 (전방/후방)
        Mat combined; hconcat(canvasF, canvasR, combined);
        
        // 상단 정보창 (네트워크 지연 시간 표시)
        string status = format("Network Latency: %lld ms", diff);
        Scalar statusColor = (diff > 50) ? Scalar(0, 165, 255) : Scalar(0, 255, 0); // 50ms 넘으면 주황색
        putText(combined, status, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.7, statusColor, 2);
        
        imshow("Sensor Fusion Result (RPi 4 Node)", combined);
        if (waitKey(1) == 'q') break;
    }

    // 자원 해제
    free(scan.points); turnOff(laser); lidarDestroy(&laser);
    close(sock);
    return 0;
}