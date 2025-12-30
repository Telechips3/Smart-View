#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>
#include "nlohmann/json.hpp"
#include "ydlidar_sdk.h"

using namespace cv;
using namespace std;
using json = nlohmann::json;

#define UDP_PORT 5005 // YOLO 탐지 데이터를 받을 UDP 포트 번호

/**
 * [차량 규격 설정]
 * 라이다 센서의 위치를 중심으로 차량의 실제 범퍼 외곽까지의 거리를 설정합니다.
 * 이 값들을 기준으로 측정된 라이다 거리에서 차량 크기를 차감합니다.
 */
const float VEH_HALF_WIDTH = 0.15f;  // 차량 폭의 절반 (중심에서 좌/우 거리)
const float VEH_FRONT_LEN = 0.20f;   // 라이다 중심에서 전방 범퍼 끝까지의 거리
const float VEH_REAR_LEN = 0.10f;    // 라이다 중심에서 후방 범퍼 끝까지의 거리

/**
 * [카메라 내부 파라미터 및 보정값]
 * 렌즈 왜곡을 펴주고 3D 점을 2D 이미지 좌표로 변환하는 데 사용됩니다.
 */
Mat mtxF = (Mat_<double>(3,3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat distF = (Mat_<double>(1,5) << -0.01189, 0.91172, -0.00182, -0.00026, -2.6566);
Mat mtxR = (Mat_<double>(3,3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);
Mat distR = (Mat_<double>(1,5) << -0.03829, 1.44263, -0.00224, -0.00417, -4.1455);

// 라이다-카메라 정렬을 위한 외부 파라미터 (미세 조정값)
float angF = -0.023f, yF = -0.050f, sF = 0.76f;
float angR = -0.023f, yR = -0.040f, sR = 0.79f;

// 감지된 객체 정보를 저장하는 구조체
struct Detection {
    int cam_id;         // 0: 전방, 1: 후방
    Rect box;           // YOLO 바운딩 박스 좌표
    float minClearance = 99.0f; // 해당 박스 내 장애물 중 가장 가까운 여유 거리
};

/**
 * [차량 외곽선 보정 함수]
 * 라이다 측정 각도에 따라 센서 중심에서 해당 방향의 범퍼 끝까지의 거리를 계산합니다.
 */
float getVehicleOffset(float angle_rad, bool isFront) {
    float abs_angle = abs(angle_rad);
    float L = isFront ? VEH_FRONT_LEN : VEH_REAR_LEN; // 전/후방 길이 선택
    float W = VEH_HALF_WIDTH;                         // 차량 폭

    if (abs_angle == 0) return L; // 정면일 경우 직진 거리 반환

    // 삼각함수를 이용하여 직사각형 차량의 경계면까지의 거리를 계산
    float boundary_dist = L / cos(abs_angle); // 앞/뒤 면까지의 거리
    float side_dist = W / sin(abs_angle);     // 좌/우 측면까지의 거리
    
    // 두 값 중 짧은 값이 실제 차량의 외곽선까지의 거리임
    return min(boundary_dist, side_dist);
}

int main() {
    // 1. UDP 통신 설정: 비차단(Non-blocking) 모드로 소켓 생성
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(UDP_PORT);
    bind(sock, (const struct sockaddr*)&addr, sizeof(addr));
    fcntl(sock, F_SETFL, O_NONBLOCK);
    char buf[8192];

    // 2. 카메라 장치 열기
    VideoCapture capF(0, CAP_V4L2); VideoCapture capR(2, CAP_V4L2);
    capF.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    capR.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));

    // 3. YDLidar SDK 초기화 및 가동
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

        // 렌즈 왜곡 보정 수행
        undistort(frameF, undF, mtxF, distF);
        undistort(frameR, undR, mtxR, distR);

        // 4. UDP로부터 YOLO 감지 데이터 수신 및 파싱
        vector<Detection> dets;
        int len = recv(sock, buf, sizeof(buf)-1, 0);
        if (len > 0) {
            buf[len] = '\0';
            try {
                auto j = json::parse(buf);
                if (j.contains("detections")) {
                    for (auto& d : j["detections"]) {
                        dets.push_back({d[0].get<int>(), 
                                        Rect((int)d[2].get<float>(), (int)d[3].get<float>(), 
                                             (int)d[4].get<float>(), (int)d[5].get<float>()), 99.0f});
                    }
                }
            } catch (...) {}
        }

        // 5. 라이다 데이터 처리 및 이미지 투영
        if (doProcessSimple(laser, &scan)) {
            for (int i = 0; i < scan.npoints; i++) {
                float dist = scan.points[i].range;  // 라이다 측정 거리 (m)
                float angle = scan.points[i].angle; // 라이다 측정 각도 (rad)
                if (dist < 0.1f || dist > 6.0f) continue; // 유효 범위 필터링

                // --- [전방 카메라 투영 및 거리 계산] ---
                float aF = angle + angF; // 각도 보정
                if (abs(aF) < 55.0f * M_PI / 180.0f) { // 전방 시야각(FOV) 제한
                    // 차량 외곽을 고려한 실제 여유 거리 계산
                    float clearance = dist - getVehicleOffset(aF, true);

                    // 3D 점을 2D 이미지 좌표로 투영 (핀홀 카메라 모델)
                    float Z = dist * cos(aF);
                    int u = (int)((mtxF.at<double>(0,0)*sF * (dist*sin(aF)) / Z) + mtxF.at<double>(0,2));
                    int v = (int)((mtxF.at<double>(1,1)*sF * yF / Z) + mtxF.at<double>(1,2));

                    if (u>=0 && u<640 && v>=0 && v<480) {
                        circle(undF, Point(u, v), 2, Scalar(0, 255, 0), -1); // 라이다 점 그리기
                        // 투영된 점이 YOLO 바운딩 박스 안에 있는지 확인
                        for (auto& d : dets) {
                            if (d.cam_id == 0 && d.box.contains(Point(u, v))) {
                                if (clearance < d.minClearance) d.minClearance = max(0.0f, clearance);
                            }
                        }
                    }
                }

                // --- [후방 카메라 투영 및 거리 계산] ---
                float aR = angle - M_PI + angR; // 후방은 180도 회전 필요
                while(aR > M_PI) aR -= 2*M_PI; while(aR < -M_PI) aR += 2*M_PI;

                if (abs(aR) < 55.0f * M_PI / 180.0f) {
                    float clearance = dist - getVehicleOffset(aR, false);
                    float Z = dist * cos(aR);
                    int u = (int)((mtxR.at<double>(0,0)*sR * (dist*sin(aR)) / Z) + mtxR.at<double>(0,2));
                    int v = (int)((mtxR.at<double>(1,1)*sR * yR / Z) + mtxR.at<double>(1,2));
                    if (u>=0 && u<640 && v>=0 && v<480) {
                        circle(undR, Point(u, v), 2, Scalar(0, 0, 255), -1);
                        for (auto& d : dets) {
                            if (d.cam_id == 1 && d.box.contains(Point(u, v))) {
                                if (clearance < d.minClearance) d.minClearance = max(0.0f, clearance);
                            }
                        }
                    }
                }
            }
        }

        // 6. 최종 이미지 출력 및 텍스트 렌더링
        for (const auto& d : dets) {
            Mat& targetImg = (d.cam_id == 0) ? undF : undR;
            rectangle(targetImg, d.box, Scalar(255, 255, 0), 2); // 바운딩 박스
            
            if (d.minClearance < 10.0f) {
                string label = format("CLR: %.2fm", d.minClearance);
                // 위험 알람: 여유 거리가 0.3m 미만이면 빨간색으로 표시
                Scalar color = (d.minClearance < 0.3f) ? Scalar(0, 0, 255) : Scalar(255, 255, 255);
                putText(targetImg, label, Point(d.box.x, d.box.y - 10), FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            }
        }

        // 전/후방 영상을 가로로 병합하여 표시
        Mat combined; hconcat(undF, undR, combined);
        imshow("Danger Distance Monitoring", combined);
        if (waitKey(1) == 'q') break;
    }

    // 자원 해제
    free(scan.points); turnOff(laser); lidarDestroy(&laser);
    close(sock);
    return 0;
}