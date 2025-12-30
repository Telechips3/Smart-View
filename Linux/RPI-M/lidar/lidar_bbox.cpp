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
#include "ydlidar_sdk.h"

using json = nlohmann::json;
using namespace std;
using namespace cv;

#define UDP_PORT 5005

// 카메라 파라미터 유지
Mat mtxF = (Mat_<double>(3,3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat mtxR = (Mat_<double>(3,3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);

float angF = -0.023f, yF = -0.150f, sF = 0.76f;
float angR = -0.023f, yR = -0.150f, sR = 0.79f;

struct LidarProj {
    Point pt; float dist; float ang;
};

// 점들을 선으로 연결해 주는 함수
void drawContour(Mat& img, const vector<LidarProj>& pts, Scalar color) {
    if (pts.size() < 2) return;
    for (size_t i = 1; i < pts.size(); i++) {
        if (abs(pts[i].dist - pts[i-1].dist) < 0.5f) {
            line(img, pts[i-1].pt, pts[i].pt, color, 2, LINE_AA);
        }
    }
}

int main() {
    // --- 1. 라이다 하드웨어 초기화 및 구동 섹션 ---
    os_init(); // YDLidar SDK 내부 운영체제 자원 초기화

    // 라이다 제어를 위한 객체 생성 (동적 할당)
    YDLidar *laser = lidarCreate(); 

    // 라이다가 연결된 시리얼 포트 경로와 통신 속도 설정
    const char *port = "/dev/ttyUSB0"; 
    int baud = 230400; // 초당 230,400 비트 전송 (YDLidar 표준 속도)

    // 생성한 라이다 객체에 포트와 보레이트 옵션을 적용
    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baud, sizeof(int));

    // 라이다 장치 초기화 시도 (성공 시 통신 준비 완료)
    initialize(laser); 

    // 라이다 모터를 가동하고 실제로 레이저 스캔 시작
    turnOn(laser); 

    // --- 2. UDP 네트워크 통신 설정 섹션 (AI 결과 수신용) ---
    // 데이터그램(DGRAM) 방식의 UDP 소켓 생성
    int sock = socket(AF_INET, SOCK_DGRAM, 0); 

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr)); // 메모리 초기화

    servaddr.sin_family = AF_INET;           // IPv4 주소 체계 사용
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY); // 모든 네트워크 인터페이스로부터 수신 허용
    servaddr.sin_port = htons(UDP_PORT);    // 지정된 포트(5005)를 네트워크 바이트 순서로 변환

    // 소켓을 설정한 주소와 포트에 결합(Bind)하여 데이터 수신 대기 상태로 전환
    bind(sock, (const struct sockaddr *)&servaddr, sizeof(servaddr));

    // --- 3. 통신 최적화 및 메모리 관리 섹션 ---
    // 수신 타임아웃(Timeout) 설정: 데이터가 안 들어올 때 무한 대기를 방지
    struct timeval tv; 
    tv.tv_sec = 0; 
    tv.tv_usec = 1000; // 1,000 마이크로초 = 0.001초 동안만 데이터 기다림

    // 소켓 옵션에 타임아웃 적용 (프로그램의 실시간성 확보를 위해 매우 중요)
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 라이다 한 바퀴 스캔 데이터를 담을 구조체 선언
    LaserFan scan;
    // 최대 2048개의 점을 담을 수 있는 메모리 공간을 미리 할당 (성능 최적화)
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * 2048); 

    // UDP로 들어올 JSON 문자열 데이터를 임시로 담아둘 버퍼(바구니)
    char buffer[8192];

    while (true) {
        // 전방(F)과 후방(R) 모니터링을 위한 빈 검은색 화면(640x480)을 매 프레임 생성
        Mat viewF = Mat::zeros(480, 640, CV_8UC3);
        Mat viewR = Mat::zeros(480, 640, CV_8UC3);
        
        // 카메라 화면 좌표로 변환된 라이다 점들을 임시로 저장할 리스트
        vector<LidarProj> ptsF, ptsR;

        // 라이다 센서로부터 최신 스캔 데이터(한 바퀴)를 가져옴
        bool lidar_ok = doProcessSimple(laser, &scan);
        if (lidar_ok) {
            // 가져온 모든 점(npoints)을 하나씩 처리
            for (int i = 0; i < scan.npoints; i++) {
                float dist = scan.points[i].range;  // 해당 점의 거리(m)
                float angle = scan.points[i].angle; // 해당 점의 각도(radian)

                // 센서 노이즈나 너무 먼 데이터(6m 이상)는 무시
                if (dist < 0.1f || dist > 6.0f) continue;

                // --- [전방 카메라 투영 로직] ---
                float aF = angle + angF; // 센서 설치 오차 각도 보정
                // 정면 기준 좌우 50도 이내의 점들만 카메라 화면에 뿌림
                if (abs(aF) < 50.0f * M_PI / 180.0f) {
                    float Z = dist * cos(aF); // 카메라에서 물체까지의 수평 거리(깊이)
                    // 삼각함수와 카메라 행렬(mtx)을 이용해 3D 거리를 2D 픽셀 X 좌표(u)로 변환
                    int u = (int)((mtxF.at<double>(0,0)*sF * (dist*sin(aF)) / Z) + mtxF.at<double>(0,2));
                    // 카메라 설치 높이(yF)를 이용해 2D 픽셀 Y 좌표(v)로 변환
                    int v = (int)((mtxF.at<double>(1,1)*sF * yF / Z) + mtxF.at<double>(1,2));
                    // 계산된 좌표가 640x480 화면 안에 들어오면 저장
                    if (u>=0 && u<640 && v>=0 && v<480) ptsF.push_back({Point(u, v), dist, angle});
                }

                // --- [후방 카메라 투영 로직] ---
                float aR = angle - M_PI + angR; // 180도 회전시켜 후방 각도로 변환
                // 각도 범위가 -PI ~ PI를 유지하도록 보정
                while(aR > M_PI) aR -= 2*M_PI; while(aR < -M_PI) aR += 2*M_PI;
                // 후방 중심 기준 좌우 50도 이내의 점들만 처리
                if (abs(aR) < 50.0f * M_PI / 180.0f) {
                    float Z = dist * cos(aR); // 후방 깊이 계산
                    int u = (int)((mtxR.at<double>(0,0)*sR * (dist*sin(aR)) / Z) + mtxR.at<double>(0,2));
                    int v = (int)((mtxR.at<double>(1,1)*sR * yR / Z) + mtxR.at<double>(1,2));
                    if (u>=0 && u<640 && v>=0 && v<480) ptsR.push_back({Point(u, v), dist, angle});
                }
            }
        }

        // 화면 좌표로 변환된 라이다 점들을 선(윤곽선)으로 연결하여 그림 (전방:초록, 후방:빨강)
        drawContour(viewF, ptsF, Scalar(0, 255, 0));
        drawContour(viewR, ptsR, Scalar(0, 0, 255));

        // 네트워크(UDP)를 통해 외부 AI 모델이 보낸 객체 인식 데이터(JSON) 수신
        int n = recv(sock, buffer, sizeof(buffer) - 1, 0);
        if (n > 0) {
            buffer[n] = '\0'; // 문자열 끝 표시
            try {
                auto j = json::parse(buffer); // 텍스트를 JSON 구조로 변환
                for (auto& det : j["detections"]) { // 인식된 모든 물체에 대해 반복
                    int cam_id = det[0]; // 어느 카메라인가? (0:전방, 1:후방)
                    float rx = det[2].get<float>(), ry = det[3].get<float>(); // 박스 좌표
                    float rw = det[4].get<float>(), rh = det[5].get<float>(); // 박스 크기

                    // 화면을 꽉 채우는 비정상적으로 큰 박스는 무시
                    if (rw > 600 || rh > 450) continue; 

                    // 박스 좌표가 화면 밖으로 나가지 않도록 고정
                    int bx = max(0, (int)rx);
                    int by = max(0, (int)ry);
                    int bw = min(640 - bx, (int)rw);
                    int bh = min(480 - by, (int)rh);

                    if (bw <= 0 || bh <= 0) continue;

                    // 물체 박스(box)와 라이다 점을 찾을 약간 더 넓은 영역(searchBox) 설정
                    Rect box(bx, by, bw, bh);
                    Rect searchBox(bx - 5, by - 5, bw + 10, bh + 10);

                    float minD = 99.0f, targetA = 0.0f;
                    bool found = false;

                    // 해당 카메라에 투영된 라이다 점 리스트 선택
                    const vector<LidarProj>& currentPts = (cam_id == 0) ? ptsF : ptsR;
                    for (const auto& p : currentPts) {
                        // 라이다 점이 AI가 그린 박스 안에 들어와 있는지 확인
                        if (searchBox.contains(p.pt)) {
                            // 박스 안의 점들 중 가장 가까운 거리값을 물체의 실제 거리로 채택
                            if (p.dist < minD) { minD = p.dist; targetA = p.ang; found = true; }
                        }
                    }

                    // 박스를 그릴 대상 화면과 색상 결정
                    Mat& targetView = (cam_id == 0) ? viewF : viewR;
                    Scalar color = (cam_id == 0) ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
                    rectangle(targetView, box, color, 2); // 화면에 AI 박스 그리기

                    if (found) { // 박스 안에 라이다 점이 매칭되었다면
                        char label[50];
                        sprintf(label, "%.2fm, %.1fdeg", minD, targetA * 180.0f / M_PI); // 거리/각도 문자열 제작
                        // 가독성을 위해 검은색 외곽선(두께 3) 위에 흰색 글씨 쓰기
                        putText(targetView, label, Point(bx, by - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 3);
                        putText(targetView, label, Point(bx, by - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
                    }
                }
            } catch (...) {} // 에러 발생 시 프로그램 중단 방지
        }

        // 전방 화면과 후방 화면을 가로로 하나로 합침
        Mat combined; hconcat(viewF, viewR, combined);
        // 통합 화면을 "Lidar Monitoring"이라는 창에 띄움
        imshow("Lidar Monitoring", combined);
        // 10ms 동안 키 입력을 기다림, 'q'를 누르면 반복문 탈출
        if (waitKey(10) == 'q') break;
    }

    // --- 프로그램 종료 및 자원 해제 섹션 ---
    // 1. 동적 할당된 메모리 해제: malloc으로 빌렸던 라이다 점 저장용 메모리를 OS에 반납
    free(scan.points); 

    // 2. 라이다 하드웨어 정지: 회전 중인 모터를 멈추고 레이저 발사를 중단하여 장치 수명 보호
    turnOff(laser); 

    // 3. 라이다 객체 파괴: 소프트웨어적으로 생성된 라이다 제어 객체를 소멸시키고 시리얼 포트 연결 해제
    lidarDestroy(&laser); 

    // 4. 네트워크 소켓 닫기: AI 데이터를 받기 위해 열어두었던 UDP 통신 통로(5005 포트)를 폐쇄
    // (이걸 안 하면 재실행 시 "Address already in use" 에러가 발생할 수 있음)
    close(sock);
    return 0;
}