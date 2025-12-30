#include "common.h"

// 카메라 파라미터(보정값) 
Mat mtxF = (Mat_<double>(3,3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat mtxR = (Mat_<double>(3,3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);

// 라이다-카메라 외부 파라미터 보정
float angF = -0.023f, yF = -0.150f, sF = 0.76f;
float angR = -0.023f, yR = -0.150f, sR = 0.79f;

struct LidarProj {
    Point pt;   // 화면상의 (x, y) 픽셀 좌표
    float dist; // 실제 물체와의 거리 (meters)
    float ang;  // 라이다 센서 기준 실제 각도 (radians)
};

int main()
{
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

    // 라이다 한 바퀴 스캔 데이터를 담을 구조체 선언
    LaserFan scan;
    // 최대 2048개의 점을 담을 수 있는 메모리 공간을 미리 할당 (성능 최적화)
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * 2048);

    while(true){
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
    }


    // 1. 동적 할당된 메모리 해제: malloc으로 빌렸던 라이다 점 저장용 메모리를 OS에 반납
    free(scan.points); 

    // 2. 라이다 하드웨어 정지: 회전 중인 모터를 멈추고 레이저 발사를 중단하여 장치 수명 보호
    turnOff(laser);

    return 0;
}