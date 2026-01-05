#include <iostream>
#include <vector>
#include <string>
#include <sys/socket.h>
#include <opencv2/opencv.hpp>
#include "common.h"
#include "ydlidar_sdk.h"

// 카메라 파라미터(보정값) 
Mat mtxF = (Mat_<double>(3,3) << 708.418, 0, 310.005, 0, 706.711, 254.354, 0, 0, 1);
Mat mtxR = (Mat_<double>(3,3) << 705.052, 0, 316.681, 0, 703.592, 251.951, 0, 0, 1);

// 라이다-카메라 외부 파라미터 보정
float angF = -0.023f, yF = -0.150f, sF = 0.76f;
float angR = -0.023f, yR = -0.150f, sR = 0.79f;


int main()
{
    int fd = shm_open(SHM_NAME_LIDAR, O_RDWR, 0666);
    if (fd == -1) {
        perror("Producer: shm_open failed");
        return -1;
    }

    LidarQueue *q = (LidarQueue *)mmap(NULL, sizeof(LidarQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (q == MAP_FAILED) {
        perror("Producer: mmap failed");
        return -1;
    }


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
    if (!initialize(laser)) {
        fprintf(stderr, "Lidar Initialize failed\n");
        return -1;
    }

    // 라이다 모터를 가동하고 실제로 레이저 스캔 시작
    turnOn(laser);

    // 라이다 한 바퀴 스캔 데이터를 담을 구조체 선언
    LaserFan scan;
    // 최대 2048개의 점을 담을 수 있는 메모리 공간을 미리 할당 (성능 최적화)
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * MAX_LIDAR_POINTS);

    printf("[Lidar Producer] Ready to produce data...\n");

    while(true){
        // 전방(F)과 후방(R) 모니터링을 위한 빈 검은색 화면(640x480)을 매 프레임 생성
        Mat viewF = Mat::zeros(480, 640, CV_8UC3);
        Mat viewR = Mat::zeros(480, 640, CV_8UC3);
        
        // 카메라 화면 좌표로 변환된 라이다 점들을 임시로 저장할 리스트
        vector<LidarProj> ptsF, ptsR;

        // 라이다 센서로부터 최신 스캔 데이터(한 바퀴)를 가져옴
        if (doProcessSimple(laser, &scan)) {
            // --- [생산자 영역 시작] ---
            // 빈 슬롯이 생길 때까지 대기
            sem_wait(&q->sem_empty);
            pthread_mutex_lock(&q->mutex);

            // 큐의 tail 위치에 데이터 쓰기
            LidarItem *item = &q->buffer[q->tail];
            item->count = 0;
            item->timestamp = (float)time(NULL);

            for (int i = 0; i < scan.npoints; i++) {
                if (item->count >= MAX_LIDAR_POINTS) break;

                float dist = scan.points[i].range;
                float angle = scan.points[i].angle;

                if (dist < 0.1f || dist > 6.0f) continue;

                // 공통 구조체에 맞게 데이터 가공
                LidarPoint lp;
                lp.dist = dist;
                lp.angle = angle;
                lp.x = dist * sin(angle); // Lidar 기준 X
                lp.y = dist * cos(angle); // Lidar 기준 Y (Depth)

                bool is_valid_proj = false;

                // 전방 투영
                float aF = angle + angF;
                if (abs(aF) < 50.0f * M_PI / 180.0f) {
                    float Z = dist * cos(aF);
                    lp.u = (float)((mtxF.at<double>(0,0)*sF * (dist*sin(aF)) / Z) + mtxF.at<double>(0,2));
                    lp.v = (float)((mtxF.at<double>(1,1)*sF * yF / Z) + mtxF.at<double>(1,2));
                    is_valid_proj = true;
                }
                // 후방 투영
                else {
                    float aR = angle - M_PI + angR;
                    while(aR > M_PI) aR -= 2*M_PI; while(aR < -M_PI) aR += 2*M_PI;
                    if (abs(aR) < 50.0f * M_PI / 180.0f) {
                        float Z = dist * cos(aR);
                        lp.u = (float)((mtxR.at<double>(0,0)*sR * (dist*sin(aR)) / Z) + mtxR.at<double>(0,2));
                        lp.v = (float)((mtxR.at<double>(1,1)*sR * yR / Z) + mtxR.at<double>(1,2));
                        is_valid_proj = true;
                    }
                }

                if (is_valid_proj) {
                    item->points[item->count++] = lp;
                }
            }

            // 큐 인덱스 이동
            q->tail = (q->tail + 1) % QUEUE_SIZE;

            pthread_mutex_unlock(&q->mutex);
            sem_post(&q->sem_full); 
        }
    }


    // 1. 동적 할당된 메모리 해제: malloc으로 빌렸던 라이다 점 저장용 메모리를 OS에 반납
    free(scan.points); 

    // 2. 라이다 하드웨어 정지: 회전 중인 모터를 멈추고 레이저 발사를 중단하여 장치 수명 보호
    turnOff(laser);

    return 0;
}