#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <arpa/inet.h>
#include <opencv2/opencv.hpp>
#include "ydlidar_sdk.h"
#include "../common.h"
#include "../spi/common/protocol.h"
#include <nlohmann/json.hpp> // JSON 파싱을 위해 추가

using namespace std;
using namespace cv;
using json = nlohmann::json;

float angF = -0.05f, yF = -0.05f, sF = 0.76f;
float angR = -0.05f, yR = -0.05f, sR = 0.79f;
volatile sig_atomic_t stop_flag = 0; // 시그널 핸들러와 공유할 인자 역할

// 차량 규격 설정(단위: 미터)
// 차량 폭
const float VEH_HALF_WIDTH = 0.08f;
// 차량 길이
const float VEH_FRONT_LEN = 0.15f;
const float VEH_REAR_LEN = 0.15f;

int fd;

void handle_sigint(int sig)
{
    printf("\n[수신] SIGINT (%d) 발생! 프로그램을 정리하고 종료합니다.\n", sig);
    stop_flag = 1;
}

// 차량 외곽선 보정 함수
float getVehicleOffset(float angle_rad, bool isFront)
{
    // 각도 정규화
    float adjusted_angle = isFront ? angle_rad : (angle_rad - M_PI);
    while (adjusted_angle > M_PI)
        adjusted_angle -= 2 * M_PI;
    while (adjusted_angle < -M_PI)
        adjusted_angle += 2 * M_PI;

    float abs_angle = std::abs(adjusted_angle);
    float L = isFront ? VEH_FRONT_LEN : VEH_REAR_LEN; // 전/후방 길이 선택
    float W = VEH_HALF_WIDTH;                         // 차량 폭

    // 코사인 값이 너무 작아지는 것 방지
    float cos_v = std::max(std::cos(abs_angle), 0.000001f);
    float sin_v = std::max(std::sin(abs_angle), 0.000001f);

    float dist_to_front = L / cos_v;
    float dist_to_side = W / sin_v;

    // 오프셋이 차체 크기보다 커지지 않도록 제한
    float offset = std::min(dist_to_front, dist_to_side);
    return std::min(offset, 0.25f); // 최대 보정치를 25cm로 제한
}

void drawContour(Mat &img, const vector<LidarPoint> &pts, Scalar color)
{
    if (pts.size() < 2)
        return;

    for (size_t i = 1; i < pts.size(); i++)
    {
        // 인접한 포인트 사이의 거리 차이가 작을 때만 선을 연결
        if (std::abs(pts[i].dist - pts[i - 1].dist) < 0.5f)
        {
            // .pt 대신 .u, .v를 사용하여 Point 객체를 생성
            Point p1((int)pts[i - 1].u, (int)pts[i - 1].v);
            Point p2((int)pts[i].u, (int)pts[i].v);

            // 이미지 범위 안인지 확인 후 그리기
            if (p1.x >= 0 && p1.x < img.cols && p2.x >= 0 && p2.x < img.cols)
            {
                line(img, p1, p2, color, 2, LINE_AA);
            }
        }
    }
}

// Base64 문자열을 바이너리 데이터로 변환하는 함수
static const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
std::vector<unsigned char> base64_decode(const std::string &in)
{
    std::vector<unsigned char> out;
    std::vector<int> T(256, -1);
    for (int i = 0; i < 64; i++)
        T[base64_chars[i]] = i;
    int val = 0, valb = -8;
    for (unsigned char c : in)
    {
        if (T[c] == -1)
            break;
        val = (val << 6) + T[c];
        valb += 6;
        if (valb >= 0)
        {
            out.push_back(char((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    return out;
}

void calibrateAndMatch(CameraQueue *cur_q, const vector<LidarPoint> &currentPts, Mat &targetView, Mat &topView, bool isfront, int64_t timestamp)
{
    CameraItem target_item; // 복사본을 저장할 로컬 변수
    const int64_t THRESHOLD = 100000;
    bool found_item = false;
    // printf("[Main] Searching Camera Queue (isfront=%d) for Timestamp: %lu\n", isfront ? 1 : 0, timestamp);

    while (!found_item)
    {
        // 큐에서 타임스탬프가 일치하는 항목을 찾음
        if (sem_trywait(&cur_q->sem_full) != 0)
        {
            // 카메라 큐가 비어있으면 일단 탈출 (다음 라이다 프레임에서 다시 시도)
            break;
        }

        pthread_mutex_lock(&cur_q->mutex);
        CameraItem *c_item = &cur_q->buffer[cur_q->head];
        int64_t diff = (int64_t)timestamp - (int64_t)c_item->timestamp;

        // 타임스탬프 비교 로직
        if (std::abs(diff) <= THRESHOLD)
        {
            // [매칭 성공] 오차 범위 내에 있음
            // printf("[Main]RPI-M: %lu, RPI-S: %lu, Diff: %ld\n", timestamp, c_item->timestamp, diff);
            memcpy(&target_item, c_item, sizeof(CameraItem));
            found_item = true;

            // 매칭된 데이터는 소모했으므로 head 이동
            cur_q->head = (cur_q->head + 1) % QUEUE_SIZE;
            pthread_mutex_unlock(&cur_q->mutex);
            sem_post(&cur_q->sem_empty);
            break;
        }
        else if (diff > THRESHOLD)
        {
            // [과거 데이터] 카메라 데이터가 라이다보다 너무 오래됨 -> 버리고 다음 것 확인
            cur_q->head = (cur_q->head + 1) % QUEUE_SIZE;
            pthread_mutex_unlock(&cur_q->mutex);
            sem_post(&cur_q->sem_empty);
        }
        else
        {
            // [미래 데이터] 카메라 데이터가 라이다보다 너무 최신임
            // 현재 라이다 프레임은 버리거나, 이 카메라 데이터를 나중에 써야 함
            // 여기서는 일단 뮤텍스만 풀고 세마포어 원복 후 종료
            pthread_mutex_unlock(&cur_q->mutex);
            sem_post(&cur_q->sem_full); // 꺼내려 했던 것을 다시 원복
            break;
        }
    }

    if (found_item)
    {
        float targetA = 0.0f;
        float minD = 1000000.0f;
        int minIDX = 0;
        int curIDX = 0;

        int size = 600;        // 창 크기
        float max_dist = 3.0f; // 최대 3미터 표시
        float scale = size / (max_dist * 2.0f);
        int center = size / 2;

        // 0.5m 단위 동심원
        for (float i = 0.5f; i <= max_dist; i += 0.5f)
        {
            int radius = (int)(i * scale);
            circle(topView, Point(center, center), radius, Scalar(60, 60, 60), 1);
            if (i <= max_dist)
            {
                putText(topView, format("%.1fm", i), Point(center + radius + 2, center),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(120, 120, 120));
            }
        }

        // printf("[Debug] Camera Data Received! Timestamp: %lu, ObjCount: %d, Data Size: %zu\n",
        //        target_item.timestamp, target_item.obj_count, strlen(target_item.data));

        
        // 이미지 복원 로직
        if (target_item.data[0] != '\0')
        {
            try
            {
                // char 배열을 std::string으로 변환
                std::string b64_str(target_item.data);

                // Base64 디코딩
                std::vector<unsigned char> decoded_bin = base64_decode(b64_str);

                if (!decoded_bin.empty())
                {
                    // 바이너리 데이터를 OpenCV용 Mat 객체로 래핑
                    cv::Mat rawData(1, decoded_bin.size(), CV_8UC1, decoded_bin.data());

                    // JPEG/PNG 압축을 풀어 실제 픽셀(Mat)로 변환
                    cv::Mat decoded_img = cv::imdecode(rawData, cv::IMREAD_COLOR);

                    if (!decoded_img.empty())
                    {
                        // targetView 크기에 맞춰 리사이즈 후 복사
                        if (!targetView.empty())
                        {
                            cv::resize(decoded_img, targetView, targetView.size());
                        }
                        else
                        {
                            targetView = decoded_img;
                        }
                    }
                    else
                    {
                        printf("Decode Failed: OpenCV imdecode could not read the buffer\n");
                    }
                }
                else
                {
                    printf("Base64 Decode Failed: Output buffer is empty\n");
                }
            }
            catch (const std::exception &e)
            {
                printf("Exception during image processing: %s\n", e.what());
            }
        }

        
        // 카메라 화면에 라이다 포인트 투영(Overlay)

        for (int i = 0; i < target_item.obj_count; i++)
        {
            if (i == 1)
                break;
            BBox &b = target_item.objects[i];
            Rect box((int)b.x, (int)b.y, (int)b.w, (int)b.h);
            Rect searchBox(box.x - 10, box.y - 10, box.width + 20, box.height + 20);

            targetA = 0.0f;
            minD = 1000000.0f;
            

            // Lidar 포인트와 BBox 매칭.
            curIDX = 0;
            for (const auto &lp : currentPts)
            {
                // 노란색 점 그리기
                if (lp.u >= 0 && lp.u < 640 && lp.v >= 0 && lp.v < 480)
                {
                    // circle(targetView, Point((int)lp.u, (int)lp.v), 2, Scalar(0, 255, 255), -1);
                }

                float getOffset = lp.dist - getVehicleOffset(lp.angle, isfront);
                // printf("Lidar Point Angle: %.2f rad, Dist: %.2f m, Vehicle Offset: %.2f m\n",
                //        lp.angle, lp.dist,
                //        getVehicleOffset(lp.angle, isfront));

                if (getOffset < minD)
                {
                    minD = getOffset;
                    targetA = lp.angle;
                    minIDX = curIDX;
                }
                curIDX++;
                if (searchBox.contains(cv::Point2f(lp.u, lp.v)))
                {
                    // 빨간색 점 그리기
                    // circle(targetView, Point((int)lp.u, (int)lp.v), 3, Scalar(0, 0, 255), -1);
                }
            }

            circle(topView, Point(center, center), 4, Scalar(255, 255, 255), -1);
            if (!targetView.empty())
            {
                curIDX = 0;
                for (const auto &lp : currentPts)
                {
                    // 이미지 범위(640x480) 내에 있는 점만 그리기

                    int px = center + (int)(lp.x * scale);
                    int py = center - (int)(lp.y * scale);

                    Scalar color;
                    int qsize = 2;
                    int tri_size = 5;
                    if (lp.u >= 0 && lp.u < targetView.cols && lp.v >= 0 && lp.v < targetView.rows)
                    {
                        if (curIDX == minIDX)
                        {
                            color = Scalar(0, 0, 255); // 가장 가까운 점은 빨간색
                            qsize = 5;
                        }
                        else if (lp.dist < 0.5f)
                        {
                            color = Scalar(0, 255, 255); // 0.5m 미만 노란색
                        }
                        else
                        {
                            color = Scalar(0, 255, 0);
                        }
                        circle(targetView, Point((int)lp.u, (int)lp.v), qsize, color, -1);
                    }

                    if (px >= 0 && px < size && py >= 0 && py < size)
                    {
                        if (curIDX == minIDX)
                        {
                            tri_size = 8;
                            vector<Point> triangle_pts;
                            
                            triangle_pts.push_back(Point(px, py - tri_size));            // 위
                            triangle_pts.push_back(Point(px - tri_size, py + tri_size)); // 좌하
                            triangle_pts.push_back(Point(px + tri_size, py + tri_size)); // 우하
                            fillPoly(topView, triangle_pts, color);
                        }
                        else
                        {
                            circle(topView, Point(px, py), 2, color, -1);
                        }
                    }

                    curIDX++;
                }

                minD = std::max(0.0f, minD);
                minD += 0.15f; // 보정치;

                UART_Packet_t p = {0};
                p.header = 0xAA;
                p.detected = isfront ? 0 : 1;
                p.distance = minD * 100.0f;
                p.class_ID = target_item.objects[i].class_id;
                p.timestamp = timestamp;
                p.bbox_x = b.x;
                p.bbox_y = b.y;
                p.bbox_h = b.h;
                p.bbox_w = b.w;

                uint8_t crc = 0;
                uint8_t *ptr = (uint8_t *)&p;
                for (int i = 0; i < (int)PACKET_SIZE - 1; i++)
                    crc ^= ptr[i];
                p.checksum = crc;
                write(fd, &p, PACKET_SIZE);

                if (isfront)
                {
                    printf("Front BBox %d: Distance = %.2f m\n", target_item.objects[i].class_id, minD);
                }
                else
                {
                    printf("Rear BBox %d: Distance = %.2f m\n", target_item.objects[i].class_id, minD);
                }
            }
        }
    }
}

int main()
{
    int fd_l = shm_open(SHM_NAME_LIDAR, O_RDWR, 0666);
    int fd_f = shm_open(SHM_NAME_FRONT_CAMERA, O_RDWR, 0666);
    int fd_b = shm_open(SHM_NAME_BACK_CAMERA, O_RDWR, 0666);

    signal(SIGINT, handle_sigint);

    LidarQueue *q_l = (LidarQueue *)mmap(NULL, sizeof(LidarQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd_l, 0);
    CameraQueue *q_f = (CameraQueue *)mmap(NULL, sizeof(CameraQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd_f, 0);
    CameraQueue *q_b = (CameraQueue *)mmap(NULL, sizeof(CameraQueue), PROT_READ | PROT_WRITE, MAP_SHARED, fd_b, 0);

    vector<LidarPoint> ptsF, ptsR;
    ptsF.reserve(MAX_LIDAR_POINTS);
    ptsR.reserve(MAX_LIDAR_POINTS);

    fd = open("/dev/stm32_spi", O_WRONLY);
    if (fd < 0)
    {
        perror("장치 열기 실패");
        return -1;
    }

    int64_t timestamp = 0;

    int frameCount = 0;
    double fps = 0.0;
    int64 startTime = cv::getTickCount();

    while (!stop_flag)
    {
        frameCount++;

        Mat viewF = Mat::zeros(480, 640, CV_8UC3);
        Mat viewR = Mat::zeros(480, 640, CV_8UC3);
        Mat topView = Mat::zeros(650, 650, CV_8UC3);

        // Lidar 데이터 가져오기
        if (sem_wait(&q_l->sem_full) == 0)
        {
            pthread_mutex_lock(&q_l->mutex);
            LidarItem *item = &q_l->buffer[q_l->head];

            // 새 데이터가 왔을 때만 기존 리스트를 지우고 새로 채움
            ptsF.clear();
            ptsR.clear();

            // printf("[Main] Lidar Data Received! Count: %d, First Dist: %.2f\n", item->count, item->points[0].dist);

            for (int i = 0; i < item->count; i++)
            {
                // 각도에 따라 전방/후방 분류
                if (abs(item->points[i].angle) < 1.57f)
                    ptsR.push_back(item->points[i]);
                else
                    ptsF.push_back(item->points[i]);
            }

            timestamp = item->timestamp;
            q_l->head = (q_l->head + 1) % QUEUE_SIZE;
            pthread_mutex_unlock(&q_l->mutex);
            sem_post(&q_l->sem_empty);
        }

        calibrateAndMatch(q_f, ptsF, viewF, topView, true, timestamp);
        calibrateAndMatch(q_b, ptsR, viewR, topView, false, timestamp);

        Mat combined;
        hconcat(viewF, viewR, combined);

        putText(combined, "FRONT", Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        putText(combined, "REAR", Point(650, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

        // FPS 계산 및 그리기 로직
        int64 currentTime = cv::getTickCount();
        // 경과 시간 계산 (초 단위)
        double timeDiff = (currentTime - startTime) / cv::getTickFrequency();

        // 1초마다 FPS 갱신
        if (timeDiff >= 1.0) {
            fps = frameCount / timeDiff;
            frameCount = 0;
            startTime = currentTime;
        }

        // 화면 좌측 상단(FRONT 밑)에 흰색으로 FPS 표시
        string fpsString = cv::format("FPS: %.1f", fps);
        putText(combined, fpsString, Point(10, 70), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
        putText(combined, fpsString, Point(650, 70), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);


        imshow("Lidar_TopView_640x640", topView);
        imshow("Main Controller - Camera Streams", combined);

        if (waitKey(1) == 'q')
            break; // 'q'를 누르면 종료
    }

    munmap(q_l, sizeof(LidarQueue));
    munmap(q_f, sizeof(CameraQueue));
    munmap(q_b, sizeof(CameraQueue));
    close(fd_l);
    close(fd_f);
    close(fd_b);
    return 0;
}
