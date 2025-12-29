#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include "nlohmann/json.hpp"
#include "ydlidar_sdk.h"

using namespace cv;
using namespace std;
using json = nlohmann::json;

#define WIN_W 640
#define WIN_H 480
#define UDP_PORT 5005
#define LIDAR_SCALE 2.0f 

struct Detection {
    int cam_id;
    Rect box;
};

void renderFusion(Mat& canvas, const vector<Point3f>& pts3d, const Mat& mtx, const Mat& dist, 
                  const Mat& rvec, const Mat& tvec, const vector<Detection>& dets, int target_id, Scalar ptColor, double fps) {
    
    if (!pts3d.empty()) {
        vector<Point2f> pts2d;
        projectPoints(pts3d, rvec, tvec, mtx, dist, pts2d);

        vector<Rect> active_boxes;
        for (const auto& d : dets) {
            if (d.cam_id == target_id) active_boxes.push_back(d.box);
        }

        for (const auto& p : pts2d) {
            Point2f scaled_p(p.x * LIDAR_SCALE, p.y * LIDAR_SCALE);
            if (scaled_p.x >= 0 && scaled_p.x < canvas.cols && scaled_p.y >= 0 && scaled_p.y < canvas.rows) {
                bool inside = false;
                for (const auto& box : active_boxes) {
                    if (box.contains(scaled_p)) { inside = true; break; }
                }
                circle(canvas, scaled_p, 3, inside ? Scalar(0, 255, 255) : ptColor, -1);
            }
        }

        for (const auto& box : active_boxes) {
            rectangle(canvas, box, Scalar(255, 0, 0), 2);
            putText(canvas, "OBJ", Point(box.x, box.y - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
        }
    }

    // FPS ���
    string fps_text = "FPS: " + to_string((int)round(fps));
    putText(canvas, fps_text, Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 2);
}

int main() {
    // 1. ��Ʈ��ŷ �� ���� ����ȭ
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    
    // ���� ���� Ȯ�� (������ ���� ����)
    int rcvbuf = 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY; 
    addr.sin_port = htons(UDP_PORT);
    bind(sock, (const struct sockaddr*)&addr, sizeof(addr));
    fcntl(sock, F_SETFL, O_NONBLOCK);

    // 2. ���� �Ķ���� �ε�
    Mat mtx_f, dist_f, mtx_b, dist_b, rvec, tvec;
    FileStorage fs_f("calib_data/front_params.yaml", FileStorage::READ);
    FileStorage fs_b("calib_data/back_params.yaml", FileStorage::READ);
    FileStorage fs_ex("calib_data/lidar_cam_extrinsic.yaml", FileStorage::READ);
    
    fs_f["camera_matrix"] >> mtx_f; fs_f["distortion_coefficients"] >> dist_f;
    fs_b["camera_matrix"] >> mtx_b; fs_b["distortion_coefficients"] >> dist_b;
    fs_ex["rvec"] >> rvec; fs_ex["tvec"] >> tvec;

    // 3. YDLIDAR ���� ��ĵ ����
    os_init();
    YDLidar *laser = lidarCreate();
    const char *port = "/dev/ttyUSB0"; int baudrate = 230400;
    setlidaropt(laser, LidarPropSerialPort, port, strlen(port));
    setlidaropt(laser, LidarPropSerialBaudrate, &baudrate, sizeof(int));

    // ���ļ� ���� (12Hz) �� ���ø� ����Ʈ ����
    float frequency = 12.0f;
    int sample_rate = 9;
    setlidaropt(laser, LidarPropScanFrequency, &frequency, sizeof(float));
    setlidaropt(laser, LidarPropSampleRate, &sample_rate, sizeof(int));

    initialize(laser); turnOn(laser);

    LaserFan scan;
    scan.points = (LaserPoint *)malloc(sizeof(LaserPoint) * 2000);
    char buf[8192];

    auto prev_time = chrono::high_resolution_clock::now();
    double fps = 0;

    cout << "\n[System High-Speed Mode] Scan Frequency: 12Hz" << endl;

    while (true) {
        auto curr_time = chrono::high_resolution_clock::now();
        chrono::duration<double> diff = curr_time - prev_time;
        prev_time = curr_time;
        fps = 1.0 / diff.count();

        Mat canvas_f = Mat::zeros(WIN_H, WIN_W, CV_8UC3);
        Mat canvas_b = Mat::zeros(WIN_H, WIN_W, CV_8UC3);

        // 4. UDP ���� (�񵿱�)
        vector<Detection> dets;
        int len = recv(sock, buf, sizeof(buf)-1, 0);
        if (len > 0) {
            buf[len] = '\0';
            try {
                auto j = json::parse(buf);
                if (j.contains("detections")) {
                    for (auto& d : j["detections"]) {
                        dets.push_back({d[0].get<int>(), Rect((int)d[2].get<float>(), (int)d[3].get<float>(), 
                                                            (int)d[4].get<float>(), (int)d[5].get<float>())});
                    }
                }
            } catch (...) {}
        }

        // 5. ǻ�� ������
        if (doProcessSimple(laser, &scan)) {
            vector<Point3f> pts3d;
            for (int i = 0; i < scan.npoints; i++) {
                float dist = scan.points[i].range * 1000.0f;
                if (dist < 150.0f) continue;
                float a = scan.points[i].angle;
                pts3d.push_back(Point3f(dist * sin(a), 0, dist * cos(a)));
            }
            renderFusion(canvas_f, pts3d, mtx_f, dist_f, rvec, tvec, dets, 0, Scalar(0, 255, 0), fps);
            renderFusion(canvas_b, pts3d, mtx_b, dist_b, rvec, tvec, dets, 1, Scalar(0, 0, 255), fps);
        }

        imshow("FRONT (Cam 0)", canvas_f);
        imshow("BACK (Cam 1)", canvas_b);
        if (waitKey(1) == 27) break;
    }

    free(scan.points); turnOff(laser); lidarDestroy(&laser);
    close(sock);
    return 0;
}