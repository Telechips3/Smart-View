#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/stat.h>

using namespace cv;
using namespace std;

void makeDir(string path) {
    mkdir(path.c_str(), 0777);
}

int main() {
    // 1. 카메라 설정 - 대역폭 절약을 위해 MJPEG 코덱 사용 설정
    VideoCapture capFront(0, CAP_V4L2);
    // video2에서 타임아웃이 나면 보통 video4나 video1 등 다른 인덱스일 수 있음
    VideoCapture capRear(2, CAP_V4L2); 

    if (!capFront.isOpened()) { cout << "Front Cam (0) 실패" << endl; }
    if (!capRear.isOpened()) { cout << "Rear Cam (2) 실패" << endl; }

    // 대역폭 문제 해결을 위한 핵심 설정 (MJPEG)
    capFront.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capRear.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    
    capFront.set(CAP_PROP_FRAME_WIDTH, 640);
    capFront.set(CAP_PROP_FRAME_HEIGHT, 480);
    capRear.set(CAP_PROP_FRAME_WIDTH, 640);
    capRear.set(CAP_PROP_FRAME_HEIGHT, 480);

    makeDir("calib_images");
    makeDir("calib_images/front");
    makeDir("calib_images/rear");

    int frontCount = 0;
    int rearCount = 0;

    cout << "VNC에서 실행 중... 화면이 뜰 때까지 잠시 기다려주세요." << endl;

    while (true) {
        Mat frameF, frameR;
        
        // 프레임을 읽어오되, 하나라도 실패하면 루프를 유지하며 재시도
        bool bF = capFront.read(frameF);
        bool bR = capRear.read(frameR);

        if (bF) {
            Mat dispF = frameF.clone();
            string txtF = "FRONT: " + to_string(frontCount);
            putText(dispF, txtF, Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
            imshow("Front Camera", dispF);
        }

        if (bR) {
            Mat dispR = frameR.clone();
            string txtR = "REAR: " + to_string(rearCount);
            putText(dispR, txtR, Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
            imshow("Rear Camera", dispR);
        }

        int key = waitKey(10); // 10ms 대기
        
        if (key == 'q') break;
        else if (key == 'f' && bF) {
            imwrite("calib_images/front/f_" + to_string(frontCount++) + ".jpg", frameF);
            cout << "Saved Front " << frontCount << endl;
        } 
        else if (key == 'r' && bR) {
            imwrite("calib_images/rear/r_" + to_string(rearCount++) + ".jpg", frameR);
            cout << "Saved Rear " << rearCount << endl;
        }
    }

    return 0;
}