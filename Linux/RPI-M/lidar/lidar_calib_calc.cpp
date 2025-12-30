// notion이랑 다른 코드 --- 이거는 data 저장해사 그거 기반으로 값 바꿈. notion은 초기값으로 줘버렸음.
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <filesystem>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

// 분석 함수 정의: 매개변수에 string saveName을 추가했습니다.
void runCalibration(string imageDir, string saveName, Size boardSize, float squareSize) {
    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;

    // 3D 좌표 생성
    vector<Point3f> obj;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            obj.push_back(Point3f(j * squareSize, i * squareSize, 0.0f));
        }
    }

    cout << "\n[진행 중] " << imageDir << " 분석..." << endl;

    Size imgSize;
    int successCount = 0;

    if (!fs::exists(imageDir)) {
        cout << "[오류] 폴더가 존재하지 않습니다: " << imageDir << endl;
        return;
    }

    for (const auto& entry : fs::directory_iterator(imageDir)) {
        Mat img = imread(entry.path().string());
        if (img.empty()) continue;

        imgSize = img.size();
        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);

        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, boardSize, corners, 
                        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            imagePoints.push_back(corners);
            objectPoints.push_back(obj);
            successCount++;
        }
    }

    if (successCount < 10) {
        cout << "[실패] " << imageDir << ": 성공한 이미지가 너무 적음 (" << successCount << "장)" << endl;
        return;
    }

    Mat cameraMatrix, distCoeffs, rvecs, tvecs;
    double rms = calibrateCamera(objectPoints, imagePoints, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    // --- [파일 저장 로직] ---
    if (!fs::exists("calib_data")) fs::create_directory("calib_data"); 

    string filePath = "calib_data/" + saveName + ".yaml";
    FileStorage fs_save(filePath, FileStorage::WRITE);
    if(fs_save.isOpened()) {
        fs_save << "camera_matrix" << cameraMatrix;
        fs_save << "distortion_coefficients" << distCoeffs;
        fs_save.release();
        cout << "--------------------------------------------" << endl;
        cout << "성공! 파일 저장 완료: " << filePath << endl;
        cout << "RMS Error: " << rms << endl;
        cout << "--------------------------------------------" << endl;
    } else {
        cout << "[오류] 파일 저장 실패!" << endl;
    }
}

int main() {
    Size boardSize(10, 7);
    float squareSize = 20.0f;

    cout << "=== 전후방 카메라 캘리브레이션 분석 시작 ===" << endl;

    // 인자 4개를 정상적으로 전달합니다.
    runCalibration("calib_images/front", "front_params", boardSize, squareSize);
    runCalibration("calib_images/rear", "back_params", boardSize, squareSize);

    // lidar_bbox 실행을 위해 필요한 외적 파라미터 기본 파일도 생성해 둡니다.
    FileStorage fs_ex("calib_data/lidar_cam_extrinsic.yaml", FileStorage::WRITE);
    fs_ex << "rvec" << Mat::zeros(3, 1, CV_64F);
    fs_ex << "tvec" << Mat::zeros(3, 1, CV_64F);
    fs_ex.release();

    cout << "\n분석 및 파일 생성이 완료되었습니다." << endl;
    return 0;
}