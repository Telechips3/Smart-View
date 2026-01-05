import cv2
import threading
import numpy as np
import time
import socket
import json
from hailo_platform import (VDevice, HEF, ConfigureParams, InferVStreams,
                            InputVStreamParams, OutputVStreamParams,
                            HailoStreamInterface, FormatType)

# --- [1. 설정 값 공간] ---
MODEL_PATH = '/usr/share/hailo-models/yolov8s_h8.hef' # Hailo 전용 모델 파일 경로
CAM_SOURCES = [0, 2]                                 # 사용할 카메라 인덱스 (예: /dev/video0, /dev/video2)
TARGET_IP = "192.168.1.13"                           # 데이터를 받을 서버 IP
UDP_PORT = 5005                                      # 데이터를 받을 UDP 포트 번호

# 클래스별 신뢰도 임계값 (0: 사람, 1: 자전거, 2: 자동차, 3: 오토바이, 5: 버스, 7: 트럭)
CLASS_THRESHOLDS = {0: 0.50, 1: 0.50, 2: 0.35, 3: 0.35, 5: 0.35, 7: 0.35}
FILTER_CLASSES = [0, 1, 2, 3, 5, 7]                  # 검출할 클래스 번호 목록

class YOLOv8PostProcess:
    """Hailo 모델의 출력 데이터를 해석하여 좌표와 클래스를 추출하는 클래스"""
    def __init__(self, thresholds):
        self.thresholds = thresholds

    def process(self, infer_results, img_w, img_h):
        detections = []
        # 모델의 출력 결과(Dictionary 형태)를 반복하며 처리
        for k, v in infer_results.items():
            raw_data = v[0] if isinstance(v, list) else v
            if isinstance(raw_data, (list, np.ndarray)) and len(raw_data) >= 80:
                # 최대 80개의 클래스 결과 확인
                for class_idx, class_dets in enumerate(raw_data[:80]):
                    if class_idx not in FILTER_CLASSES: continue
                    
                    score_thresh = self.thresholds.get(class_idx, 0.5)
                    for det in class_dets:
                        # 신뢰도(Confidence Score)가 기준치 이상인 경우만 추출
                        if len(det) >= 5 and det[4] >= score_thresh:
                            ymin, xmin, ymax, xmax = det[:4]

                            # [좌표 예외 처리 및 정규화]
                            # 1. 0.0 ~ 1.0 범위를 벗어나지 않도록 클리핑 (화면 밖 검출 방지)
                            xmin_c = max(0.0, min(1.0, float(xmin)))
                            ymin_c = max(0.0, min(1.0, float(ymin)))
                            xmax_c = max(0.0, min(1.0, float(xmax)))
                            ymax_c = max(0.0, min(1.0, float(ymax)))

                            # 2. 비율 기반 좌표를 실제 이미지 픽셀 크기로 변환
                            px = xmin_c * img_w
                            py = ymin_c * img_h
                            pw = (xmax_c - xmin_c) * img_w
                            ph = (ymax_c - ymin_c) * img_h

                            # 특정 클래스(5: 버스, 7: 트럭)는 2(자동차)로 통합 관리
                            effective_cid = 2 if class_idx in [5, 7] else class_idx

                            detections.append({
                                "id": int(effective_cid),
                                "x": round(px, 1),
                                "y": round(py, 1),
                                "w": round(pw, 1),
                                "h": round(ph, 1)
                            })
        return detections

class CameraStream:
    """카메라 영상을 별도의 쓰레드에서 캡처하여 끊김 없는 스트리밍을 제공하는 클래스"""
    def __init__(self, cam_id):
        self.cam_id = cam_id
        self.cap = cv2.VideoCapture(cam_id)
        # 해상도 설정 (640x480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.frame = None
        self.is_connected = False
        self.stopped = False

    def start(self):
        # 데몬 쓰레드로 시작 (메인 프로그램 종료 시 자동 종료)
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
                self.is_connected = True
            else:
                # 연결 끊김 시 재시도 로직
                self.is_connected = False
                self.cap.release()
                time.sleep(2)
                self.cap = cv2.VideoCapture(self.cam_id)
            time.sleep(0.01) # CPU 점유율 방지용 미세 대기

def main():
    # 1. 전송용 UDP 소켓 및 Hailo 가속기 장치 초기화
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    hef = HEF(MODEL_PATH)
    target = VDevice() # Hailo-8 장치 할당
    
    # 2. 모델 구성 및 네트워크 그룹 설정
    params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_group = target.configure(hef, params)[0]
    
    # 모델의 입력 형태(가로, 세로) 확인 (보통 640x640)
    input_info = hef.get_input_vstream_infos()[0]
    model_w, model_h = input_info.shape[1], input_info.shape[0]

    # 3. 카메라 스트림 및 후처리 객체 생성
    streams = [CameraStream(src).start() for src in CAM_SOURCES]
    post_proc = YOLOv8PostProcess(CLASS_THRESHOLDS)

    # 4. 입출력 가상 스트림 설정 (입력: UINT8 이미지, 출력: FLOAT32 예측값)
    input_params = InputVStreamParams.make_from_network_group(network_group, format_type=FormatType.UINT8)
    output_params = OutputVStreamParams.make_from_network_group(network_group, format_type=FormatType.FLOAT32)

    print(f"Streaming JSON to {TARGET_IP}:{UDP_PORT}")

    # 5. 메인 추론 루프
    with network_group.activate(), InferVStreams(network_group, input_params, output_params) as pipeline:
        while True:
            for cam_id, stream in enumerate(streams):
                # 전송할 데이터 기본 구조 (JSON 패킷)
                packet = {
                    "cam_id": cam_id,
                    "timestamp": time.time(),
                    "status": 1 if stream.is_connected else 0, # 1: 정상, 0: 연결 유실
                    "detections": []
                }

                if stream.is_connected and stream.frame is not None:
                    frame = stream.frame
                    # 모델 크기에 맞춰 이미지 리사이징
                    resized = cv2.resize(frame, (model_w, model_h))
                    
                    # Hailo 가속기로 추론(Inference) 실행
                    results = pipeline.infer({input_info.name: np.expand_dims(resized, axis=0)})
                    
                    # 결과를 해석(Post-process)하여 좌표 목록 획득
                    packet["detections"] = post_proc.process(results, frame.shape[1], frame.shape[0])

                # 최종 데이터를 JSON으로 직렬화하여 UDP 전송
                sock.sendto(json.dumps(packet).encode(), (TARGET_IP, UDP_PORT))

            time.sleep(0.01) # 전체 루프 속도 조절

if __name__ == "__main__":
    main()
