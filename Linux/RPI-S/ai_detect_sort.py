import cv2
import threading
import numpy as np
import time
import socket
import json
import base64
from hailo_platform import (VDevice, HEF, ConfigureParams, InferVStreams,
                            InputVStreamParams, OutputVStreamParams,
                            HailoStreamInterface, FormatType)

# ==========================================
# 1. 설정 및 상수 (Configuration)
# ==========================================
MODEL_PATH = '/usr/share/hailo-models/yolov8s_h8.hef'  # Hailo 가속기용 모델 파일 경로
CAM_SOURCES = [0, 2]                                  # 연결된 카메라 인덱스 리스트
TARGET_IP = "192.168.1.13"                            # 데이터를 받을 목적지 IP
UDP_PORT = 5005                                       # UDP 포트 번호
JPEG_QUALITY = 40                                     # 이미지 압축 품질 (UDP 용량 제한 때문)

# [카메라 예상 거리 계산용] 거리 = (초점거리 * 실제높이) / 이미지상 픽셀높이
FOCAL_LENGTHS = {0: 352.46, 2: 352.46}                # 카메라별 초점 거리
REAL_HEIGHTS = {                                      # 클래스별 실제 평균 높이 (미터 단위)
    0: 1.7,   # Person
    1: 1.1,   # Bicycle
    2: 1.5,   # Car
    3: 0.8,   # Motorcycle
    5: 3.3,   # Bus
    7: 3.5    # Truck
}

# [필터링 및 라벨링]
CLASS_NAMES = {0: "Person", 1: "Bicycle", 2: "Car", 3: "Motorcycle", 5: "Bus", 7: "Truck"}
CLASS_THRESHOLDS = {0: 0.50, 1: 0.50, 2: 0.35, 3: 0.35, 5: 0.35, 7: 0.35} # 클래스별 신뢰도 임계값
FILTER_CLASSES = [0, 1, 2, 3, 5, 7]                  # 추론 결과에서 추출할 클래스 목록


# ==========================================
# 2. 유틸리티 함수 (Utility Functions)
# ==========================================

def draw_detections(frame, detections):
    """
    프레임 위에 객체의 경계 상자(Bounding Box)와 정보를 덧씌움.
    """
    viz_frame = frame.copy()
    for det in detections:
        x, y, w, h = int(det['x']), int(det['y']), int(det['w']), int(det['h'])
        label_name = CLASS_NAMES.get(det['id'], f"ID:{det['id']}")
        
        # 박스 및 텍스트 시각화
        color = (0, 255, 0) # 초록색
        cv2.rectangle(viz_frame, (x, y), (x + w, y + h), color, 2)
        
        label_text = f"{label_name} {det['d']}m"
        (t_w, t_h), baseline = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(viz_frame, (x, y - t_h - baseline), (x + t_w, y), color, -1)
        cv2.putText(viz_frame, label_text, (x, y - baseline), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    return viz_frame

def encode_image_to_base64(frame, quality=40):
    """
    OpenCV 이미지를 JPEG로 압축한 후 Base64 문자열로 변환 (네트워크 전송용)
    """
    try:
        success, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not success:
            return None
        return base64.b64encode(buffer).decode('utf-8')
    except Exception as e:
        print(f"Image encoding error: {e}")
        return None


# ==========================================
# 3. 객체 탐지 클래스 (Core Classes)
# ==========================================

class YOLOv8PostProcess:
    """
    Hailo 가속기에서 나온 Raw Output을 해석하여 객체 정보로 변환
    """
    def __init__(self, thresholds):
        self.thresholds = thresholds

    def process(self, infer_results, img_w, img_h, cam_idx):
        detections = []
        f_length = FOCAL_LENGTHS.get(cam_idx, 352.46)

        # Hailo 출력 데이터 구조를 순회 (YOLOv8 구조 기준)
        for _, v in infer_results.items():
            raw_data = v[0] if isinstance(v, list) else v
            if not isinstance(raw_data, (list, np.ndarray)) or len(raw_data) < 80:
                continue

            for class_idx, class_dets in enumerate(raw_data[:80]):
                if class_idx not in FILTER_CLASSES: 
                    continue
                
                score_thresh = self.thresholds.get(class_idx, 0.5)

                for det in class_dets:
                    if len(det) >= 5 and det[4] >= score_thresh:
                        # 좌표 정규화 해제 (0.0~1.0 -> 픽셀 단위)
                        ymin, xmin, ymax, xmax = det[:4]
                            # 좌표의 최소값(0.0)과 최대값(1.0) 제한
                        xmin_c = max(0.0, min(1.0, float(xmin)))
                        ymin_c = max(0.0, min(1.0, float(ymin)))
                        xmax_c = max(0.0, min(1.0, float(xmax)))
                        ymax_c = max(0.0, min(1.0, float(ymax)))
                            # 픽셀 단위 좌표 계산
                        px, py = xmin_c * img_w, ymin_c * img_h
                        pw, ph = (xmax_c - xmin_c) * img_w, (ymax_c - ymin_c) * img_h

                        # 거리 계산 (공식: 실제높이 * 초점거리 / 이미지상 높이)
                        real_h = REAL_HEIGHTS.get(class_idx, 1.0)
                        distance = (f_length * real_h) / ph if ph > 0 else 0
                        
                        # 버스/트럭은 자동차(2) ID로 통합 처리 후 시각화
                        effective_cid = 2 if class_idx in [5, 7] else class_idx

                        detections.append({
                            "id": int(effective_cid),
                            "x": round(px, 1), "y": round(py, 1),
                            "w": round(pw, 1), "h": round(ph, 1),
                            "d": round(float(distance), 2)
                        })
        return detections

class CameraStream:
    """
    카메라 영상을 별도의 스레드에서 끊임없이 읽어오는 클래스 (지연 방지용)
    """
    def __init__(self, cam_id):
        self.cam_id = cam_id
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.frame = None
        self.is_connected = False
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
                self.is_connected = True
            else:
                self.is_connected = False
                self.cap.release()
                time.sleep(2) # 재연결 시도 대기
                self.cap = cv2.VideoCapture(self.cam_id)
            time.sleep(0.01)

# ==========================================
# 4. Main Loop
# ==========================================

def main():
    # UDP 소켓 초기화
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65535) # 송신 버퍼 최대화
    except: pass

    # Hailo 장치 준비
    vdevice = VDevice()
    hef = HEF(MODEL_PATH)
    
    # 모델의 입출력 설정 로드
    configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_group = vdevice.configure(hef, configure_params)[0]
    
    input_info = hef.get_input_vstream_infos()[0]
    model_w, model_h = input_info.shape[1], input_info.shape[0]

    # 카메라 스트림 시작
    streams = [CameraStream(src).start() for src in CAM_SOURCES]
    post_proc = YOLOv8PostProcess(CLASS_THRESHOLDS)

    # 입출력 스트림 파라미터 생성
    in_params = InputVStreamParams.make_from_network_group(network_group, format_type=FormatType.UINT8)
    out_params = OutputVStreamParams.make_from_network_group(network_group, format_type=FormatType.FLOAT32)

    print(f"Smart-View: Streaming visualized images to {TARGET_IP}:{UDP_PORT}")

    # 가속기 활성화 및 추론 루프
    with network_group.activate(), InferVStreams(network_group, in_params, out_params) as pipeline:
        while True:
            for cam_idx, stream in enumerate(streams):
                # 전송할 기본 데이터 구조 (Packet)
                packet = {
                    "cam_id": cam_idx,
                    "timestamp": int(time.time() * 1000000), # 마이크로초 단위 타임스탬프
                    "status": 1 if stream.is_connected else 0,
                    "detections": [],
                    "image_base64": None
                }

                if stream.is_connected and stream.frame is not None:
                    # 1. 이미지 전처리 (모델 크기에 맞춤)
                    frame = stream.frame
                    resized = cv2.resize(frame, (model_w, model_h))
                    
                    # 2. Hailo 가속기 추론 실행
                    results = pipeline.infer({input_info.name: np.expand_dims(resized, axis=0)})
                    
                    # 3. 후처리 (박스 좌표 및 거리 계산)
                    detections = post_proc.process(results, frame.shape[1], frame.shape[0], cam_idx)
                    packet["detections"] = detections

                    # 4. 시각화 및 인코딩
                    viz_frame = draw_detections(frame, detections)
                    img_str = encode_image_to_base64(viz_frame, quality=JPEG_QUALITY)
                    packet["image_base64"] = img_str

                # 5. UDP 전송
                try:
                    json_data = json.dumps(packet).encode()
                    # UDP 패킷 크기 제한(약 65KB) 확인
                    if len(json_data) > 65000:
                        print("Warning: Packet too large ({len(json_data)} bytes). Reduce JPEG_QUALITY.")
                        packet["image_base64"] = None
                        json_data = json.dumps(packet).encode()
                    
                    sock.sendto(json_data, (TARGET_IP, UDP_PORT))
                except Exception as e:
                    print(f"Socket error: {e}")

            time.sleep(0.01) # CPU 부하 방지용 미세 대기

if __name__ == "__main__":
    main()