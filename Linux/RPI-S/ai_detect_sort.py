import cv2
import threading
import numpy as np
import time
import socket
import json
from boxmot import OcSort
from hailo_platform import (VDevice, HEF, ConfigureParams, InferVStreams, 
                            InputVStreamParams, OutputVStreamParams, 
                            HailoStreamInterface, FormatType)

# --- [1. 기본 설정 및 통신 정보] ---
MODEL_PATH = '/usr/share/hailo-models/yolov8s_h8.hef' # Hailo 전용 모델 파일 경로
CAM_SOURCES = [0, 2]         # 연결된 카메라 번호 (USB 포트 위치에 따라 다름)
BOARD_B_IP = "192.168.10.11" # 데이터를 받을 상대방 보드의 IP 주소
UDP_PORT = 5005              # 데이터를 보낼 네트워크 포트 번호

SCORE_THRESHOLD = 0.50       # AI 탐지 신뢰도 (0.50 미만은 무시)
# 보드 B로 전송할 대상 클래스 번호 (0:사람, 1:자전거, 2:자동차, 3:오토바이)
FILTER_CLASSES = [0, 1, 2, 3] 
ALLOWED_NAMES = {0: 'Person', 1: 'Bicycle', 2: 'Car', 3: 'Motorcycle'}

# --- [2. YOLOv8 후처리 클래스] ---
# AI 모델이 내뱉는 원시 데이터(Raw Data)를 좌표와 점수 형태로 가공하는 역할
class YOLOv8PostProcess:
    def __init__(self, score_thresh):
        self.score_thresh = score_thresh

    def process(self, infer_results, img_w, img_h):
        detections = []
        for k, v in infer_results.items():
            raw_data = v[0] if isinstance(v, list) else v
            # YOLOv8은 보통 80개의 클래스 정보를 출력함
            if isinstance(raw_data, (list, np.ndarray)) and len(raw_data) == 80:
                for class_idx, class_dets in enumerate(raw_data):
                    if class_idx not in FILTER_CLASSES: continue # 필터링 대상만 처리
                    if class_dets is None or len(class_dets) == 0: continue
                    
                    for det in class_dets:
                        if len(det) >= 5:
                            score = float(det[4]) # 4번 인덱스: 신뢰도 점수
                            if score >= self.score_thresh:
                                ymin, xmin, ymax, xmax = det[:4] # 0~3번: 박스 좌표(비율값)
                                # 0~1 사이의 비율값을 실제 영상 픽셀 좌표(px)로 변환
                                detections.append([
                                    float(xmin * img_w), float(ymin * img_h), 
                                    float(xmax * img_w), float(ymax * img_h), 
                                    float(score), float(class_idx)
                                ])
        if not detections:
            return np.empty((0, 6), dtype=np.float32)
        return np.array(detections, dtype=np.float32)

# --- [3. 추적 결과 시각화 및 데이터 추출 함수] ---
def draw_tracks(frame, tracks, cam_id):
    """ 화면에 추적 상자를 그리고, 보드 B로 보낼 데이터를 리스트에 정리 """
    current_frame_dets = []
    for trk in tracks:
        # OcSort 출력 구조: [x1, y1, x2, y2, track_id, confidence, class_id]
        if len(trk) < 7: continue 
        
        x1, y1, x2, y2 = trk[:4]
        tid = int(trk[4])   # 추적 번호 (아이디)
        conf = float(trk[5]) # 점수
        cid = int(trk[6])   # 클래스 번호
        
        if cid in FILTER_CLASSES:
            w, h = x2 - x1, y2 - y1 # 가로, 세로 크기 계산
            
            # 보드 B 전송용 리스트 생성 (소수점 첫째자리까지 반올림)
            current_frame_dets.append([
                cam_id, cid, round(float(x1), 1), round(float(y1), 1),
                round(float(w), 1), round(float(h), 1),
                round(float(conf), 2), tid
            ])

            # 화면 시각화 (사각형 및 라벨 그리기)
            name = ALLOWED_NAMES.get(cid, "Object")
            color = (0, 255, 0) if cam_id == 0 else (255, 0, 0) # 카메라별 색상 구분
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            label = f"{name} #{tid} ({conf:.2f})"
            cv2.putText(frame, label, (int(x1), int(y1)-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
    return current_frame_dets

# --- [4. 멀티스레딩 카메라 입력 클래스] ---
# 카메라 읽기와 메인 로직을 분리하여 영상 끊김(딜레이)을 방지
class CameraStream:
    def __init__(self, cam_id):
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 카메라 입력 가로 640
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) # 카메라 입력 세로 480
        self.frame = None
        self.stopped = False

    def start(self):
        # 별도의 스레드에서 카메라 영상을 계속 업데이트함
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret: self.frame = frame
            else: time.sleep(0.01)

    def stop(self):
        self.stopped = True
        self.cap.release()

# --- [5. 메인 실행 루프] ---
def main():
    # 1. 네트워크 통신 소켓(UDP) 준비
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 2. Hailo-8 하드웨어 가속기 및 AI 모델 로드
    hef = HEF(MODEL_PATH)
    target = VDevice() # Hailo 장치 연결
    params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_group = target.configure(hef, params)[0]
    input_info = hef.get_input_vstream_infos()[0]
    model_w, model_h = input_info.shape[1], input_info.shape[0] # 모델 입력 크기 (640x640)

    # 3. 각 카메라용 추적기(OcSort) 및 카메라 스트림 객체 생성
    tracker0 = OcSort(det_thresh=SCORE_THRESHOLD, asso_threshold=0.3, use_byte=False)
    tracker2 = OcSort(det_thresh=SCORE_THRESHOLD, asso_threshold=0.3, use_byte=False)
    stream0 = CameraStream(CAM_SOURCES[0]).start()
    stream2 = CameraStream(CAM_SOURCES[1]).start()
    post_proc = YOLOv8PostProcess(SCORE_THRESHOLD)
    
    prev_time = time.time()
    print(f"보드 A(Hailo) 작동 중... 대상 IP: {BOARD_B_IP}:{UDP_PORT}")

    try:
        # Hailo-8 파이프라인 활성화 (추론 준비)
        with network_group.activate(), InferVStreams(network_group, 
             InputVStreamParams.make_from_network_group(network_group, format_type=FormatType.UINT8),
             OutputVStreamParams.make_from_network_group(network_group, format_type=FormatType.FLOAT32)) as pipeline:
            
            while True:
                f0, f2 = stream0.frame, stream2.frame
                if f0 is None or f2 is None: continue # 두 카메라 모두 영상이 들어올 때까지 대기

                all_detections = [] # 보드 B로 보낼 전체 통합 리스트
                processed_views = [] # 화면 표시용 리스트

                # 듀얼 카메라 루프 (i=0: 카메라0, i=1: 카메라2)
                for i, (frame, tracker) in enumerate(zip([f0.copy(), f2.copy()], [tracker0, tracker2])):
                    # (1) AI 추론: 모델 크기(640x640)로 리사이즈 후 Hailo에 입력
                    resized = cv2.resize(frame, (model_w, model_h))
                    results = pipeline.infer({input_info.name: np.expand_dims(resized, axis=0)})
                    
                    # (2) 후처리: 추론 결과를 픽셀 좌표로 변환
                    dets = post_proc.process(results, frame.shape[1], frame.shape[0])
                    
                    # (3) 추적(Tracking): 이전 프레임과 객체를 매칭하여 ID 부여
                    tracks = tracker.update(dets, frame)
                    
                    # (4) 결과 정리: 화면에 그리고 전송용 리스트(cam_id 포함)에 추가
                    cam_dets = draw_tracks(frame, tracks, cam_id=i)
                    all_detections.extend(cam_dets)
                    
                    # 모니터링 화면용 크기 조절 (480x360)
                    processed_views.append(cv2.resize(frame, (480, 360)))

                # 4. 네트워크 전송: 최종 JSON 패킷 구성 및 보드 B로 쏘기
                packet = {
                    "f_id": time.time(),         # 프레임 타임스탬프 (ID 역할)
                    "detections": all_detections # 통합 탐지 데이터
                }
                sock.sendto(json.dumps(packet).encode(), (BOARD_B_IP, UDP_PORT))

                # 5. 성능 표시 및 화면 합치기
                combined = np.hstack(processed_views) # 가로로 두 영상 합치기
                fps = 1 / (time.time() - prev_time)
                prev_time = time.time()
                cv2.putText(combined, f"FPS: {fps:.1f}", (20, 40), 
                            cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 255), 2)
                
                # 최종 결과창 출력
                cv2.imshow("Hailo-8 Dual Cam + UDP Send", combined)
                if cv2.waitKey(1) == ord('q'): break # 'q' 누르면 종료
    finally:
        # 종료 시 자원 해제
        stream0.stop(); stream2.stop()
        sock.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()