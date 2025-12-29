import cv2
import time
import socket
import json
import multiprocessing as mp
import os
import numpy as np
from ultralytics import YOLO
from boxmot import OcSort

# --- 설정 ---
BOARD_B_IP = "192.168.10.11"
UDP_PORT = 5005
FILTER_CLASSES = [0, 1, 2, 3]

def camera_worker(cam_index, cam_id, core_id, queue):
    # 프로세스를 특정 코어에 고정 (Core 0, Core 1)
    os.sched_setaffinity(0, {core_id})
    
    # OpenVINO 모델 로드
    try:
        model = YOLO('yolo11n_openvino_model', task='detect')
    except:
        print(f"Cam {cam_id}: OpenVINO 모델을 찾을 수 없어 .pt로 실행합니다. (느려짐 주의)")
        model = YOLO('yolo11n.pt')

    # 사용자가 요청한 OcSort 설정
    tracker = OcSort(
        det_thresh=0.45, 
        asso_threshold=0.3, 
        use_byte=False
    )

    cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    prev_time = time.perf_counter()

    while True:
        ret, frame = cap.read()
        if not ret: continue

        # OpenVINO 추론
        results = model.predict(frame, imgsz=320, verbose=False, device='cpu')[0]
        dets = results.boxes.data.cpu().numpy()
        
        current_dets = []
        # 드로잉용 원본 프레임
        draw_frame = frame.copy()

        # OcSort 트래킹
        if len(dets) > 0:
            tracked_objects = tracker.update(dets, frame)
            for t in tracked_objects:
                x1, y1, x2, y2, track_id, conf, cls = t[:7]
                if int(cls) in FILTER_CLASSES:
                    current_dets.append([
                        cam_id, int(cls), round(float(x1),1), round(float(y1),1),
                        round(float(x2-x1),1), round(float(y2-y1),1),
                        round(float(conf),2), int(track_id)
                    ])
                    # 시각화
                    color = (0, 255, 0) if cam_id == 0 else (255, 0, 0)
                    cv2.rectangle(draw_frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    cv2.putText(draw_frame, f"ID:{int(track_id)}", (int(x1), int(y1)-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        curr_time = time.perf_counter()
        fps = 1.0 / (curr_time - prev_time)
        prev_time = curr_time

        if queue.full():
            try: queue.get_nowait()
            except: pass
        queue.put({"frame": draw_frame, "dets": current_dets, "fps": fps})

if __name__ == '__main__':
    mp.set_start_method('spawn', force=True)
    
    q0 = mp.Queue(maxsize=1)
    q1 = mp.Queue(maxsize=1)

    # Core 0과 Core 1에 각각 할당
    p0 = mp.Process(target=camera_worker, args=(0, 0, 0, q0))
    p1 = mp.Process(target=camera_worker, args=(2, 1, 1, q1))
    
    p0.start()
    p1.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_res = {0: None, 1: None}
    
    print("보드 A: OcSort + OpenVINO 멀티프로세싱 작동 중...")

    try:
        while True:
            # 데이터 수집
            while not q0.empty(): last_res[0] = q0.get()
            while not q1.empty(): last_res[1] = q1.get()

            if last_res[0] is None or last_res[1] is None:
                time.sleep(0.01)
                continue

            # 1. UDP 전송
            packet = {"f_id": time.time(), "detections": last_res[0]["dets"] + last_res[1]["dets"]}
            sock.sendto(json.dumps(packet).encode(), (BOARD_B_IP, UDP_PORT))

            # 2. 화면 표시
            combined_view = np.hstack((last_res[0]["frame"], last_res[1]["frame"]))
            combined_view = cv2.resize(combined_view, (800, 300)) 

            avg_fps = (last_res[0]["fps"] + last_res[1]["fps"]) / 2
            cv2.putText(combined_view, f"OcSort+OpenVINO FPS: {avg_fps:.1f}", (20, 280), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow('RPi 5 Performance Monitoring', combined_view)
            if cv2.waitKey(1) == 27: break

    except KeyboardInterrupt:
        pass
    finally:
        p0.terminate()
        p1.terminate()
        cv2.destroyAllWindows()