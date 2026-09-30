"""YOLO 결과 저장과 선택적 로컬 추론을 담당하는 모델입니다.

SP의 통신 방식은 아직 구현하지 않습니다. 향후 수신 코드는 결과를
기존 웹 형식으로 변환한 뒤 VisionState.update()에 전달합니다.
"""
# 1. 결과 복사·로그·동시 접근 보호 모듈
from copy import deepcopy
import logging
from threading import Lock


# 웹에 전달할 객체 인식 결과 및 영상 저장
class VisionState:
    """웹 API가 읽을 최신 검출 목록과 JPEG 영상 한 쌍을 보관합니다."""

    # 검출 결과와 프레임의 초기 상태 및 잠금 준비
    def __init__(self):
        self._lock = Lock()
        self._detections = []
        self._jpeg = None

    # 로컬 추론 또는 향후 SP 수신 결과 저장
    def update(self, detections, jpeg):
        """검출 목록과 JPEG bytes를 함께 갱신합니다. 영상이 없으면 None을 전달합니다."""
        # SP 수신 처리와 HTTP 조회가 동시에 실행돼도 결과 한 쌍을 유지합니다.
        # 목록을 복사하여 호출자가 원본을 수정해도 저장된 결과가 변하지 않게 합니다.
        with self._lock:
            self._detections = deepcopy(detections)
            self._jpeg = jpeg

    # 웹 API에 최신 결과 복사본 전달
    def snapshot(self):
        """호출자가 저장된 검출 목록을 직접 수정하지 않도록 복사본을 반환합니다."""
        with self._lock:
            return deepcopy(self._detections), self._jpeg


# 2. SP 연동 전 개발용 로컬 YOLO 실행
def run_yolo_loop(state, stop):
    """SP 연동 전 선택적으로 사용하는 로컬 웹캠·YOLO 데모입니다."""
    # 기본 웹 실행에서는 OpenCV와 Ultralytics를 불러오지 않습니다.
    import cv2
    # YOLO 모델 로드: 실패하면 로그를 남기고 종료
    try:
        from ultralytics import YOLO
        model = YOLO('yolov8n.pt')
    except Exception:
        logging.exception("YOLO initialization failed")
        return

    # 개발용 웹캠 연결: SP 수신 연결에서는 실행하지 않음
    cap = cv2.VideoCapture(0)

    try:
        # 서버 종료 신호가 올 때까지 프레임 수집 및 객체 인식 반복
        while not stop.is_set():
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and model:
                    # 객체 추론 및 검출 목록 생성
                    results = model(frame, verbose=False, conf=0.3)
                    detections = []
                    for r in results:
                        for box in r.boxes:
                            x1, y1, x2, y2 = box.xyxy[0].tolist()
                            conf = float(box.conf[0])
                            cls_id = int(box.cls[0])
                            cls_name = model.names[cls_id]
                            # 웹에서 사용할 클래스·신뢰도·박스 좌표 정리
                            detections.append({
                                "class_id": cls_id,
                                "class_name": cls_name,
                                "confidence": round(conf, 2),
                                "bbox": {"x1": round(x1, 1), "y1": round(y1, 1), "x2": round(x2, 1), "y2": round(y2, 1)}
                            })
                            # 영상에 검출 객체의 테두리 표시
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 136), 2)

                    # 웹 전송용 JPEG 변환 및 최신 결과 저장
                    encoded, jpeg = cv2.imencode('.jpg', frame)
                    if encoded:
                        # 로컬 추론 결과도 SP 수신 결과와 같은 저장 경로를 사용합니다.
                        state.update(detections, jpeg.tobytes())

            # 종료 신호를 확인하며 다음 프레임 처리까지 대기
            stop.wait(0.04)
    finally:
        # 서버 종료 또는 추론 오류 시 카메라 자원 반환
        cap.release()
