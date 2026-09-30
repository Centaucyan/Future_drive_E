"""SP에서 수신한 최신 검출 결과와 JPEG 영상을 웹 API와 공유합니다."""
# 결과 복사 및 동시 접근 보호 모듈
from copy import deepcopy
from threading import Lock


# 웹에 전달할 객체 인식 결과 및 영상 저장
class VisionState:
    """웹 API가 읽을 최신 검출 목록과 JPEG 영상 한 쌍을 보관합니다."""

    # 검출 결과와 프레임의 초기 상태 및 잠금 준비
    def __init__(self):
        self._lock = Lock()
        self._detections = []
        self._jpeg = None

    # SP에서 수신한 최신 검출 결과와 영상 저장
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
