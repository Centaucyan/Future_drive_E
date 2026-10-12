"""브리지 JPEG 프레임을 가져와 웹 서버에서 YOLO 처리한다."""

from __future__ import annotations

import ipaddress
import os
import threading
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlsplit, urlunsplit
from urllib.request import Request, urlopen

import cv2
import numpy as np
from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL_PATH = PROJECT_ROOT / "ros2_ws/models/futuredrive_yolo26n_best.pt"
MAX_FRAME_BYTES = 12 * 1024 * 1024


class FrameSourceError(RuntimeError):
    """원격 브리지에서 입력 프레임을 가져오지 못한 경우."""


class YoloProcessingError(RuntimeError):
    """YOLO 모델 또는 이미지 처리에 실패한 경우."""


def normalize_bridge_url(value: str) -> str:
    """내부 네트워크의 HTTP(S) 브리지 URL만 허용한다."""
    parsed = urlsplit(value.strip())
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise ValueError("브리지 주소는 http:// 또는 https:// 형식이어야 합니다.")
    if parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError("브리지 주소에 인증 정보, 쿼리 또는 fragment를 넣을 수 없습니다.")

    host = parsed.hostname
    if host != "localhost":
        try:
            address = ipaddress.ip_address(host)
        except ValueError as error:
            raise ValueError("브리지 주소는 localhost 또는 내부 IP 주소여야 합니다.") from error
        if address.is_global:
            raise ValueError("외부 공개 IP를 카메라 브리지로 사용할 수 없습니다.")

    path = parsed.path.rstrip("/")
    return urlunsplit((parsed.scheme, parsed.netloc, path, "", ""))


class YoloFrameProcessor:
    """한 모델을 공유해 Robot별 브리지 프레임을 순차적으로 추론한다."""

    def __init__(self) -> None:
        self.model_path = Path(os.getenv("YOLO_MODEL_PATH", str(DEFAULT_MODEL_PATH)))
        self.device = os.getenv("YOLO_DEVICE", "cpu")
        self.confidence = float(os.getenv("YOLO_CONFIDENCE", "0.30"))
        self.max_detections = int(os.getenv("YOLO_MAX_DETECTIONS", "50"))
        self.jpeg_quality = int(os.getenv("YOLO_JPEG_QUALITY", "85"))
        self._model: YOLO | None = None
        self._inference_lock = threading.Lock()
        self._cache_lock = threading.Lock()
        self._cache: dict[str, tuple[str, bytes]] = {}

    def _load_model(self) -> YOLO:
        if self._model is not None:
            return self._model
        if not self.model_path.is_file():
            raise YoloProcessingError(f"YOLO 모델을 찾을 수 없습니다: {self.model_path}")
        self._model = YOLO(str(self.model_path))
        self._model.to(self.device)
        return self._model

    @staticmethod
    def _fetch_frame(bridge_url: str, stamp: str) -> bytes:
        query = urlencode({"t": stamp}) if stamp else ""
        url = f"{bridge_url}/camera-frame"
        if query:
            url = f"{url}?{query}"
        request = Request(url, headers={"Accept": "image/jpeg"})
        try:
            with urlopen(request, timeout=3) as response:
                content_type = response.headers.get_content_type()
                if content_type not in {"image/jpeg", "image/jpg"}:
                    raise FrameSourceError(
                        f"브리지 카메라 응답 형식이 JPEG가 아닙니다: {content_type}"
                    )
                data = response.read(MAX_FRAME_BYTES + 1)
        except HTTPError as error:
            raise FrameSourceError(
                f"브리지 카메라 요청 실패: HTTP {error.code}"
            ) from error
        except (URLError, TimeoutError, OSError) as error:
            raise FrameSourceError("브리지 카메라에 연결할 수 없습니다.") from error

        if not data or len(data) > MAX_FRAME_BYTES:
            raise FrameSourceError("브리지 카메라 프레임 크기가 올바르지 않습니다.")
        return data

    def process(self, bridge_url: str, stamp: str) -> bytes:
        bridge_url = normalize_bridge_url(bridge_url)
        with self._cache_lock:
            cached = self._cache.get(bridge_url)
            if cached and stamp and cached[0] == stamp:
                return cached[1]

        encoded_input = self._fetch_frame(bridge_url, stamp)
        frame = cv2.imdecode(np.frombuffer(encoded_input, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise YoloProcessingError("브리지 JPEG 프레임을 디코딩할 수 없습니다.")

        started = time.perf_counter()
        try:
            with self._inference_lock:
                result = self._load_model()(
                    frame,
                    conf=self.confidence,
                    max_det=self.max_detections,
                    device=self.device,
                    verbose=False,
                )[0]
                annotated = result.plot()
        except YoloProcessingError:
            raise
        except Exception as error:
            raise YoloProcessingError(f"YOLO 추론 실패: {error}") from error

        elapsed = max(time.perf_counter() - started, 1e-6)
        object_count = len(result.boxes) if result.boxes is not None else 0
        cv2.putText(
            annotated,
            f"WEB YOLO | FPS: {1.0 / elapsed:.1f} | Objects: {object_count}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        success, encoded_output = cv2.imencode(
            ".jpg",
            annotated,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not success:
            raise YoloProcessingError("YOLO 결과를 JPEG로 인코딩할 수 없습니다.")

        output = encoded_output.tobytes()
        with self._cache_lock:
            self._cache[bridge_url] = (stamp, output)
            if len(self._cache) > 8:
                self._cache.pop(next(iter(self._cache)))
        return output


yolo_frame_processor = YoloFrameProcessor()
