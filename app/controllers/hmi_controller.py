"""실제 HMI 화면과 인식 결과 조회 API를 제공합니다."""
import asyncio
from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from app.models.image_stream_model import image_stream_model
from app.models.yahboom_yolo import YoloDetector
from fastapi.templating import Jinja2Templates

# 기존 HMI의 경로와 응답 형식을 유지합니다.
router = APIRouter()
BASE_DIR = Path(__file__).resolve().parents[1]

templates = Jinja2Templates(
    directory="app/views"
)

# 4. HMI 메인 화면 제공
@router.get("/")
@router.get("/index.html")
@router.get("/views/hmi_main.html", include_in_schema=False)
async def home():
    return FileResponse(BASE_DIR / "views" / "hmi_main.html")


# 5. 객체 인식 결과를 JSON으로 제공
@router.get("/api/yolo-detections")
async def detections(request: Request):
    state = request.app.state.vision

    # 모델의 최신 결과를 기존 API 응답 형식으로 제공합니다.
    items, _ = state.snapshot()

    return {
        "num_detections": len(items),
        "detections": items
    }

@router.get("/image")
async def image_page(request: Request):
    """
    이미지 스트리밍 화면
    """
    return templates.TemplateResponse(
        request=request,
        name="image_stream.html",
        context={
            "title": "Image Raw Stream",
        },
    )

@router.get("/image/stream")
async def image_stream():
    """
    MJPEG 스트리밍 endpoint
    """

    return StreamingResponse(
        image_stream_model.generate_mjpeg(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "Expires": "0",
        },
    )

# 6. 카메라 JPEG 영상을 MJPEG 스트림으로 제공
@router.get("/api/cam-stream")
async def camera_stream(request: Request):
    state = request.app.state.vision

    # 추론이나 SP 통신 없이 모델에 저장된 영상만 전송합니다.
    _, jpeg = state.snapshot()

    if jpeg is None:
        return JSONResponse(
            {"detail": "Camera stream not ready"},
            status_code=503
        )

    # 연결된 브라우저에 최신 프레임을 반복 전송
    async def frames():
        while not await request.is_disconnected():

            _, jpeg = state.snapshot()

            if jpeg is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )

            await asyncio.sleep(0.05)

    return StreamingResponse(
        frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )
