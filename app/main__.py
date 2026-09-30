"""FastAPI 서버. 실행: python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8080"""
# 1. 서버 실행 및 HTTP 응답 모듈
import asyncio
import os
from contextlib import asynccontextmanager
from pathlib import Path
from threading import Event, Thread

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

# YOLO 처리 및 인식 결과 저장 모델
from app.models.model_load import VisionState, run_yolo_loop

# HTML·CSS·JS 파일을 찾기 위한 app 폴더 경로
BASE_DIR = Path(__file__).resolve().parent


# 2. 서버 시작·종료 및 선택적 로컬 YOLO 실행 관리
@asynccontextmanager
async def lifespan(application):
    # 환경 변수를 켠 경우에만 모델의 로컬 데모를 시작하고 종료 시 정리합니다.
    stop = Event()
    worker = None
    if os.getenv("HMI_LOCAL_VISION") == "1":
        worker = Thread(target=run_yolo_loop, args=(application.state.vision, stop), daemon=True)
        worker.start()
    try:
        yield
    finally:
        # 종료 신호 전달 후 로컬 추론 스레드를 최대 3초 대기
        stop.set()
        if worker is not None:
            await asyncio.to_thread(worker.join, 3)


# 3. FastAPI 앱 생성 및 공용 인식 상태 연결
app = FastAPI(lifespan=lifespan)
# 향후 SP 수신 코드도 이 인스턴스의 update()로 결과를 전달합니다.
app.state.vision = VisionState()
# 브라우저에서 사용하는 CSS·JS 정적 파일 제공
app.mount("/assets", StaticFiles(directory=BASE_DIR / "assets"), name="assets")


# HMI 메인 화면 제공
@app.get("/")
@app.get("/index.html")
@app.get("/views/hmi_main.html", include_in_schema=False)
async def home():
    return FileResponse(BASE_DIR / "views" / "hmi_main.html")


# 객체 인식 결과를 JSON으로 제공
@app.get("/api/yolo-detections")
async def detections(request: Request):
    state = request.app.state.vision
    # 모델의 최신 결과를 기존 API 응답 형식으로 제공합니다.
    items, _ = state.snapshot()
    return {"num_detections": len(items), "detections": items}


# 카메라 JPEG 영상을 MJPEG 스트림으로 제공
@app.get("/api/cam-stream")
async def camera_stream(request: Request):
    state = request.app.state.vision
    # 추론이나 SP 통신 없이 모델에 저장된 영상만 전송합니다.
    _, jpeg = state.snapshot()
    if jpeg is None:
        return JSONResponse({"detail": "Camera stream not ready"}, status_code=503)

    # 연결된 브라우저에 최신 프레임을 반복 전송
    async def frames():
        while not await request.is_disconnected():
            _, jpeg = state.snapshot()
            if jpeg is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                       + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
            await asyncio.sleep(0.05)

    return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")


# 전체 긴급정지 요청: EP 미연동 상태를 오류로 안내
@app.post("/api/master-emergency-stop")
async def emergency_stop():
    return JSONResponse({"detail": "EP emergency-stop integration is not configured"}, status_code=501)
