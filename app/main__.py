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

# ROS2 통신 모듈 (테스트용)
import rclpy
from std_msgs.msg import String

# YOLO 처리 및 인식 결과 저장 모델
from app.models.model_load import VisionState, run_yolo_loop

# HTML·CSS·JS 파일을 찾기 위한 app 폴더 경로
BASE_DIR = Path(__file__).resolve().parent


# 2. 서버 시작·종료 및 ROS2 통신 초기화
@asynccontextmanager
async def lifespan(application):

    # ------------------------------------------
    # ROS2 Publisher 초기화
    # ------------------------------------------

    rclpy.init()

    ros_node = None
    stop = Event()
    worker = None

    try:
        ros_node = rclpy.create_node("hmi_control_test")

        ros_publisher = ros_node.create_publisher(
            String,
            "/ap_test/control/request",
            10
        )

        # FastAPI 내부에서 ROS2 Publisher를 사용할 수 있도록 저장
        application.state.ros_node = ros_node
        application.state.ros_publisher = ros_publisher

        print(
            "[ROS2] Publisher initialized: /ap_test/control/request",
            flush=True
        )

        # ------------------------------------------
        # 기존 선택적 로컬 YOLO 실행
        # ------------------------------------------

        if os.getenv("HMI_LOCAL_VISION") == "1":
            worker = Thread(
                target=run_yolo_loop,
                args=(application.state.vision, stop),
                daemon=True
            )
            worker.start()

        # 서버 실행 유지
        yield

    finally:
        # ------------------------------------------
        # 서버 종료 시 자원 정리
        # ------------------------------------------

        stop.set()

        if worker is not None:
            await asyncio.to_thread(worker.join, 3)

        # ROS2 노드 종료
        if ros_node is not None:
            ros_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("[ROS2] Test publisher shutdown", flush=True)


# 3. FastAPI 앱 생성 및 공용 인식 상태 연결
app = FastAPI(lifespan=lifespan)

# 향후 SP 수신 코드도 이 인스턴스의 update()로 결과를 전달합니다.
app.state.vision = VisionState()

# 브라우저에서 사용하는 CSS·JS 정적 파일 제공
app.mount(
    "/assets",
    StaticFiles(directory=BASE_DIR / "assets"),
    name="assets"
)


# 4. HMI 메인 화면 제공
@app.get("/")
@app.get("/index.html")
@app.get("/views/hmi_main.html", include_in_schema=False)
async def home():
    return FileResponse(BASE_DIR / "views" / "hmi_main.html")


# 5. 객체 인식 결과를 JSON으로 제공
@app.get("/api/yolo-detections")
async def detections(request: Request):
    state = request.app.state.vision

    # 모델의 최신 결과를 기존 API 응답 형식으로 제공합니다.
    items, _ = state.snapshot()

    return {
        "num_detections": len(items),
        "detections": items
    }


# 6. 카메라 JPEG 영상을 MJPEG 스트림으로 제공
@app.get("/api/cam-stream")
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



# 전체 긴급정지 버튼 - ROS2 통신 테스트 모드
@app.post("/api/master-emergency-stop")
async def emergency_stop(request: Request):

    # 테스트용 ROS2 메시지 생성
    msg = String()
    msg.data = "STOP"

    # 실제 로봇 제어 Topic이 아닌 테스트 전용 Topic에 발행
    request.app.state.ros_publisher.publish(msg)

    print(
        "[HMI ALL STOP -> ROS2 TEST] STOP published!",
        flush=True
    )

    return {
        "success": True,
        "command": "STOP",
        "topic": "/ap_test/control/request",
        "test_only": True,
        "executed": False,
        "message": "ROS2 test message published. Robot stop is not confirmed."
    }



# ============================================================
# 8. ROS2 STOP 통신 테스트 API (새로 추가)
# ============================================================

@app.post("/api/test/stop")
async def test_stop(request: Request):

    # ROS2 String 메시지 생성
    msg = String()
    msg.data = "STOP"

    # 테스트 전용 ROS2 Topic에 메시지 발행
    request.app.state.ros_publisher.publish(msg)

    # FastAPI 실행 터미널에 출력
    print(
        "[FastAPI -> ROS2] STOP message published!",
        flush=True
    )

    # 웹으로 HTTP 응답 반환
    return {
        "success": True,
        "command": "STOP",
        "topic": "/ap_test/control/request",
        "test_only": True,
        "executed": False
    }