"""FastAPI 서버. 실행: python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8080"""

# 1. 서버 실행 및 HTTP 응답 모듈
import asyncio
import os
from contextlib import asynccontextmanager
from pathlib import Path
from threading import Event, Thread

from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles

# ROS2 통신 모듈 (테스트용)
import rclpy
from app.models.ap_ros_model import HMIControlModel
from app.controllers import hmi_controller, control_controller

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
        # 실제 ROS2 제어 모델을 생성하고 Controller에 전달합니다.
        ros_node = HMIControlModel()
        application.state.ros_model = ros_node

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


# HMI 화면과 조회 API 등록: 샘플 index 화면 대신 실제 HMI를 제공합니다.
app.include_router(hmi_controller.router)


# 요청이 속한 서버의 ROS2 모델을 Controller에 전달합니다.
async def get_ros_model(request: Request):
    return request.app.state.ros_model


app.include_router(control_controller.create_router(get_ros_model))
