"""FastAPI 서버. 실행: python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8080"""

# 1. 서버 실행 및 HTTP 응답 모듈
import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from threading import Thread

from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles

# ROS2 통신 모듈 (테스트용)
import rclpy
from rclpy.executors import SingleThreadedExecutor
from app.models.ap_ros_model import HMIControlModel
from app.controllers import hmi_controller, web_controller
from app.controllers.ros_decet import SPVisionReceiver
from app.models.image_stream_model import image_stream_model

# YOLO 처리 및 인식 결과 저장 모델
from app.models.model_load import VisionState

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
    sp_receiver = None
    executor = None
    spin_thread = None

    


    try:
        # 실제 ROS2 제어 모델을 생성하고 Controller에 전달합니다.
        ros_node = HMIControlModel()
        application.state.ros_model = ros_node
        image_stream_model.start()

        print(
            "[ROS2] Publisher initialized: /ap_test/control/request",
            flush=True
        )

        # 웹 API와 동일한 상태 객체를 SP 수신 노드에 전달합니다.
        sp_receiver = SPVisionReceiver(application.state.vision)
        application.state.sp_receiver = sp_receiver

        # 두 노드를 함께 spin합니다. 수신 콜백은 순차 실행되어 최신 값끼리 덮어쓰지 않습니다.
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)
        executor.add_node(sp_receiver)
        spin_thread = Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # 서버 실행 유지
        yield

    finally:
        # ------------------------------------------
        # 서버 종료 시 자원 정리
        # ------------------------------------------

        # 콜백 실행을 먼저 끝내고 노드를 해제하여 종료 중 접근을 방지합니다.
        if executor is not None:
            await asyncio.to_thread(executor.shutdown)
        if spin_thread is not None:
            await asyncio.to_thread(spin_thread.join)
        if sp_receiver is not None:
            sp_receiver.destroy_node()

        # ROS2 노드 종료
        if ros_node is not None:
            ros_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        image_stream_model.stop()
        print("[ROS2] Test publisher shutdown", flush=True)


# 3. FastAPI 앱 생성 및 공용 인식 상태 연결
app = FastAPI(lifespan=lifespan)

# SP 수신 노드와 HMI API가 이 인스턴스 하나를 공유합니다.
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


app.include_router(web_controller.create_router(get_ros_model))
