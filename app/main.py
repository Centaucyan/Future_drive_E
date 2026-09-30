import threading

import rclpy
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from app.models.ros_model import ROSModel
from app.controllers import web_controller


# --------------------------------------------------
# ROS2 초기화
# --------------------------------------------------
rclpy.init()

ros_model = ROSModel()


# --------------------------------------------------
# ROS2 Spin
# FastAPI와 동시에 실행하기 위해 별도 Thread 사용
# --------------------------------------------------
def ros_spin():

    rclpy.spin(ros_model)


ros_thread = threading.Thread(
    target=ros_spin,
    daemon=True
)

ros_thread.start()


# --------------------------------------------------
# FastAPI
# --------------------------------------------------
app = FastAPI()


# Static Files
app.mount(
    "/static",
    StaticFiles(directory="app/statics"),
    name="static"
)


# ROS Model을 Controller에 전달
web_controller.set_ros_model(ros_model)


# Controller 등록
app.include_router(
    web_controller.router
)


# --------------------------------------------------
# 종료 처리
# --------------------------------------------------
@app.on_event("shutdown")
def shutdown_event():

    ros_model.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()