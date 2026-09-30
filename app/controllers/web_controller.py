"""실제 로봇 정지를 확인하지 않는 ROS2 STOP 테스트 API입니다."""
from fastapi import APIRouter, Depends


def create_router(get_ros_model):
    """서버가 제공하는 모델 조회 함수를 받아 요청마다 모델을 주입합니다."""
    router = APIRouter()

    # 전체 긴급정지 버튼 - ROS2 통신 테스트 모드
    @router.post("/api/master-emergency-stop")
    async def emergency_stop(ros_model=Depends(get_ros_model)):

        # 주입받은 모델을 통해 기존 테스트 전용 Topic에 발행합니다.
        ros_model.publish_stop()

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

    @router.post("/api/test/stop")
    async def test_stop(ros_model=Depends(get_ros_model)):

        # 메시지 생성과 ROS2 발행은 모델에 위임합니다.
        ros_model.publish_stop()

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

    return router
