"""화면과 브라우저용 설정 API를 제공하는 Controller."""
import asyncio
import json
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel

from app.models.bridge_manager import bridge_manager
from app.models.viewer_model import viewer_settings
from app.models.yolo_frame_processor import (
    FrameSourceError,
    YoloProcessingError,
    yolo_frame_processor,
)


class InitialPosePayload(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"


class BridgeConnectPayload(BaseModel):
    robot_id: str
    domain_id: int
    robot_ip: str | None = None


def _post_platform(path: str, payload: dict) -> dict:
    request = Request(
        viewer_settings.platform_bridge_url.rstrip("/") + path,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=5) as response:
            # HTTP 200이어도 HTML이나 손상된 JSON이 올 수 있으므로 502로 명확히 구분한다.
            try:
                result = json.loads(response.read().decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError) as error:
                raise HTTPException(
                    502, "플랫폼 제어 API 응답을 해석할 수 없습니다."
                ) from error
            if not isinstance(result, dict):
                raise HTTPException(502, "플랫폼 제어 API 응답 형식이 올바르지 않습니다.")
            return result
    except HTTPError as error:
        # 플랫폼 오류 본문까지 손상된 경우에도 원래 HTTP 상태 코드는 유지한다.
        try:
            detail = json.loads(error.read().decode("utf-8")).get("detail")
        except (AttributeError, json.JSONDecodeError, OSError, UnicodeDecodeError):
            detail = None
        raise HTTPException(error.code, detail or "플랫폼 제어 API가 요청을 거부했습니다.") from error
    except (URLError, TimeoutError, OSError, ValueError) as error:
        raise HTTPException(503, "플랫폼 제어 API에 연결할 수 없습니다.") from error


def create_router(view_path: Path) -> APIRouter:
    router = APIRouter()

    @router.get("/", include_in_schema=False)
    @router.get("/index.html", include_in_schema=False)
    async def viewer() -> FileResponse:
        return FileResponse(view_path)

    @router.get("/api/viewer-config")
    async def viewer_config() -> dict[str, str]:
        return viewer_settings.to_dict()

    @router.get("/api/health")
    async def health() -> dict[str, str]:
        return {"status": "ok"}

    @router.get("/api/yolo-frame")
    async def yolo_frame(bridge: str, t: str = "") -> Response:
        try:
            # PyTorch/OpenCV 추론은 웹 서버 프로세스의 주 실행 스레드에서 실행한다.
            # 로봇 상태와 제어 WebSocket은 브라우저가 각 브리지에 직접 연결하므로
            # 프레임 추론 중에도 로봇 제어 경로는 영향을 받지 않는다.
            frame = yolo_frame_processor.process(bridge, t)
        except ValueError as error:
            raise HTTPException(422, str(error)) from error
        except FrameSourceError as error:
            raise HTTPException(502, str(error)) from error
        except YoloProcessingError as error:
            raise HTTPException(503, str(error)) from error
        return Response(
            content=frame,
            media_type="image/jpeg",
            headers={"Cache-Control": "no-store"},
        )

    @router.post("/api/bridges/connect")
    async def connect_bridge(payload: BridgeConnectPayload) -> dict:
        try:
            return await bridge_manager.connect(payload.robot_id, payload.domain_id, payload.robot_ip)
        except ValueError as error:
            raise HTTPException(422, str(error)) from error
        except (OSError, RuntimeError) as error:
            raise HTTPException(503, str(error)) from error

    @router.delete("/api/bridges/{robot_id}")
    async def disconnect_bridge(robot_id: str) -> dict[str, bool]:
        # 잘못된 입력은 422, 실제 프로세스 종료 실패는 재시도 가능한 503으로 반환한다.
        try:
            return {"stopped": await bridge_manager.disconnect(robot_id)}
        except ValueError as error:
            raise HTTPException(422, str(error)) from error
        except (OSError, RuntimeError) as error:
            raise HTTPException(503, str(error)) from error

    @router.post("/api/control/initialpose")
    async def initial_pose(payload: InitialPosePayload) -> dict:
        return await asyncio.to_thread(_post_platform, "/initialpose", payload.model_dump())

    return router
