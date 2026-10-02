"""화면과 브라우저용 설정 API를 제공하는 Controller."""
import asyncio
import json
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel

from app.models.bridge_manager import bridge_manager
from app.models.viewer_model import viewer_settings


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
            return json.loads(response.read().decode("utf-8"))
    except HTTPError as error:
        try:
            detail = json.loads(error.read().decode("utf-8")).get("detail")
        except (json.JSONDecodeError, UnicodeDecodeError):
            detail = None
        raise HTTPException(error.code, detail or "플랫폼 제어 API가 요청을 거부했습니다.") from error
    except (URLError, TimeoutError, OSError) as error:
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
        return {"stopped": await bridge_manager.disconnect(robot_id)}

    @router.post("/api/control/initialpose")
    async def initial_pose(payload: InitialPosePayload) -> dict:
        return await asyncio.to_thread(_post_platform, "/initialpose", payload.model_dump())

    return router
