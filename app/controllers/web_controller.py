"""화면과 브라우저용 설정 API를 제공하는 Controller."""
import asyncio
import json
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel

from app.models.viewer_model import viewer_settings


class InitialPosePayload(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"


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

    @router.post("/api/control/initialpose")
    async def initial_pose(payload: InitialPosePayload) -> dict:
        return await asyncio.to_thread(_post_platform, "/initialpose", payload.model_dump())

    return router
