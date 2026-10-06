"""야붐카 RViz 웹 뷰어 FastAPI 애플리케이션."""
from pathlib import Path
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from app.controllers.web_controller import create_router
from app.models.bridge_manager import bridge_manager


BASE_DIR = Path(__file__).resolve().parent

@asynccontextmanager
async def lifespan(_app: FastAPI):
    yield
    await bridge_manager.close()


app = FastAPI(title="Yahboom RViz Web Viewer", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=BASE_DIR / "statics"), name="static")
app.include_router(create_router(BASE_DIR / "views" / "index.html"))
