"""야붐카 RViz 웹 뷰어 FastAPI 애플리케이션."""
from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from app.controllers.web_controller import create_router


BASE_DIR = Path(__file__).resolve().parent

app = FastAPI(title="Yahboom RViz Web Viewer")
app.mount("/static", StaticFiles(directory=BASE_DIR / "statics"), name="static")
app.include_router(create_router(BASE_DIR / "views" / "index.html"))
