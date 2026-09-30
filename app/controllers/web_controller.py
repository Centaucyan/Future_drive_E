from fastapi import APIRouter, Request
from fastapi.templating import Jinja2Templates

from app.models.ros_model import ROSModel


router = APIRouter()

templates = Jinja2Templates(
    directory="app/views"
)


# ROS Model은 main.py에서 주입
ros_model: ROSModel = None


def set_ros_model(model: ROSModel):
    global ros_model
    ros_model = model


@router.get("/")
async def index(request: Request):

    message = ros_model.get_message()

    return templates.TemplateResponse(
        request=request,
        name="index.html",
        context={
            "message": message
        }
    )