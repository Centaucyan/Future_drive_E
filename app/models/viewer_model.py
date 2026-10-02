"""RViz 웹 뷰어에서 사용하는 서버 설정 모델."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class ViewerSettings:
    title: str = "퓨처드라이브 관제센터 · ROS Live"
    rosbridge_url: str = "ws://localhost:9090"
    control_api_url: str = "/api/control"
    platform_bridge_url: str = "http://127.0.0.1:8765"
    fixed_frame: str = "map"
    base_frame: str = "base_footprint"

    def to_dict(self) -> dict[str, str]:
        return asdict(self)


viewer_settings = ViewerSettings()
