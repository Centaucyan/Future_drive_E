# 관리 파일

웹 서버와 rosbridge를 독립 실행하고 ROS 통신 환경을 변경할 때 사용하는 파일입니다.

- `start-web.sh`: FastAPI 웹 서버 실행
- `start-bridge.sh`: rosbridge WebSocket 실행
- `ros-env.sh`: ROS Domain과 DDS 환경 변수
- `fastdds-pc.xml`: 현재 PC용 Fast DDS 네트워크 설정

팀 프로젝트의 통합 실행 방식이 정해지면 이 설정을 그 방식에 맞춰 관리합니다.
