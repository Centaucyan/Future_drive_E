# 관리 파일

웹 서버와 로봇 상태·제어 브리지를 실행하는 파일입니다.

- `start-web.sh`: FastAPI 웹 서버 실행
- `start-platform-bridge.sh`: Autonomy Studio 시각화와 별개로 상태·제어 브리지 실행
- `status-platform-bridge.sh`: 독립 브리지 컨테이너와 상태 API 확인
- `stop-platform-bridge.sh`: 독립 브리지 중지·삭제
- `start-bridge.sh`: rosbridge WebSocket 실행
- `ros-env.sh`: ROS Domain과 DDS 환경 변수
- `fastdds-pc.xml`: 현재 PC용 Fast DDS 네트워크 설정

## 독립 브리지 사용

Autonomy Studio에서 로봇을 연결하고 필요한 모듈을 실행한 뒤 사용합니다. 시각화 창은 열지 않아도 됩니다.

```bash
cd ~/Future_drive_E
bash app/management/start-platform-bridge.sh
bash app/management/start-web.sh
```

브리지 상태 확인과 종료:

```bash
bash app/management/status-platform-bridge.sh
bash app/management/stop-platform-bridge.sh
```

스크립트는 Autonomy Studio가 마지막으로 생성한 Fast DDS peer 프로필과 로컬 브리지 이미지를 재사용합니다. 특정 프로필이나 포트를 선택해야 하면 환경 변수로 지정할 수 있습니다.

```bash
FASTDDS_PROFILE="$HOME/autonomy-studio/runtime/nav2/docker/fastdds-peer-100-64-0-21-d77.xml" \
BRIDGE_PORT=8765 \
bash app/management/start-platform-bridge.sh
```

기존 `start-bridge.sh`는 이전 rosbridge 방식이 필요할 때만 사용합니다.
