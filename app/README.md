# AP 웹 실행 및 파일 역할

현재 서버는 FastAPI입니다. FastAPI도 HTTP로 브라우저와 통신합니다. JS는 버튼 처리·지도 그리기·검출 결과 표시를 담당하므로 필요합니다.

# 설치 라이브러리 (ROS2 제외)

```bash
python3 -m pip install fastapi uvicorn jinja2
```

## 실행

```bash
#경로 접속
source /opt/ros/humble/setup.bash

# 또는 프로젝트 루트에서
python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8080
```

접속: `http://localhost:8080/`. 다른 PC는 서버 PC의 IP와 8080 포트를 사용합니다.

## 현재 파일

- `main.py`: FastAPI 앱, 화면/API 응답, 모델의 선택적 로컬 YOLO 시작·종료 관리.
- `app.py`: 기존 실행 명령을 위한 Uvicorn 진입점.
- `controllers/ros_decet.py`: 향후 ROS2 수신 구현 자리. 사용하지 않는 초안 import 제거.
- `models/model_load.py`: 검출 결과·JPEG 저장 모델과 선택적 로컬 YOLO 추론. 역할별 한국어 주석을 포함합니다.
- `views/hmi_main.html`: HTML과 기존 인라인 이벤트, 카메라 재요청 처리.
- `assets/js/controllers/hmi_control.js`: 로봇 선택·시뮬레이션 상태·제어 버튼.
- `assets/js/views/hmi_map.js`: 지도 그리기와 기존 이동 시뮬레이션.
- `assets/js/views/hmi_cam.js`: 검출 결과 조회·배지 갱신. 누락된 초기화를 연결했고 빈 render 메서드는 제거했습니다.
- `assets/css/hmi_style.css`: 현재 사용 중인 스타일. 사용되지 않는 선택자는 제거했습니다.

기존 `controllers/app.py`의 표준 HTTP 서버는 FastAPI로 대체하여 제거했습니다.

## API 및 연동 범위

- `/`, `/index.html`, `/views/hmi_main.html`: 메인 화면.
- `/assets/...`: CSS·JS.
- `/api/yolo-detections`: 기존 `{num_detections, detections}` JSON.
- `/api/cam-stream`: MJPEG. 준비 전 503; 브라우저는 오류 시 3초 후 재요청.
- `POST /api/master-emergency-stop`: EP 미연동으로 501. 실패 시 정지 완료 배너를 표시하지 않습니다.

기본 실행은 YOLO를 실행하지 않습니다. 기존 로컬 데모는 OpenCV·Ultralytics·카메라·모델을 준비한 뒤 `HMI_LOCAL_VISION=1 python3 app/app.py`로 실행합니다. 서버 종료 시 종료 신호를 전달하고 최대 3초 기다립니다.

SP·EP 실제 연동은 아직 없습니다. 로봇 위치·주행거리·경로는 기존 시뮬레이션입니다. 서버 기본 검출 목록이 비어 있는 것만으로 SP 연결 여부를 판단할 수 없습니다.

## SP 연동 시 사용할 모델 경계

SP의 토픽·메시지 형식이 정해지면 수신 코드에서 기존 웹 응답 형식으로 변환한 뒤 `app.state.vision.update(detections, jpeg)`를 호출합니다. `detections`는 JSON으로 표현 가능한 검출 목록이며, 현재 로컬 데모 항목은 `class_id`, `class_name`, `confidence`, `bbox`를 가집니다. `jpeg`는 JPEG bytes 또는 영상이 없을 때 None입니다. 웹 API는 `snapshot()`으로 결과를 읽습니다.

이 변경은 SP 수신 프로토콜이나 실제 연결을 구현하지 않습니다. 기본 실행과 API 주소는 동일하며 로컬 YOLO는 계속 선택 실행입니다.
