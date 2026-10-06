# EP팀 프로젝트 구조

- firmware/opencr_motor : OpenCR 펌웨어 (PC의 Arduino IDE로 업로드, 파이에는 불필요)
- ros2_ws/src/my_robot_bringup : 브리지(cmd_vel/odom), 회피 노드, launch, 설정
- (SP팀 감지 토픽은 vision_msgs/Detection2DArray 사용 -> 별도 msg 패키지 불필요)

시리얼 프로토콜 v1
  PC -> OpenCR : V,좌raw,우raw
  OpenCR -> PC : F,ms,좌tick,우tick / U,ms,거리mm,estop / # 디버그

## SLAM 사용 (요약)
파이:  docker compose up -d robot   (lidar:=true, /dev/ttyUSB0 주석 해제 후)
PC  :  docker compose -f docker-compose.pc.yml up slam
       xhost +local:docker && docker compose -f docker-compose.pc.yml --profile viewer up rviz
저장:  docker compose -f docker-compose.pc.yml --profile shell run --rm shell
       ros2 run nav2_map_server map_saver_cli -f /ep_ws/maps/my_map

## 카메라 / SP팀 연동
파이:  robot.launch.py ... camera:=true            (압축 이미지 /image_raw/compressed 발행)
PC  :  ros2 run image_transport republish compressed raw \
         --ros-args -r in/compressed:=/image_raw/compressed -r out:=/camera/image_raw
회피 노드는 /yolo/detections (class 0=4륜, 1=2륜, 2=사람, 거리=pose.position.z)를 구독
