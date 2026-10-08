#!/bin/bash

CONFIG=$HOME/config.ini
test -f "$CONFIG" || { echo "설정 파일이 없습니다: $CONFIG"; exit 1; }

# 호스트네임(HOSTNAME)으로 ROS_DOMAIN_ID 판별
HOST_NAME=$(hostname)

case "$HOST_NAME" in
  *hkit4*)
    DOMAIN_ID=40
    ;;
  *hkit5*)
    DOMAIN_ID=50
    ;;
  *)
    echo "경고: 알 수 없는 호스트네임($HOST_NAME)입니다. 기본값 90으로 설정합니다."
    DOMAIN_ID=50
    ;;
esac

echo "=========================================="
echo "호스트네임: $HOST_NAME -> ROS_DOMAIN_ID: $DOMAIN_ID"
echo "=========================================="

docker run -it --rm \
  --name ep_car \
  --privileged \
  --net=host \
  -e ROS_DOMAIN_ID="$DOMAIN_ID" \
  --mount type=bind,src="$CONFIG",dst=/root/config.ini,readonly \
  -v /dev:/dev \
  -v /home/$HOST_NAME/ep_part/ros2_ws:/root/ep_part/ros2_ws \
  -w /root \
  ros:humble_ep_v01 \
  /bin/bash