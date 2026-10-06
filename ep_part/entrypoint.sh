CONFIG=/home/hkit5/config.ini
test -f "$CONFIG" || { echo "설정 파일이 없습니다: $CONFIG"; exit 1; }

docker run -it \
  --name ep_car \
  --privileged \
  --net=host \
  --mount type=bind,src="$CONFIG",dst=/root/config.ini,readonly \
  -v /dev:/dev \
  -w /root \
  ros:humble \
  /bin/bash