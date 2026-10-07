FROM ros:humble

# 패키지 설치 시 대화형 프롬프트 방지
ENV DEBIAN_FRONTEND=noninteractive

# 요청하신 패키지들을 하나의 RUN 명령으로 묶어 레이어 최적화 및 캐시 정리
RUN apt-get update && apt-get install -y \
    ros-humble-usb-cam \
    ros-humble-compressed-image-transport \
    ros-humble-rplidar-ros \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 환경 설정을 .bashrc에 등록
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root

CMD ["/bin/bash"]