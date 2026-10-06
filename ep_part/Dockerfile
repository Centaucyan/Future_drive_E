FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-serial \
    python3-opencv \
    ros-${ROS_DISTRO}-vision-msgs \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# 라이다 드라이버 (RPLiDAR 가정, 모델이 다르면 교체)
RUN apt-get update && (apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rplidar-ros \
    || echo "WARNING: rplidar-ros not installed") && rm -rf /var/lib/apt/lists/*

WORKDIR /ep_ws
COPY ros2_ws/src ./src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
