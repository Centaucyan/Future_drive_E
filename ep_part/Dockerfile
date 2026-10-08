FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive
ARG MESON_VERSION=1.11.2
ARG LIBCAMERA_VERSION=v0.7.2+rpt20260817
ARG RPICAM_APPS_VERSION=v1.12.0

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV PATH="/opt/meson-venv/bin:${PATH}"

# ROS 패키지, Python 이미지 처리 모듈, 카메라 빌드 의존성
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    ros-humble-usb-cam \
    ros-humble-compressed-image-transport \
    ros-humble-rplidar-ros \
    ros-humble-vision-msgs \
    python3-serial \
    python3-opencv \
    python3-numpy \
    python3-venv \
    git \
    g++ \
    cmake \
    pkg-config \
    ninja-build \
    libyaml-dev \
    libgnutls28-dev \
    openssl \
    libtiff5-dev \
    libjpeg-dev \
    libpng-dev \
    libboost-program-options-dev \
    libexif-dev \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Meson과 libcamera 빌드에 필요한 Python 모듈을 같은 가상환경에 설치
RUN python3 -m venv /opt/meson-venv \
    && /opt/meson-venv/bin/pip install --no-cache-dir --upgrade pip \
    && /opt/meson-venv/bin/pip install --no-cache-dir \
       "meson==${MESON_VERSION}" \
       Jinja2 PyYAML ply \
    && /opt/meson-venv/bin/python -c \
       "import jinja2, yaml, ply; print('Python build modules OK')" \
    && meson --version

# Raspberry Pi용 libcamera 빌드 및 설치
RUN mkdir -p /tmp/camera-build \
    && cd /tmp/camera-build \
    && git clone --depth=1 --branch "${LIBCAMERA_VERSION}" \
       https://github.com/raspberrypi/libcamera.git \
    && cd libcamera \
    && meson setup build \
       --buildtype=release \
       -Dpipelines=rpi/vc4,rpi/pisp \
       -Dipas=rpi/vc4,rpi/pisp \
       -Dv4l2=true \
       -Dgstreamer=disabled \
       -Dtest=false \
       -Dlc-compliance=disabled \
       -Dcam=disabled \
       -Dqcam=disabled \
       -Ddocumentation=disabled \
       -Dpycamera=disabled \
    && ninja -C build -j2 \
    && ninja -C build install \
    && ldconfig

# rpicam-vid 등 카메라 명령 빌드 및 설치
RUN cd /tmp/camera-build \
    && git clone --depth=1 --branch "${RPICAM_APPS_VERSION}" \
       https://github.com/raspberrypi/rpicam-apps.git \
    && cd rpicam-apps \
    && meson setup build \
       -Denable_libav=disabled \
       -Denable_drm=disabled \
       -Denable_egl=disabled \
       -Denable_qt=disabled \
       -Denable_opencv=disabled \
       -Denable_tflite=disabled \
       -Denable_hailo=disabled \
    && ninja -C build -j2 \
    && ninja -C build install \
    && ldconfig \
    && rpicam-vid --version \
    && rpicam-hello --version \
    && rm -rf /tmp/camera-build

# ROS 2 언더레이 및 오버레이 워크스페이스 자동 소싱
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ep_part/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root
CMD ["/bin/bash"]