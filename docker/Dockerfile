# Luna R2 ROS 2 Humble image for the Jetson Orin NX 16 GB (JetPack 6.2, L4T r36.4, Ubuntu 22.04).
# JetPack 6 is Ubuntu 22.04, where ROS 2 Humble is ordinary apt packages: no ROS-from-source image any more (R1's
# Xavier on JetPack 5 / Ubuntu 20.04 needed dustynv's). l4t-jetpack brings CUDA for whisper.cpp.
# NOT BUILT on a Jetson yet. The ROS part (same apt packages, OrbbecSDK_ROS2, sllidar_ros2, luna_robot) builds and its
# colcon test passes on x86 ros:humble-ros-base (README_TR.md, "R2 doğrulama").
#   docker compose -f docker/compose.yaml build      (on the Orin, from the repository root)
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 ROS_DISTRO=humble
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg2 ca-certificates git cmake build-essential \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base ros-humble-robot-state-publisher ros-humble-tf2-ros \
    ros-humble-slam-toolbox ros-humble-nav2-controller ros-humble-nav2-planner ros-humble-nav2-navfn-planner \
    ros-humble-nav2-behaviors ros-humble-nav2-bt-navigator ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-velocity-smoother ros-humble-nav2-collision-monitor ros-humble-nav2-regulated-pure-pursuit-controller \
    ros-humble-nav2-costmap-2d ros-humble-nav2-msgs ros-humble-nav2-map-server \
    ros-humble-apriltag-ros ros-humble-image-transport ros-humble-camera-info-manager \
    python3-colcon-common-extensions python3-rosdep python3-numpy python3-yaml python3-pytest python3-pip \
    python3-opencv python3-usb alsa-utils \
 && rm -rf /var/lib/apt/lists/*
# python3-opencv is Ubuntu's 4.5.4: FaceDetectorYN (YuNet) is there, and it is built with GStreamer for the CSI
# face camera (nvarguscamerasrc, needs /tmp/argus_socket from the host: compose.yaml).

# Speech: whisper.cpp's whisper-server on the Orin's GPU (sm_87), Piper for the Turkish voice.
RUN git clone --depth 1 -b v1.9.5 https://github.com/ggml-org/whisper.cpp /opt/whisper.cpp \
 && cmake -S /opt/whisper.cpp -B /opt/whisper.cpp/build -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87 \
    -DWHISPER_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /opt/whisper.cpp/build -j"$(nproc)" --target whisper-server \
 && install -m 755 /opt/whisper.cpp/build/bin/whisper-server /usr/local/bin/ \
 && python3 -m pip install --no-cache-dir "piper-tts[http]==1.8.0"

WORKDIR /opt/luna_ws
# Drivers built from source: Gemini 336 (OrbbecSDK_ROS2 v2, the 330 series needs v2), RPLIDAR C1, micro-ROS agent.
RUN mkdir -p src && cd src \
 && git clone --depth 1 -b v2.9.3 https://github.com/orbbec/OrbbecSDK_ROS2.git \
 && git clone --depth 1 https://github.com/Slamtec/sllidar_ros2.git \
 && git clone --depth 1 -b humble https://github.com/micro-ROS/micro_ros_setup.git
COPY luna_ws/src/luna_robot src/luna_robot
RUN . /opt/ros/humble/setup.sh \
 && rosdep init && rosdep update --rosdistro humble \
 && apt-get update && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
 && rm -rf /var/lib/apt/lists/* \
 && colcon build --packages-up-to orbbec_camera sllidar_ros2 micro_ros_setup luna_robot \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && . install/setup.sh \
 && ros2 run micro_ros_setup create_agent_ws.sh \
 && ros2 run micro_ros_setup build_agent.sh \
 && rm -rf build log
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "luna_robot", "luna.launch.py"]
