# Luna ROS 2 Humble image for Jetson AGX Xavier (JetPack 5.1.x / L4T r35, Ubuntu 20.04 host).
# ROS inside dustynv images is built from source: extra ROS packages must be built from source too.
# NOT BUILT in this delivery (needs a Jetson or an arm64 L4T build host).
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

# Voice: arecord/aplay for the XVF3800, pyusb for its DoA control interface (NOT BUILT, see README_TR.md).
RUN apt-get update && apt-get install -y --no-install-recommends alsa-utils python3-usb \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/luna_ws
RUN mkdir -p src && cd src \
 && git clone --depth 1 -b humble https://github.com/micro-ROS/micro_ros_setup.git \
 && git clone --depth 1 https://github.com/Slamtec/sllidar_ros2.git
COPY luna_ws/src/luna_robot src/luna_robot
RUN . /opt/ros/humble/install/setup.sh \
 && colcon build --packages-select micro_ros_setup sllidar_ros2 luna_robot --symlink-install \
 && . install/setup.sh \
 && ros2 run micro_ros_setup create_agent_ws.sh \
 && ros2 run micro_ros_setup build_agent.sh
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "luna_robot", "luna.launch.py"]
