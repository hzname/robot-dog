# Robot Dog 2.0 — runtime image for the robot (arm64) or a PC (amd64).
#
#   docker build -f docker/Dockerfile -t robot-dog:2.0 .
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=lyrical -t robot-dog:2.0-lyrical .
#
# ros-base plus std_srvs (the driver's release_fault service type) and diagnostic_msgs
# (the nodes' diagnostics bus): nothing else
# outside it (no xacro, no joy, no ros2_control), which keeps the image small on
# the Pi.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
# The Pi has 2 GB RAM: keep parallel C++ jobs low.
ARG BUILD_JOBS=2

SHELL ["/bin/bash", "-c"]
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-std-srvs \
      ros-${ROS_DISTRO}-diagnostic-msgs \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY ros2_ws/src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --parallel-workers 1 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --packages-skip dog_gazebo \
 && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
ENV ROS_DISTRO=${ROS_DISTRO}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "dog_bringup", "robot.launch.py"]
