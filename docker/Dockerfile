# Robot Dog 2.0 — runtime image for the robot (arm64) or a PC (amd64).
#
#   docker build -f docker/Dockerfile -t robot-dog:2.0 .
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=lyrical -t robot-dog:2.0-lyrical .
#
# ros-base plus std_srvs (the driver's release_fault service type). diagnostic_msgs
# comes with ros-base; nothing else is added
# outside it (no xacro, no joy, no ros2_control), which keeps the image small on
# the Pi.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
# The Pi has 2 GB RAM: keep parallel C++ jobs low.
ARG BUILD_JOBS=2

SHELL ["/bin/bash", "-c"]
# std_srvs is not in ros-base, and installing it alone is a trap: apt takes the
# current build of it while the image keeps its older interface neighbours, and the
# typesupport of the new package then calls symbols they do not have - the node dies
# at startup with "symbol lookup error" (exit 127). Upgrading the whole installed set
# in the same layer keeps the interface stack consistent.
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-std-srvs \
 && apt-get upgrade -y --no-install-recommends \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY ros2_ws/src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --parallel-workers 1 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --packages-skip dog_gazebo \
 && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
# Preflight ships inside the image and the entrypoint runs it before the launch:
# a step someone can forget is not a check (CAL-09). The smoke script is what CI
# runs to prove the image can start the stack.
COPY docker/ci_smoke.sh /ws/docker/ci_smoke.sh
COPY docker/check_state.py /ws/docker/check_state.py
COPY tools/preflight /ws/tools/preflight
ENV ROS_DISTRO=${ROS_DISTRO}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "dog_bringup", "robot.launch.py"]
