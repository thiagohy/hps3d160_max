# ROS 1 Noetic environment for the HPS-3D160 system.
#
# Ubuntu 24.04 (the WSL host) has no ROS 1 Noetic binaries, so the ROS build
# and the RViz validation run inside this container.
#
#   docker build -t hps3d160-ros -f docker/ros.Dockerfile .
#   ./scripts/ros_docker.sh build      # build the workspace inside the container
#   ./scripts/ros_docker.sh run        # roscore + node + rviz
FROM ros:noetic-perception

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-rviz \
        ros-noetic-rqt-rviz \
        ros-noetic-rqt-graph \
        ros-noetic-tf2-ros \
        ros-noetic-tf2-geometry-msgs \
        ros-noetic-image-transport \
        ros-noetic-xacro \
        x11-apps \
        x11-utils \
        xvfb \
        imagemagick \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=noetic
WORKDIR /home/thiagohy/ros_ws
