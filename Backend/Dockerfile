FROM osrf/ros:noetic-desktop-focal

RUN apt-get update && apt-get install -y \
ros-noetic-desktop-full \
ros-noetic-rosbridge-server \
ros-noetic-catkin \
neovim 

RUN useradd -m -s /bin/bash -u 1000 user
USER user
WORKDIR /workspace
