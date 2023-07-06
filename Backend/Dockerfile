FROM osrf/ros:noetic-desktop-buster

RUN apt-get update && apt-get install -y \
ros-noetic-desktop-full=1.5.0-1* \
ros-noetic-rosbridge-server \
ros-noetic-catkin \
&& rm -rf /var/lib/apt/lists/*
