FROM osrf/ros:noetic-desktop-focal

RUN apt-get update && apt-get install -y \
ros-noetic-desktop-full \
ros-noetic-rosbridge-server \
ros-noetic-catkin \
gdb \
iproute2 \
wireless-tools \
neovim \
lcov

RUN useradd -m -s /bin/bash -u 1000 user
RUN echo '. /workspace/devel/setup.bash' >> /home/user/.bashrc
USER user
WORKDIR /workspace
