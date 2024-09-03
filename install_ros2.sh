#!/bin/bash

set -e

add-apt-repository universe

# add the ROS 2 GPG key with apt
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install dependencies to build your own ROS packages
apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
    libgl1-mesa-glx \
    libglib2.0-0 \
    python3 \
    python3-pip \
    python3-rosdep \
	python3-wstool\
	python3-distutils \
	python3-psutil \
    python3-tk \
    git \
	ffmpeg \
	&& rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages
apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-rclpy \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 build tools and dependencies
apt-get install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    && rm -rf /var/lib/apt/lists/*