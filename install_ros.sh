#!/bin/bash

set -e

# Install ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies to build your own ROS packages
apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
    libgl1-mesa-glx \
    libglib2.0-0 \
    python3 \
    python3-pip \
	python3-rosdep \
	python3-rosinstall \
	python3-rosinstall-generator \
	python3-wstool\
    build-essential \
	python3-pip \
	python3-distutils \
	python3-psutil \
    python3-tk \
    git \
	ffmpeg \
	&& rm -rf /var/lib/apt/lists/*
		
#>>>note: rosdep update is not included because otherwise only root will have access to the database 
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init 
