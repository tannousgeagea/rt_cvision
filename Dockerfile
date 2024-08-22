# This is an auto generated Dockerfile for ros:perception
# generated from docker_images/create_ros_image.Dockerfile.em
FROM nvidia/cuda:11.5.2-cudnn8-runtime-ubuntu20.04

# Maintainer instructions has been deprecated, instead use LABEL
LABEL maintainer="tannous.geagea@wasteant.com"

# Versionining as "b-beta, a-alpha, rc - release candidate"
LABEL com.wasteant.version="1.1b1"

# [CHECK] Whether it is convenient to use the local user values or create ENV variables, or run everyhting with root
ENV ROS_DISTRO=noetic
ARG user
ARG userid
ARG group
ARG groupid

# Install other necessary packages and dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
    apt-utils \
	vim \
	git \
	iputils-ping \
	net-tools \
	netcat \
	ssh \
    curl \
    lsb-release \
    wget \
    zip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies to build your own ROS packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
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

# Install libraries
RUN pip3 install ultralytics
RUN pip3 install opencv-python
RUN pip3 install albumentations
RUN pip3 install natsort
RUN pip3 install psycopg2
RUN pip3 install schedule
RUN pip3 install numpy
RUN pip3 install pandas
RUN pip3 install matplotlib
RUN pip3 install pymongo
RUN pip3 install imutils
RUN pip3 install fpdf
RUN pip3 install pytz
RUN pip3 install pyyaml
RUN pip3 install tqdm
RUN pip3 install supervisor
RUN pip3 install fastapi[standard]
RUN pip3 install uvicorn[standard]
RUN pip3 install flower
RUN pip3 install redis
RUN pip3 install celery
RUN pip3 install asgi_correlation_id
RUN pip3 install django==4.2
RUN pip3 install gunicorn
RUN pip3 install requests
RUN pip3 install python-redis-lock
RUN pip3 install grpcio
RUN pip3 install grpcio-tools
RUN pip3 install confluent-kafka==2.0.2
RUN pip3 install lapx>=0.5.2

# upgrade everything
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -q -y \
   && rm -rf /var/lib/apt/lists/*

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
	sudo \
	&& rm -rf /var/lib/apt/lists/*

# # Set up users and groups
RUN addgroup --gid $groupid $group && \
	adduser --uid $userid --gid $groupid --disabled-password --gecos '' --shell /bin/bash $user && \
	echo "$user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$user && \
	chmod 0440 /etc/sudoers.d/$user

# # # Create initial workspace 
RUN mkdir -p /home/$user/src
RUN mkdir -p /home/$user/data

RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin_init_workspace /home/$user/src"  
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; cd /home/$user; catkin_make"

RUN /bin/bash -c "echo source /opt/ros/noetic/setup.bash >> /home/$user/.bashrc"
RUN /bin/bash -c "echo source /home/$user/devel/setup.bash >> /home/$user/.bashrc"
RUN /bin/bash -c "chown -R $user:$user /home/$user/"

# setup entrypoint
ENV ROS_DISTRO=noetic

# Create directory for Supervisor logs
RUN mkdir -p /var/log/supervisor && \
    chmod -R 755 /var/log/supervisor
	
COPY ./supervisord.conf /etc/supervisord.conf
COPY ./entrypoint.sh /home/.
RUN /bin/bash -c "chown $user:$user /home/entrypoint.sh"

ENTRYPOINT /bin/bash -c ". /home/entrypoint.sh"