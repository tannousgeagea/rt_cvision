# This is an auto generated Dockerfile for ros:perception
# generated from docker_images/create_ros_image.Dockerfile.em
ARG CUDA_VERSION=12.5.1-cudnn-runtime-ubuntu22.04

FROM nvidia/cuda:${CUDA_VERSION}

LABEL maintainer="tannous.geagea@wasteant.com"
LABEL com.wasteant.version="1.1b1"

# Set non-interactive mode for apt-get
ENV DEBIAN_FRONTEND=noninteractive

# [CHECK] Whether it is convenient to use the local user values or create ENV variables, or run everyhting with root
ARG user
ARG userid
ARG group
ARG groupid
ARG ros_distro=humble
ARG ros_version=ros2
ARG installation_folder="./installation_files"

# Set environment variables
ENV ROS_VERSION=${ros_version}
ENV ROS_DISTRO=${ros_distro}

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
    sudo \
    && rm -rf /var/lib/apt/lists/*


# Install minimal packages for ros install
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
    curl \
    gnupg2 \
    build-essential \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Copy installation scripts into the container
COPY install_ros.sh /install_ros.sh
COPY install_ros2.sh /install_ros2.sh

# Make the scripts executable
RUN chmod +x /install_ros.sh /install_ros2.sh
RUN echo "USER: $user | ROS_VERSION: $ros_version" && echo "ROS_DISTRO: $ros_distro"

# Install dependencies and ROS version based on the environment variable
RUN if [ "$ros_version" = "ros1" ]; then /install_ros.sh; \
    elif [ "$ros_version" = "ros2" ]; then /install_ros2.sh; \
    else echo "Invalid ROS_VERSION ${ros_version} specified, must be 'ros1' or 'ros2'"; exit 1; \
    fi

# Install libraries
RUN pip3 install ultralytics
RUN pip3 install opencv-python
RUN pip3 install albumentations
RUN pip3 install natsort
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
RUN pip3 install confluent-kafka
RUN pip3 install scp
RUN pip3 install lapx>=0.5.2
RUN pip3 install mlflow
RUN pip3 install azureml-mlflow

# upgrade everything
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -q -y \
   && rm -rf /var/lib/apt/lists/*

# # Set up users and groups
RUN addgroup --gid $groupid $group && \
	adduser --uid $userid --gid $groupid --disabled-password --gecos '' --shell /bin/bash $user && \
	echo "$user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$user && \
	chmod 0440 /etc/sudoers.d/$user

RUN mkdir -p /home/$user/src
RUN mkdir -p /media/$user

RUN /bin/bash -c "chown -R $user:$user /home/$user/"
RUN /bin/bash -c "chown -R $user:$user /media/$user/"

# Source the ROS 2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Create directory for Supervisor logs
RUN mkdir -p /var/log/supervisor && \
    chmod -R 755 /var/log/supervisor
	

COPY . /home/${user}/src

COPY ./supervisord.conf /etc/supervisord.conf
COPY ./entrypoint.sh /home/.
RUN /bin/bash -c "chown $user:$user /home/entrypoint.sh"

ENTRYPOINT /bin/bash -c ". /home/entrypoint.sh"