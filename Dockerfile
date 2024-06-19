# Pull base image with pre installed ROS2 Humble
FROM osrf/ros:humble-desktop-full-jammy

# Set the timezone non-interactively
ENV TZ=UTC

# command-line arguments
ARG DEBIAN_FRONTEND=noninteractive
ARG USER
ARG PASSWORD
ARG USER_ID
ARG GROUP_ID
ARG OPENCV_VERSION
ARG CUDA_ARCH_BIN

# Update and install packages without interactive prompts
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git vim nano sudo ca-certificates lsb-release gnupg2 \
    build-essential cmake wget unzip gnupg tar \
    libfreeimage3 libfreeimage-dev zip libasio-dev libtinyxml2-dev \
    libjsoncpp-dev libcurl4-openssl-dev libtins-dev libpcap-dev libglfw3-dev libglew-dev\
    libeigen3-dev \

# Set timezone
RUN apt update && \
    apt install -yq tzdata && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

# Installing gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] 
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null


# Set working directory and user
WORKDIR /home/ROS2

# install python and other packages
RUN apt-get update && && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip ignition-fortress ros-humble-ros-ign-bridge \
    ros-humble-gazebo-ros-pkgs  \
    ros-humble-teleop-twist-keyboard    \
    ros-humble-turtlebot3-gazebo   ros-humble-xacro \
    ros-humble-camera-calibration-parsers
    ros-humble-camera-info-manager
    ros-humble-launch-testing-ament-cmake
    ros-humble-camera-calibration
    ros-humble-sensor-msgs ros-humble-sensor-msgs-py
    ros-humble-navigation2 ros-humble-nav2-bringup

# Clone main project for deliverybot
RUN git clone https://github.com/AbhiKhoyani/deliverybot.git src/deliverybot

# Clone multi lidar calibration project for extrinsic calibration
RUN git clone https://github.com/AbhiKhoyani/multi_lidar_calibration_ros2.git src/multi_lidar_calibration_ros2

# Building RTABMAP from source to include multiple camera
RUN git clone https://github.com/introlab/rtabmap.git src/rtabmap
RUN git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros \
    && rosdep update && rosdep install --from-paths src --ignore-src -r -y  \
    && export MAKEFLAGS="-j6"   \
    && colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release

WORKDIR /home/ROS2