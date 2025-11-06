# Base: Humble on Ubuntu 22.04 (ARM64 variant will be pulled automatically on Pi 5)
FROM ros:humble-ros-base

# Locale & basics
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales ca-certificates gnupg curl \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Common dev tools for ROS workspaces
RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-pip python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Zenoh RMW for ROS 2 Humble
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Control stack (provides hardware_interface)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Joy node for joystick input
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*

# Optional helpful Python tools
RUN pip3 install --no-cache-dir vcstool

# Python dependencies for kiwi_robot package
RUN pip3 install --no-cache-dir pyserial opencv-python-headless numpy feetech-servo-sdk

# Default to Zenoh; you can override at runtime with -e RMW_IMPLEMENTATION=...
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Nice-to-have: auto-source ROS in interactive shells
RUN bash -lc "echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc"

# Default working directory inside container
WORKDIR /root/ros2_ws

# Keep it interactive by default
CMD ["/bin/bash"]
