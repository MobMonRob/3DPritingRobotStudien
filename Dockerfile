# ROS 2 Humble Dockerfile with MoveIt 2 and UR driver
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8

# Install essential C++ and ROS 2 tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    python3-vcstool \
    bash-completion \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install MoveIt 2 + UR driver
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-ros-planning \
    ros-$ROS_DISTRO-moveit-ros-planning-interface \
    ros-$ROS_DISTRO-moveit-ros-visualization \
    ros-$ROS_DISTRO-moveit-ros-move-group \
    ros-$ROS_DISTRO-moveit-ros-perception \
    ros-$ROS_DISTRO-moveit-resources-panda-description \
    ros-$ROS_DISTRO-ur \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep
RUN rosdep update

# Create workspace
WORKDIR /ros_ws
RUN mkdir -p src

# Automatically source ROS 2 and workspace on container start
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Start interactive bash
CMD ["bash"]
