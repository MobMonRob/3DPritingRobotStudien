# Use official Ubuntu 22.04 image
FROM ubuntu:22.04

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Update and install common packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        sudo \
        curl \
        wget \
        git \
        procps && \
    rm -rf /var/lib/apt/lists/*

# Create a non-root user with sudo privileges
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to the new user
USER $USERNAME

# Base setup
RUN sudo apt update
RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository -y universe
RUN sudo apt install curl -y
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
RUN sudo dpkg -i /tmp/ros2-apt-source.deb
RUN sudo apt update
RUN sudo apt upgrade
RUN sudo ln -fs /usr/share/zoneinfo/Europe/Berlin /etc/localtime
RUN sudo DEBIAN_FRONTEND=noninteractive apt install -y tzdata
RUN sudo apt install ros-humble-desktop -y
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN sudo apt install python3-rosdep -y
RUN sudo rosdep init
RUN rosdep update
RUN sudo apt update
RUN sudo apt dist-upgrade
RUN sudo apt install python3-colcon-common-extensions -y
RUN sudo apt install python3-colcon-mixin -y
RUN mkdir -p /home/$USERNAME/colcon_ws
WORKDIR /home/$USERNAME/colcon_ws
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default
RUN mkdir -p /home/$USERNAME/ws_moveit/src
WORKDIR /home/$USERNAME/ws_moveit/src
RUN git clone -b humble https://github.com/moveit/moveit2_tutorials
RUN sudo apt install -y python3-vcstool
RUN vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
RUN sudo apt remove ros-humble-moveit*

RUN sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
WORKDIR /home/$USERNAME/ws_moveit
RUN sudo apt update && sudo apt install -y ros-humble-ament-cmake ros-humble-ament-cmake-core
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --mixin release"

RUN echo "source /home/$USERNAME/ws_moveit/install/setup.bash" >> /home/$USERNAME/.bashrc
RUN sudo apt-get install ros-humble-ros-gz -y
RUN sudo apt-get update
RUN sudo apt-get install lsb-release gnupg -y
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install -y ignition-fortress


WORKDIR /home/developer/ws_moveit/src
RUN git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
RUN git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git

RUN sudo rosdep update && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

WORKDIR /home/developer/ws_moveit
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --mixin release"

WORKDIR /home/developer/ws_moveit/src
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && ros2 pkg create kronos --build-type ament_cmake --dependencies rclcpp moveit_ros_planning_interface"
RUN rm -rf /home/developer/ws_moveit/src/kronos/*

