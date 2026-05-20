FROM docker.io/osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install core dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# Install UR robot packages, MoveIt2, and Gazebo integration
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ur \
    ros-humble-ur-robot-driver \
    ros-humble-ur-description \
    ros-humble-ur-moveit-config \
    ros-humble-moveit \
    ros-humble-moveit-planners \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-ros-visualization \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-kdl-parser \
    ros-humble-urdf \
    libnlopt-dev \
    libnlopt-cxx-dev \
    liborocos-kdl-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Build TRAC-IK from source (rolling branch, patched for Humble)
RUN mkdir -p /tmp/trac_ik_ws/src && \
    cd /tmp/trac_ik_ws/src && \
    git clone --branch rolling https://bitbucket.org/traclabs/trac_ik.git && \
    sed -i 's|#include <urdf/model.hpp>|#include <urdf/model.h>|g' \
        /tmp/trac_ik_ws/src/trac_ik/trac_ik_lib/src/trac_ik.cpp \
        /tmp/trac_ik_ws/src/trac_ik/trac_ik_kinematics_plugin/src/trac_ik_kinematics_plugin.cpp && \
    cd /tmp/trac_ik_ws && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select trac_ik_lib --cmake-args -DCMAKE_BUILD_TYPE=Release" && \
    cp -r /tmp/trac_ik_ws/install/trac_ik_lib/* /opt/ros/humble/ && \
    rm -rf /tmp/trac_ik_ws

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Source ROS2 and workspace in every interactive shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash 2>/dev/null" >> /root/.bashrc

# Also set PATH so ros2 is found in non-interactive shells (docker exec bash -c)
ENV PATH="/opt/ros/humble/bin:${PATH}"
ENV AMENT_PREFIX_PATH="/ros2_ws/install/ur3e_control:/opt/ros/humble"
ENV COLCON_PREFIX_PATH="/ros2_ws/install"
ENV CMAKE_PREFIX_PATH="/ros2_ws/install/ur3e_control:/opt/ros/humble"
ENV PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/ros2_ws/install/ur3e_control/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/dist-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
ENV LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/ros2_ws/install/ur3e_control/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu"

# Copy the entrypoint
COPY entrypoint.sh /usr/local/bin/ur3e_entrypoint.sh
RUN chmod +x /usr/local/bin/ur3e_entrypoint.sh

# Copy the package source
COPY src/ur3e_control /ros2_ws/src/ur3e_control

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

ENTRYPOINT ["/usr/local/bin/ur3e_entrypoint.sh"]
CMD ["bash"]
