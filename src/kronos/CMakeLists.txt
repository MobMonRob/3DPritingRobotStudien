# This CMakeLists.txt is configured for the kronos package.

cmake_minimum_required(VERSION 3.10)
project(kronos)

# Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# The moveit_ros_planning_interface includes all necessary MoveIt components, 
# but we explicitly include MoveIt's core components for clarity and reliability.
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Compiler Configuration ---
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Set compiler flags for C++ standard, warnings, and optimization
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -O3)
endif()

# --- Executable Definition ---
# Add the source file to be compiled into an executable
add_executable(mover_node src/Kronos.cpp)

# --- Link Libraries and Include Directories ---
# Link the executable against its dependencies
ament_target_dependencies(mover_node
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
)

# --- Installation ---
# Install the executable so it can be run with 'ros2 run'
install(TARGETS
  mover_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the package.xml for the ROS 2 environment
ament_package()