cmake_minimum_required(VERSION 3.8)
project(ur3e_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(ur3e_controller src/ur3e_controller.cpp)
ament_target_dependencies(ur3e_controller
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(ur3e_trac_ik_controller src/ur3e_trac_ik_controller.cpp)
ament_target_dependencies(ur3e_trac_ik_controller
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  trac_ik_lib
  orocos_kdl
  kdl_parser
  controller_manager_msgs
)

add_executable(ur3e_printer src/ur3e_printer.cpp)
ament_target_dependencies(ur3e_printer
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  shape_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  controller_manager_msgs
)

install(TARGETS ur3e_controller ur3e_trac_ik_controller ur3e_printer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config toolpaths rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
