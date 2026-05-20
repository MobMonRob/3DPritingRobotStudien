"""
Launch the ur3e_controller node with the MoveIt parameters it needs.

Usage (inside the container, with the simulation already running):
  ros2 launch ur3e_control ur3e_controller.launch.py
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def load_yaml(package_name, file_path):
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(full_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    ur_moveit_config = get_package_share_directory("ur_moveit_config")
    ur_description = get_package_share_directory("ur_description")

    # Process URDF xacro
    robot_description_content = xacro.process_file(
        os.path.join(ur_description, "urdf", "ur.urdf.xacro"),
        mappings={"name": "ur", "ur_type": "ur3e"},
    ).toxml()
    robot_description = {"robot_description": robot_description_content}

    # Process SRDF xacro
    robot_description_semantic_content = xacro.process_file(
        os.path.join(ur_moveit_config, "srdf", "ur.srdf.xacro"),
        mappings={"name": "ur"},
    ).toxml()
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Kinematics
    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = kinematics_yaml.get("/**", {}).get(
        "ros__parameters", {}
    )

    controller_node = Node(
        package="ur3e_control",
        executable="ur3e_controller",
        name="ur3e_controller",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            controller_node,
        ]
    )
