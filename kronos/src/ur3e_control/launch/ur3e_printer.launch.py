"""
Launch the UR3e 3D-printer node.

Usage (simulation must be running):
  ros2 launch ur3e_control ur3e_printer.launch.py \
      toolpath_file:=/ros2_ws/src/ur3e_control/toolpaths/demo_square.csv
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    ur3e_control = get_package_share_directory("ur3e_control")

    # URDF
    robot_description_content = xacro.process_file(
        os.path.join(ur_description, "urdf", "ur.urdf.xacro"),
        mappings={"name": "ur", "ur_type": "ur3e"},
    ).toxml()
    robot_description = {"robot_description": robot_description_content}

    # SRDF
    robot_description_semantic_content = xacro.process_file(
        os.path.join(ur_moveit_config, "srdf", "ur.srdf.xacro"),
        mappings={"name": "ur"},
    ).toxml()
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Kinematics — use our TRAC-IK override
    kinematics_yaml = load_yaml("ur3e_control", "config/kinematics.yaml")
    robot_description_kinematics = kinematics_yaml.get("/**", {}).get(
        "ros__parameters", {}
    )

    toolpath_file = LaunchConfiguration("toolpath_file")

    printer_node = Node(
        package="ur3e_control",
        executable="ur3e_printer",
        name="ur3e_printer",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"toolpath_file": toolpath_file},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            DeclareLaunchArgument(
                "toolpath_file",
                default_value="",
                description="Path to CSV or G-code toolpath file",
            ),
            printer_node,
        ]
    )
