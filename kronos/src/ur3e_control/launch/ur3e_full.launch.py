"""
Launch the UR3e simulation: fake hardware driver + MoveIt2 + RViz.

Usage (inside the container):
  ros2 launch ur3e_control ur3e_full.launch.py

Then, in a separate terminal, run the controller (must use launch to load kinematics):
  ros2 launch ur3e_control ur3e_controller.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")

    # 1. UR driver with fake (simulated) hardware
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver"),
                    "launch",
                    "ur_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_fake_hardware": "true",
            "launch_rviz": "false",
            "robot_ip": "xxx.xxx.xxx.xxx",
            "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
    )

    # 2. MoveIt2 + RViz  (delayed to let the driver spin up)
    moveit_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur_moveit_config"),
                            "launch",
                            "ur_moveit.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "launch_rviz": "true",
                    "use_sim_time": "true",
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            driver_launch,
            moveit_launch,
        ]
    )
