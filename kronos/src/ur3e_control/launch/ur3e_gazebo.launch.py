"""Launch UR3e with fake (simulated) hardware via ur_robot_driver + RViz."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            # Launch the UR driver in simulated (fake) hardware mode
            IncludeLaunchDescription(
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
                    "launch_rviz": "true",
                    "robot_ip": "xxx.xxx.xxx.xxx",
                }.items(),
            ),
        ]
    )
