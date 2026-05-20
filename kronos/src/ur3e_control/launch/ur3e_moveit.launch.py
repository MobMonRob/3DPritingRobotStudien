"""Launch MoveIt2 + RViz for the UR3e, connecting to a running robot or simulation."""

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
            # Use the official ur_moveit_config launch
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
                }.items(),
            ),
        ]
    )
