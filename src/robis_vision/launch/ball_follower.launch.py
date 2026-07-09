from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robis_vision_share = FindPackageShare("robis_vision")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [robis_vision_share, "config", "ball_follower.yaml"]
                ),
            ),
            Node(
                package="robis_vision",
                executable="ball_follower",
                name="ball_follower",
                output="screen",
                emulate_tty=True,
                parameters=[params_file],
            ),
        ]
    )
