from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robis_gazebo_share = FindPackageShare("robis_gazebo")
    robis_vision_share = FindPackageShare("robis_vision")

    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    spawn_z = LaunchConfiguration("spawn_z")
    gz_args = LaunchConfiguration("gz_args")
    params_file = LaunchConfiguration("params_file")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robis_gazebo_share, "launch", "simulation.launch.py"])
        ),
        launch_arguments={
            "world": world,
            "robot_name": robot_name,
            "spawn_z": spawn_z,
            "gz_args": gz_args,
        }.items(),
    )

    ball_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robis_vision_share, "launch", "ball_follower.launch.py"])
        ),
        launch_arguments={"params_file": params_file}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [robis_gazebo_share, "worlds", "turtlebot3_like.world"]
                ),
            ),
            DeclareLaunchArgument("robot_name", default_value="robis_d2x"),
            DeclareLaunchArgument("spawn_z", default_value="0.20"),
            DeclareLaunchArgument("gz_args", default_value="-r -v 4"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [robis_vision_share, "config", "ball_follower_demo.yaml"]
                ),
            ),
            simulation,
            TimerAction(period=4.0, actions=[ball_follower]),
        ]
    )
