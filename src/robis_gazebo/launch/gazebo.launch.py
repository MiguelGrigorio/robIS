from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")
    robis_gazebo_share = FindPackageShare("robis_gazebo")

    default_world = PathJoinSubstitution([robis_gazebo_share, "worlds", "turtlebot3_like.world"])

    world = LaunchConfiguration("world")
    gz_args = LaunchConfiguration("gz_args")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gz_args", default_value="-r -v 4"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
                ),
                launch_arguments={"gz_args": [gz_args, " ", world]}.items(),
            ),
        ]
    )
