from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robis_bringup_share = FindPackageShare("robis_bringup")
    robis_vision_share = FindPackageShare("robis_vision")

    video_device = LaunchConfiguration("video_device")
    params_file = LaunchConfiguration("params_file")
    enable_lidar = LaunchConfiguration("enable_lidar")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robis_bringup_share, "launch", "real_full.launch.py"])
        ),
        launch_arguments={
            "stack": "none",
            "enable_camera": "false",
            "enable_lidar": enable_lidar,
        }.items(),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robis_vision_share, "launch", "usb_camera.launch.py"])
        ),
        launch_arguments={"video_device": video_device}.items(),
    )

    ball_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robis_vision_share, "launch", "ball_follower.launch.py"])
        ),
        launch_arguments={"params_file": params_file}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("enable_lidar", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [robis_vision_share, "config", "ball_follower.yaml"]
                ),
            ),
            robot,
            camera,
            ball_follower,
        ]
    )
