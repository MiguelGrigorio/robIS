"""Common physical drivers, with optional SLAM, localization and Nav2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def include(package, filename, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), "launch", filename,
        ])), launch_arguments=arguments.items(), condition=condition,
    )


def stack_in(*values):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration("stack"), "' in ", repr(values),
    ]))


def generate_launch_description():
    bringup = FindPackageShare("robis_bringup")
    navigation = FindPackageShare("robis_navigation")
    description = FindPackageShare("robis_description")
    config = lambda package, name: PathJoinSubstitution([package, "config", name])
    arguments = [
        DeclareLaunchArgument("stack", default_value="none", choices=[
            "none", "slam", "mapping", "localization", "navigation",
        ]),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("nav_rviz", default_value="false"),
        DeclareLaunchArgument("nav_start_delay", default_value="5.0"),
        DeclareLaunchArgument("nav_params_file", default_value=config(navigation, "nav2_params.yaml")),
        DeclareLaunchArgument("slam_params_file", default_value=config(navigation, "slam_toolbox.yaml")),
        DeclareLaunchArgument("model", default_value=PathJoinSubstitution([
            description, "urdf", "robis.urdf.xacro",
        ])),
        DeclareLaunchArgument("description_params_file", default_value=config(description, "robot_parameters.yaml")),
        DeclareLaunchArgument("odrive_params_file", default_value=config(bringup, "odrive_real.yaml")),
        DeclareLaunchArgument("lidar_params_file", default_value=config(bringup, "ydlidar_real.yaml")),
        DeclareLaunchArgument("camera_params_file", default_value=config(bringup, "usb_cam_basic.yaml")),
        DeclareLaunchArgument("camera_info_url", default_value=""),
    ]
    arguments += [DeclareLaunchArgument(name, default_value=default) for name, default in [
        ("enable_base", "true"), ("enable_lidar", "true"),
        ("enable_camera", "true"), ("enable_foxglove", "false"),
    ]]
    hardware_arguments = {name: LaunchConfiguration(name) for name in [
        "enable_base", "enable_lidar", "enable_camera", "enable_foxglove",
        "model", "description_params_file", "odrive_params_file", "lidar_params_file",
        "camera_params_file", "camera_info_url",
    ]}
    navigation_arguments = {
        "params_file": LaunchConfiguration("nav_params_file"),
        "use_sim_time": "false", "rviz": LaunchConfiguration("nav_rviz"),
    }
    return LaunchDescription(arguments + [
        include("robis_bringup", "basic_real.launch.py", hardware_arguments),
        include("robis_navigation", "slam.launch.py", {
            "params_file": LaunchConfiguration("slam_params_file"),
            "use_sim_time": "false", "rviz": "false",
        }, stack_in("slam", "mapping")),
        include("robis_navigation", "localization.launch.py", {
            **navigation_arguments, "map": LaunchConfiguration("map"), "rviz": "false",
        }, stack_in("localization", "navigation")),
        TimerAction(period=LaunchConfiguration("nav_start_delay"), actions=[
            include("robis_navigation", "navigation.launch.py", navigation_arguments,
                    stack_in("mapping", "navigation")),
        ]),
    ])
