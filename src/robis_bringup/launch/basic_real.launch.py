"""Hardware-only bringup, independent of Nav2, Gazebo and semantic packages."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose the base, fixed sensor frames, lidar and USB camera."""
    bringup = FindPackageShare("robis_bringup")
    description = FindPackageShare("robis_description")
    return LaunchDescription([
        DeclareLaunchArgument("enable_base", default_value="true"),
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("simulation_mode", default_value="false"),
        DeclareLaunchArgument("enable_foxglove", default_value="false"),
        DeclareLaunchArgument("odrive_params_file", default_value=PathJoinSubstitution(
            [bringup, "config", "odrive_real.yaml"])),
        DeclareLaunchArgument("lidar_params_file", default_value=PathJoinSubstitution(
            [bringup, "config", "ydlidar_real.yaml"])),
        DeclareLaunchArgument("camera_params_file", default_value=PathJoinSubstitution(
            [bringup, "config", "usb_cam_basic.yaml"])),
        DeclareLaunchArgument("camera_info_url", default_value=""),
        DeclareLaunchArgument("model", default_value=PathJoinSubstitution(
            [description, "urdf", "robis.urdf.xacro"])),
        DeclareLaunchArgument("description_params_file", default_value=PathJoinSubstitution(
            [description, "config", "robot_parameters.yaml"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup, "launch", "foxglove.launch.py",
            ])),
            launch_arguments={"use_sim_time": "false"}.items(),
            condition=IfCondition(LaunchConfiguration("enable_foxglove")),
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{
                "use_sim_time": False,
                "robot_description": ParameterValue(Command([
                    FindExecutable(name="xacro"), " ",
                    LaunchConfiguration("model"),
                    " sim_flavor:=none params_file:=",
                    LaunchConfiguration("description_params_file"),
                ]), value_type=str),
            }],
            output="screen",
        ),
        Node(
            package="robis_odrive", executable="odrive_node", name="odrive_node",
            parameters=[LaunchConfiguration("odrive_params_file"), {
                "simulation_mode": ParameterValue(
                    LaunchConfiguration("simulation_mode"), value_type=bool),
            }],
            condition=IfCondition(LaunchConfiguration("enable_base")), output="screen",
        ),
        # A normal rclcpp node; URDF is the only publisher of lidar_link's fixed TF.
        Node(
            package="robis_ydlidar", executable="robis_ydlidar_node",
            name="robis_ydlidar_node",
            parameters=[LaunchConfiguration("lidar_params_file")],
            condition=IfCondition(LaunchConfiguration("enable_lidar")), output="screen",
        ),
        Node(
            package="robis_vision", executable="usb_camera_node", name="usb_camera_node",
            parameters=[LaunchConfiguration("camera_params_file"), {
                "camera_info_url": LaunchConfiguration("camera_info_url"),
            }],
            condition=IfCondition(LaunchConfiguration("enable_camera")), output="screen",
        ),
    ])
