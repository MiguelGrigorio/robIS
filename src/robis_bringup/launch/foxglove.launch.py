"""Optional observation of the ROS graph through the packaged Foxglove bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_bridge(context):
    """Select concrete capabilities after evaluating the optional teleop flag."""
    capabilities = ['connectionGraph', 'assets']
    if IfCondition(LaunchConfiguration('enable_teleop')).evaluate(context):
        capabilities.append('clientPublish')
    return [Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        name=LaunchConfiguration('node_name'), output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robis_bringup'), 'config', 'foxglove.yaml',
            ]),
            {
                'capabilities': capabilities,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool,
                ),
                'address': LaunchConfiguration('address'),
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            },
        ],
    )]


def generate_launch_description():
    """Launch observation and opt-in manual control."""
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("address", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("node_name", default_value="foxglove_bridge"),
        DeclareLaunchArgument("preview_fps", default_value="30.0"),
        DeclareLaunchArgument("enable_image_preview", default_value="true"),
        DeclareLaunchArgument("enable_teleop", default_value="false"),
        Node(
            package="robis_vision", executable="image_preview",
            name="foxglove_image_preview", output="screen",
            condition=IfCondition(LaunchConfiguration("enable_image_preview")),
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool,
                ),
                "fps": ParameterValue(LaunchConfiguration("preview_fps"), value_type=float),
                "jpeg_quality": 70,
            }],
        ),
        Node(
            package='robis_bringup', executable='foxglove_teleop.py',
            output='screen', condition=IfCondition(LaunchConfiguration('enable_teleop')),
        ),
        OpaqueFunction(function=launch_bridge),
    ])
