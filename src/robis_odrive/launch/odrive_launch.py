from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    simulation_mode = LaunchConfiguration("simulation_mode")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    simulation_mode_bool = ParameterValue(simulation_mode, value_type=bool)
    publish_odom_tf_bool = ParameterValue(publish_odom_tf, value_type=bool)

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_mode", default_value="false"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="odrive/odom"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            Node(
                package="robis_odrive",
                executable="odrive_node",
                name="odrive_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "simulation_mode": simulation_mode_bool,
                        "cmd_vel_topic": cmd_vel_topic,
                        "odom_topic": odom_topic,
                        "wheel_track": 0.278,
                        "tyre_circumference": 0.5,
                        "publish_odom_tf": publish_odom_tf_bool,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_broadcaster",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "base_footprint",
                ],
            ),
        ]
    )
