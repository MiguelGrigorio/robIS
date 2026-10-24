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
            DeclareLaunchArgument("connect_on_startup", default_value="true"),
            DeclareLaunchArgument("engage_on_startup", default_value="false"),
            DeclareLaunchArgument("odometry_mode", default_value="encoder"),
            DeclareLaunchArgument("allow_angular_motion", default_value="false"),
            DeclareLaunchArgument("safety_profile", default_value=""),
            DeclareLaunchArgument("axis_for_right", default_value="0"),
            DeclareLaunchArgument("encoder_yaw_sign", default_value="1.0"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
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
                        "connect_on_startup": ParameterValue(LaunchConfiguration("connect_on_startup"), value_type=bool),
                        "engage_on_startup": ParameterValue(LaunchConfiguration("engage_on_startup"), value_type=bool),
                        "odometry_mode": LaunchConfiguration("odometry_mode"),
                        "allow_angular_motion": ParameterValue(LaunchConfiguration("allow_angular_motion"), value_type=bool),
                        "safety_profile": LaunchConfiguration("safety_profile"),
                        "axis_for_right": ParameterValue(LaunchConfiguration("axis_for_right"), value_type=int),
                        "encoder_yaw_sign": ParameterValue(LaunchConfiguration("encoder_yaw_sign"), value_type=float),
                        "publish_odom_tf": publish_odom_tf_bool,
                    }
                ],
            ),
        ]
    )
