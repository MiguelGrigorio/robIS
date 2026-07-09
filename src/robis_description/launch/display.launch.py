from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("robis_description")

    default_model = PathJoinSubstitution([package_share, "urdf", "robis.urdf.xacro"])
    default_params_file = PathJoinSubstitution([package_share, "config", "robot_parameters.yaml"])
    default_rviz_config = PathJoinSubstitution([package_share, "rviz", "robis.rviz"])

    model = LaunchConfiguration("model")
    params_file = LaunchConfiguration("params_file")
    rvizconfig = LaunchConfiguration("rvizconfig")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model,
                " ",
                "params_file:=",
                params_file,
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("rvizconfig", default_value=default_rviz_config),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=UnlessCondition(use_joint_state_publisher_gui),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_joint_state_publisher_gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rvizconfig],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
