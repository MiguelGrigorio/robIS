from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robis_description_share = FindPackageShare("robis_description")
    robis_gazebo_share = FindPackageShare("robis_gazebo")

    default_model = PathJoinSubstitution([robis_description_share, "urdf", "robis.urdf.xacro"])
    default_world = PathJoinSubstitution([robis_gazebo_share, "worlds", "turtlebot3_like.world"])
    default_bridge_config = PathJoinSubstitution([robis_gazebo_share, "config", "bridge_topics.yaml"])

    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    bridge_config = LaunchConfiguration("bridge_config")
    robot_name = LaunchConfiguration("robot_name")
    spawn_z = LaunchConfiguration("spawn_z")
    gz_args = LaunchConfiguration("gz_args")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model,
                " ",
                "sim_flavor:=gz",
            ]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robis_gazebo_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world, "gz_args": gz_args}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("/tf", "/demo/tf"),
            ("/tf_static", "/demo/tf_static"),
            ("/joint_states", "/demo/joint_states"),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_config,
                "use_sim_time": True,
            }
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-z",
            spawn_z,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        remappings=[
            ("/tf", "/demo/tf"),
            ("/tf_static", "/demo/tf_static"),
        ],
        arguments=[
            "0.22",
            "0",
            "0.275",
            "0",
            "0",
            "0",
            "base_footprint",
            "robis_d2x/base_footprint/ydlidar",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("bridge_config", default_value=default_bridge_config),
            DeclareLaunchArgument("robot_name", default_value="robis_d2x"),
            DeclareLaunchArgument("spawn_z", default_value="0.20"),
            DeclareLaunchArgument("gz_args", default_value="-r -v 4"),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    PathJoinSubstitution([robis_description_share, ".."]),
                    ":",
                    PathJoinSubstitution([robis_gazebo_share, ".."]),
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            gazebo,
            robot_state_publisher,
            bridge,
            static_tf,
            TimerAction(period=2.0, actions=[spawn_robot]),
        ]
    )
