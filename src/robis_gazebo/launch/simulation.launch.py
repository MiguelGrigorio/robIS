from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
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
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    gz_args = LaunchConfiguration("gz_args")
    enable_ground_truth_sensors = LaunchConfiguration("enable_ground_truth_sensors")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model,
                " ",
                "sim_flavor:=gz",
                " ",
                "enable_ground_truth_sensors:=",
                enable_ground_truth_sensors,
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

    # Keep the command bridge explicit. In ros_gz_bridge Jazzy the YAML-loaded
    # ROS_TO_GZ Twist rule can create a ROS subscription without forwarding the
    # samples to Gazebo, while the typed command-line bridge is reliable.
    cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="cmd_vel_bridge",
        output="screen",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
    )

    ground_truth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ground_truth_bridge",
        output="screen",
        parameters=[{
            "config_file": PathJoinSubstitution([
                robis_gazebo_share, "config", "ground_truth_bridge_topics.yaml",
            ]),
            "use_sim_time": True,
        }],
        condition=IfCondition(enable_ground_truth_sensors),
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
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-Y",
            spawn_yaw,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("bridge_config", default_value=default_bridge_config),
            DeclareLaunchArgument("robot_name", default_value="robis"),
            DeclareLaunchArgument("spawn_z", default_value="0.20"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("gz_args", default_value="-r -v 4"),
            DeclareLaunchArgument("enable_ground_truth_sensors", default_value="false"),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    PathJoinSubstitution([robis_description_share, ".."]),
                    ":",
                    PathJoinSubstitution([robis_gazebo_share, ".."]),
                    ":",
                    PathJoinSubstitution([robis_gazebo_share, "models"]),
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            gazebo,
            robot_state_publisher,
            bridge,
            cmd_vel_bridge,
            ground_truth_bridge,
            # Gazebo needs time to advertise robot_description/create services,
            # especially over SSH X11 and on the first GPU context creation.
            TimerAction(period=5.0, actions=[spawn_robot]),
        ]
    )
