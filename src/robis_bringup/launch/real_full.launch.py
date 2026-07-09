from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _stack_condition(stack, value):
    return IfCondition(PythonExpression(["'", stack, "' == '", value, "'"]))


def generate_launch_description():
    robis_bringup_share = FindPackageShare("robis_bringup")
    robis_description_share = FindPackageShare("robis_description")
    robis_navigation_share = FindPackageShare("robis_navigation")

    model = LaunchConfiguration("model")
    description_params_file = LaunchConfiguration("description_params_file")
    odrive_params_file = LaunchConfiguration("odrive_params_file")
    lidar_params_file = LaunchConfiguration("lidar_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    enable_base = LaunchConfiguration("enable_base")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_camera = LaunchConfiguration("enable_camera")
    stack = LaunchConfiguration("stack")
    map_file = LaunchConfiguration("map")
    nav_params_file = LaunchConfiguration("nav_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav_rviz = LaunchConfiguration("nav_rviz")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model,
                " ",
                "params_file:=",
                description_params_file,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    odrive = Node(
        package="robis_odrive",
        executable="odrive_node",
        name="odrive_node",
        output="screen",
        emulate_tty=True,
        parameters=[odrive_params_file],
        condition=IfCondition(enable_base),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robis_ydlidar"), "launch", "ydlidar_launch.py"]
            )
        ),
        launch_arguments={"params_file": lidar_params_file}.items(),
        condition=IfCondition(enable_lidar),
    )

    camera = Node(
        package="robis_vision",
        executable="usb_camera_node",
        name="usb_camera_node",
        output="screen",
        emulate_tty=True,
        parameters=[camera_params_file, {"image_topic": "/camera/image_raw"}],
        condition=IfCondition(enable_camera),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robis_navigation_share, "launch", "slam.launch.py"])
        ),
        launch_arguments={
            "params_file": slam_params_file,
            "use_sim_time": "false",
            "rviz": nav_rviz,
        }.items(),
        condition=_stack_condition(stack, "slam"),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robis_navigation_share, "launch", "localization.launch.py"])
        ),
        launch_arguments={
            "map": map_file,
            "params_file": nav_params_file,
            "use_sim_time": "false",
            "rviz": nav_rviz,
        }.items(),
        condition=_stack_condition(stack, "localization"),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robis_navigation_share, "launch", "navigation.launch.py"])
        ),
        launch_arguments={
            "map": map_file,
            "params_file": nav_params_file,
            "use_sim_time": "false",
            "rviz": nav_rviz,
        }.items(),
        condition=_stack_condition(stack, "navigation"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=PathJoinSubstitution(
                    [robis_description_share, "urdf", "robis.urdf.xacro"]
                ),
            ),
            DeclareLaunchArgument(
                "description_params_file",
                default_value=PathJoinSubstitution(
                    [robis_description_share, "config", "robot_parameters.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "odrive_params_file",
                default_value=PathJoinSubstitution(
                    [robis_bringup_share, "config", "odrive_real.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "lidar_params_file",
                default_value=PathJoinSubstitution(
                    [robis_bringup_share, "config", "ydlidar_real.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=PathJoinSubstitution(
                    [robis_bringup_share, "config", "usb_cam_real.yaml"]
                ),
            ),
            DeclareLaunchArgument("enable_base", default_value="true"),
            DeclareLaunchArgument("enable_lidar", default_value="true"),
            DeclareLaunchArgument("enable_camera", default_value="false"),
            DeclareLaunchArgument("stack", default_value="none"),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [robis_navigation_share, "maps", "blank_map.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "nav_params_file",
                default_value=PathJoinSubstitution(
                    [robis_navigation_share, "config", "nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [robis_navigation_share, "config", "slam_toolbox.yaml"]
                ),
            ),
            DeclareLaunchArgument("nav_rviz", default_value="true"),
            robot_state_publisher,
            odrive,
            lidar,
            camera,
            slam,
            localization,
            navigation,
        ]
    )
