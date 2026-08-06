from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace


def _stack_condition(stack, value):
    return IfCondition(PythonExpression(["'", stack, "' == '", value, "'"]))


def generate_launch_description():
    robis_gazebo_share = FindPackageShare("robis_gazebo")
    robis_navigation_share = FindPackageShare("robis_navigation")

    stack = LaunchConfiguration("stack")
    map_file = LaunchConfiguration("map")
    nav_params_file = LaunchConfiguration("nav_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav_rviz = LaunchConfiguration("nav_rviz")
    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    spawn_z = LaunchConfiguration("spawn_z")
    gz_args = LaunchConfiguration("gz_args")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robis_gazebo_share, "launch", "simulation.launch.py"])
        ),
        launch_arguments={
            "world": world,
            "robot_name": robot_name,
            "spawn_z": spawn_z,
            "gz_args": gz_args,
        }.items(),
    )

    slam = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([robis_navigation_share, "launch", "slam.launch.py"])
                ),
                launch_arguments={
                    "params_file": slam_params_file,
                    "use_sim_time": "true",
                    "rviz": nav_rviz,
                }.items(),
            ),
        ],
        condition=_stack_condition(stack, "slam"),
    )
    localization = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([robis_navigation_share, "launch", "localization.launch.py"])
                ),
                launch_arguments={
                    "map": map_file,
                    "params_file": nav_params_file,
                    "use_sim_time": "true",
                    "rviz": nav_rviz,
                }.items(),
            ),
        ],
        condition=_stack_condition(stack, "localization"),
    )

    navigation = [
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [robis_navigation_share, "launch", "localization.launch.py"]
                        )
                    ),
                    launch_arguments={
                        "map": map_file,
                        "params_file": nav_params_file,
                        "use_sim_time": "true",
                        "rviz": "false",
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [robis_navigation_share, "launch", "navigation.launch.py"]
                        )
                    ),
                    launch_arguments={
                        "map": map_file,
                        "params_file": nav_params_file,
                        "use_sim_time": "true",
                        "rviz": nav_rviz,
                    }.items(),
                ),
            ],
            condition=_stack_condition(stack, "navigation"),
        )
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("stack", default_value="slam"),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [robis_navigation_share, "maps", "blank_map.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "nav_params_file",
                default_value=PathJoinSubstitution(
                    [robis_navigation_share, "config", "nav2_params_demo.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [robis_navigation_share, "config", "slam_toolbox_demo.yaml"]
                ),
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [robis_gazebo_share, "worlds", "turtlebot3_like.world"]
                ),
            ),
            DeclareLaunchArgument("robot_name", default_value="robis"),
            DeclareLaunchArgument("spawn_z", default_value="0.20"),
            DeclareLaunchArgument("gz_args", default_value="-r -v 4"),
            DeclareLaunchArgument("nav_rviz", default_value="true"),
            simulation,
            slam,
            localization,
            *navigation,
        ]
    )
