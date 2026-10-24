from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    robis_navigation_share = FindPackageShare("robis_navigation")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    rviz_start_delay = LaunchConfiguration("rviz_start_delay")
    # Rewrite nested costmaps too; a Node-level override only reaches the parent.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    navigation_nodes = [
        Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=[configured_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")] + (
                [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]
                if name == "velocity_smoother" else
                [("cmd_vel", "cmd_vel_nav")]
                if name in ("controller_server", "behavior_server") else []
            ),
        )
        for package, executable, name in [
            ("nav2_controller", "controller_server", "controller_server"),
            ("nav2_smoother", "smoother_server", "smoother_server"),
            ("nav2_planner", "planner_server", "planner_server"),
            ("nav2_behaviors", "behavior_server", "behavior_server"),
            ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
            ("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"),
            ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother"),
        ]
    ]
    navigation_nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": lifecycle_nodes,
                }
            ],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [robis_navigation_share, "config", "nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [robis_navigation_share, "rviz", "navigation.rviz"]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_start_delay", default_value="8.0"),
            *navigation_nodes,
            # Resolve the period while this include's configuration is in scope.
            # A parent TimerAction restores its context before the async wait runs.
            OpaqueFunction(
                function=lambda context: [TimerAction(
                    period=float(rviz_start_delay.perform(context)), actions=[rviz_node],
                )],
                condition=IfCondition(rviz),
            ),
        ]
    )
