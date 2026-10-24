from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    robis_navigation_share = FindPackageShare("robis_navigation")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    # Rewrite per-node values as well as the parent launch's clock.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    # Own the three nodes used by robIS; nav2_bringup also pulls robot simulators.
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    localization = [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=[configured_params, {"yaml_filename": map_file}],
             remappings=remappings),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             parameters=[configured_params], remappings=remappings),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen", parameters=[{
                 "use_sim_time": use_sim_time, "autostart": autostart,
                 "node_names": ["map_server", "amcl"],
             }]),
    ]

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
                "map",
                default_value=PathJoinSubstitution(
                    [robis_navigation_share, "maps", "blank_map.yaml"]
                ),
            ),
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
            *localization,
            rviz_node,
        ]
    )
