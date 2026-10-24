"""Complete Gazebo, Nav2 and semantic-navigation extras/research stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _mode_is(mode, expected):
    return IfCondition(PythonExpression(["'", mode, "' == '", expected, "'"]))


def _include(package_share, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", launch_file])
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    gazebo_share = FindPackageShare("robis_gazebo")
    navigation_share = FindPackageShare("robis_navigation")
    semantic_share = FindPackageShare("robis_semantic_navigation")

    mode = LaunchConfiguration("mode")
    map_file = LaunchConfiguration("map")
    nav_params = LaunchConfiguration("nav_params_file")
    mapping_nav_params = LaunchConfiguration("mapping_nav_params_file")
    slam_params = LaunchConfiguration("slam_params_file")
    detector_backend = LaunchConfiguration("detector_backend")
    detector_device = LaunchConfiguration("detector_device")
    detector_model = LaunchConfiguration("detector_model")
    open_vocab_model = LaunchConfiguration("open_vocab_model")
    enable_ollama = LaunchConfiguration("enable_ollama")
    semantic_map = LaunchConfiguration("semantic_map")
    map_checksum = LaunchConfiguration("map_checksum")
    semantic_params = LaunchConfiguration("semantic_params_file")
    range_estimator = LaunchConfiguration("range_estimator")
    use_depth_hint = LaunchConfiguration("use_depth_hint")
    enable_localization_evaluator = LaunchConfiguration(
        "enable_localization_evaluator"
    )
    enable_reference_oracle = LaunchConfiguration("enable_reference_oracle")
    enable_detector_evaluator = LaunchConfiguration("enable_detector_evaluator")
    ground_truth_file = LaunchConfiguration("ground_truth_file")
    localization_output_csv = LaunchConfiguration("localization_output_csv")
    experiment_environment = LaunchConfiguration("experiment_environment")
    trial_id = LaunchConfiguration("trial_id")
    rviz = LaunchConfiguration("rviz")
    rviz_start_delay = LaunchConfiguration("rviz_start_delay")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_dataset_recorder = LaunchConfiguration("enable_dataset_recorder")

    simulation = _include(
        gazebo_share,
        "simulation.launch.py",
        {
            "world": LaunchConfiguration("world"),
            "gz_args": LaunchConfiguration("gz_args"),
            "enable_ground_truth_sensors": LaunchConfiguration("enable_ground_truth_sensors"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
        },
    )

    mapping = GroupAction(
        condition=_mode_is(mode, "mapping"),
        actions=[
            _include(
                navigation_share,
                "slam.launch.py",
                {"params_file": slam_params, "use_sim_time": "true", "rviz": "false"},
            ),
            TimerAction(
                period=LaunchConfiguration("nav_start_delay"),
                actions=[
                    _include(
                        navigation_share,
                        "navigation.launch.py",
                        {
                            "params_file": mapping_nav_params,
                            "use_sim_time": "true",
                            "rviz": "false",
                            "rviz_start_delay": rviz_start_delay,
                        },
                    )
                ],
            ),
        ],
    )
    navigation = GroupAction(
        condition=_mode_is(mode, "navigation"),
        actions=[
            _include(
                navigation_share,
                "localization.launch.py",
                {
                    "map": map_file,
                    "params_file": nav_params,
                    "use_sim_time": "true",
                    "rviz": "false",
                },
            ),
            _include(
                navigation_share,
                "navigation.launch.py",
                {
                    "params_file": nav_params,
                    "use_sim_time": "true",
                    "rviz": "false",
                    "rviz_start_delay": rviz_start_delay,
                },
            ),
        ],
    )
    semantics = _include(
        semantic_share,
        "semantic_navigation.launch.py",
        {
            "detector_backend": detector_backend,
            "detector_device": detector_device,
            "detector_model": detector_model,
            "detector_image_size": LaunchConfiguration("detector_image_size"),
            "detector_inference_hz": LaunchConfiguration("detector_inference_hz"),
            "detector_cpu_threads": LaunchConfiguration("detector_cpu_threads"),
            "open_vocab_model": open_vocab_model,
            "enable_ollama": enable_ollama,
            "semantic_map": semantic_map,
            "map_checksum": map_checksum,
            **{name: LaunchConfiguration(name) for name in [
                "enable_external_memory", "external_environment_id",
                "external_max_age_sec", "external_max_sensitivity_m",
            ]},
            "params_file": semantic_params,
            "range_estimator": range_estimator,
            "mask_pixel_tolerance_px": LaunchConfiguration("mask_pixel_tolerance_px"),
            "localization_frame": LaunchConfiguration("localization_frame"),
            "use_depth_hint": use_depth_hint,
            "enable_localization_evaluator": enable_localization_evaluator,
            "enable_reference_oracle": enable_reference_oracle,
            "enable_detector_evaluator": enable_detector_evaluator,
            "detector_evaluation_output_csv": LaunchConfiguration(
                "detector_evaluation_output_csv"
            ),
            "ground_truth_file": ground_truth_file,
            "localization_output_csv": localization_output_csv,
            "environment": experiment_environment,
            "trial_id": trial_id,
            "world_to_map_origin_x": LaunchConfiguration("spawn_x"),
            "world_to_map_origin_y": LaunchConfiguration("spawn_y"),
            "world_to_map_origin_yaw": LaunchConfiguration("spawn_yaw"),
            "use_sim_time": "true",
        },
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="semantic_rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )
    dataset_recorder = Node(
        package="robis_vision",
        executable="semantic_dataset_recorder",
        name="semantic_dataset_recorder",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "output_directory": LaunchConfiguration("dataset_output_directory"),
            "maximum_frames": LaunchConfiguration("dataset_maximum_frames"),
            "frame_stride": LaunchConfiguration("dataset_frame_stride"),
            "save_depth": LaunchConfiguration("dataset_save_depth"),
        }],
        condition=IfCondition(enable_dataset_recorder),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="mapping",
                description="mapping (SLAM) or navigation (saved map + AMCL)",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [gazebo_share, "worlds", "semantic_lab.world"]
                ),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [navigation_share, "maps", "blank_map.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "nav_params_file",
                default_value=PathJoinSubstitution(
                    [navigation_share, "config", "nav2_params_demo.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "mapping_nav_params_file",
                default_value=PathJoinSubstitution(
                    [navigation_share, "config", "nav2_params_demo.yaml"]
                ),
                description="Nav2 profile consuming the map updated by SLAM",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [navigation_share, "config", "slam_toolbox_demo.yaml"]
                ),
            ),
            DeclareLaunchArgument("detector_backend", default_value="oracle"),
            DeclareLaunchArgument("detector_device", default_value="auto"),
            DeclareLaunchArgument("detector_model", default_value="yolo26n.pt"),
            DeclareLaunchArgument("detector_image_size", default_value="640"),
            DeclareLaunchArgument("detector_inference_hz", default_value="5.0"),
            DeclareLaunchArgument("detector_cpu_threads", default_value="2"),
            DeclareLaunchArgument("open_vocab_model", default_value=""),
            DeclareLaunchArgument("enable_ollama", default_value="false"),
            DeclareLaunchArgument("range_estimator", default_value="bbox_cluster"),
            DeclareLaunchArgument("mask_pixel_tolerance_px", default_value="0"),
            DeclareLaunchArgument("localization_frame", default_value="map"),
            DeclareLaunchArgument("use_depth_hint", default_value="false"),
            DeclareLaunchArgument(
                "enable_localization_evaluator", default_value="false"
            ),
            DeclareLaunchArgument("enable_reference_oracle", default_value="false"),
            DeclareLaunchArgument("enable_detector_evaluator", default_value="false"),
            DeclareLaunchArgument(
                "detector_evaluation_output_csv",
                default_value="/tmp/robis_localization/detector_samples.csv",
            ),
            DeclareLaunchArgument("ground_truth_file", default_value=""),
            DeclareLaunchArgument(
                "localization_output_csv",
                default_value="/tmp/robis_localization/localization_samples.csv",
            ),
            DeclareLaunchArgument(
                "experiment_environment", default_value="simulation"
            ),
            DeclareLaunchArgument("trial_id", default_value="trial_000"),
            DeclareLaunchArgument(
                "semantic_map", default_value="/robis_ws/data/environments/semantic_map.yaml"
            ),
            DeclareLaunchArgument("map_checksum", default_value=""),
        DeclareLaunchArgument("enable_external_memory", default_value="false"),
        DeclareLaunchArgument("external_environment_id", default_value=""),
        DeclareLaunchArgument("external_max_age_sec", default_value="3.0"),
        DeclareLaunchArgument("external_max_sensitivity_m", default_value="0.6"),
            DeclareLaunchArgument(
                "semantic_params_file",
                default_value=PathJoinSubstitution(
                    [semantic_share, "config", "semantic_navigation.yaml"]
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_start_delay", default_value="8.0"),
            DeclareLaunchArgument(
                "nav_start_delay",
                default_value="12.0",
                description="Delay Nav2 in mapping mode until SLAM has published its first map",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [navigation_share, "rviz", "navigation.rviz"]
                ),
            ),
            DeclareLaunchArgument("gz_args", default_value="-r -v 3"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("enable_dataset_recorder", default_value="false"),
            DeclareLaunchArgument(
                "dataset_output_directory",
                default_value="/robis_ws/data/datasets/semantic_lab",
            ),
            DeclareLaunchArgument("dataset_maximum_frames", default_value="300"),
            DeclareLaunchArgument("dataset_frame_stride", default_value="2"),
            DeclareLaunchArgument("dataset_save_depth", default_value="true"),
            DeclareLaunchArgument(
                "enable_ground_truth_sensors",
                default_value=PythonExpression([
                    "'", detector_backend, "' == 'oracle' or '",
                    enable_reference_oracle, "' == 'true' or '",
                    enable_detector_evaluator, "' == 'true' or '",
                    enable_dataset_recorder, "' == 'true'",
                ]),
                description="Enable extras/research cameras only for oracle/evaluation/dataset runs",
            ),
            simulation,
            mapping,
            navigation,
            semantics,
            TimerAction(period=rviz_start_delay, actions=[rviz_node]),
            dataset_recorder,
        ]
    )
