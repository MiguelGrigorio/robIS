"""Perception, localization and persistent semantic-navigation action server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def backend_is(name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration("detector_backend"), "' == '", name, "'"]))


def generate_launch_description():
    share = FindPackageShare("robis_semantic_navigation")
    vision_share = FindPackageShare("robis_vision")
    config = LaunchConfiguration("params_file")
    oracle_config = PathJoinSubstitution([
        vision_share, "config", "oracle_detector.yaml"
    ])
    common = [{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    return LaunchDescription([
        DeclareLaunchArgument("detector_backend", default_value="yolo26"),
        DeclareLaunchArgument("detector_device", default_value="auto"),
        DeclareLaunchArgument("detector_model", default_value="yolo26n.pt"),
        DeclareLaunchArgument("detector_image_size", default_value="640"),
        DeclareLaunchArgument("detector_inference_hz", default_value="5.0"),
        DeclareLaunchArgument("detector_cpu_threads", default_value="2"),
        DeclareLaunchArgument("open_vocab_model", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("localization_frame", default_value="map"),
        DeclareLaunchArgument("enable_ollama", default_value="false"),
        DeclareLaunchArgument("range_estimator", default_value="bbox_cluster"),
        DeclareLaunchArgument("mask_pixel_tolerance_px", default_value="0"),
        DeclareLaunchArgument("use_depth_hint", default_value="true"),
        DeclareLaunchArgument("enable_localization_evaluator", default_value="false"),
        DeclareLaunchArgument("enable_reference_oracle", default_value="false"),
        DeclareLaunchArgument("enable_detector_evaluator", default_value="false"),
        DeclareLaunchArgument(
            "detector_evaluation_output_csv",
            default_value="/tmp/robis_localization/detector_samples.csv",
        ),
        DeclareLaunchArgument("ground_truth_file", default_value=""),
        DeclareLaunchArgument("localization_output_csv", default_value="/tmp/robis_localization/localization_samples.csv"),
        DeclareLaunchArgument("environment", default_value="simulation"),
        DeclareLaunchArgument("trial_id", default_value="trial_000"),
        DeclareLaunchArgument("world_to_map_origin_x", default_value="0.0"),
        DeclareLaunchArgument("world_to_map_origin_y", default_value="0.0"),
        DeclareLaunchArgument("world_to_map_origin_yaw", default_value="0.0"),
        DeclareLaunchArgument("semantic_map", default_value="/robis_ws/data/environments/semantic_map.yaml"),
        DeclareLaunchArgument("map_checksum", default_value=""),
        DeclareLaunchArgument("enable_external_memory", default_value="false"),
        DeclareLaunchArgument("external_environment_id", default_value=""),
        DeclareLaunchArgument("external_max_age_sec", default_value="3.0"),
        DeclareLaunchArgument("external_max_sensitivity_m", default_value="0.6"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([share, "config", "semantic_navigation.yaml"])),
        Node(package="robis_vision", executable="yolo26_detector", name="yolo26_detector",
             parameters=[config, {
                 "device": ParameterValue(LaunchConfiguration("detector_device"), value_type=str),
                 "model": LaunchConfiguration("detector_model"),
                 "image_size": ParameterValue(LaunchConfiguration("detector_image_size"), value_type=int),
                 "inference_hz": ParameterValue(LaunchConfiguration("detector_inference_hz"), value_type=float),
                 "cpu_threads": ParameterValue(LaunchConfiguration("detector_cpu_threads"), value_type=int),
             }] + common,
             output="screen", condition=backend_is("yolo26")),
        Node(package="robis_vision", executable="open_vocab_detector", name="open_vocab_detector",
             parameters=[config, {
                 "backend": LaunchConfiguration("detector_backend"),
                 "device": ParameterValue(LaunchConfiguration("detector_device"), value_type=str),
                 "model": LaunchConfiguration("open_vocab_model"),
                 "image_size": ParameterValue(LaunchConfiguration("detector_image_size"), value_type=int),
                 "inference_hz": ParameterValue(LaunchConfiguration("detector_inference_hz"), value_type=float),
                 "cpu_threads": ParameterValue(LaunchConfiguration("detector_cpu_threads"), value_type=int),
             }] + common,
             output="screen", condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration("detector_backend"), "' in ['yoloe', 'yolo_world']"
             ]))),
        Node(package="robis_vision", executable="oracle_detector", name="oracle_detector",
             parameters=[oracle_config, config] + common,
             output="screen", condition=backend_is("oracle")),
        Node(package="robis_vision", executable="oracle_detector",
             name="oracle_detector", namespace="reference", parameters=[
                 oracle_config, config, {
                 "detections_topic": "/semantic_perception/reference_detections",
                 "mask_topic": "/semantic_perception/reference_mask",
                 "instance_mask_topic": "/semantic_perception/reference_instance_mask",
             }] + common, output="screen",
             condition=IfCondition(LaunchConfiguration("enable_reference_oracle"))),
        Node(package="robis_vision", executable="detector_evaluator",
             name="detector_evaluator", parameters=[{
                 "output_csv": LaunchConfiguration(
                     "detector_evaluation_output_csv"
                 ),
                 "trial_id": LaunchConfiguration("trial_id"),
             }] + common, output="screen",
             condition=IfCondition(LaunchConfiguration("enable_detector_evaluator"))),
        Node(package="robis_vision", executable="ollama_detector", name="ollama_detector",
             parameters=[config, {"enabled": LaunchConfiguration("enable_ollama")}] + common,
             condition=IfCondition(LaunchConfiguration("enable_ollama")), output="screen"),
        Node(package="robis_semantic_navigation", executable="object_localizer", name="object_localizer",
             parameters=[config, {
                 "map_frame": LaunchConfiguration("localization_frame"),
                 "range_estimator": LaunchConfiguration("range_estimator"),
                 "mask_pixel_tolerance_px": ParameterValue(
                     LaunchConfiguration("mask_pixel_tolerance_px"), value_type=int),
                 "use_depth_hint": ParameterValue(
                     LaunchConfiguration("use_depth_hint"), value_type=bool
                 ),
             }] + common, output="screen"),
        Node(package="robis_semantic_navigation", executable="localization_evaluator",
             name="localization_evaluator", parameters=[{
                 "ground_truth_file": LaunchConfiguration("ground_truth_file"),
                 "output_csv": LaunchConfiguration("localization_output_csv"),
                 "environment": LaunchConfiguration("environment"),
                 "trial_id": LaunchConfiguration("trial_id"),
                 "range_estimator": LaunchConfiguration("range_estimator"),
                 "world_to_map_origin_x": ParameterValue(
                     LaunchConfiguration("world_to_map_origin_x"), value_type=float
                 ),
                 "world_to_map_origin_y": ParameterValue(
                     LaunchConfiguration("world_to_map_origin_y"), value_type=float
                 ),
                 "world_to_map_origin_yaw": ParameterValue(
                     LaunchConfiguration("world_to_map_origin_yaw"), value_type=float
                 ),
             }] + common, output="screen",
             condition=IfCondition(LaunchConfiguration("enable_localization_evaluator"))),
        Node(package="robis_semantic_navigation", executable="semantic_navigation_server",
             name="semantic_navigation_server",
             parameters=[config, {
                 "semantic_map": LaunchConfiguration("semantic_map"),
                 "map_checksum": LaunchConfiguration("map_checksum"),
                 "enable_external_memory": ParameterValue(
                     LaunchConfiguration("enable_external_memory"), value_type=bool),
                 "external_environment_id": LaunchConfiguration("external_environment_id"),
                 "external_max_age_sec": ParameterValue(
                     LaunchConfiguration("external_max_age_sec"), value_type=float),
                 "external_max_sensitivity_m": ParameterValue(
                     LaunchConfiguration("external_max_sensitivity_m"), value_type=float),
             }] + common,
             output="screen"),
    ])
