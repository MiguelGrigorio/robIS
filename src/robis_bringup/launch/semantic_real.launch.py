"""Physical counterpart of semantic_simulation: shared perception and actions."""

from pathlib import Path
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), "launch", filename,
        ])), launch_arguments=arguments.items(),
    )


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value("mode")
    if value("enable_external_memory").lower() == "true":
        checksum = value("map_checksum")
        limits = [float(value(name)) for name in ("external_max_age_sec", "external_max_sensitivity_m")]
        if (not value("external_environment_id").strip() or len(checksum) != 64
                or any(c not in '0123456789abcdef' for c in checksum)
                or any(not math.isfinite(v) or v <= 0 for v in limits)):
            raise RuntimeError("Memoria externa exige ambiente, SHA-256 do mapa e limites positivos")
    if mode == "navigation" and not Path(value("map")).is_file():
        raise RuntimeError("mode:=navigation exige map:=/caminho/mapa.yaml existente")
    if value("enable_camera").lower() == "true":
        filename = value("camera_info_url").removeprefix("file://")
        if not filename or not Path(filename).is_file():
            raise RuntimeError(
                "Percepcao fisica exige camera_info_url:=/caminho/calibracao_usb.yaml. "
                "Use basic_real.launch.py para capturar sem calibracao. "
                "config/calib/calib_rt*.npz pertence as cameras fixas do ambiente.")
    hardware = {name: LaunchConfiguration(name) for name in [
        "map", "enable_base", "enable_lidar", "enable_camera", "enable_foxglove",
        "camera_info_url", "camera_params_file", "odrive_params_file", "lidar_params_file",
        "description_params_file", "nav_params_file", "slam_params_file", "nav_start_delay",
    ]}
    semantics = {name: LaunchConfiguration(name) for name in [
        "detector_backend", "detector_device", "detector_model", "detector_image_size",
        "detector_inference_hz", "detector_cpu_threads", "semantic_map", "map_checksum",
        "range_estimator", "mask_pixel_tolerance_px",
        "enable_external_memory", "external_environment_id", "external_max_age_sec",
        "external_max_sensitivity_m",
    ]}
    return [
        include("robis_bringup", "real_full.launch.py", {**hardware, "stack": mode}),
        include("robis_semantic_navigation", "semantic_navigation.launch.py", {
            **semantics, "use_sim_time": "false", "use_depth_hint": "false",
            "enable_ollama": "false", "params_file": LaunchConfiguration("semantic_params_file"),
        }),
    ]


def generate_launch_description():
    bringup = FindPackageShare("robis_bringup")
    navigation = FindPackageShare("robis_navigation")
    semantic = FindPackageShare("robis_semantic_navigation")
    description = FindPackageShare("robis_description")
    config = lambda package, name: PathJoinSubstitution([package, "config", name])
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mapping", choices=["mapping", "navigation"]),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("camera_info_url", default_value=""),
        DeclareLaunchArgument("camera_params_file", default_value=config(bringup, "usb_cam_basic.yaml")),
        DeclareLaunchArgument("odrive_params_file", default_value=config(bringup, "odrive_real.yaml")),
        DeclareLaunchArgument("lidar_params_file", default_value=config(bringup, "ydlidar_real.yaml")),
        DeclareLaunchArgument("description_params_file", default_value=config(description, "robot_parameters.yaml")),
        DeclareLaunchArgument("nav_params_file", default_value=config(navigation, "nav2_params.yaml")),
        DeclareLaunchArgument("slam_params_file", default_value=config(navigation, "slam_toolbox.yaml")),
        DeclareLaunchArgument("semantic_params_file", default_value=config(semantic, "semantic_navigation.yaml")),
        DeclareLaunchArgument("nav_start_delay", default_value="5.0"),
        DeclareLaunchArgument("detector_backend", default_value="yolo26", choices=["yolo26", "external"]),
        DeclareLaunchArgument("detector_device", default_value="cpu"),
        DeclareLaunchArgument("detector_model", default_value="/opt/robis-models/yolo26n-seg.pt"),
        DeclareLaunchArgument("detector_image_size", default_value="320"),
        DeclareLaunchArgument("detector_inference_hz", default_value="1.0"),
        DeclareLaunchArgument("detector_cpu_threads", default_value="2"),
        DeclareLaunchArgument("range_estimator", default_value="mask_projected_cluster"),
        DeclareLaunchArgument("mask_pixel_tolerance_px", default_value="2"),
        DeclareLaunchArgument("semantic_map", default_value="/robis_ws/data/environments/semantic_map.yaml"),
        DeclareLaunchArgument("map_checksum", default_value=""),
        DeclareLaunchArgument("enable_external_memory", default_value="false"),
        DeclareLaunchArgument("external_environment_id", default_value=""),
        DeclareLaunchArgument("external_max_age_sec", default_value="3.0"),
        DeclareLaunchArgument("external_max_sensitivity_m", default_value="0.6"),
        *[DeclareLaunchArgument(name, default_value=default) for name, default in [
            ("enable_base", "true"), ("enable_lidar", "true"),
            ("enable_camera", "true"), ("enable_foxglove", "false"),
        ]],
        OpaqueFunction(function=start),
    ])
