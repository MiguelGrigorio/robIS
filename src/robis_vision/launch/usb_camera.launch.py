from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    video_device = LaunchConfiguration("video_device")
    width = LaunchConfiguration("image_width")
    height = LaunchConfiguration("image_height")
    framerate = LaunchConfiguration("framerate")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("image_width", default_value="640"),
            DeclareLaunchArgument("image_height", default_value="480"),
            DeclareLaunchArgument("framerate", default_value="30.0"),
            DeclareLaunchArgument("frame_id", default_value="camera_link"),
            Node(
                package="robis_vision",
                executable="usb_camera_node",
                name="usb_camera_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "image_topic": image_topic,
                        "video_device": video_device,
                        "image_width": ParameterValue(width, value_type=int),
                        "image_height": ParameterValue(height, value_type=int),
                        "framerate": ParameterValue(framerate, value_type=float),
                        "frame_id": frame_id,
                    }
                ],
            ),
        ]
    )
