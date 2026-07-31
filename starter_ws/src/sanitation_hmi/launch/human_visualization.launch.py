"""Launch the map-first console against the currently active ROS graph."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("operator_token", default_value=""),
            DeclareLaunchArgument("camera_topic", default_value="/camera/color/image_raw"),
            Node(
                package="sanitation_hmi",
                executable="sanitation_hmi_server",
                name="sanitation_human_visualization",
                output="screen",
                arguments=[
                    "--ros",
                    "--host",
                    LaunchConfiguration("host"),
                    "--port",
                    LaunchConfiguration("port"),
                    "--operator-token",
                    LaunchConfiguration("operator_token"),
                    "--camera-topic",
                    LaunchConfiguration("camera_topic"),
                ],
            ),
        ]
    )
