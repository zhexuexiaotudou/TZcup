"""Start the fail-closed PC DOSOD + EdgeSAM product adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    artifact_root = LaunchConfiguration("artifact_root")
    topic_defaults = {
        "front_rgb_topic": "/sensors/front_rgbd/depth/image_rect_raw/image",
        "front_depth_topic": "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
        "front_camera_info_topic": "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
        "wrist_rgb_topic": "/sensors/wrist_rgbd/depth/image_rect_raw/image",
        "wrist_depth_topic": "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image",
        "wrist_camera_info_topic": "/sensors/wrist_rgbd/depth/image_rect_raw/camera_info",
        "rear_left_rgb_topic": "/sensors/rear_left_fisheye/image_raw",
        "rear_right_rgb_topic": "/sensors/rear_right_fisheye/image_raw",
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "artifact_root",
                description="Absolute directory containing verified formal PC model artifacts",
            ),
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in topic_defaults.items()
            ],
            Node(
                package="sanitation_perception",
                executable="pc_open_vocab_product_adapter",
                name="pc_open_vocab_product_adapter",
                output="screen",
                parameters=[{
                    "artifact_root": artifact_root,
                    "use_sim_time": True,
                    **{name: LaunchConfiguration(name) for name in topic_defaults},
                }],
            ),
        ]
    )
