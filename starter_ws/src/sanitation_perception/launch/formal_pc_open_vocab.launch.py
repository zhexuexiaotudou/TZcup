"""Start the fail-closed PC DOSOD + EdgeSAM product adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    artifact_root = LaunchConfiguration("artifact_root")
    capture_root = LaunchConfiguration("intermediate_capture_root")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "artifact_root",
                description="Absolute directory containing verified formal PC model artifacts",
            ),
            DeclareLaunchArgument("intermediate_capture_root", default_value=""),
            Node(
                package="sanitation_perception",
                executable="pc_open_vocab_product_adapter",
                name="pc_open_vocab_product_adapter",
                output="screen",
                parameters=[{"artifact_root": artifact_root, "intermediate_capture_root": capture_root, "use_sim_time": True}],
            ),
        ]
    )
