from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("sanitation_coverage"), "config", "coverage.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="opennav_coverage",
                executable="opennav_coverage",
                name="coverage_server",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="coverage_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": ["coverage_server"],
                    }
                ],
            ),
        ]
    )
