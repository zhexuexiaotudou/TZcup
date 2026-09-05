"""Run the formal DOSOD + EdgeSAM gate without starting placeholder inference."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    contract = PathJoinSubstitution(
        [FindPackageShare("sanitation_perception"), "config", "formal_open_vocab_perception.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("repository_root"),
            DeclareLaunchArgument("platform", default_value="pc"),
            DeclareLaunchArgument("artifact_root", default_value=""),
            DeclareLaunchArgument(
                "output", default_value="/tmp/formal_perception_preflight.json"
            ),
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "sanitation_perception", "formal_perception_preflight",
                    "--contract", contract,
                    "--repository-root", LaunchConfiguration("repository_root"),
                    "--platform", LaunchConfiguration("platform"),
                    "--artifact-root", LaunchConfiguration("artifact_root"),
                    "--output", LaunchConfiguration("output"),
                ],
                output="screen",
            ),
        ]
    )

