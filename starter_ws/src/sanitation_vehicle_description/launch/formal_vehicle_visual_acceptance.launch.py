"""Launch the formal vehicle in the deterministic three-camera studio world."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package = FindPackageShare("sanitation_vehicle_description")
    formal_launch = PathJoinSubstitution([package, "launch", "formal_vehicle_sim.launch.py"])
    world = PathJoinSubstitution([package, "worlds", "formal_vehicle_visual_acceptance.sdf"])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(formal_launch),
            launch_arguments={
                "world": world,
                "gui": "false",
                "bodywork_visible": "true",
                "start_controllers": "true",
            }.items(),
        )
    ])
