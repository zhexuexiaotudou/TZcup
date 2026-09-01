"""Launch the formal vehicle in the deterministic functional-view studio."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package = FindPackageShare("sanitation_vehicle_description")
    formal_launch = PathJoinSubstitution([package, "launch", "formal_vehicle_sim.launch.py"])
    default_world = PathJoinSubstitution(
        [package, "worlds", "formal_vehicle_visual_acceptance.sdf"]
    )
    world = LaunchConfiguration("world")
    bodywork_visible = LaunchConfiguration("bodywork_visible")
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description=(
                "Visual studio SDF. Formal capture supplies a source-bound "
                "triggered derivative so only one 1600x1000 camera renders at a time."
            ),
        ),
        DeclareLaunchArgument(
            "bodywork_visible",
            default_value="true",
            description=(
                "Render the installed product body panels. Set false only for the "
                "service-inspection capture that exposes mounted components."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(formal_launch),
            launch_arguments={
                "world": world,
                "gui": "false",
                "headless_rendering": "true",
                "bodywork_visible": bodywork_visible,
                "start_controllers": "true",
                "enable_safety_manager": "false",
                "start_localization": "false",
                "high_bandwidth_sensor_runtime": "false",
                "visual_acceptance_runtime": "true",
            }.items(),
        )
    ])
