"""Start the human-readable Gazebo scene and vehicle without any web UI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    sim_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_bringup"), "launch", "sim.launch.py"]
    )
    structured_world = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_worlds"),
            "worlds",
            "sanitation_structured_world.sdf",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless_rendering", default_value="true"),
            DeclareLaunchArgument("world_file", default_value=structured_world),
            DeclareLaunchArgument(
                "world_name", default_value="sanitation_structured_world"
            ),
            DeclareLaunchArgument("spawn_x", default_value="-8.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("random_seed", default_value="0"),
            DeclareLaunchArgument("camera_profile", default_value="production"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "headless_rendering": LaunchConfiguration("headless_rendering"),
                    "world_file": LaunchConfiguration("world_file"),
                    "world_name": LaunchConfiguration("world_name"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                    "random_seed": LaunchConfiguration("random_seed"),
                    "camera_profile": LaunchConfiguration("camera_profile"),
                }.items(),
            ),
        ]
    )
