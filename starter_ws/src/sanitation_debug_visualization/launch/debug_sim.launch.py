from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    fixed_frame = LaunchConfiguration("fixed_frame")
    headless_rendering = LaunchConfiguration("headless_rendering")

    sim_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_bringup"), "launch", "sim.launch.py"]
    )
    debug_launch = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_debug_visualization"),
            "launch",
            "debug_visualization.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("fixed_frame", default_value="base_link"),
            DeclareLaunchArgument("headless_rendering", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "gui": gui,
                    "headless_rendering": headless_rendering,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(debug_launch),
                launch_arguments={
                    "rviz": rviz,
                    "fixed_frame": fixed_frame,
                    "use_sim_time": "true",
                }.items(),
            ),
        ]
    )
