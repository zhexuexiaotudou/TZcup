"""Start Gazebo, SLAM, the safety gate, and the browser supervision console."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    sim_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_bringup"), "launch", "sim.launch.py"]
    )
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_navigation"), "launch", "slam.launch.py"]
    )
    structured_world = PathJoinSubstitution(
        [FindPackageShare("sanitation_worlds"), "worlds", "sanitation_structured_world.sdf"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless_rendering", default_value="true"),
            DeclareLaunchArgument("slam", default_value="true"),
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("operator_token", default_value=""),
            DeclareLaunchArgument("world_file", default_value=structured_world),
            DeclareLaunchArgument("world_name", default_value="sanitation_structured_world"),
            DeclareLaunchArgument("camera_profile", default_value="production"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "headless_rendering": LaunchConfiguration("headless_rendering"),
                    "world_file": LaunchConfiguration("world_file"),
                    "world_name": LaunchConfiguration("world_name"),
                    "camera_profile": LaunchConfiguration("camera_profile"),
                }.items(),
            ),
            TimerAction(
                period=15.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(slam_launch),
                        condition=IfCondition(LaunchConfiguration("slam")),
                        launch_arguments={"rviz": "false", "use_sim_time": "true"}.items(),
                    )
                ],
            ),
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
                    "/camera/color/image_raw",
                ],
            ),
        ]
    )
