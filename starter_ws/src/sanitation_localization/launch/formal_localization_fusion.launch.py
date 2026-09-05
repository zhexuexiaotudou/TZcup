"""Start the formal wheel/IMU/GNSS/lidar-map localization fusion chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    parameters = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_local_fusion = LaunchConfiguration("start_local_fusion")
    start_navsat_transform = LaunchConfiguration("start_navsat_transform")
    start_global_fusion = LaunchConfiguration("start_global_fusion")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_local_fusion", default_value="true"),
            DeclareLaunchArgument("start_navsat_transform", default_value="true"),
            DeclareLaunchArgument("start_global_fusion", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sanitation_localization"),
                        "config",
                        "formal_fusion.yaml",
                    ]
                ),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="local_ekf",
                condition=IfCondition(start_local_fusion),
                parameters=[parameters, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odom")],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                condition=IfCondition(start_navsat_transform),
                parameters=[parameters, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("imu", "/imu/data"),
                    ("gps/fix", "/gnss/fix"),
                    ("odometry/filtered", "/odom"),
                    ("odometry/gps", "/odometry/gps"),
                ],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="global_ekf",
                condition=IfCondition(start_global_fusion),
                parameters=[parameters, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("odometry/filtered", "/localization/fused_odom"),
                ],
                output="screen",
            ),
        ]
    )
