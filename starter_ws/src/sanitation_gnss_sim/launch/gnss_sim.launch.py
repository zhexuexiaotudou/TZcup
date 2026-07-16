import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("sanitation_gnss_sim")
    default_config = os.path.join(package_share, "config", "profiles.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("gnss_config_file", default_value=default_config),
            DeclareLaunchArgument("profile", default_value="rtk_fixed"),
            DeclareLaunchArgument("random_seed", default_value="0"),
            Node(
                package="sanitation_gnss_sim",
                executable="sanitation_gnss_sim",
                name="sanitation_gnss_sim",
                output="screen",
                parameters=[
                    LaunchConfiguration("gnss_config_file"),
                    {
                        "profile": LaunchConfiguration("profile"),
                        "random_seed": LaunchConfiguration("random_seed"),
                    },
                ],
            ),
        ]
    )
