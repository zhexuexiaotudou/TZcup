# Copyright 2026 Sanitation Vehicle Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('sanitation_bringup')
    navigation_share = get_package_share_directory('sanitation_navigation')
    worlds_share = get_package_share_directory('sanitation_worlds')
    gnss_share = get_package_share_directory('sanitation_gnss_sim')
    refiner_share = get_package_share_directory('sanitation_scan_refiner')

    sim_launch = os.path.join(bringup_share, 'launch', 'sim.launch.py')
    gnss_launch = os.path.join(gnss_share, 'launch', 'gnss_sim.launch.py')
    hybrid_launch = os.path.join(
        refiner_share, 'launch', 'hybrid_localization.launch.py'
    )
    default_world = os.path.join(
        worlds_share, 'worlds', 'sanitation_structured_world.sdf'
    )
    default_map = os.path.join(
        navigation_share, 'maps', 'stage4v_surveyed_reference.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('gui', default_value='false'),
            DeclareLaunchArgument('world_file', default_value=default_world),
            DeclareLaunchArgument('map_file', default_value=default_map),
            DeclareLaunchArgument('random_seed', default_value='0'),
            DeclareLaunchArgument('gnss_profile', default_value='rtk_fixed'),
            DeclareLaunchArgument(
                'fusion_mode', default_value='hybrid_rtk_scan_imu_wheel'
            ),
            DeclareLaunchArgument('lidar_samples', default_value='360'),
            DeclareLaunchArgument('lidar_update_rate', default_value='10'),
            DeclareLaunchArgument('spawn_x', default_value='-8.0'),
            DeclareLaunchArgument('spawn_y', default_value='0.0'),
            DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
            DeclareLaunchArgument('world_to_map_x', default_value='8.0'),
            DeclareLaunchArgument('world_to_map_y', default_value='0.0'),
            DeclareLaunchArgument('world_to_map_yaw', default_value='0.0'),
            DeclareLaunchArgument('enable_scan_refiner', default_value='true'),
            DeclareLaunchArgument('publish_map_to_odom', default_value='true'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    'gui': LaunchConfiguration('gui'),
                    'headless_rendering': 'true',
                    'use_sim_time': use_sim_time,
                    'world_file': LaunchConfiguration('world_file'),
                    'random_seed': LaunchConfiguration('random_seed'),
                    'lidar_samples': LaunchConfiguration('lidar_samples'),
                    'lidar_update_rate': LaunchConfiguration('lidar_update_rate'),
                    'spawn_x': LaunchConfiguration('spawn_x'),
                    'spawn_y': LaunchConfiguration('spawn_y'),
                    'spawn_yaw': LaunchConfiguration('spawn_yaw'),
                    'world_to_map_x': LaunchConfiguration('world_to_map_x'),
                    'world_to_map_y': LaunchConfiguration('world_to_map_y'),
                    'world_to_map_yaw': LaunchConfiguration('world_to_map_yaw'),
                }.items(),
            ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        'yaml_filename': LaunchConfiguration('map_file'),
                    }
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='stage4v_map_lifecycle_manager',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        'autostart': True,
                        'node_names': ['map_server'],
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gnss_launch),
                launch_arguments={
                    'profile': LaunchConfiguration('gnss_profile'),
                    'random_seed': LaunchConfiguration('random_seed'),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hybrid_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'fusion_mode': LaunchConfiguration('fusion_mode'),
                    'prior_topic': '/localization/fused_pose',
                    'enable_scan_refiner': LaunchConfiguration(
                        'enable_scan_refiner'
                    ),
                    'publish_map_to_odom': LaunchConfiguration(
                        'publish_map_to_odom'
                    ),
                    'initial_pose_x': '0.0',
                    'initial_pose_y': '0.0',
                    'initial_pose_yaw': '0.0',
                }.items(),
            ),
        ]
    )
