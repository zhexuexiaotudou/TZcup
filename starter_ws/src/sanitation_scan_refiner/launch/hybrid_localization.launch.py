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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('sanitation_scan_refiner')
    default_config = os.path.join(
        package_share, 'config', 'stage4v_hybrid.yaml'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'hybrid_config_file', default_value=default_config
            ),
            DeclareLaunchArgument(
                'fusion_mode', default_value='hybrid_rtk_scan_imu_wheel'
            ),
            DeclareLaunchArgument('prior_topic', default_value='/amcl_pose'),
            DeclareLaunchArgument('enable_scan_refiner', default_value='true'),
            DeclareLaunchArgument('publish_map_to_odom', default_value='true'),
            DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
            DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
            DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
            Node(
                package='sanitation_scan_refiner',
                executable='scan_refiner_node',
                name='scan_refiner',
                output='screen',
                condition=IfCondition(
                    LaunchConfiguration('enable_scan_refiner')
                ),
                parameters=[
                    LaunchConfiguration('hybrid_config_file'),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'prior_topic': LaunchConfiguration('prior_topic'),
                    },
                ],
            ),
            Node(
                package='sanitation_scan_refiner',
                executable='hybrid_global_fuser_node',
                name='hybrid_global_fuser',
                output='screen',
                parameters=[
                    LaunchConfiguration('hybrid_config_file'),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'mode': LaunchConfiguration('fusion_mode'),
                        'publish_map_to_odom': LaunchConfiguration(
                            'publish_map_to_odom'
                        ),
                        'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                        'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                        'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
                    },
                ],
            ),
        ]
    )
