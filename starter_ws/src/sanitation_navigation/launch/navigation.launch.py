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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    package_share = get_package_share_directory('sanitation_navigation')
    nav2_params = os.path.join(package_share, 'config', 'nav2.yaml')
    default_map = os.path.join(
        package_share, 'maps', 'sanitation_test_map.yaml'
    )
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')

    localization_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'autostart': LaunchConfiguration('autostart'),
                    'node_names': ['map_server', 'amcl'],
                }
            ],
        ),
    ]

    filter_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_filter_mask_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file}],
            remappings=[('map', 'keepout_filter_mask')],
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_costmap_filter_info_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='speed_filter_mask_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file}],
            remappings=[('map', 'speed_filter_mask')],
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='speed_costmap_filter_info_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='filter_lifecycle_manager',
            output='screen',
            parameters=[params_file],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument('params_file', default_value=nav2_params),
            DeclareLaunchArgument('map_file', default_value=default_map),
            *localization_nodes,
            GroupAction(
                [
                    SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav2_launch),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'autostart': LaunchConfiguration('autostart'),
                            'params_file': params_file,
                            'use_composition': 'False',
                            'use_localization': 'False',
                        }.items(),
                    ),
                ]
            ),
            *filter_nodes,
            Node(
                package='sanitation_safety',
                executable='velocity_gate',
                name='velocity_gate',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )
