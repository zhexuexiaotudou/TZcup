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
from launch.substitutions import PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString


def generate_launch_description():
    package_share = get_package_share_directory('sanitation_navigation')
    nav2_params = os.path.join(package_share, 'config', 'nav2.yaml')
    default_map = os.path.join(
        package_share, 'maps', 'sanitation_test_map.yaml'
    )
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
    )
    slam_localization_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'localization_launch.py'
    )
    default_slam_localization_params = os.path.join(
        package_share, 'config', 'slam_localization.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    configured_params = ReplaceString(
        source_file=params_file,
        replacements={
            '__ACKERMANN_NAV_TO_POSE_BT__': os.path.join(
                package_share, 'behavior_trees', 'navigate_to_pose_ackermann.xml'
            ),
            '__ACKERMANN_NAV_THROUGH_POSES_BT__': os.path.join(
                package_share,
                'behavior_trees',
                'navigate_through_poses_ackermann.xml',
            ),
        },
    )
    map_file = LaunchConfiguration('map_file')
    keepout_map = LaunchConfiguration('keepout_map')
    speed_map = LaunchConfiguration('speed_map')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    localization_backend = LaunchConfiguration('localization_backend')
    enable_filters = LaunchConfiguration('enable_filters')
    slam_params_file = LaunchConfiguration('slam_params_file')
    amcl_condition = IfCondition(
        PythonExpression(["'", localization_backend, "' == 'amcl'"])
    )
    slam_condition = IfCondition(
        PythonExpression(["'", localization_backend, "' == 'slam_toolbox'"])
    )
    external_condition = IfCondition(
        PythonExpression(["'", localization_backend, "' == 'external'"])
    )
    auto01_height_banded_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('footprint_profile'),
            "' == 'auto01_g1_height_banded'",
        ])
    )
    auto01_g2_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('footprint_profile'),
            "' in ('auto01_g2_v5_retracted', "
            "'autonomous_navigation_profile_v1', 'auto12_efficiency_v1')",
        ])
    )

    localization_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            condition=amcl_condition,
            parameters=[configured_params, {'yaml_filename': map_file}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            condition=amcl_condition,
            parameters=[configured_params, {
                'initial_pose.x': ParameterValue(initial_pose_x, value_type=float),
                'initial_pose.y': ParameterValue(initial_pose_y, value_type=float),
                'initial_pose.yaw': ParameterValue(initial_pose_yaw, value_type=float),
            }],
            remappings=[
                ('amcl_pose', LaunchConfiguration('localization_pose_topic')),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            condition=amcl_condition,
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'autostart': LaunchConfiguration('autostart'),
                    'node_names': ['map_server', 'amcl'],
                }
            ],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            condition=external_condition,
            parameters=[configured_params, {'yaml_filename': map_file}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            condition=external_condition,
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'autostart': LaunchConfiguration('autostart'),
                    'node_names': ['map_server'],
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
            condition=IfCondition(enable_filters),
            parameters=[configured_params, {'yaml_filename': keepout_map}],
            remappings=[('map', 'keepout_filter_mask')],
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_costmap_filter_info_server',
            output='screen',
            condition=IfCondition(enable_filters),
            parameters=[configured_params],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='speed_filter_mask_server',
            output='screen',
            condition=IfCondition(enable_filters),
            parameters=[configured_params, {'yaml_filename': speed_map}],
            remappings=[('map', 'speed_filter_mask')],
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='speed_costmap_filter_info_server',
            output='screen',
            condition=IfCondition(enable_filters),
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='filter_lifecycle_manager',
            output='screen',
            condition=IfCondition(enable_filters),
            parameters=[configured_params],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument('params_file', default_value=nav2_params),
            DeclareLaunchArgument(
                'footprint_profile', default_value='production',
                description=(
                    'production or an opt-in external profile; params_file '
                    'must carry matching polygons'
                ),
            ),
            DeclareLaunchArgument('map_file', default_value=default_map),
            DeclareLaunchArgument('keepout_map', default_value=default_map),
            DeclareLaunchArgument('speed_map', default_value=default_map),
            DeclareLaunchArgument('operational_profile', default_value='localization_coverage'),
            DeclareLaunchArgument('max_linear_velocity', default_value='0.45'),
            DeclareLaunchArgument('max_angular_velocity', default_value='0.35'),
            DeclareLaunchArgument(
                'safety_startup_stopped',
                default_value='false',
                description=(
                    'Engineering compatibility default. Product launch must '
                    'override this to true and require an operator clear.'
                ),
            ),
            DeclareLaunchArgument(
                'safety_require_supervisor',
                default_value='false',
                description=(
                    'Engineering compatibility default. Product launch must '
                    'require the product supervisor heartbeat.'
                ),
            ),
            DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
            DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
            DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
            DeclareLaunchArgument(
                'localization_backend',
                default_value='amcl',
                description=(
                    'amcl, slam_toolbox, or external. The external backend '
                    'keeps the map server but delegates fused pose and '
                    'map-to-odom ownership to a separate localization stack.'
                ),
            ),
            DeclareLaunchArgument(
                'localization_pose_topic',
                default_value='/amcl_pose',
                description=(
                    'Canonical global pose output. Product launch remaps AMCL '
                    'to /localization/fused_pose; engineering launches retain '
                    'the Nav2-compatible /amcl_pose default.'
                ),
            ),
            DeclareLaunchArgument(
                'enable_filters', default_value='true',
                description=(
                    'Disable only for first-principles mapping acceptance; '
                    'normal product navigation keeps safety filters enabled.'
                ),
            ),
            DeclareLaunchArgument(
                'slam_params_file', default_value=default_slam_localization_params
            ),
            *localization_nodes,
            Node(
                package='sanitation_navigation',
                executable='scan_self_filter',
                name='scan_self_filter',
                output='screen',
                condition=auto01_height_banded_condition,
                parameters=[configured_params],
            ),
            Node(
                package='sanitation_navigation',
                executable='pointcloud_self_filter',
                name='pointcloud_self_filter',
                output='screen',
                condition=auto01_g2_condition,
                parameters=[configured_params],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_localization_launch),
                condition=slam_condition,
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params_file,
                    'autostart': 'true',
                }.items(),
            ),
            GroupAction(
                [
                    SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav2_launch),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'autostart': LaunchConfiguration('autostart'),
                            'params_file': configured_params,
                            'use_composition': 'False',
                            'use_localization': 'False',
                        }.items(),
                    ),
                ]
            ),
            *filter_nodes,
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='ground_collision_monitor',
                output='screen',
                condition=auto01_height_banded_condition,
                parameters=[configured_params],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='ground_collision_lifecycle_manager',
                output='screen',
                condition=auto01_height_banded_condition,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': LaunchConfiguration('autostart'),
                    'node_names': ['ground_collision_monitor'],
                }],
            ),
            Node(
                package='sanitation_safety',
                executable='safety_authority',
                name='safety_authority',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'startup_emergency_stopped': ParameterValue(
                        LaunchConfiguration('safety_startup_stopped'),
                        value_type=bool,
                    ),
                    'require_supervisor_heartbeat': ParameterValue(
                        LaunchConfiguration('safety_require_supervisor'),
                        value_type=bool,
                    ),
                }],
            ),
            Node(
                package='sanitation_safety',
                executable='velocity_gate',
                name='velocity_gate',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'profile_name': LaunchConfiguration('operational_profile'),
                    'max_linear_velocity': ParameterValue(max_linear_velocity, value_type=float),
                    'max_angular_velocity': ParameterValue(max_angular_velocity, value_type=float),
                }],
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
