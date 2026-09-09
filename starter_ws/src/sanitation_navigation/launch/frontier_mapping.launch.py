# Copyright 2026 Sanitation Vehicle Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Lean Nav2 bringup for the first-map frontier mission only.

The regular ``navigation.launch.py`` intentionally delegates to Nav2's full
bringup and remains the only profile used by cleaning and product runs.  A
frontier mapper sends only ``NavigateToPose`` goals, so Route, waypoint and
docking servers would be idle here.  This launch starts the smallest Nav2 set
that preserves planning, recovery, velocity smoothing, collision monitoring
and both costmap filters.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


MAPPING_LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
    "bt_navigator",
]


def generate_launch_description():
    package_share = get_package_share_directory("sanitation_navigation")
    default_params = os.path.join(package_share, "config", "nav2.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    keepout_map = LaunchConfiguration("keepout_map")
    speed_map = LaunchConfiguration("speed_map")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={"autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    navigation_nodes = GroupAction(
        [
            SetRemap(src="/cmd_vel", dst="/cmd_vel_nav"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params, {
                    "navigators": ["navigate_to_pose"],
                }],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": MAPPING_LIFECYCLE_NODES,
                }],
            ),
        ]
    )

    filter_nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="keepout_filter_mask_server",
            output="screen",
            parameters=[params_file, {"yaml_filename": keepout_map}],
            remappings=[("map", "keepout_filter_mask")],
        ),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="keepout_costmap_filter_info_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="speed_filter_mask_server",
            output="screen",
            parameters=[params_file, {"yaml_filename": speed_map}],
            remappings=[("map", "speed_filter_mask")],
        ),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="speed_costmap_filter_info_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="filter_lifecycle_manager",
            output="screen",
            parameters=[params_file],
        ),
    ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("keepout_map", default_value=""),
            DeclareLaunchArgument("speed_map", default_value=""),
            navigation_nodes,
            *filter_nodes,
        ]
    )
