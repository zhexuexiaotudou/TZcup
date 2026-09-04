"""Launch the formal vehicle plus the strict 20-cube physical scene."""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from sanitation_manipulation.formal_20_cube_scene import load_scene_manifest


def _spawn_actions(context):
    manifest_path = Path(LaunchConfiguration("manifest").perform(context)).resolve()
    _, specs = load_scene_manifest(manifest_path)
    cube = PathJoinSubstitution(
        [FindPackageShare("sanitation_manipulation"), "urdf", "material_cube.urdf.xacro"]
    )
    actions = []
    for index, spec in enumerate(specs):
        red, green, blue = spec.color_rgb
        description = ParameterValue(
            Command(
                [
                    "xacro ", cube,
                    " material:=", spec.material,
                    " color_r:=", f"{red:.6f}",
                    " color_g:=", f"{green:.6f}",
                    " color_b:=", f"{blue:.6f}",
                ]
            ),
            value_type=str,
        )
        actions.append(
            TimerAction(
                period=2.0 + index * 0.20,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        parameters=[{"robot_description": description}],
                        arguments=[
                            "-param", "robot_description",
                            "-name", spec.model_name,
                            "-x", f"{spec.x_m:.9f}",
                            "-y", f"{spec.y_m:.9f}",
                            "-z", f"{spec.z_m + 0.002:.9f}",
                            "-Y", f"{spec.yaw_rad:.9f}",
                        ],
                        output="screen",
                    )
                ],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    manifest = LaunchConfiguration("manifest")
    base_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_manipulation"), "launch", "formal_cube_pick_place.launch.py"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("manifest"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "gui": "false",
                    "physics_engine": "gz-physics-dartsim-plugin",
                    "spawn_single_cube": "false",
                    # This formal scene retains every deposited cube as an
                    # independent Gazebo rigid body.  Do not permit the
                    # historical aggregate dry-mass path alongside it.
                    "dry_accounting_mode": "physical_resident",
                }.items(),
            ),
            OpaqueFunction(function=_spawn_actions),
        ]
    )
