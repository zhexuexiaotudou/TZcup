"""Start the complete PC product demo without creating package dependency cycles."""

import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validate_product_inputs(context):  # type: ignore[no-untyped-def]
    artifact_root = Path(
        context.perform_substitution(
            LaunchConfiguration("perception_artifact_root")
        )
    ).resolve()
    checkpoint = Path(
        context.perform_substitution(LaunchConfiguration("policy_checkpoint"))
    ).resolve()
    maximum_distance = float(
        context.perform_substitution(
            LaunchConfiguration("maximum_task_distance_m")
        )
    )
    if not artifact_root.is_dir():
        raise RuntimeError("product demo requires a perception artifact directory")
    if not checkpoint.is_file():
        raise RuntimeError("product demo requires a frozen policy checkpoint")
    if not math.isfinite(maximum_distance) or maximum_distance <= 0.0:
        raise RuntimeError(
            "product demo requires a positive same-map FullCoverage distance"
        )
    return []


def generate_launch_description() -> LaunchDescription:
    campus_launch = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_formal_campus_integration"),
            "launch",
            "formal_campus_map_lifecycle.launch.py",
        ]
    )
    perception_launch = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_perception"),
            "launch",
            "formal_pc_open_vocab.launch.py",
        ]
    )
    active_cleaning_launch = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_active_cleaning"),
            "launch",
            "formal_active_cleaning.launch.py",
        ]
    )
    manipulation_launch = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_manipulation"),
            "launch",
            "formal_physical_grasp.launch.py",
        ]
    )
    runtime_root = LaunchConfiguration("saved_map_artifact_dir")
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("world", description="Generated public/world.sdf"),
            DeclareLaunchArgument("world_name", default_value="campus_formal"),
            DeclareLaunchArgument(
                "episode_manifest",
                description="Generated public/episode_manifest.json",
            ),
            DeclareLaunchArgument("pedestrian_schedule", default_value=""),
            DeclareLaunchArgument("start_pedestrians", default_value="true"),
            DeclareLaunchArgument(
                "saved_map_artifact_dir",
                description=(
                    "First-task SLAM directory with a passing "
                    "map_lifecycle_manifest.json"
                ),
            ),
            DeclareLaunchArgument(
                "perception_artifact_root",
                description="Verified DOSOD and EdgeSAM PC artifact directory",
            ),
            DeclareLaunchArgument(
                "policy_checkpoint",
                description="Frozen truth-free active-cleaning Q checkpoint",
            ),
            DeclareLaunchArgument(
                "maximum_task_distance_m",
                description="Successful FullCoverage distance for this exact map",
            ),
            DeclareLaunchArgument(
                "episode_seed",
                description="Seed of the one frozen random cleaning episode",
            ),
            OpaqueFunction(function=_validate_product_inputs),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(campus_launch),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "world": LaunchConfiguration("world"),
                    "world_name": LaunchConfiguration("world_name"),
                    "episode_manifest": LaunchConfiguration("episode_manifest"),
                    "pedestrian_schedule": LaunchConfiguration("pedestrian_schedule"),
                    "start_pedestrians": LaunchConfiguration("start_pedestrians"),
                    "map_artifact_dir": runtime_root,
                    "mission_mode": "cleaning",
                    "cleaning_planner": "rl_dirt_priority",
                    # The active policy and the coverage baseline are mutually
                    # exclusive command owners.  Product mode always disables
                    # the baseline coverage server.
                    "start_coverage": "false",
                }.items(),
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="sanitation_product_demo_integration",
                        executable="simulation_operator_gate",
                        name="formal_product_demo_operator_gate",
                        output="screen",
                        parameters=[{"use_sim_time": True}],
                    )
                ],
            ),
            TimerAction(
                period=50.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(perception_launch),
                        launch_arguments={
                            "artifact_root": LaunchConfiguration(
                                "perception_artifact_root"
                            )
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=55.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(active_cleaning_launch),
                        launch_arguments={
                            "runtime_root": runtime_root,
                            "policy_checkpoint": LaunchConfiguration(
                                "policy_checkpoint"
                            ),
                            "maximum_task_distance_m": LaunchConfiguration(
                                "maximum_task_distance_m"
                            ),
                            "episode_seed": LaunchConfiguration("episode_seed"),
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=58.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(manipulation_launch)
                    )
                ],
            ),
        ]
    )
