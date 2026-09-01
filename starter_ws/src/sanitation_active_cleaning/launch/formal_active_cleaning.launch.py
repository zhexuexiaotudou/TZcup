"""Start formal product observation and validated Nav2 trajectory adapters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    runtime_root = LaunchConfiguration("runtime_root")
    policy_checkpoint = LaunchConfiguration("policy_checkpoint")
    maximum_task_distance_m = LaunchConfiguration("maximum_task_distance_m")
    episode_seed = LaunchConfiguration("episode_seed")
    config = PathJoinSubstitution(
        [FindPackageShare("sanitation_active_cleaning"), "config", "formal_runtime.yaml"]
    )
    shared_map_parameters = {
        "occupancy_map": PathJoinSubstitution([runtime_root, "occupancy.yaml"]),
        "mission_geometry": PathJoinSubstitution([runtime_root, "mission_geometry.yaml"]),
        "materialization_contract": PathJoinSubstitution(
            [runtime_root, "materialization_contract.yaml"]
        ),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "runtime_root",
                description="Formal public campus materialization directory",
            ),
            DeclareLaunchArgument(
                "policy_checkpoint",
                description="Frozen truth-free formal Q-policy checkpoint",
            ),
            DeclareLaunchArgument(
                "maximum_task_distance_m",
                description="Same-map successful FullCoverage task distance",
            ),
            DeclareLaunchArgument(
                "episode_seed",
                description="Seed of the one frozen random cleaning episode",
            ),
            Node(
                package="sanitation_active_cleaning",
                executable="formal_observation_bridge",
                name="formal_active_cleaning_observation_bridge",
                output="screen",
                parameters=[config, shared_map_parameters, {"use_sim_time": True}],
            ),
            Node(
                package="sanitation_active_cleaning",
                executable="formal_trajectory_executor",
                name="formal_active_cleaning_trajectory_executor",
                output="screen",
                parameters=[
                    config,
                    {"mission_geometry": shared_map_parameters["mission_geometry"]},
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="sanitation_active_cleaning",
                executable="formal_policy_planner",
                name="formal_active_cleaning_policy_planner",
                output="screen",
                parameters=[
                    config,
                    shared_map_parameters,
                    {"policy_checkpoint": policy_checkpoint},
                    {
                        "maximum_task_distance_m": ParameterValue(
                            maximum_task_distance_m, value_type=float
                        ),
                        "episode_seed": ParameterValue(episode_seed, value_type=int),
                    },
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="sanitation_active_cleaning",
                executable="formal_cleaning_coordinator",
                name="formal_active_cleaning_cleaning_coordinator",
                output="screen",
                parameters=[config, {"use_sim_time": True}],
            ),
        ]
    )
