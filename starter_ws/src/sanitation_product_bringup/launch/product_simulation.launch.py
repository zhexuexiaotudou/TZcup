"""Single product runtime topology with strict safety and no GT subscribers."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    output_dir = LaunchConfiguration("output_dir")
    transport_partition = LaunchConfiguration("transport_partition")
    sim_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_bringup"), "launch", "sim.launch.py"]
    )
    navigation_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_navigation"), "launch", "navigation.launch.py"]
    )
    hybrid_localization_launch = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_scan_refiner"),
            "launch",
            "hybrid_localization.launch.py",
        ]
    )
    coverage_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_coverage"), "launch", "coverage.launch.py"]
    )
    cleaning_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_bringup"), "launch", "product_cleaning.launch.py"]
    )
    hmi_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_hmi"), "launch", "human_visualization.launch.py"]
    )
    default_world = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_worlds"),
            "worlds",
            "sanitation_test_world.sdf",
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("pipeline_manifest"),
        DeclareLaunchArgument("artifact_root"),
        DeclareLaunchArgument("mission_id"),
        DeclareLaunchArgument("mission_config"),
        DeclareLaunchArgument("dynamic_map_path"),
        DeclareLaunchArgument("cleanable_polygon_json"),
        DeclareLaunchArgument("output_dir"),
        DeclareLaunchArgument("operator_token"),
        DeclareLaunchArgument("map_file"),
        DeclareLaunchArgument("keepout_map"),
        DeclareLaunchArgument("speed_map"),
        DeclareLaunchArgument("navigation_params_file"),
        DeclareLaunchArgument("world_file", default_value=default_world),
        DeclareLaunchArgument("world_name", default_value="sanitation_test_world"),
        DeclareLaunchArgument("spawn_x", default_value="-8.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("world_to_map_x", default_value="8.0"),
        DeclareLaunchArgument("world_to_map_y", default_value="0.0"),
        DeclareLaunchArgument("world_to_map_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "transport_partition",
            default_value=[
                "tzcup_product_",
                EnvironmentVariable("ROS_DOMAIN_ID", default_value="0"),
            ],
            description=(
                "Gazebo Transport partition. The ROS domain-derived default "
                "prevents concurrent trials from cross-publishing clock/sensors."
            ),
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Product runtime is headless; Gazebo GUI is explicit operator opt-in.",
        ),
        DeclareLaunchArgument("headless_rendering", default_value="true"),
        DeclareLaunchArgument("random_seed", default_value="0"),
        SetEnvironmentVariable("GZ_PARTITION", transport_partition),
        SetEnvironmentVariable("IGN_PARTITION", transport_partition),
        Node(
            package="sanitation_safety",
            executable="product_supervisor",
            name="product_supervisor",
            output="screen",
            respawn=True,
            respawn_delay=0.05,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "gui": LaunchConfiguration("gui"),
                "headless_rendering": LaunchConfiguration("headless_rendering"),
                "random_seed": LaunchConfiguration("random_seed"),
                "world_file": LaunchConfiguration("world_file"),
                "world_name": LaunchConfiguration("world_name"),
                "spawn_x": LaunchConfiguration("spawn_x"),
                "spawn_y": LaunchConfiguration("spawn_y"),
                "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                "drive_model": "ackermann",
                # Velocity gate owns the product watchdog and fails closed on
                # stale commands or a missing safety heartbeat.  Do not run
                # the upstream same-topic command_timeout publisher in parallel.
                "enable_command_timeout": "false",
                "camera_profile": "production",
                "enable_training_gt": "false",
                "enable_evaluation_gt": "false",
                "cleaning_width": "1.32",
            }.items(),
        ),
        Node(
            package="sanitation_gnss_sim",
            executable="dual_navsat_adapter",
            name="dual_navsat_adapter",
            output="screen",
            respawn=True,
            respawn_delay=1.0,
            parameters=[{
                "use_sim_time": use_sim_time,
                "profile": "rtk_fixed",
                "random_seed": LaunchConfiguration("random_seed"),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hybrid_localization_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "fusion_mode": "rtk_imu_wheel",
                "enable_scan_refiner": "false",
                "publish_map_to_odom": "true",
                "respawn_fuser": "true",
                "initial_pose_x": "0.0",
                "initial_pose_y": "0.0",
                "initial_pose_yaw": LaunchConfiguration("spawn_yaw"),
                "world_to_map_x": LaunchConfiguration("world_to_map_x"),
                "world_to_map_y": LaunchConfiguration("world_to_map_y"),
                "world_to_map_yaw": LaunchConfiguration("world_to_map_yaw"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": LaunchConfiguration("navigation_params_file"),
                "map_file": LaunchConfiguration("map_file"),
                "keepout_map": LaunchConfiguration("keepout_map"),
                "speed_map": LaunchConfiguration("speed_map"),
                "footprint_profile": "production",
                "localization_backend": "external",
                "localization_pose_topic": "/localization/fused_pose",
                "safety_startup_stopped": "true",
                "safety_require_supervisor": "true",
                "enable_filters": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(coverage_launch),
            launch_arguments={"footprint_profile": "production"}.items(),
        ),
        Node(
            package="sanitation_coverage",
            executable="coverage_probe",
            name="product_coverage_controller",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "config_path": LaunchConfiguration("mission_config"),
                "manual_start": True,
                "allow_ground_truth_evaluation": False,
                "require_product_task_drain": True,
                "output_path": PathJoinSubstitution(
                    [output_dir, "coverage_metrics.json"]
                ),
                "path_output_path": PathJoinSubstitution(
                    [output_dir, "coverage_path.json"]
                ),
                "trajectory_output_path": PathJoinSubstitution(
                    [output_dir, "coverage_trajectory.csv"]
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cleaning_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "pipeline_manifest": LaunchConfiguration("pipeline_manifest"),
                "artifact_root": LaunchConfiguration("artifact_root"),
                "mission_id": LaunchConfiguration("mission_id"),
                "dynamic_map_path": LaunchConfiguration("dynamic_map_path"),
                "resume_same_mission": "false",
                "cleanable_polygon_json": LaunchConfiguration(
                    "cleanable_polygon_json"
                ),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hmi_launch),
            launch_arguments={
                "operator_token": LaunchConfiguration("operator_token"),
            }.items(),
        ),
    ])
