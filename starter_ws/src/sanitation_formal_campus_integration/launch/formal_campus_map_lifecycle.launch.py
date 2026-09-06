"""Formal no-wall campus: first-task SLAM or saved-map cleaning lifecycle."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from sanitation_formal_campus_integration.contract import materialize_nav2_config
from sanitation_formal_campus_integration.map_lifecycle_core import (
    load_campus_map_contract,
    prepare_public_lifecycle_artifacts,
    validate_saved_map_artifact,
)
from sanitation_formal_campus_integration.nav2_mode_config import (
    configure_collision_monitor_sources,
)
from sanitation_formal_campus_integration.saved_map_coverage_core import (
    DRY_CLEANING_SPEED_PROFILE,
    FORMAL_MAX_LINEAR_SPEED_MPS,
    FORMAL_OPERATION_WIDTH_M,
    MAPPING_SAFE_SPEED_PROFILE,
    load_formal_operation_speed_profile,
)
import yaml


def _runtime_actions(context):  # type: ignore[no-untyped-def]
    mode = context.perform_substitution(LaunchConfiguration("mission_mode"))
    if mode not in {"mapping", "cleaning"}:
        raise RuntimeError("mission_mode must be mapping or cleaning")
    cleaning_planner = context.perform_substitution(
        LaunchConfiguration("cleaning_planner")
    )
    if cleaning_planner not in {"full_coverage", "rl_dirt_priority"}:
        raise RuntimeError(
            "cleaning_planner must be full_coverage or rl_dirt_priority"
        )
    manifest_path = Path(
        context.perform_substitution(LaunchConfiguration("episode_manifest"))
    )
    artifact_root = Path(
        context.perform_substitution(LaunchConfiguration("map_artifact_dir"))
    ).resolve()
    contract = load_campus_map_contract(manifest_path)
    if mode == "cleaning":
        # This launch-time gate prevents AMCL/Nav2 from even starting with an
        # unqualified, wrong-map or tampered artifact.
        validate_saved_map_artifact(artifact_root, contract)
        support = {
            "keepout_map": artifact_root / "geofence_keepout.yaml",
            "speed_map": artifact_root / "neutral_speed.yaml",
        }
    else:
        if (artifact_root / "map_lifecycle_manifest.json").exists():
            raise RuntimeError(
                "mapping mode cannot overwrite a finalized first-task map"
            )
        support = prepare_public_lifecycle_artifacts(contract, artifact_root)
    base_params = Path(
        context.perform_substitution(LaunchConfiguration("base_nav2_params_file"))
    )
    base_slam_params = Path(
        context.perform_substitution(LaunchConfiguration("base_slam_params_file"))
    )
    motion_profile = Path(
        context.perform_substitution(LaunchConfiguration("motion_profile_file"))
    )
    speed_profile_file = Path(
        context.perform_substitution(LaunchConfiguration("operation_speed_profile_file"))
    )
    requested_speed_profile = context.perform_substitution(
        LaunchConfiguration("operation_speed_profile")
    )
    if mode == "mapping" and requested_speed_profile != MAPPING_SAFE_SPEED_PROFILE:
        raise RuntimeError("mapping mode must retain the mapping_safe speed profile")
    speed_profile = load_formal_operation_speed_profile(
        speed_profile_file, requested_speed_profile
    )
    nav2, cleaning_width = materialize_nav2_config(
        base_params,
        motion_profile,
        clean_path_speed_mps=speed_profile.maximum_linear_speed_mps,
    )
    if not math.isclose(cleaning_width, FORMAL_OPERATION_WIDTH_M, abs_tol=1e-9):
        raise RuntimeError("formal saved-map cleaning width must be exactly 1.32 m")
    smoother_speed = float(
        nav2["velocity_smoother"]["ros__parameters"]["max_velocity"][0]
    )
    if not math.isclose(smoother_speed, speed_profile.maximum_linear_speed_mps, abs_tol=1e-9):
        raise RuntimeError("formal saved-map maximum linear speed disagrees with profile")
    # AMCL supplies a lidar-map pose measurement only. The global EKF owns
    # map->odom in cleaning mode, so AMCL must never publish the same TF edge.
    nav2["amcl"]["ros__parameters"]["tf_broadcast"] = False
    # Make the product command chain explicit. Nav2 generates the virtual
    # Ackermann-constrained command, smoother feeds the scan-based collision
    # monitor, and only /cmd_vel_gate reaches whole_vehicle_safety_manager.
    # Jazzy velocity_smoother uses the fixed cmd_vel/cmd_vel_smoothed topic
    # interface. navigation.launch.py remaps its cmd_vel input to
    # /cmd_vel_nav; the output remains /cmd_vel_smoothed.
    # Pin the transitional Jazzy message contract instead of depending on a
    # distro patch-level default: the installed controller, smoother,
    # collision monitor and whole-vehicle manager all use unstamped Twist up
    # to /cmd_vel_gate.  The safety manager alone stamps its final command for
    # diff_drive_controller.
    for node_name in (
        "controller_server",
        "velocity_smoother",
        "collision_monitor",
    ):
        nav2[node_name]["ros__parameters"]["enable_stamped_cmd_vel"] = False
    nav2["collision_monitor"]["ros__parameters"].update({
        "cmd_vel_in_topic": "/cmd_vel_smoothed",
        "cmd_vel_out_topic": "/cmd_vel_gate",
    })
    # Mapping has no high-bandwidth 3D publisher by contract.  Narrow only
    # that runtime's collision monitor to the live, self-filtered 2D scan;
    # saved-map cleaning retains the formal high-bandwidth source set.
    configure_collision_monitor_sources(nav2, mission_mode=mode)
    canonical_scan = "/scan/navigation"
    nav2["amcl"]["ros__parameters"]["scan_topic"] = canonical_scan
    nav2["collision_monitor"]["ros__parameters"]["scan"]["topic"] = canonical_scan
    for costmap_name in ("local_costmap", "global_costmap"):
        nav2[costmap_name][costmap_name]["ros__parameters"][
            "obstacle_layer"
        ]["scan"]["topic"] = canonical_scan
    if mode == "mapping":
        # slam_toolbox initially sizes /map around laser returns.  At the
        # fixed open-boundary start, the physical base can lie just outside
        # that first tiny grid even though the lidar origin is inside it.  A
        # rolling global planning window keeps the robot in bounds while the
        # explorer still submits only sensor-known-free frontier goals.  Live
        # scan obstacles, public geofence filters and collision_monitor remain
        # active; no world occupancy is introduced.
        global_costmap = nav2["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]
        global_costmap.update({
            "rolling_window": True,
            # Nav2 declares these parameters as integers in the base config.
            # A YAML float override aborts planner_server before activation.
            "width": 30,
            "height": 30,
            "track_unknown_space": False,
        })
        global_costmap["plugins"] = [
            plugin
            for plugin in global_costmap["plugins"]
            if plugin != "static_layer"
        ]
        global_costmap.pop("static_layer", None)
    generated_nav2 = Path(tempfile.gettempdir()) / (
        f"tzcup_map_lifecycle_nav2_{os.getpid()}.yaml"
    )
    generated_nav2.write_text(yaml.safe_dump(nav2, sort_keys=False), encoding="utf-8")
    slam = yaml.safe_load(base_slam_params.read_text(encoding="utf-8"))
    slam["slam_toolbox"]["ros__parameters"]["scan_topic"] = canonical_scan
    # The final campus contains fixed houses, poles and service assets. Lidar
    # scan matching and loop closure therefore remain active; wheel+IMU EKF
    # provides the smooth odom prior and GNSS is an independent drift gate.
    # slam_toolbox remains the sole map->odom authority while mapping.
    slam_params = slam["slam_toolbox"]["ros__parameters"]
    slam_params["use_scan_matching"] = True
    slam_params["use_scan_barycenter"] = True
    slam_params["do_loop_closing"] = True
    # Karto builds each scan's map bounding box from readings at or below its
    # raster threshold, while an endpoint is occupied only below threshold by
    # KT_TOLERANCE.  Normalize physical +Inf no-return samples to the exact
    # threshold: the ray expands known free space but cannot form a 12 m ring.
    expected_sensor_range_max = 30.0
    slam_max_laser_range = float(slam_params["max_laser_range"])
    normalized_no_return_range = 12.0
    if slam_max_laser_range != normalized_no_return_range:
        raise RuntimeError(
            "slam max_laser_range must exactly match the normalized formal "
            "lidar no-return range"
        )
    if normalized_no_return_range >= expected_sensor_range_max:
        raise RuntimeError(
            "normalized no-return range must remain inside the physical range"
        )
    generated_slam = Path(tempfile.gettempdir()) / (
        f"tzcup_map_lifecycle_slam_{os.getpid()}.yaml"
    )
    generated_slam.write_text(
        yaml.safe_dump(slam, sort_keys=False), encoding="utf-8"
    )
    base_coverage_params = Path(
        context.perform_substitution(LaunchConfiguration("base_coverage_params_file"))
    )
    coverage = yaml.safe_load(base_coverage_params.read_text(encoding="utf-8"))
    coverage_parameters = coverage["coverage_server"]["ros__parameters"]
    transport_footprint = json.loads(
        nav2["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    )
    coverage_parameters["operation_width"] = cleaning_width
    coverage_parameters["robot_width"] = max(
        point[1] for point in transport_footprint
    ) - min(point[1] for point in transport_footprint)
    generated_coverage = Path(tempfile.gettempdir()) / (
        f"tzcup_map_lifecycle_coverage_{os.getpid()}.yaml"
    )
    generated_coverage.write_text(
        yaml.safe_dump(coverage, sort_keys=False), encoding="utf-8"
    )
    coverage_evidence_dir = Path(
        context.perform_substitution(LaunchConfiguration("coverage_evidence_dir"))
        or str(artifact_root / "saved_map_cleaning_runtime")
    ).resolve()

    base_launch = PathJoinSubstitution([
        FindPackageShare("sanitation_formal_campus_integration"),
        "launch",
        "formal_campus.launch.py",
    ])
    navigation_launch = PathJoinSubstitution([
        FindPackageShare("sanitation_navigation"), "launch", "navigation.launch.py"
    ])
    slam_launch = PathJoinSubstitution([
        FindPackageShare("sanitation_navigation"), "launch", "slam.launch.py"
    ])
    scan_filter_params = PathJoinSubstitution([
        FindPackageShare("sanitation_formal_campus_integration"),
        "config",
        "formal_utm30lx_self_filter.yaml",
    ])
    mapping = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mission_mode"), "' == 'mapping'"])
    )
    cleaning_coverage = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("mission_mode"), "' == 'cleaning' and '",
            LaunchConfiguration("cleaning_planner"), "' == 'full_coverage' and '",
            LaunchConfiguration("start_coverage"), "'.lower() in ('1','true','yes','on')",
        ])
    )
    return [
        # Reuse the formal physical vehicle, sensors, safety and pedestrians.
        # Its legacy world-materialized maps are deliberately not connected to
        # any product node in this lifecycle launch.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "world": LaunchConfiguration("world"),
                "world_name": LaunchConfiguration("world_name"),
                "episode_manifest": LaunchConfiguration("episode_manifest"),
                "pedestrian_schedule": LaunchConfiguration("pedestrian_schedule"),
                "start_pedestrians": LaunchConfiguration("start_pedestrians"),
                "enable_dynamic_footprint_runtime_test_override": LaunchConfiguration(
                    "enable_dynamic_footprint_runtime_test_override"
                ),
                "start_navigation": "false",
                "mission_mode": mode,
                "localization_backend": "slam" if mode == "mapping" else "amcl",
                "start_coverage": "false",
                "materialize_static_maps": "false",
                "runtime_artifact_dir": str(artifact_root),
                "high_bandwidth_sensor_runtime": (
                    "false" if mode == "mapping" else "true"
                ),
                        "motion_profile_file": LaunchConfiguration("motion_profile_file"),
                        "operation_speed_profile_file": LaunchConfiguration(
                            "operation_speed_profile_file"
                        ),
                "operation_speed_profile": LaunchConfiguration(
                            "operation_speed_profile"
                        ),
                "max_linear_velocity": LaunchConfiguration("max_linear_velocity"),
                "speed_qualification_state": LaunchConfiguration(
                    "speed_qualification_state"
                ),
                "base_nav2_params_file": LaunchConfiguration("base_nav2_params_file"),
            }.items(),
        ),
        # The formal UTM has two mesh-proven fixed self-occlusion sectors.
        # Produce one filtered scan before any consumer starts; SLAM, both
        # costmaps and collision_monitor all consume this same topic.
        Node(
            package="sanitation_formal_campus_integration",
            executable="formal-scan-self-filter",
            name="formal_scan_self_filter",
            parameters=[
                scan_filter_params,
                {
                    "use_sim_time": True,
                    "no_return_replacement_m": normalized_no_return_range,
                },
            ],
            output="screen",
        ),
        TimerAction(
            period=20.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_launch),
                    condition=mapping,
                    launch_arguments={
                        "use_sim_time": "true",
                        "params_file": str(generated_slam),
                        # whole_vehicle_safety_manager is the sole final writer;
                        # never start the legacy /cmd_vel republisher here.
                        "start_velocity_gate": "false",
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch),
                    launch_arguments={
                        "use_sim_time": "true",
                        "params_file": str(generated_nav2),
                        "footprint_profile": "formal_transport_stowed",
                        "map_file": str(artifact_root / "occupancy.yaml"),
                        "keepout_map": str(support["keepout_map"]),
                        "speed_map": str(support["speed_map"]),
                        "initial_pose_x": "0.0",
                        "initial_pose_y": "0.0",
                        "initial_pose_yaw": "0.0",
                        "localization_backend": "external" if mode == "mapping" else "amcl",
                        "start_velocity_gate": "false",
                    }.items(),
                ),
            ],
        ),
        TimerAction(
            period=25.0,
            actions=[
                # Jazzy nav2_bringup above owns collision_monitor and includes
                # it in lifecycle_manager_navigation. Do not create a second
                # same-name scan gate: two publishers on /cmd_vel_gate make
                # safety ownership ambiguous.
                Node(
                    package="sanitation_formal_campus_integration",
                    executable="formal-map-lifecycle-manager",
                    parameters=[{
                        "use_sim_time": True,
                        "mode": mode,
                        "episode_manifest": str(manifest_path),
                        "artifact_directory": str(artifact_root),
                        "support_artifacts_prepared": True,
                        "mapping_pose_source": (
                            "wheel_imu_ekf_lidar_scan_matching_gnss_consistency"
                        ),
                    }],
                    output="screen",
                ),
                Node(
                    package="sanitation_formal_campus_integration",
                    executable="formal-frontier-explorer",
                    condition=mapping,
                    parameters=[{
                        "use_sim_time": True,
                        "episode_manifest": str(manifest_path),
                    }],
                    output="screen",
                ),
            ],
        ),
        TimerAction(
            period=45.0,
            actions=[
                Node(
                    package="opennav_coverage",
                    executable="opennav_coverage",
                    name="coverage_server",
                    condition=cleaning_coverage,
                    parameters=[str(generated_coverage)],
                    output="screen",
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="formal_saved_map_coverage_lifecycle_manager",
                    condition=cleaning_coverage,
                    parameters=[{
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": ["coverage_server"],
                    }],
                ),
            ],
        ),
        TimerAction(
            period=55.0,
            actions=[
                Node(
                    package="sanitation_formal_campus_integration",
                    executable="formal-saved-map-coverage-executor",
                    name="formal_saved_map_coverage_executor",
                    condition=cleaning_coverage,
                    parameters=[{
                        "use_sim_time": True,
                        "mission_geometry_path": str(
                            artifact_root / "mission_geometry.yaml"
                        ),
                        "output_path": str(
                            coverage_evidence_dir / "coverage_execution.json"
                        ),
                        "operation_width_m": cleaning_width,
                        "maximum_linear_speed_mps": smoother_speed,
                        "operation_speed_profile": speed_profile.name,
                    }],
                    output="screen",
                )
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    repository_root = EnvironmentVariable("TZCUP_REPOSITORY_ROOT", default_value=".")
    return LaunchDescription([
        DeclareLaunchArgument("mission_mode", default_value="mapping"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world"),
        DeclareLaunchArgument("world_name", default_value="campus_formal"),
        DeclareLaunchArgument("episode_manifest"),
        DeclareLaunchArgument("map_artifact_dir"),
        DeclareLaunchArgument("pedestrian_schedule", default_value=""),
        DeclareLaunchArgument("start_pedestrians", default_value="true"),
        DeclareLaunchArgument(
            "enable_dynamic_footprint_runtime_test_override",
            default_value="false",
            description=(
                "Test-only dynamic-footprint override; base motion remains inhibited"
            ),
        ),
        DeclareLaunchArgument("start_coverage", default_value="true"),
        DeclareLaunchArgument("coverage_evidence_dir", default_value=""),
        DeclareLaunchArgument(
            "cleaning_planner",
            default_value="full_coverage",
            description="full_coverage fallback or rl_dirt_priority product planner",
        ),
        DeclareLaunchArgument(
            "motion_profile_file",
            default_value=PathJoinSubstitution([
                repository_root,
                "config",
                "high_fidelity_vehicle",
                "formal_motion_cleaning_profile.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "operation_speed_profile_file",
            default_value=PathJoinSubstitution([
                repository_root,
                "config",
                "high_fidelity_vehicle",
                "formal_operation_speed_profiles.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "operation_speed_profile",
            default_value=DRY_CLEANING_SPEED_PROFILE,
            description=(
                "Explicit formal runtime profile; cleaning defaults to dry 1.0 m/s, "
                "while mapping must select mapping_safe."
            ),
        ),
        DeclareLaunchArgument(
            "max_linear_velocity",
            default_value="0.45",
            description=(
                "Final whole-vehicle cap; retained at 0.45 m/s except for an "
                "explicit isolated requalification invocation."
            ),
        ),
        DeclareLaunchArgument(
            "speed_qualification_state",
            default_value="none",
            description=(
                "Fail-closed isolated same-map dry coverage state; all other "
                "modes remain capped at 0.45 m/s."
            ),
        ),
        DeclareLaunchArgument(
            "base_nav2_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("sanitation_navigation"), "config", "nav2.yaml"
            ]),
        ),
        DeclareLaunchArgument(
            "base_slam_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("sanitation_navigation"), "config", "slam.yaml"
            ]),
        ),
        DeclareLaunchArgument(
            "base_coverage_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("sanitation_coverage"),
                "config",
                "coverage_skid_steer_optimized.yaml",
            ]),
        ),
        OpaqueFunction(function=_runtime_actions),
    ])
