"""Integrate the formal skid-steer vehicle with one generated campus episode."""

from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path
from types import SimpleNamespace

import yaml
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from sanitation_formal_campus_integration.campus_materializer import (
    materialize_campus_artifacts,
)
from sanitation_formal_campus_integration.contract import (
    materialize_nav2_config,
    resolve_spawn_pose,
)
from sanitation_formal_campus_integration.saved_map_coverage_core import (
    DRY_CLEANING_SPEED_PROFILE,
    load_formal_operation_speed_profile,
)


def _runtime_actions(context):  # type: ignore[no-untyped-def]
    motion_profile = Path(
        context.perform_substitution(LaunchConfiguration("motion_profile_file"))
    )
    base_nav2 = Path(
        context.perform_substitution(LaunchConfiguration("base_nav2_params_file"))
    )
    speed_profile_file = Path(
        context.perform_substitution(LaunchConfiguration("operation_speed_profile_file"))
    )
    speed_profile = load_formal_operation_speed_profile(
        speed_profile_file,
        context.perform_substitution(LaunchConfiguration("operation_speed_profile")),
    )
    episode_manifest = Path(
        context.perform_substitution(LaunchConfiguration("episode_manifest"))
    )
    world = Path(context.perform_substitution(LaunchConfiguration("world")))
    if not motion_profile.is_file():
        raise RuntimeError(f"formal motion profile not found: {motion_profile}")
    if not episode_manifest.is_file():
        raise RuntimeError(f"public episode manifest not found: {episode_manifest}")
    if not world.is_file():
        raise RuntimeError(f"public scenario world not found: {world}")
    spawn_values = {
        name: float(context.perform_substitution(LaunchConfiguration(name)))
        for name in ("spawn_x", "spawn_y", "spawn_yaw")
    }
    source_pose = resolve_spawn_pose(
        episode_manifest,
        spawn_x=None if math.isnan(spawn_values["spawn_x"]) else spawn_values["spawn_x"],
        spawn_y=None if math.isnan(spawn_values["spawn_y"]) else spawn_values["spawn_y"],
        spawn_yaw=(
            None if math.isnan(spawn_values["spawn_yaw"]) else spawn_values["spawn_yaw"]
        ),
    )
    artifact_directory_text = context.perform_substitution(
        LaunchConfiguration("runtime_artifact_dir")
    )
    artifact_directory = (
        Path(artifact_directory_text)
        if artifact_directory_text
        else Path(tempfile.gettempdir()) / f"tzcup_formal_campus_{os.getpid()}"
    )
    materialize_static_maps = context.perform_substitution(
        LaunchConfiguration("materialize_static_maps")
    ).strip().lower() in {"1", "true", "yes", "on"}
    if materialize_static_maps:
        artifacts = materialize_campus_artifacts(
            episode_manifest,
            world,
            motion_profile,
            artifact_directory,
            resolution=float(
                context.perform_substitution(LaunchConfiguration("map_resolution"))
            ),
            safety_margin_m=float(
                context.perform_substitution(LaunchConfiguration("map_safety_margin"))
            ),
            slow_zone_width_m=float(
                context.perform_substitution(LaunchConfiguration("slow_zone_width"))
            ),
            slow_zone_percent=int(
                context.perform_substitution(LaunchConfiguration("slow_zone_percent"))
            ),
            start_pose_override=source_pose,
        )
    else:
        # The first-map product lifecycle must not construct a world-derived
        # occupancy map, even as an unused side effect.  It supplies only
        # public geofence masks; SLAM is the sole occupancy-map producer.
        artifacts = SimpleNamespace(
            world_name=context.perform_substitution(LaunchConfiguration("world_name")),
            start_pose=source_pose,
            occupancy_map=artifact_directory / "occupancy.yaml",
            keepout_map=artifact_directory / "geofence_keepout.yaml",
            speed_map=artifact_directory / "neutral_speed.yaml",
            mission_geometry=artifact_directory / "mission_geometry.yaml",
        )
    configured_world_name = context.perform_substitution(
        LaunchConfiguration("world_name")
    )
    if configured_world_name != artifacts.world_name:
        raise RuntimeError(
            "world_name launch argument disagrees with public world SDF: "
            f"argument={configured_world_name}, sdf={artifacts.world_name}"
        )
    pedestrian_schedule_text = context.perform_substitution(
        LaunchConfiguration("pedestrian_schedule")
    )
    pedestrian_schedule = (
        Path(pedestrian_schedule_text)
        if pedestrian_schedule_text
        else episode_manifest.parent.parent
        / "environment"
        / "pedestrian_schedule.json"
    )
    pedestrians_enabled = context.perform_substitution(
        LaunchConfiguration("start_pedestrians")
    ).strip().lower() in {"1", "true", "yes", "on"}
    if pedestrians_enabled and not pedestrian_schedule.is_file():
        raise RuntimeError(
            "environment pedestrian schedule is required when start_pedestrians=true: "
            f"{pedestrian_schedule}"
        )
    nav2_config, cleaning_width = materialize_nav2_config(
        base_nav2,
        motion_profile,
        clean_path_speed_mps=speed_profile.maximum_linear_speed_mps,
    )
    generated_nav2 = Path(tempfile.gettempdir()) / (
        f"tzcup_formal_campus_nav2_{os.getpid()}.yaml"
    )
    generated_nav2.write_text(
        yaml.safe_dump(nav2_config, sort_keys=False), encoding="utf-8"
    )

    formal_launch = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_vehicle_description"),
            "launch",
            "formal_vehicle_sim.launch.py",
        ]
    )
    manipulation_model = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_manipulation"),
            "urdf",
            "formal_manipulation_acceptance.urdf.xacro",
        ]
    )
    manipulation_launch = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_manipulation"),
            "launch",
            "formal_physical_grasp.launch.py",
        ]
    )
    navigation_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_navigation"), "launch", "navigation.launch.py"]
    )
    localization_launch = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_localization"),
            "launch",
            "formal_localization_fusion.launch.py",
        ]
    )
    localization_backend = context.perform_substitution(
        LaunchConfiguration("localization_backend")
    ).strip()
    start_global_fusion = "false" if localization_backend == "slam" else "true"
    coverage_params = PathJoinSubstitution(
        [FindPackageShare("sanitation_coverage"), "config", "coverage_skid_steer_optimized.yaml"]
    )
    position_hold_controllers = [
        "arm_controller",
        "gripper_controller",
        "cleaning_controller",
        "storage_controller",
        "service_controller",
    ]
    safety_switched_controllers = [
        "brush_controller",
        "recovery_controller",
    ]

    return [
        LogInfo(
            msg=[
                "Formal campus integration uses canonical skid-steer profile ",
                str(motion_profile),
                "; operation speed profile: ",
                speed_profile.name,
                "; generated Nav2 params: ",
                str(generated_nav2),
                "; public-only maps and mission geometry: ",
                str(artifact_directory),
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(formal_launch),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "world": LaunchConfiguration("world"),
                "model": manipulation_model,
                "manipulation_sim_interfaces": "true",
                # The integration layer loads the base controller and leaves
                # all managed actuators inactive for the safety manager.
                "start_controllers": "false",
                # The campus layer starts the only whole-vehicle safety
                # manager below because it must consume Nav2's /cmd_vel_gate.
                # Leaving the description launch's default manager enabled
                # creates two switch-controller clients which alternately
                # activate and deactivate the brush/recovery controllers.
                "enable_safety_manager": "false",
                "start_simulation_safety_inputs": LaunchConfiguration(
                    "start_simulation_safety_inputs"
                ),
                "start_localization": "true",
                "simulation_initial_estop_active": LaunchConfiguration(
                    "simulation_initial_estop_active"
                ),
                "high_bandwidth_sensor_runtime": LaunchConfiguration(
                    "high_bandwidth_sensor_runtime"
                ),
                # Place the vehicle before NavSat/local-EKF startup.  A later
                # SetPose changes the GNSS world datum but not wheel/IMU odom,
                # producing a false ~98 m GNSS-consistency failure.
                "spawn_x": str(source_pose[0]),
                "spawn_y": str(source_pose[1]),
                "spawn_yaw": str(source_pose[2]),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manipulation_launch),
        ),
        # Dynamic pedestrian motion still uses this public-world service. Its
        # presence does not authorize moving the vehicle after initial create.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="formal_campus_set_pose_bridge",
            arguments=[
                PathJoinSubstitution(
                    [
                        "/world",
                        LaunchConfiguration("world_name"),
                        "set_pose@ros_gz_interfaces/srv/SetEntityPose",
                    ]
                )
            ],
            output="screen",
        ),
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    name="formal_campus_position_controller_spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        *position_hold_controllers,
                        "--controller-manager",
                        "/controller_manager",
                        "--controller-manager-timeout",
                        "180",
                        "--service-call-timeout",
                        "60",
                        "--switch-timeout",
                        "60",
                        "--activate-as-group",
                    ],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    name="formal_campus_velocity_controller_loader",
                    arguments=[
                        *safety_switched_controllers,
                        "--controller-manager",
                        "/controller_manager",
                        "--controller-manager-timeout",
                        "180",
                        "--service-call-timeout",
                        "60",
                        "--switch-timeout",
                        "60",
                        "--inactive",
                    ],
                    output="screen",
                ),
            ],
        ),
        Node(
            package="sanitation_formal_campus_integration",
            executable="formal-legacy-topic-adapter",
            name="formal_legacy_topic_adapter",
            output="screen",
            parameters=[{
                "use_sim_time": True,
            }],
        ),
        Node(
            package="sanitation_formal_campus_integration",
            executable="formal-dynamic-footprint-manager",
            name="formal_dynamic_footprint_manager",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "motion_profile_file": LaunchConfiguration("motion_profile_file"),
                # Default-off test endpoint: production launches never create
                # it. The ROS-only footprint gate must opt in explicitly.
                "enable_runtime_test_override": ParameterValue(
                    LaunchConfiguration(
                        "enable_dynamic_footprint_runtime_test_override"
                    ),
                    value_type=bool,
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "use_sim_time": "true",
                "start_local_fusion": "false",
                # The vehicle launch already owns the one navsat_transform
                # instance. This include adds only the saved-map global EKF.
                "start_navsat_transform": "false",
                "start_global_fusion": start_global_fusion,
            }.items(),
        ),
        Node(
            package="sanitation_safety",
            executable="whole_vehicle_safety_manager",
            name="whole_vehicle_safety_manager",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    # Nav2 collision_monitor emits its checked Twist on
                    # /cmd_vel_gate.  The formal manager is the sole
                    # TwistStamped controller writer.
                    "command_input_topic": "/cmd_vel_gate",
                    "base_command_output_topic": "/base_controller/cmd_vel",
                    "max_linear_velocity": ParameterValue(
                        LaunchConfiguration("max_linear_velocity"), value_type=float
                    ),
                    "max_angular_velocity": ParameterValue(
                        LaunchConfiguration("max_angular_velocity"), value_type=float
                    ),
                    # Empty/default values are deliberately not eligible for
                    # high speed.  The lifecycle wrapper supplies the exact
                    # dry same-map scope when it has independently qualified it.
                    "mission_mode": LaunchConfiguration("mission_mode"),
                    "operation_speed_profile": LaunchConfiguration(
                        "operation_speed_profile"
                    ),
                    "speed_qualification_state": LaunchConfiguration(
                        "speed_qualification_state"
                    ),
                }
            ],
        ),
        # Fast DDS participant discovery becomes nondeterministic when the
        # simulator, bridges, all Nav2 servers, filters and coverage are
        # forked in one scheduler tick on WSL.  Start the vehicle graph first,
        # then add the autonomy groups in bounded phases.  The E-stop remains
        # asserted throughout these delays, so staging cannot permit motion.
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package="sanitation_campus_scenario",
                    executable="sanitation-campus-pedestrian-driver",
                    name="campus_pedestrian_driver",
                    condition=IfCondition(LaunchConfiguration("start_pedestrians")),
                    parameters=[
                        {
                            "use_sim_time": True,
                            "schedule_path": str(pedestrian_schedule),
                        }
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=20.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch),
                    condition=IfCondition(LaunchConfiguration("start_navigation")),
                    launch_arguments={
                        "use_sim_time": "true",
                        "params_file": str(generated_nav2),
                        "footprint_profile": "formal_transport_stowed",
                        "map_file": str(artifacts.occupancy_map),
                        "keepout_map": str(artifacts.keepout_map),
                        "speed_map": str(artifacts.speed_map),
                        "initial_pose_x": str(artifacts.start_pose[0]),
                        "initial_pose_y": str(artifacts.start_pose[1]),
                        "initial_pose_yaw": str(artifacts.start_pose[2]),
                        "localization_backend": LaunchConfiguration("localization_backend"),
                        # The whole-vehicle manager above owns the only safety
                        # gate. Do not start the legacy /cmd_vel writer.
                        "start_velocity_gate": "false",
                    }.items(),
                )
            ],
        ),
        TimerAction(
            period=45.0,
            actions=[
                Node(
                    package="opennav_coverage",
                    executable="opennav_coverage",
                    name="coverage_server",
                    condition=IfCondition(LaunchConfiguration("start_coverage")),
                    parameters=[
                        coverage_params,
                        {
                            "robot_width": cleaning_width,
                            "operation_width": cleaning_width,
                        },
                    ],
                    output="screen",
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="formal_campus_coverage_lifecycle_manager",
                    condition=IfCondition(LaunchConfiguration("start_coverage")),
                    parameters=[
                        {
                            "use_sim_time": True,
                            "autostart": True,
                            "node_names": ["coverage_server"],
                        }
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    repository_root = EnvironmentVariable("TZCUP_REPOSITORY_ROOT", default_value=".")
    default_motion_profile = PathJoinSubstitution(
        [
            repository_root,
            "config",
            "high_fidelity_vehicle",
            "formal_motion_cleaning_profile.yaml",
        ]
    )
    default_nav2 = PathJoinSubstitution(
        [FindPackageShare("sanitation_navigation"), "config", "nav2.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("world", description="Generated public/world.sdf path"),
            DeclareLaunchArgument("world_name", default_value="campus_formal"),
            DeclareLaunchArgument(
                "episode_manifest",
                description="Generated public/episode_manifest.json path",
            ),
            DeclareLaunchArgument("pedestrian_schedule", default_value=""),
            DeclareLaunchArgument("start_pedestrians", default_value="true"),
            DeclareLaunchArgument(
                "start_simulation_safety_inputs", default_value="true"
            ),
            # Power-on remains fail-closed. Operators must explicitly command
            # A live operator gate must continuously command main_power=true
            # and emergency_stop=false before motion is allowed.
            DeclareLaunchArgument(
                "simulation_initial_estop_active", default_value="true"
            ),
            DeclareLaunchArgument(
                "high_bandwidth_sensor_runtime", default_value="true"
            ),
            DeclareLaunchArgument("spawn_x", default_value="nan"),
            DeclareLaunchArgument("spawn_y", default_value="nan"),
            DeclareLaunchArgument("spawn_yaw", default_value="nan"),
            DeclareLaunchArgument(
                "motion_profile_file", default_value=default_motion_profile
            ),
            DeclareLaunchArgument(
                "enable_dynamic_footprint_runtime_test_override",
                default_value="false",
                description=(
                    "Test-only dynamic-footprint override endpoint. Keep false "
                    "outside the ROS-only runtime footprint gate."
                ),
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
            ),
            DeclareLaunchArgument(
                "base_nav2_params_file", default_value=default_nav2
            ),
            DeclareLaunchArgument("runtime_artifact_dir", default_value=""),
            DeclareLaunchArgument(
                "materialize_static_maps",
                default_value="true",
                description=(
                    "Create the legacy public-world occupancy map. The formal "
                    "SLAM lifecycle sets this false so no world-derived map is "
                    "created or connected."
                ),
            ),
            DeclareLaunchArgument("map_resolution", default_value="0.10"),
            DeclareLaunchArgument("map_safety_margin", default_value="0.15"),
            DeclareLaunchArgument("slow_zone_width", default_value="1.0"),
            DeclareLaunchArgument("slow_zone_percent", default_value="50"),
            DeclareLaunchArgument("start_navigation", default_value="true"),
            DeclareLaunchArgument("start_coverage", default_value="true"),
            DeclareLaunchArgument("localization_backend", default_value="amcl"),
            DeclareLaunchArgument("mission_mode", default_value=""),
            DeclareLaunchArgument("max_linear_velocity", default_value="0.45"),
            DeclareLaunchArgument("max_angular_velocity", default_value="0.35"),
            DeclareLaunchArgument("speed_qualification_state", default_value="none"),
            OpaqueFunction(function=_runtime_actions),
        ]
    )
