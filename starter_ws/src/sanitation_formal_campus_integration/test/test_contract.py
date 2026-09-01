import ast
import json
from pathlib import Path
import sys

import pytest
import yaml


PACKAGE = Path(__file__).parents[1]
ROOT = Path(__file__).parents[4]
sys.path.insert(0, str(PACKAGE))

from sanitation_formal_campus_integration.contract import (  # noqa: E402
    IntegrationContractError,
    materialize_nav2_config,
    resolve_spawn_pose,
)


MOTION_PROFILE = ROOT / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml"
BASE_NAV2 = ROOT / "starter_ws/src/sanitation_navigation/config/nav2.yaml"
INTEGRATION_CONFIG = PACKAGE / "config/formal_campus_integration.yaml"
LAUNCH = PACKAGE / "launch/formal_campus.launch.py"
RUNTIME_RUNNER = ROOT / "scripts/run_formal_campus_runtime.sh"
RUNTIME_VALIDATOR = ROOT / "scripts/validate_formal_campus_runtime.py"
RUNTIME_ISOLATION = ROOT / "scripts/run_formal_runtime_isolation.sh"


def test_integration_contract_names_skid_steer_and_canonical_profile():
    contract = yaml.safe_load(INTEGRATION_CONFIG.read_text(encoding="utf-8"))
    assert contract["vehicle_contract"]["kinematic_model"] == "four_wheel_skid_steer"
    assert contract["vehicle_contract"]["physical_steering_claim"] is False
    assert contract["motion_profile"]["repository_relative_path"] == (
        "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml"
    )
    assert contract["command_contract"]["formal_controller_type"] == (
        "geometry_msgs/msg/TwistStamped"
    )
    profile = yaml.safe_load(MOTION_PROFILE.read_text(encoding="utf-8"))
    drive = profile["drive"]
    assert drive["kinematic_model"] == "four_wheel_skid_steer"
    assert drive["steering_joint_names"] == []
    assert drive["canonical_planning_kinematic_constraint"] == (
        "curvature_limited_reference_path_for_skid_steer"
    )
    assert drive["canonical_constraint_claim"]["physical_steering_claim"] is False
    assert drive["canonical_constraint_claim"]["runtime_tracking_status"] == (
        "pending_skid_steer_tracking_validation"
    )
    assert contract["command_contract"]["nav2_controller_topic"] == "/cmd_vel_nav"
    assert contract["command_contract"]["nav2_collision_checked_topic"] == (
        "/cmd_vel_gate"
    )
    assert contract["command_contract"]["converter_owner"] == (
        "sanitation_safety/whole_vehicle_safety_manager"
    )


def test_nav2_costmaps_are_materialized_from_formal_motion_profile():
    config, cleaning_width = materialize_nav2_config(BASE_NAV2, MOTION_PROFILE)
    profile = yaml.safe_load(MOTION_PROFILE.read_text(encoding="utf-8"))
    expected = json.dumps(
        profile["motion_footprints"]["transport_stowed"]["footprint_xy_m"],
        separators=(",", ":"),
    )
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    global_ = config["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    assert local == expected
    assert global_ == expected
    assert "0.4,0.36" not in local.lower()
    assert cleaning_width == pytest.approx(1.32)

    # The real four-wheel base remains skid-steer.  RPP follows a
    # curvature-limited reference path without inserting an in-place
    # rotate-to-heading manoeuvre or claiming physical steering joints.
    assert "front-steered" not in BASE_NAV2.read_text(encoding="utf-8")
    controller = config["controller_server"]["ros__parameters"]
    for name in ("FollowPath", "CleanPath", "RepairPath"):
        assert controller[name]["allow_reversing"] is True
        assert controller[name]["use_rotate_to_heading"] is False


def test_materializer_rejects_ackermann_or_legacy_small_footprint(tmp_path):
    profile = yaml.safe_load(MOTION_PROFILE.read_text(encoding="utf-8"))
    profile["drive"]["kinematic_model"] = "ackermann"
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(profile), encoding="utf-8")
    with pytest.raises(IntegrationContractError, match="skid-steer"):
        materialize_nav2_config(BASE_NAV2, bad)

    profile = yaml.safe_load(MOTION_PROFILE.read_text(encoding="utf-8"))
    profile["motion_footprints"]["transport_stowed"]["footprint_xy_m"] = [
        [0.40, 0.36],
        [0.40, -0.36],
        [-0.40, -0.36],
        [-0.40, 0.36],
    ]
    bad.write_text(yaml.safe_dump(profile), encoding="utf-8")
    with pytest.raises(IntegrationContractError, match="legacy small footprint"):
        materialize_nav2_config(BASE_NAV2, bad)


def test_spawn_pose_defaults_to_public_manifest_and_allows_explicit_override(tmp_path):
    manifest = tmp_path / "episode_manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "vehicle_start_pose_map": {
                    "x_m": -98.0,
                    "y_m": 1.25,
                    "yaw_rad": 0.4,
                }
            }
        ),
        encoding="utf-8",
    )
    assert resolve_spawn_pose(manifest) == (-98.0, 1.25, 0.4)
    assert resolve_spawn_pose(manifest, spawn_x=-90.0, spawn_yaw=1.0) == (
        -90.0,
        1.25,
        1.0,
    )
    with pytest.raises(IntegrationContractError, match="finite"):
        resolve_spawn_pose(manifest, spawn_x=float("nan"))


def test_launch_is_parseable_and_keeps_safety_and_controller_ownership_explicit():
    source = LAUNCH.read_text(encoding="utf-8")
    ast.parse(source)
    assert "formal_vehicle_sim.launch.py" in source
    assert "formal_manipulation_acceptance.urdf.xacro" in source
    assert '"manipulation_sim_interfaces": "true"' in source
    assert "formal_physical_grasp.launch.py" in source
    assert '"start_controllers": "false"' in source
    assert '"enable_safety_manager": "false"' in source
    assert 'executable="whole_vehicle_safety_manager"' in source
    assert '"command_input_topic": "/cmd_vel_gate"' in source
    assert '"base_command_output_topic": "/base_controller/cmd_vel"' in source
    assert '"--inactive"' in source
    assert '"service_controller"' in source
    assert 'DeclareLaunchArgument("spawn_x"' in source
    assert 'DeclareLaunchArgument("spawn_y"' in source
    assert 'DeclareLaunchArgument("spawn_yaw"' in source
    assert "ackermann" not in source.lower()

    navigation_launch = (
        ROOT / "starter_ws/src/sanitation_navigation/launch/navigation.launch.py"
    ).read_text(encoding="utf-8")
    ast.parse(navigation_launch)
    assert "DeclareLaunchArgument('start_velocity_gate', default_value='true')" in navigation_launch
    assert "condition=IfCondition(LaunchConfiguration('start_velocity_gate'))" in navigation_launch


def test_launch_stages_dds_participants_and_bounds_controller_discovery():
    source = LAUNCH.read_text(encoding="utf-8")
    ast.parse(source)
    for period in (8.0, 12.0, 15.0, 20.0, 45.0):
        assert f"period={period}" in source
    assert '"--controller-manager-timeout"' in source
    assert '"180"' in source
    assert '"--service-call-timeout"' in source
    assert '"--switch-timeout"' in source
    assert source.index("period=20.0") < source.index("period=45.0")


def test_formal_campus_runner_locks_local_dds_and_fails_closed():
    runner = RUNTIME_RUNNER.read_text(encoding="utf-8")
    isolation = RUNTIME_ISOLATION.read_text(encoding="utf-8")
    assert runner.index("source /opt/ros/jazzy/setup.bash") < runner.index("set -u")
    assert runner.index('source "${stage1_setup}"') < runner.index(
        'source "${runtime_setup}"'
    ) < runner.index('source "${campus_setup}"')
    assert 'source "${repo_root}/scripts/run_formal_runtime_isolation.sh"' in runner
    assert 'formal_runtime_configure "${ROS_DOMAIN_ID}"' in runner
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST' in isolation
    assert 'simulation_initial_estop_active:=true' in runner
    assert "use 0..101 or 215..231" in isolation
    assert (
        '"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch '
        'sanitation_formal_campus_integration'
    ) in runner
    assert 'formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"' in runner
    assert 'kill -INT "${pid}"' in isolation
    assert "for signal in TERM KILL" in isolation
    assert 'kill -"${signal}" -- "-${pid}"' in isolation
    assert 'if (( status != 0 ))' in runner

    validator = RUNTIME_VALIDATOR.read_text(encoding="utf-8")
    ast.parse(validator)
    for marker in (
        '"/controller_manager"',
        '"/coverage_server"',
        '"/map"',
        '"/scan"',
        '"/sensors/lidar_3d/points"',
        '"/odom"',
        '"/emergency_stop"',
    ):
        assert marker in validator
    assert "all(probe.estop_values)" in validator
    assert '"PASSED" if all(gates.values()) else "BLOCKED"' in validator


def test_topic_adapter_contract_covers_formal_sensor_and_legacy_odom_names():
    contract = yaml.safe_load(INTEGRATION_CONFIG.read_text(encoding="utf-8"))
    assert contract["topic_aliases"] == {
        "/sensors/lidar_2d/scan": "/scan",
        "/sensors/imu/data": "/imu/data",
        "/sensors/gnss/fix": "/gnss/fix",
        "/sensors/front_rgbd/depth/image_rect_raw/image": "/camera/color/image_raw",
        "/sensors/front_rgbd/depth/image_rect_raw/depth_image": "/camera/depth/image_rect_raw",
        "/sensors/front_rgbd/depth/image_rect_raw/points": "/camera/depth/color/points",
        "/sensors/front_rgbd/depth/image_rect_raw/camera_info": "/camera/color/camera_info",
    }
    source = (
        PACKAGE
        / "sanitation_formal_campus_integration/topic_adapter.py"
    ).read_text(encoding="utf-8")
    ast.parse(source)
    assert "Odometry" not in source
    assert "base_controller/odom" not in source
    assert "publish_selected_odom" not in source
    assert "odom/unfiltered" not in source

    native = contract["native_sensor_topics"]
    assert native["/sensors/lidar_3d/points"] == "sensor_msgs/msg/PointCloud2"
    assert native["/sensors/wrist_rgbd/depth/image_rect_raw/points"] == (
        "sensor_msgs/msg/PointCloud2"
    )
    formal_launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    high_bandwidth_bridges = yaml.safe_load(
        (
            ROOT
            / "starter_ws/src/sanitation_vehicle_description/config/formal_high_bandwidth_sensor_bridge.yaml"
        ).read_text(encoding="utf-8")
    )
    configured_high_bandwidth_topics = {
        row["ros_topic_name"] for row in high_bandwidth_bridges
    }
    for topic in [
        *contract["topic_aliases"].keys(),
        *native.keys(),
    ]:
        assert (
            f'"{topic}@' in formal_launch
            or topic in configured_high_bandwidth_topics
        )

    localization = contract["localization_contract"]
    assert localization["default_backend"] == "nav2_amcl"
    assert localization["map_observation_topic"] == "/scan/navigation"
    assert localization["selected_odom_source"] == "/local_ekf"
    assert localization["raw_wheel_odom_source"] == "A300DrivetrainPlantSystem"
    assert localization["gnss_mapping_consistency_topic"] == "/odometry/gps"
    assert localization["gnss_used_for_mapping_consistency"] is True
    assert localization["gnss_fused_into_saved_map_backend"] is True
    nav2 = yaml.safe_load(BASE_NAV2.read_text(encoding="utf-8"))
    # The shared base Nav2 file remains reusable, but the formal lifecycle
    # materializes every scan consumer onto the canonical self-filter output.
    lifecycle = (
        PACKAGE / "launch" / "formal_campus_map_lifecycle.launch.py"
    ).read_text(encoding="utf-8")
    assert 'canonical_scan = "/scan/navigation"' in lifecycle
    assert 'nav2["amcl"]["ros__parameters"]["scan_topic"] = canonical_scan' in lifecycle
    assert nav2["bt_navigator"]["ros__parameters"]["odom_topic"] == "/odom"
    assert nav2["collision_monitor"]["ros__parameters"]["cmd_vel_in_topic"] == (
        "cmd_vel_smoothed"
    )
    assert nav2["collision_monitor"]["ros__parameters"]["cmd_vel_out_topic"] == (
        "/cmd_vel_gate"
    )
    controllers = yaml.safe_load(
        (
            ROOT
            / "starter_ws/src/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml"
        ).read_text(encoding="utf-8")
    )
    manager = controllers["controller_manager"]["ros__parameters"]
    assert "base_controller" not in manager
    assert "diff_drive_controller/DiffDriveController" not in str(controllers)
