import ast
import json
import math
from pathlib import Path
import sys

import pytest
import yaml


PACKAGE = Path(__file__).parents[1]
ROOT = Path(__file__).parents[4]
sys.path.insert(0, str(PACKAGE))

from sanitation_formal_campus_integration.contract import (  # noqa: E402
    IntegrationContractError,
    _navigation_inset_radius,
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
    local_parameters = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_parameters = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local_parameters["footprint_padding"] == pytest.approx(0.01)
    assert global_parameters["footprint_padding"] == pytest.approx(0.01)
    assert local_parameters["inflation_layer"]["inflation_radius"] == pytest.approx(0.56)
    assert global_parameters["inflation_layer"]["inflation_radius"] == pytest.approx(0.56)
    assert "0.4,0.36" not in local.lower()
    assert cleaning_width == pytest.approx(1.32)

    dry_config, _ = materialize_nav2_config(
        BASE_NAV2, MOTION_PROFILE, clean_path_speed_mps=1.0
    )
    assert dry_config["controller_server"]["ros__parameters"]["CleanPath"][
        "desired_linear_vel"
    ] == pytest.approx(1.0)
    assert dry_config["velocity_smoother"]["ros__parameters"]["max_velocity"][0] == pytest.approx(1.0)
    # Transit and recovery controller settings stay at their safety baseline;
    # only dry CleanPath is selected by the explicit runtime profile.
    assert dry_config["controller_server"]["ros__parameters"]["FollowPath"][
        "desired_linear_vel"
    ] == pytest.approx(0.45)

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


@pytest.mark.parametrize(
    ("padding", "message"),
    [(None, "exactly one"), (float("nan"), "must be finite"), (-0.01, "nonnegative")],
)
def test_materializer_rejects_missing_nonfinite_or_negative_footprint_padding(
    tmp_path, padding, message
):
    profile = yaml.safe_load(MOTION_PROFILE.read_text(encoding="utf-8"))
    if padding is None:
        profile.pop("nav2_footprint_padding_m")
    else:
        profile["nav2_footprint_padding_m"] = padding
    bad = tmp_path / "bad-padding-profile.yaml"
    bad.write_text(yaml.safe_dump(profile), encoding="utf-8")
    with pytest.raises(IntegrationContractError, match=message):
        materialize_nav2_config(BASE_NAV2, bad)


def test_materializer_rejects_duplicate_top_level_footprint_padding(tmp_path):
    source = MOTION_PROFILE.read_text(encoding="utf-8")
    duplicate = source.replace(
        "nav2_footprint_padding_m: 0.01",
        "nav2_footprint_padding_m: 0.01\nnav2_footprint_padding_m: 0.02",
        1,
    )
    path = tmp_path / "duplicate-padding-profile.yaml"
    path.write_text(duplicate, encoding="utf-8")
    with pytest.raises(IntegrationContractError, match="exactly one nav2_footprint_padding_m"):
        materialize_nav2_config(BASE_NAV2, path)


def test_materializer_rejects_costmap_inflation_at_or_below_enabled_inset_boundary(tmp_path):
    profile = yaml.safe_load(MOTION_PROFILE.read_text(encoding="utf-8"))
    # The disabled arm has a 1.05 m inradius and must not make the 0.56 m
    # navigation costmaps fail.  Only navigation_allowed footprints contribute.
    assert profile["motion_footprints"]["arm_deployed"]["navigation_allowed"] is False
    base = yaml.safe_load(BASE_NAV2.read_text(encoding="utf-8"))
    local = base["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = base["global_costmap"]["global_costmap"]["ros__parameters"]
    local["inflation_layer"]["inflation_radius"] = 0.55
    global_["inflation_layer"]["inflation_radius"] = 0.56
    boundary = tmp_path / "boundary-nav2.yaml"
    boundary.write_text(yaml.safe_dump(base), encoding="utf-8")
    with pytest.raises(IntegrationContractError, match="local_costmap inflation_radius"):
        materialize_nav2_config(boundary, MOTION_PROFILE)

    local["inflation_layer"]["inflation_radius"] = 0.56
    global_["inflation_layer"]["inflation_radius"] = 0.55
    inconsistent = tmp_path / "inconsistent-nav2.yaml"
    inconsistent.write_text(yaml.safe_dump(base), encoding="utf-8")
    with pytest.raises(IntegrationContractError, match="global_costmap inflation_radius"):
        materialize_nav2_config(inconsistent, MOTION_PROFILE)

    global_["inflation_layer"]["inflation_radius"] = 0.56
    accepted = tmp_path / "accepted-nav2.yaml"
    accepted.write_text(yaml.safe_dump(base), encoding="utf-8")
    config, _ = materialize_nav2_config(accepted, MOTION_PROFILE)
    assert config["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"][
        "inflation_radius"
    ] == pytest.approx(0.56)


def test_navigation_inset_radius_uses_nav2_sign_padded_vertices_not_plain_addition():
    profile = yaml.safe_load(MOTION_PROFILE.read_text(encoding="utf-8"))
    for footprint in profile["motion_footprints"].values():
        footprint["navigation_allowed"] = False
    profile["motion_footprints"]["transport_stowed"].update(
        {
            "navigation_allowed": True,
            "footprint_xy_m": [
                [0.40, 0.20],
                [-0.20, 0.40],
                [-0.40, -0.20],
                [0.20, -0.40],
            ],
        }
    )
    original = _navigation_inset_radius(profile, 0.0)
    padded = _navigation_inset_radius(profile, 0.01)
    # For the rotated diamond, Nav2's per-coordinate sign expansion produces
    # the edge (0.41, 0.21) -> (-0.21, 0.41), whose origin distance is not
    # the original inradius plus 0.01.
    expected = 0.2122 / math.hypot(0.20, 0.62)
    assert padded == pytest.approx(expected)
    assert padded != pytest.approx(original + 0.01)


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

    manifest.write_text(
        json.dumps(
            {
                "vehicle_start_pose_source_world": {"x_m": -98.0, "y_m": 1.25, "yaw_rad": 0.4},
                "vehicle_start_pose_map": {"x_m": 999.0, "y_m": 1.25, "yaw_rad": 0.4},
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(IntegrationContractError, match="disagree"):
        resolve_spawn_pose(manifest)


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
    # The public start must reach initial creation, not a delayed SetPose that
    # leaves NavSat and local wheel/IMU odom with different origins.
    for axis, index in (("x", 0), ("y", 1), ("yaw", 2)):
        assert f'"spawn_{axis}": str(source_pose[{index}])' in source
    assert 'executable="formal-spawn-initializer"' not in source
    # The bridge remains for the independent dynamic-pedestrian driver. The
    # absent vehicle initializer, not removal of the service, prevents a
    # post-start vehicle teleport.
    assert 'name="formal_campus_set_pose_bridge"' in source
    assert 'set_pose@ros_gz_interfaces/srv/SetEntityPose' in source
    assert "ackermann" not in source.lower()

    formal_vehicle_launch = (
        ROOT / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    ast.parse(formal_vehicle_launch)
    for axis in ("x", "y", "yaw"):
        assert f'DeclareLaunchArgument("spawn_{axis}", default_value="0.0")' in formal_vehicle_launch
    assert '"-x", spawn_x, "-y", spawn_y, "-Y", spawn_yaw, "-z", "0.005"' in formal_vehicle_launch
    assert '"-z", "0.005"' in formal_vehicle_launch

    lifecycle = (PACKAGE / "launch/formal_campus_map_lifecycle.launch.py").read_text(
        encoding="utf-8"
    )
    ast.parse(lifecycle)
    assert '"operation_speed_profile"' in lifecycle
    assert "default_value=DRY_CLEANING_SPEED_PROFILE" in lifecycle
    assert "mapping mode must retain the mapping_safe speed profile" in lifecycle
    assert "clean_path_speed_mps=speed_profile.maximum_linear_speed_mps" in lifecycle

    navigation_launch = (
        ROOT / "starter_ws/src/sanitation_navigation/launch/navigation.launch.py"
    ).read_text(encoding="utf-8")
    ast.parse(navigation_launch)
    assert "DeclareLaunchArgument('start_velocity_gate', default_value='true')" in navigation_launch
    assert "condition=IfCondition(LaunchConfiguration('start_velocity_gate'))" in navigation_launch


def test_launch_stages_dds_participants_and_bounds_controller_discovery():
    source = LAUNCH.read_text(encoding="utf-8")
    ast.parse(source)
    # The former 8 s slot performed a post-start SetPose. Creation now uses
    # the public start pose, while the remaining DDS/controller stages stay
    # ordered and bounded.
    for period in (12.0, 15.0, 20.0, 45.0):
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
    assert 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' in isolation
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
    product_bridge = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/FormalVehicleProductNativeBridge.cc"
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
            topic in product_bridge
            or topic in configured_high_bandwidth_topics
        )
    # Control-plane aliases moved from parameter_bridge into the native
    # product bridge; the high-bandwidth topics remain governed by the YAML
    # contract above.
    assert 'NativeBridgeSupport("formal_vehicle_product_native_bridge")' in product_bridge
    for ros_type, gazebo_type in (
        ("sensor_msgs::msg::LaserScan", "gz::msgs::LaserScan"),
        ("sensor_msgs::msg::NavSatFix", "gz::msgs::NavSat"),
        ("sensor_msgs::msg::Imu", "gz::msgs::IMU"),
    ):
        assert f"GazeboToRosEndpoint<{ros_type}, {gazebo_type}>" in product_bridge
    product_executable = formal_launch.index(
        'executable="formal_vehicle_product_native_bridge"'
    )
    product_node = formal_launch[
        formal_launch.rfind("Node(", 0, product_executable) :
        formal_launch.index("),", product_executable)
    ]
    assert 'name="formal_vehicle_product_bridge"' in product_node
    assert 'package="sanitation_gazebo_control"' in product_node
    assert 'parameter_bridge' not in product_node

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
