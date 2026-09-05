from pathlib import Path
import sys

import yaml


sys.path.insert(0, str(Path(__file__).resolve().parent))
from formal_vehicle_sensor_runtime_contract import (  # noqa: E402
    A300_WHEEL_JOINTS,
    FORMAL_SENSOR_GROUPS,
    SOURCE_FREQUENCY_SAMPLE_TARGETS,
    STREAM_CONTRACTS,
    observed_frequency_hz,
    validate_runtime_contract,
)


ROOT = Path(__file__).resolve().parents[1]


def test_sensor_collector_binds_current_plant_and_all_formal_sensors():
    source = (ROOT / "scripts/collect_formal_vehicle_sensor_runtime.py").read_text(
        encoding="utf-8"
    )
    for topic in (
        "/sensors/lidar_2d/scan",
        "/sensors/lidar_3d/points",
        "/sensors/front_rgbd/depth/image_rect_raw/image",
        "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
        "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
        "/sensors/front_rgbd/infra1/image_rect_raw",
        "/sensors/front_rgbd/infra1/image_rect_raw/camera_info",
        "/sensors/front_rgbd/infra2/image_rect_raw",
        "/sensors/front_rgbd/infra2/image_rect_raw/camera_info",
        "/sensors/wrist_rgbd/depth/image_rect_raw/image",
        "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image",
        "/sensors/wrist_rgbd/depth/image_rect_raw/camera_info",
        "/sensors/wrist_rgbd/infra1/image_rect_raw",
        "/sensors/wrist_rgbd/infra1/image_rect_raw/camera_info",
        "/sensors/wrist_rgbd/infra2/image_rect_raw",
        "/sensors/wrist_rgbd/infra2/image_rect_raw/camera_info",
        "/sensors/rear_left_fisheye/image_raw",
        "/sensors/rear_left_fisheye/camera_info",
        "/sensors/rear_right_fisheye/image_raw",
        "/sensors/rear_right_fisheye/camera_info",
        "/sensors/gnss/fix",
        "/sensors/imu/data",
        "/formal_vehicle/encoders/a300/counts",
        "/formal_vehicle/encoders/a300/joint_states",
        "/odom/unfiltered",
    ):
        assert topic in source
    assert "libA300DrivetrainPlantSystem.so" in source
    assert "CameraInfo" in source
    assert '"all_camera_info_intrinsics_valid"' in source
    assert "tzcup_formal_vehicle_headless_runtime_v5" in source
    assert '"base_controller" not in states' in source
    assert "wastewater_payload_clamp_kg\": 8.30" in source
    assert 'findall(".//wastewater_capacity_kg")' in source
    assert '"fisheye_camera_info_matches_nominal_equisolid_gazebo_projection"' in source
    assert "expected_fisheye_distortion" in source
    assert "validate_runtime_contract" in source
    assert "acceptance_session_binding" in source
    assert "preembedded_sensor_world_binding" in source
    assert "validate_preembedded_sensor_world" in source
    assert 'temporary.replace(output)' in source


def test_runtime_contract_names_exactly_the_eight_formal_sensor_groups():
    assert tuple(FORMAL_SENSOR_GROUPS) == (
        "utm30lx_2d_lidar",
        "mid360_3d_lidar",
        "front_d435",
        "wrist_d435",
        "rear_left_fisheye",
        "rear_right_fisheye",
        "zed_f9p_gnss",
        "imu",
    )


def test_sensor_runner_uses_safety_manager_and_fresh_overlay():
    source = (ROOT / "scripts/run_formal_vehicle_sensor_runtime.sh").read_text(
        encoding="utf-8"
    )
    assert "FORMAL_SENSOR_RUNTIME_SETUP" in source
    assert "enable_safety_manager:=true" in source
    assert "simulation_initial_estop_active:=true" in source
    assert "collect_formal_vehicle_sensor_runtime.py" in source
    assert "runtime_binding" in source
    assert "stale_paths=(" in source
    assert "archive or isolate every prior attempt artifact" in source
    assert "validate_formal_fov_occlusion.py" in source
    assert "FORMAL_ACCEPTANCE_SESSION" in source
    assert "FORMAL_SENSOR_SNAPSHOT" in source
    assert "FORMAL_SENSOR_FOV_OUTPUT" in source
    assert '"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch' in source
    # The formal world uses Ogre2 Sensors.  A server-only launch without the
    # offscreen renderer creates bridge endpoints but produces no GPU lidar or
    # camera frames, so the complete stream gate must opt into headless
    # rendering rather than weakening its required topics.
    assert "gui:=false headless_rendering:=true" in source
    assert "high_bandwidth_sensor_runtime:=true" in source
    assert "prepare_formal_preembedded_sensor_world.py" in source
    assert "spawn_robot:=false" in source
    assert "preembedded_sensor_world" in source
    assert "FORMAL_SENSOR_PREEMBEDDED_MODEL_POSE" in source
    assert '--model-pose "${preembedded_model_pose}"' in source
    assert 'installed_package_share="$(ros2 pkg prefix --share sanitation_vehicle_description)"' in source
    assert 'expected_package_share="${install_root}/share/sanitation_vehicle_description"' in source
    assert "resolves outside the frozen runtime install" in source
    assert 'installed_controller_config="${installed_package_share}/config/formal_vehicle_controllers.yaml"' in source
    assert '--controller-config "${installed_controller_config}"' in source
    assert '--runtime-install-root "${install_root}"' in source
    assert "--preembedded-report" in source
    assert "--preembedded-world" in source
    assert 'formal_runtime_memory_preflight "${memory_preflight_prefix}"' in source
    assert (
        'formal_runtime_start_memory_watchdog "${launch_pid}" '
        '"${memory_watchdog_prefix}"'
    ) in source
    assert source.index("validate_formal_fov_occlusion.py") < source.index(
        "ros2 launch sanitation_vehicle_description"
    )


def test_sensor_runtime_gate_reconciles_both_frozen_snapshot_hashes():
    contract = (
        ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml"
    ).read_text(encoding="utf-8")
    sensor_gate = contract.split("  sensor_runtime:\n", 1)[1].split(
        "  manipulator_trajectory:\n", 1
    )[0]
    assert (
        "snapshot_urdf_hash_field: "
        "acceptance_session_binding.snapshot.expanded_urdf_sha256"
    ) in sensor_gate
    assert (
        "snapshot_source_hash_field: "
        "acceptance_session_binding.snapshot.source_inventory_sha256"
    ) in sensor_gate


def test_collector_does_not_shadow_rclpy_subscription_registry():
    source = (ROOT / "scripts/collect_formal_vehicle_sensor_runtime.py").read_text(
        encoding="utf-8"
    )
    assert "self._topic_subscriptions: dict[str, Any] = {}" in source
    assert "self._description_subscription = self.create_subscription(" in source
    assert "self._topic_subscriptions.append(" not in source
    assert "self._subscriptions = []" not in source


def test_collector_bounds_and_retires_high_bandwidth_subscriptions():
    source = (ROOT / "scripts/collect_formal_vehicle_sensor_runtime.py").read_text(
        encoding="utf-8"
    )
    assert "HistoryPolicy.KEEP_LAST" in source
    assert "depth=1" in source
    assert "ReliabilityPolicy.BEST_EFFORT" in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "RELIABLE_FRAGMENTED_TOPICS" in source
    assert "controller_plane_ready" in source
    assert "node.start_topic_subscriptions()" in source
    assert "node.retire_ready_subscriptions()" in source
    assert "self.destroy_subscription(subscription)" in source
    assert "self.controllers.remove_pending_request(future)" in source
    assert "rclpy.try_shutdown()" in source
    assert "SOURCE_FREQUENCY_SAMPLE_TARGETS.get(topic, 3)" in source


def test_transient_sensitive_cadence_uses_stable_bounded_source_timestamp_windows():
    assert SOURCE_FREQUENCY_SAMPLE_TARGETS == {
        "/sensors/front_rgbd/depth/image_rect_raw/image": 10,
        "/sensors/wrist_rgbd/depth/image_rect_raw/image": 32,
        "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image": 32,
        "/sensors/wrist_rgbd/infra1/image_rect_raw": 32,
        "/sensors/wrist_rgbd/infra2/image_rect_raw": 32,
        "/sensors/imu/data": 50,
    }
    source = (ROOT / "scripts/collect_formal_vehicle_sensor_runtime.py").read_text(
        encoding="utf-8"
    )
    assert "len(unique_source_stamps) >= required_source_stamps" in source
    assert '"observed_source_timestamp_sample_counts"' in source
    assert '"source_frequency_sample_targets"' in source


def test_high_bandwidth_and_visual_bridges_are_bounded_and_isolated_from_controls():
    source = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    product = source.split('name="formal_vehicle_product_bridge"', 1)[1].split(
        'name="formal_vehicle_high_bandwidth_sensor_bridge"', 1
    )[0]
    high_bandwidth = source.split(
        'name="formal_vehicle_high_bandwidth_sensor_bridge"', 1
    )[1].split('name="formal_vehicle_visual_bridge"', 1)[0]
    high_bandwidth_config = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/config/formal_high_bandwidth_sensor_bridge.yaml"
    ).read_text(encoding="utf-8")
    rows = yaml.safe_load(high_bandwidth_config)
    visual_config = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/config/formal_visual_sensor_bridge.yaml"
    ).read_text(encoding="utf-8")
    visual_rows = yaml.safe_load(visual_config)
    assert "/sensors/front_rgbd/depth/image_rect_raw/image" not in product
    assert "/formal_visual/front_left" not in product
    assert "formal_high_bandwidth_sensor_bridge.yaml" in high_bandwidth
    assert "/sensors/front_rgbd/depth/image_rect_raw/image" in high_bandwidth_config
    assert "/sensors/front_rgbd/depth/image_rect_raw/points" in high_bandwidth_config
    assert high_bandwidth_config.count("lazy: true") == 21
    assert high_bandwidth_config.count("subscriber_queue: 1") == 21
    assert high_bandwidth_config.count("publisher_queue: 1") == 21
    assert high_bandwidth_config.count("qos_profile: SENSOR_DATA") == 18
    assert high_bandwidth_config.count("qos_profile: SYSTEM_DEFAULT") == 3
    assert len(rows) == 21
    assert len({row["ros_topic_name"] for row in rows}) == len(rows)
    assert all(row["direction"] == "GZ_TO_ROS" for row in rows)
    assert all(row["ros_topic_name"].startswith("/sensors/") for row in rows)
    reliable_topics = {
        row["ros_topic_name"]
        for row in rows
        if row["qos_profile"] == "SYSTEM_DEFAULT"
    }
    assert reliable_topics == {
        "/sensors/lidar_3d/points",
        "/sensors/rear_left_fisheye/image_raw",
        "/sensors/rear_right_fisheye/image_raw",
    }
    assert 'package="ros_gz_image"' in source
    assert 'executable="image_bridge"' in source
    assert "arguments=visual_image_topics" in source
    # Formal 1600x1000 raw visual frames are about 4.8 MB each. The Gazebo
    # image bridge and capture subscriber intentionally use Reliable QoS so
    # those triggered frames are not dropped; navigation sensor bridges remain
    # bounded SENSOR_DATA streams above.
    assert 'parameters=[{"qos": "default"}]' in source
    assert len(visual_rows) == 19
    assert all(row["qos_profile"] == "DEFAULT_RELIABLE" for row in visual_rows)
    assert "condition=IfCondition(visual_acceptance_runtime)" in source
    assert '"visual_acceptance_runtime",\n                default_value="false"' in source
    assert "formal_visual_sensor_bridge.yaml" in source
    assert "/formal_visual/front_left" in visual_config
    assert len(visual_rows) == 19
    assert len({row["ros_topic_name"] for row in visual_rows}) == 19
    assert all(
        row["ros_topic_name"] == row["gz_topic_name"] for row in visual_rows
    )
    assert all(row["ros_type_name"] == "sensor_msgs/msg/Image" for row in visual_rows)
    assert all(row["gz_type_name"] == "gz.msgs.Image" for row in visual_rows)
    assert all(row["direction"] == "GZ_TO_ROS" for row in visual_rows)
    assert all(row["qos_profile"] == "DEFAULT_RELIABLE" for row in visual_rows)
    assert all("subscriber_queue" not in row for row in visual_rows)
    assert all("publisher_queue" not in row for row in visual_rows)
    assert all("lazy" not in row for row in visual_rows)


def test_formal_launch_can_disable_only_dynamic_usercommands_spawn():
    source = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument(\n                "spawn_robot"' in source
    create = source.split('package="ros_gz_sim",\n                executable="create"', 1)[1].split(
        'name="formal_vehicle_product_bridge"', 1
    )[0]
    assert "condition=IfCondition(spawn_robot)" in create


def test_formal_d435_ir_cameras_publish_explicit_camera_info_topics():
    source = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/sensor_suite.xacro"
    ).read_text(encoding="utf-8")
    assert "<camera_info_topic>${ir_left_topic}/camera_info</camera_info_topic>" in source
    assert "<camera_info_topic>${ir_right_topic}/camera_info</camera_info_topic>" in source


def _valid_runtime_rows():
    samples = {topic: 6 for topic in STREAM_CONTRACTS}
    metadata = {
        topic: {"frame_id": contract.get("frame_id", "")}
        for topic, contract in STREAM_CONTRACTS.items()
    }
    for topic, contract in STREAM_CONTRACTS.items():
        if "size" in contract:
            metadata[topic].update(width=contract["size"][0], height=contract["size"][1])
    metadata["/sensors/lidar_2d/scan"].update(
        range_min_m=0.1,
        range_max_m=30.0,
        angle_min_rad=-2.356194,
        angle_max_rad=2.356194,
        range_count=1080,
    )
    metadata["/sensors/lidar_3d/points"].update(width=1800, height=64, point_step=32)
    metadata["/sensors/gnss/fix"].update(latitude=39.9, longitude=116.4, altitude=45.0)
    metadata["/sensors/imu/data"]["finite_measurement"] = True
    metadata["/formal_vehicle/encoders/a300/counts"].update(
        layout_label="joint_order:" + ",".join(A300_WHEEL_JOINTS), data_length=4
    )
    metadata["/formal_vehicle/encoders/a300/joint_states"].update(
        joint_names=list(A300_WHEEL_JOINTS), finite_position_velocity=True
    )
    metadata["/odom/unfiltered"]["child_frame_id"] = "base_footprint"
    observed = {
        topic: contract.get("nominal_hz")
        for topic, contract in STREAM_CONTRACTS.items()
    }
    return samples, metadata, observed


def test_dependency_free_runtime_contract_accepts_exact_sensor_and_encoder_chain():
    samples, metadata, observed = _valid_runtime_rows()
    result = validate_runtime_contract(samples, metadata, observed)
    assert result["passed"] is True
    assert all(result["passed_checks"].values())


def test_dependency_free_runtime_contract_rejects_frame_rate_range_and_encoder_drift():
    samples, metadata, observed = _valid_runtime_rows()
    metadata["/sensors/imu/data"]["frame_id"] = "base_link"
    observed["/sensors/lidar_3d/points"] = 4.9
    metadata["/sensors/lidar_2d/scan"]["range_max_m"] = 29.0
    metadata["/formal_vehicle/encoders/a300/counts"]["layout_label"] = "wrong"
    result = validate_runtime_contract(samples, metadata, observed)
    assert result["passed"] is False
    assert "/sensors/imu/data" in result["frame_errors"]
    assert "/sensors/lidar_3d/points" in result["frequency_errors"]
    assert result["passed_checks"]["utm30lx_runtime_range_and_270deg_scan_exact"] is False
    assert result["passed_checks"]["a300_four_wheel_encoder_feedback_structured_and_finite"] is False


def test_source_timestamp_frequency_requires_three_monotonic_samples():
    assert observed_frequency_hz([0, 100, 100]) is None
    assert observed_frequency_hz([1_000_000_000, 1_100_000_000, 1_200_000_000]) == 10.0
