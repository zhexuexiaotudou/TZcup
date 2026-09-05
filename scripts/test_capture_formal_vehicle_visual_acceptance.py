from __future__ import annotations

import ast
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]


def _launch_node_binds_config(
    launch_text: str, *, node_name: str, config_name: str
) -> bool:
    tree = ast.parse(launch_text)
    for call in (
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ):
        keywords = {
            item.arg: item.value for item in call.keywords if item.arg is not None
        }
        name = keywords.get("name")
        if not isinstance(name, ast.Constant) or name.value != node_name:
            continue
        parameters = keywords.get("parameters")
        if parameters is None:
            continue
        for mapping in (
            node for node in ast.walk(parameters) if isinstance(node, ast.Dict)
        ):
            for key, value in zip(mapping.keys, mapping.values, strict=True):
                if isinstance(key, ast.Constant) and key.value == "config_file":
                    if any(
                        isinstance(node, ast.Constant) and node.value == config_name
                        for node in ast.walk(value)
                    ):
                        return True
    return False


def _launch_node_has_if_condition(
    launch_text: str, *, node_name: str, configuration_variable: str
) -> bool:
    tree = ast.parse(launch_text)
    for call in (
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ):
        keywords = {
            item.arg: item.value for item in call.keywords if item.arg is not None
        }
        name = keywords.get("name")
        condition = keywords.get("condition")
        if not isinstance(name, ast.Constant) or name.value != node_name:
            continue
        return (
            isinstance(condition, ast.Call)
            and isinstance(condition.func, ast.Name)
            and condition.func.id == "IfCondition"
            and len(condition.args) == 1
            and isinstance(condition.args[0], ast.Name)
            and condition.args[0].id == configuration_variable
        )
    return False


def _literal_assignment(name: str):
    source = (ROOT / "scripts/capture_formal_vehicle_visual_acceptance.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    return next(
        ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name for target in node.targets)
    )


def _load_module_without_ros():
    # The CI lane does not install ROS Python packages. Keep deterministic image
    # metric tests local by extracting the pure helper definition at runtime.
    namespace = {
        "np": np,
        "hashlib": hashlib,
        "json": json,
        "math": math,
        "os": os,
        "Path": Path,
        "subprocess": subprocess,
        "time": time,
        "ET": ET,
        "ROOT": ROOT,
        "TOPICS": _literal_assignment("TOPICS"),
        "WORLD_NAME": "formal_vehicle_visual_acceptance",
        "MODEL_NAME": "tzcup_formal_sanitation_vehicle",
        "CaptureNode": object,
        "VisualAcceptanceError": RuntimeError,
        "RuntimeGateError": RuntimeError,
    }
    source = open(__file__.replace("test_capture_", "capture_"), encoding="utf-8").read()
    start = source.index("def frame_metrics")
    end = source.index("\n\ndef capture(args", start)
    exec(source[start:end], namespace)
    return namespace


def test_frame_metrics_distinguish_visible_and_black_images() -> None:
    frame_metrics = _load_module_without_ros()["frame_metrics"]
    visible = np.zeros((720, 1280, 3), dtype=np.uint8)
    visible[:, :640] = 180
    black = np.zeros_like(visible)
    assert frame_metrics(visible)["luminance_stddev"] > 8.0
    assert frame_metrics(visible)["near_black_fraction"] < 0.95
    assert frame_metrics(black)["near_black_fraction"] == 1.0
    validate = _load_module_without_ros()["validate_frame_metrics"]
    try:
        validate("empty", frame_metrics(black))
    except RuntimeError as error:
        assert "black or visually empty" in str(error)
    else:
        raise AssertionError("empty image was accepted")


def test_bodywork_profile_is_verified_from_expanded_robot_description() -> None:
    validate_bodywork_profile = _load_module_without_ros()["validate_bodywork_profile"]
    marker = "package://sanitation_vehicle_description/meshes/project/bodywork/"
    product_description = "".join(f'{marker}panel_{index}.stl' for index in range(43))
    assert validate_bodywork_profile(product_description, "product") == 43
    assert validate_bodywork_profile("<robot/>", "service") == 0
    for description, profile in (("<robot/>", "product"), (product_description, "service")):
        try:
            validate_bodywork_profile(description, profile)
        except ValueError:
            pass
        else:
            raise AssertionError(f"mislabeled {profile} capture was accepted")


def test_ground_truth_gate_rejects_studio_drift_and_accepts_stationary_vehicle() -> None:
    validate = _load_module_without_ros()["validate_ground_truth_pose"]
    initial = {"x": 0.004, "y": -0.003, "z": 0.005, "yaw": 0.002}
    final = {"x": 0.008, "y": -0.004, "z": 0.005, "yaw": 0.003}
    accepted = validate(
        initial,
        final,
        max_spawn_planar_m=0.10,
        max_run_planar_m=0.05,
        max_abs_yaw_rad=0.05,
    )
    assert accepted["passed"] is True
    assert accepted["violations"] == []
    drifted = validate(
        initial,
        {"x": -2.88, "y": 0.1, "z": 0.005, "yaw": 0.20},
        max_spawn_planar_m=0.10,
        max_run_planar_m=0.05,
        max_abs_yaw_rad=0.05,
    )
    assert drifted["passed"] is False
    assert len(drifted["violations"]) == 3


def test_visual_launch_forwards_bodywork_selection() -> None:
    launch_source = (
        ROOT
        / "starter_ws"
        / "src"
        / "sanitation_vehicle_description"
        / "launch"
        / "formal_vehicle_visual_acceptance.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument(\n            "bodywork_visible"' in launch_source
    assert '"bodywork_visible": bodywork_visible' in launch_source

    runner_source = (
        ROOT / "scripts" / "run_formal_vehicle_visual_acceptance.sh"
    ).read_text(encoding="utf-8")
    assert "capture_profile product true" in runner_source
    assert "capture_profile service false" in runner_source
    assert '--bodywork-profile "${profile}"' in runner_source
    assert '--renderer-log "${launch_log}"' in runner_source
    assert '--runtime-binding "${runtime_binding}"' in runner_source
    assert 'runtime_binding="${run_root}/runtime_gate_binding.json"' in runner_source
    assert "formal_source_bound_preflight" in runner_source
    assert runner_source.index("formal_source_bound_preflight") < runner_source.index(
        'source "${runtime_install}/setup.bash"'
    ) < runner_source.index("ros2 launch")
    assert "prepare_formal_triggered_visual_world.py" in runner_source
    assert 'world:="${triggered_visual_world}"' in runner_source
    assert "--trigger-cameras-sequentially" in runner_source
    assert '--triggered-world-report "${run_root}/triggered_visual_world_report.json"' in runner_source
    assert '--camera-contract-world "${triggered_visual_world}"' in runner_source

    assert 'DeclareLaunchArgument(\n            "world"' in launch_source
    assert '"world": world' in launch_source


def test_visual_runner_fails_closed_before_service_profile_handoff() -> None:
    runner = (ROOT / "scripts/run_formal_vehicle_visual_acceptance.sh").read_text(
        encoding="utf-8"
    )
    product_call = runner.index('capture_profile product true')
    handoff_call = runner.index(
        'prepare_service_profile_after_product "${run_root}/formal_vehicle_visual_acceptance"'
    )
    service_call = runner.index('capture_profile service false')
    assert product_call < handoff_call < service_call
    assert 'formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"' in runner
    assert 'FORMAL_VISUAL_LAST_COMPLETED_PARTITION="${GZ_PARTITION}"' in runner
    assert 'formal_runtime_cleanup_partition "${FORMAL_VISUAL_LAST_COMPLETED_PARTITION}"' in runner
    assert "product visual profile still has partition processes at service handoff" in runner
    assert "FORMAL_VISUAL_PROFILE_FILE_CACHE_RECLAIM_ATTEMPTED" in runner
    assert 'getattr(os, "posix_fadvise", None)' in runner
    assert 'getattr(os, "POSIX_FADV_DONTNEED", None)' in runner
    assert 'os.O_WRONLY | os.O_CREAT | os.O_EXCL' in runner
    assert 'getattr(os, "O_NOFOLLOW", 0)' in runner
    assert '"advised_file_count": advised' in runner
    assert 'except (OSError, ValueError):' in runner
    assert "bounded 60-second / 5-second" in runner
    assert 'formal_runtime_memory_preflight "${handoff_prefix}" || return "$?"' in runner
    assert runner.index('formal_runtime_cleanup_partition "${FORMAL_VISUAL_LAST_COMPLETED_PARTITION}"') < runner.index(
        'formal_runtime_memory_preflight "${handoff_prefix}"'
    ) < service_call

    parser_flags = set(
        re.findall(r'parser\.add_argument\("(--[a-z0-9-]+)"', (
            ROOT / "scripts/capture_formal_vehicle_visual_acceptance.py"
        ).read_text(encoding="utf-8"))
    )
    capture_call = runner.split(
        'scripts/capture_formal_vehicle_visual_acceptance.py', 1
    )[1].split("|| result=$?", 1)[0]
    runner_flags = set(re.findall(r"--[a-z0-9-]+", capture_call))
    assert runner_flags <= parser_flags


def test_sequential_trigger_capture_is_fail_closed_and_one_camera_at_a_time(
    tmp_path: Path,
) -> None:
    namespace = _load_module_without_ros()
    topics = namespace["TOPICS"]
    trigger_topic = namespace["camera_trigger_topic"]
    assert trigger_topic(topics["front_left"]) == "/formal_visual/front_left/trigger"
    try:
        trigger_topic("/unapproved/camera")
    except RuntimeError as error:
        assert "outside the formal topic contract" in str(error)
    else:
        raise AssertionError("an unapproved camera trigger topic was accepted")

    capture_source = (
        ROOT / "scripts/capture_formal_vehicle_visual_acceptance.py"
    ).read_text(encoding="utf-8")
    assert '"gz",' in capture_source
    assert '"gz.msgs.Boolean"' in capture_source
    assert '"data: true"' in capture_source
    assert "for name, image_topic in TOPICS.items():" in capture_source
    assert "name not in node.frames" in capture_source
    assert '"sequential_gazebo_trigger"' in capture_source
    assert '"maximum_simultaneously_triggered_cameras"' in capture_source
    assert '"camera_trigger_order"' in capture_source
    assert '"camera_trigger_command_count"' in capture_source
    assert '"gz", "topic", "-i", "-t", trigger_topic' in capture_source
    assert '"subscriber_ready": True' in capture_source
    assert "triggered visual camera produced an unsolicited frame" in capture_source
    assert '"frame_received_after_trigger_completion": True' in capture_source
    assert "args.camera_contract_world" in capture_source

    triggered_world = tmp_path / "triggered.sdf"
    triggered_world.write_text("<sdf version='1.10'/>", encoding="utf-8")
    report_path = tmp_path / "triggered.json"
    report = {
        "status": "FORMAL_TRIGGERED_VISUAL_WORLD_PREPARED",
        "passed": True,
        "camera_count": len(topics),
        "all_camera_contract_fields_preserved": True,
        "all_cameras_triggered": True,
        "all_cameras_use_default_trigger_topic": True,
        "output_world": str(triggered_world.resolve()),
        "output_world_sha256": hashlib.sha256(triggered_world.read_bytes()).hexdigest(),
        "source_world": "source.sdf",
        "source_world_sha256": "source-sha",
        "camera_contract_sha256_after": "contract-sha",
        "trigger_bindings": [
            {
                "image_topic": topic,
                "trigger_topic": f"{topic}/trigger",
                "uses_default_trigger_topic": True,
            }
            for topic in topics.values()
        ],
    }
    report_path.write_text(json.dumps(report), encoding="utf-8")
    binding = namespace["validate_triggered_world_report"](
        report_path, triggered_world
    )
    assert binding["passed"] is True
    assert binding["camera_count"] == 19
    other_world = tmp_path / "other.sdf"
    other_world.write_text("<sdf version='1.10'/>", encoding="utf-8")
    try:
        namespace["validate_triggered_world_report"](report_path, other_world)
    except RuntimeError as error:
        assert "differs from the launched triggered world" in str(error)
    else:
        raise AssertionError("a divergent triggered world was accepted")


def test_sequential_trigger_state_machine_never_has_two_cameras_in_flight() -> None:
    namespace = _load_module_without_ros()
    topics = namespace["TOPICS"]
    calls: list[tuple[str, str]] = []
    persisted: list[str] = []
    state: dict[str, object] = {"pending": None, "peak_held_frame_bytes": 0}

    class FakeRclpy:
        @staticmethod
        def ok() -> bool:
            return True

        @staticmethod
        def spin_once(node, timeout_sec: float) -> None:
            del timeout_sec
            pending = state["pending"]
            if pending is None:
                return
            stamp = len(node.frames) + 1
            message = SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(sec=stamp, nanosec=stamp)
                )
            )
            # One full-resolution RGB allocation proves that the state machine
            # observes the large-frame retention bound without lowering the
            # formal 1600x1000 contract in its test double.
            pixels = np.empty((1000, 1600, 3), dtype=np.uint8)
            node.frames[pending] = (message, pixels)
            state["peak_held_frame_bytes"] = max(
                int(state["peak_held_frame_bytes"]),
                sum(frame[1].nbytes for frame in node.frames.values()),
            )
            node.frame_received_monotonic[pending] = time.monotonic()
            calls.append(("frame", pending))
            state["pending"] = None

    def fake_ready(image_topic: str, deadline: float) -> dict[str, object]:
        assert deadline > time.monotonic()
        name = next(name for name, topic in topics.items() if topic == image_topic)
        assert state["pending"] is None
        calls.append(("ready", name))
        return {"subscriber_ready": True}

    def fake_publish(image_topic: str, timeout_s: float) -> dict[str, object]:
        assert timeout_s > 0.0
        name = next(name for name, topic in topics.items() if topic == image_topic)
        assert state["pending"] is None
        state["pending"] = name
        calls.append(("trigger", name))
        return {
            "completed_monotonic_seconds": time.monotonic(),
            "trigger_topic": f"{image_topic}/trigger",
        }

    def fake_ros_ready(node, image_topic: str, deadline: float) -> dict[str, object]:
        del node
        assert deadline > time.monotonic()
        name = next(name for name, topic in topics.items() if topic == image_topic)
        assert state["pending"] is None
        calls.append(("ros_ready", name))
        return {"publisher_ready": True, "expected_reliability": "RELIABLE"}

    node = SimpleNamespace(frames={}, frame_received_monotonic={})

    def start_camera_capture(name: str) -> None:
        assert not node.frames
        calls.append(("subscribe", name))

    def stop_camera_capture(name: str) -> None:
        calls.append(("unsubscribe", name))

    def release_camera_frame(name: str) -> None:
        node.frames.pop(name, None)
        node.frame_received_monotonic.pop(name, None)
        calls.append(("release", name))

    def persist_frame(name: str, message, pixels) -> None:
        del message
        assert set(node.frames) == {name}
        assert pixels.shape == (1000, 1600, 3)
        assert pixels.nbytes == 4_800_000
        persisted.append(name)
        calls.append(("persist", name))

    node.start_camera_capture = start_camera_capture
    node.stop_camera_capture = stop_camera_capture
    node.release_camera_frame = release_camera_frame
    namespace["rclpy"] = FakeRclpy
    namespace["wait_for_camera_trigger_subscriber"] = fake_ready
    namespace["wait_for_ros_camera_publisher"] = fake_ros_ready
    namespace["publish_camera_trigger"] = fake_publish
    traces = namespace["capture_frames_sequentially"](
        node,
        time.monotonic() + 5.0,
        persist_frame=persist_frame,
        frame_wait_seconds=0.01,
    )
    expected_calls = [
        (phase, name)
        for name in topics
        for phase in (
            "subscribe",
            "ready",
            "ros_ready",
            "trigger",
            "frame",
            "persist",
            "unsubscribe",
            "release",
        )
    ]
    assert calls == expected_calls
    assert state["pending"] is None
    assert persisted == list(topics)
    assert node.frames == {}
    assert state["peak_held_frame_bytes"] == 4_800_000
    assert list(traces) == list(topics)
    assert all(len(trace) == 1 for trace in traces.values())
    assert all(
        trace[0]["frame_received_after_trigger_completion"] is True
        and trace[0]["accepted"] is True
        for trace in traces.values()
    )


def test_sequential_capture_contract_releases_large_frames_and_subscriptions() -> None:
    source = (ROOT / "scripts/capture_formal_vehicle_visual_acceptance.py").read_text(
        encoding="utf-8"
    )
    assert "visual capture may subscribe to only one camera at a time" in source
    assert "node.start_camera_capture(name)" in source
    assert "node.stop_camera_capture(name)" in source
    assert "node.release_camera_frame(name)" in source
    assert source.index("persist_frame(name, message, pixels)") < source.index(
        "node.stop_camera_capture(name)"
    ) < source.index("node.release_camera_frame(name)")
    assert "entry alone would retain a full prior frame" in source
    assert source.index("node.release_camera_frame(name)") < source.index(
        "message = None\n            pixels = None"
    )
    assert "frames = dict(node.frames)" not in source.split(
        "if args.trigger_cameras_sequentially:", 1
    )[1].split("else:", 1)[0]


def test_failed_capture_manifest_binds_partial_png_evidence(tmp_path: Path) -> None:
    partial_frame_artifacts = _load_module_without_ros()["partial_frame_artifacts"]
    (tmp_path / "rear_right.png").write_bytes(b"partial-rear")
    (tmp_path / "front_left.png").write_bytes(b"partial-front")
    (tmp_path / "manifest.json").write_text("{}", encoding="utf-8")
    assert partial_frame_artifacts(tmp_path) == {
        "partial_frame_file_count": 2,
        "partial_frame_files": ["front_left.png", "rear_right.png"],
    }
    source = (ROOT / "scripts/capture_formal_vehicle_visual_acceptance.py").read_text(
        encoding="utf-8"
    )
    assert "**partial_frame_artifacts(args.output)" in source


def test_committed_legacy_visuals_are_explicitly_invalidated_until_rerender() -> None:
    for directory, profile in (
        ("formal_vehicle_visual_acceptance", "product"),
        ("formal_vehicle_service_visual_acceptance", "service"),
    ):
        root = ROOT / "reports" / "engineering" / directory
        manifest_path = root / "manifest.json"
        if not manifest_path.exists():
            # A final-acceptance session archives the previous manifest before
            # rendering.  Retained PNGs are then only unaccepted visual aids;
            # absence of a manifest is a fail-closed invalidation, not a pass.
            assert list(root.glob("*.png"))
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest["schema_version"] < 4:
            assert any("png_sha256" not in frame for frame in manifest["frames"].values())
            continue
        assert manifest["schema_version"] == 4
        assert manifest["camera_count"] == 19
        assert manifest["status"] == (
            f"GAZEBO_OGRE2_{profile.upper()}_NINETEEN_FUNCTIONAL_VIEW_CAPTURE_PASSED"
        )
        assert manifest["bodywork_profile"] == profile
        assert manifest["bodywork_profile_verified_from_robot_description"] is True
        if profile == "product":
            assert manifest["bodywork_visual_mesh_count"] >= 40
        else:
            assert manifest["bodywork_visual_mesh_count"] == 0
        assert all(frame["png_sha256"] for frame in manifest["frames"].values())


def test_visual_studio_limits_dds_bandwidth_before_controller_startup() -> None:
    root = Path(__file__).resolve().parents[1]
    launch = (
        root
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_visual_acceptance.launch.py"
    ).read_text(encoding="utf-8")
    world = (
        root
        / "starter_ws/src/sanitation_vehicle_description/worlds/formal_vehicle_visual_acceptance.sdf"
    ).read_text(encoding="utf-8")
    assert '"high_bandwidth_sensor_runtime": "false"' in launch
    assert '"visual_acceptance_runtime": "true"' in launch
    assert '"headless_rendering": "true"' in launch
    assert '"start_localization": "false"' in launch
    assert '"enable_safety_manager": "false"' in launch
    assert world.count("<update_rate>0.2</update_rate>") == 19
    assert "<update_rate>3</update_rate>" not in world


def test_all_nineteen_visual_studio_cameras_are_bridged_to_ros() -> None:
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    bridge_config = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/config/formal_visual_sensor_bridge.yaml"
    )
    rows = yaml.safe_load(bridge_config.read_text(encoding="utf-8"))
    topics = {row["ros_topic_name"]: row for row in rows}
    assert "formal_visual_sensor_bridge.yaml" in launch
    assert 'headless_rendering = LaunchConfiguration("headless_rendering")' in launch
    assert 'DeclareLaunchArgument(\n                "headless_rendering",\n                default_value="false"' in launch
    assert "IfElseSubstitution(" in launch
    assert '" --headless-rendering"' in launch
    assert 'package="ros_gz_image"' in launch
    assert 'executable="image_bridge"' in launch
    assert "arguments=visual_image_topics" in launch
    assert 'parameters=[{"qos": "default"}]' in launch
    assert 'row["qos_profile"] != "DEFAULT_RELIABLE"' in launch
    assert "BEST_EFFORT dropped fragmented loopback samples" in launch
    assert "condition=IfCondition(visual_acceptance_runtime)" in launch
    assert "high_bandwidth_bridges_enabled = PythonExpression(" in launch
    assert 'start_high_bandwidth_sensor_bridges = LaunchConfiguration(' in launch
    for node_name in (
        "formal_vehicle_high_bandwidth_sensor_bridge",
        "formal_fisheye_camera_info_publisher",
    ):
        assert _launch_node_has_if_condition(
            launch,
            node_name=node_name,
            configuration_variable="high_bandwidth_bridges_enabled",
        )
    for name in (
        "front_left",
        "rear_right",
        "top_cleaning",
        "sensor_tower_detail",
        "front_sensor_detail",
        "arm_mount_detail",
        "dry_deposition_detail",
        "cleaning_head_detail",
        "rear_service_detail",
        "power_compute_detail",
        "storage_recovery_detail",
        "rear_left_sensor_detail",
        "rear_right_sensor_detail",
        "drivetrain_detail",
        "inertial_power_detail",
        "dry_deposition_internal",
        "power_safety_internal",
        "charge_interface_detail",
        "drain_interface_detail",
    ):
        row = topics[f"/formal_visual/{name}"]
        assert row["ros_topic_name"] == row["gz_topic_name"]
        assert row["ros_type_name"] == "sensor_msgs/msg/Image"
        assert row["gz_type_name"] == "gz.msgs.Image"
        assert row["direction"] == "GZ_TO_ROS"
        assert row["qos_profile"] == "DEFAULT_RELIABLE"
        assert "subscriber_queue" not in row
        assert "publisher_queue" not in row
        assert "lazy" not in row


def test_visual_capture_qos_matches_the_reliable_large_frame_bridge() -> None:
    capture = (ROOT / "scripts/capture_formal_vehicle_visual_acceptance.py").read_text(
        encoding="utf-8"
    )
    assert "HistoryPolicy" in capture
    assert "self._image_qos = QoSProfile(" in capture
    assert "history=HistoryPolicy.KEEP_LAST" in capture
    assert "depth=1" in capture
    assert "reliability=ReliabilityPolicy.RELIABLE" in capture
    assert "durability=DurabilityPolicy.VOLATILE" in capture
    assert "self._image_qos," in capture
    assert "wait_for_ros_camera_publisher" in capture
    assert '"expected_reliability": "RELIABLE"' in capture
    assert "self._on_image(key, message), 2" not in capture


def test_semantic_projection_gate_rejects_wrong_camera_and_out_of_frame_target() -> None:
    namespace = _load_module_without_ros()
    validate = namespace["validate_target_projection"]
    camera = {
        "pose_xyz_rpy": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "horizontal_fov_rad": 0.8,
        "width": 1600,
        "height": 1000,
    }
    target = {
        "target_xyz_m": [1.0, 0.0, 0.0],
        "target_entities": ["charge_port_housing_link"],
        "minimum_projected_target_pixels": 64,
    }
    assert validate("correct", camera, target)["passed"] is True
    for bad_target in ([0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]):
        mutated = dict(target, target_xyz_m=bad_target)
        try:
            validate("wrong", camera, mutated)
        except RuntimeError as error:
            assert "do not project inside" in str(error)
        else:
            raise AssertionError("off-camera target was accepted")


def test_semantic_target_entities_are_real_visible_urdf_links() -> None:
    namespace = _load_module_without_ros()
    validate = namespace["validate_target_entities"]
    source = (ROOT / "scripts/capture_formal_vehicle_visual_acceptance.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    targets = next(
        ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "VIEW_TARGETS" for target in node.targets)
    )
    for contract in targets.values():
        contract["minimum_projected_target_pixels"] = 64
    robot_description = (ROOT / "reports/engineering/formal_competition_vehicle.urdf").read_text(
        encoding="utf-8"
    )
    assert set(validate(robot_description, targets)) == set(targets)
    broken = dict(targets)
    broken["front_left"] = dict(targets["front_left"], target_entities=["missing_link"])
    try:
        validate(robot_description, broken)
    except RuntimeError as error:
        assert "absent from URDF" in str(error)
    else:
        raise AssertionError("missing target entity was accepted")


def test_capture_waits_for_controllers_on_the_full_180_link_vehicle() -> None:
    source = (ROOT / "scripts/capture_formal_vehicle_visual_acceptance.py").read_text(
        encoding="utf-8"
    )
    assert "min(args.timeout, 90.0)" in source
    assert "node.start_robot_description_subscription()" in source
    assert source.index("node.start_robot_description_subscription()") > source.index(
        "node.command_folded_arm()"
    )
    assert "invalid_frames: list[str]" in source
    assert source.index("PillowImage.fromarray") < source.index("if invalid_frames:")
    assert "read_ground_truth_with_retry" in source
    assert "validate_ground_truth_pose" in source
    assert "failure_manifest_persisted" in source
    assert "write_bound_manifest(manifest_path, running, runtime_gate_binding)" in source
    assert 'parser.add_argument("--runtime-binding", type=Path, required=True)' in source
    assert "load_runtime_gate_binding(args.runtime_binding)" in source
    assert "inspect_renderer_log(args.renderer_log)" in source


def test_profile_manifest_embeds_exact_binding_and_writes_sibling_sidecar(
    tmp_path: Path,
) -> None:
    namespace = _load_module_without_ros()
    binding = {
        "schema_version": 1,
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        },
        "runtime_closure_binding": {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        },
    }
    manifest_path = tmp_path / "manifest.json"
    namespace["write_bound_manifest"](
        manifest_path,
        {"status": "GAZEBO_OGRE2_PRODUCT_NINETEEN_FUNCTIONAL_VIEW_CAPTURE_PASSED"},
        binding,
    )
    sidecar = manifest_path.with_name(manifest_path.name + ".runtime_binding.json")
    assert json.loads(sidecar.read_text(encoding="utf-8")) == binding
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["runtime_gate_binding"] == binding
    assert manifest["acceptance_session_binding"] == binding["acceptance_session_binding"]
    assert manifest["runtime_closure_binding"] == binding["runtime_closure_binding"]


def test_invalid_visual_binding_fails_closed_before_any_manifest_is_written(
    tmp_path: Path,
) -> None:
    namespace = _load_module_without_ros()
    namespace["load_binding"] = lambda _: (_ for _ in ()).throw(
        RuntimeError("invalid binding")
    )
    try:
        namespace["load_runtime_gate_binding"](tmp_path / "missing-binding.json")
    except RuntimeError as error:
        assert "missing or invalid" in str(error)
    else:
        raise AssertionError("invalid runtime binding was accepted")
    assert not list(tmp_path.iterdir())


def test_renderer_log_gate_rejects_scene_entity_zero(tmp_path: Path) -> None:
    inspect_renderer_log = _load_module_without_ros()["inspect_renderer_log"]
    clean = tmp_path / "clean.log"
    clean.write_text("Gazebo server ready\n", encoding="utf-8")
    assert inspect_renderer_log(clean)["passed"] is True
    broken = tmp_path / "broken.log"
    broken.write_text(
        "[Err] [SceneManager.cc:615] Could not find visual for entity: 0\n",
        encoding="utf-8",
    )
    try:
        inspect_renderer_log(broken)
    except RuntimeError as error:
        assert "scene_entity_zero=1" in str(error)
    else:
        raise AssertionError("SceneManager entity-zero corruption was accepted")


def test_every_camera_axis_intersects_its_engineering_target_inside_half_hfov() -> None:
    capture_source = (
        ROOT / "scripts/capture_formal_vehicle_visual_acceptance.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(capture_source)
    targets = None
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "VIEW_TARGETS"
            for target in node.targets
        ):
            targets = ast.literal_eval(node.value)
            break
    assert targets is not None

    world_path = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/worlds/formal_vehicle_visual_acceptance.sdf"
    )
    world = ET.parse(world_path).getroot().find("world")
    assert world is not None
    cameras = {}
    for model in world.findall("model"):
        sensor = model.find("./link/sensor")
        if sensor is None or sensor.get("type") != "camera":
            continue
        name = sensor.get("name")
        pose = [float(value) for value in model.findtext("pose", "").split()]
        hfov = float(sensor.findtext("./camera/horizontal_fov", "0"))
        cameras[name] = (pose, hfov)
    assert set(cameras) == set(targets)

    for name, row in targets.items():
        pose, hfov = cameras[name]
        x, y, z, _roll, pitch, yaw = pose
        optical = (
            math.cos(yaw) * math.cos(pitch),
            math.sin(yaw) * math.cos(pitch),
            -math.sin(pitch),
        )
        target = row["target_xyz_m"]
        ray = (target[0] - x, target[1] - y, target[2] - z)
        distance = math.sqrt(sum(value * value for value in ray))
        cosine = sum(optical[index] * ray[index] for index in range(3)) / distance
        offset = math.acos(max(-1.0, min(1.0, cosine)))
        assert distance > 0.25, name
        assert offset < hfov / 2.0, (name, offset, hfov / 2.0)


def test_nineteen_view_crosswalk_covers_every_registered_id_and_physical_link() -> None:
    namespace = _load_module_without_ros()
    validate = namespace["validate_engineering_visual_crosswalk"]
    robot_description = (
        ROOT / "reports/engineering/formal_competition_vehicle.urdf"
    ).read_text(encoding="utf-8")
    cameras = namespace["camera_projection_contracts"](
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/worlds/"
        "formal_vehicle_visual_acceptance.sdf"
    )
    register = yaml.safe_load(
        (ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml").read_text(
            encoding="utf-8"
        )
    )
    arm_joints = _literal_assignment("FOLDED_ARM_JOINTS")
    arm_positions = _literal_assignment("FOLDED_ARM_POSITIONS")
    visual_joint_positions = dict(zip(arm_joints, arm_positions))
    visual_joint_positions.update(
        {"robotiq_85_left_knuckle_joint": 0.20, "dry_deposit_gate_joint": 1.05}
    )
    report = validate(
        robot_description,
        cameras,
        register,
        position_views=_literal_assignment("FUNCTION_POSITION_VIEWS"),
        sensor_views=_literal_assignment("SENSOR_INSTALLATION_VIEWS"),
        assembly_views=_literal_assignment("MECHANICAL_SUBASSEMBLY_VIEWS"),
        inspection_links=_literal_assignment("VIEW_INSPECTION_LINKS"),
        joint_positions=visual_joint_positions,
    )
    assert report["passed"] is True
    assert report["camera_count"] == 19
    assert report["functional_position_count"] == 38
    assert report["sensor_installation_count"] == 9
    assert report["mechanical_subassembly_count"] == 18
    assert report["required_registered_physical_link_count"] >= 100
    assert report["inspected_physical_link_count"] >= report[
        "required_registered_physical_link_count"
    ]
    for link_name in (
        "lidar_2d_link",
        "lidar_3d_link",
        "gnss_antenna_link",
        "zed_f9p_module_reference_link",
        "front_rgbd_link",
        "rear_left_fisheye_link",
        "rear_right_fisheye_link",
        "imu_link",
        "wrist_rgbd_link",
        "ur5e_wrist_3_link",
        "robotiq_85_left_finger_tip_link",
        "left_side_brush_motor_stator_link",
        "left_side_brush_bristle_sectors_link",
        "right_side_brush_bristle_sectors_link",
        "central_roller_motor_stator_link",
        "recovery_pump_motor_link",
        "wastewater_tank_link",
        "dry_bin_link",
        "emergency_stop_plunger_link",
        "charge_connector_lock_link",
        "wastewater_drain_valve_actuator_link",
    ):
        assert report["link_projection_reports"][link_name]["passed"] is True


def test_crosswalk_rejects_missing_position_and_misassigned_physical_link() -> None:
    namespace = _load_module_without_ros()
    validate = namespace["validate_engineering_visual_crosswalk"]
    robot_description = (
        ROOT / "reports/engineering/formal_competition_vehicle.urdf"
    ).read_text(encoding="utf-8")
    cameras = namespace["camera_projection_contracts"](
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/worlds/"
        "formal_vehicle_visual_acceptance.sdf"
    )
    register = yaml.safe_load(
        (ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml").read_text(
            encoding="utf-8"
        )
    )
    position_views = _literal_assignment("FUNCTION_POSITION_VIEWS")
    sensor_views = _literal_assignment("SENSOR_INSTALLATION_VIEWS")
    assembly_views = _literal_assignment("MECHANICAL_SUBASSEMBLY_VIEWS")
    inspection_links = _literal_assignment("VIEW_INSPECTION_LINKS")
    arm_joints = _literal_assignment("FOLDED_ARM_JOINTS")
    arm_positions = _literal_assignment("FOLDED_ARM_POSITIONS")
    joint_positions = dict(zip(arm_joints, arm_positions))
    joint_positions.update(
        {"robotiq_85_left_knuckle_joint": 0.20, "dry_deposit_gate_joint": 1.05}
    )

    missing_position = dict(position_views)
    missing_position.pop("emergency_stop")
    try:
        validate(
            robot_description,
            cameras,
            register,
            position_views=missing_position,
            sensor_views=sensor_views,
            assembly_views=assembly_views,
            inspection_links=inspection_links,
            joint_positions=joint_positions,
        )
    except RuntimeError as error:
        assert "functional_positions visual crosswalk mismatch" in str(error)
    else:
        raise AssertionError("missing functional-position visual coverage was accepted")

    wrong_view = {name: list(values) for name, values in inspection_links.items()}
    wrong_view["front_sensor_detail"].remove("front_rgbd_link")
    wrong_view.setdefault("drain_interface_detail", []).append("front_rgbd_link")
    try:
        validate(
            robot_description,
            cameras,
            register,
            position_views=position_views,
            sensor_views=sensor_views,
            assembly_views=assembly_views,
            inspection_links=wrong_view,
            joint_positions=joint_positions,
        )
    except RuntimeError as error:
        assert "physical links do not use its assigned inspection views" in str(error)
    else:
        raise AssertionError("wrong-camera physical-link assignment was accepted")


def test_service_profile_crosswalk_excludes_hidden_body_skin_but_keeps_internal_hardware() -> None:
    namespace = _load_module_without_ros()
    root = ET.parse(ROOT / "reports/engineering/formal_competition_vehicle.urdf").getroot()
    marker = "package://sanitation_vehicle_description/meshes/project/bodywork/"
    for link in root.findall("link"):
        for visual in list(link.findall("visual")):
            mesh = visual.find("geometry/mesh")
            if mesh is not None and mesh.get("filename", "").startswith(marker):
                link.remove(visual)
    robot_description = ET.tostring(root, encoding="unicode")
    assert namespace["validate_bodywork_profile"](robot_description, "service") == 0

    targets = _literal_assignment("VIEW_TARGETS")
    for contract in targets.values():
        contract["minimum_projected_target_pixels"] = 64
    for name, entities in _literal_assignment("SERVICE_TARGET_ENTITY_OVERRIDES").items():
        targets[name]["target_entities"] = entities
    assert set(namespace["validate_target_entities"](robot_description, targets)) == set(
        targets
    )

    cameras = namespace["camera_projection_contracts"](
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/worlds/"
        "formal_vehicle_visual_acceptance.sdf"
    )
    register = yaml.safe_load(
        (ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml").read_text(
            encoding="utf-8"
        )
    )
    joint_positions = dict(
        zip(
            _literal_assignment("FOLDED_ARM_JOINTS"),
            _literal_assignment("FOLDED_ARM_POSITIONS"),
        )
    )
    joint_positions.update(
        {"robotiq_85_left_knuckle_joint": 0.20, "dry_deposit_gate_joint": 1.05}
    )
    report = namespace["validate_engineering_visual_crosswalk"](
        robot_description,
        cameras,
        register,
        position_views=_literal_assignment("FUNCTION_POSITION_VIEWS"),
        sensor_views=_literal_assignment("SENSOR_INSTALLATION_VIEWS"),
        assembly_views=_literal_assignment("MECHANICAL_SUBASSEMBLY_VIEWS"),
        inspection_links=_literal_assignment("VIEW_INSPECTION_LINKS"),
        joint_positions=joint_positions,
        bodywork_profile="service",
    )
    assert report["bodywork_profile"] == "service"
    assert report["required_registered_physical_link_count"] == 96
    assert report["inspected_physical_link_count"] == 142
    assert "bodywork_lower_tub_link" not in report["link_projection_reports"]
    assert report["link_projection_reports"]["power_distribution_box_link"]["passed"] is True
    assert report["link_projection_reports"]["wastewater_tank_link"]["passed"] is True
