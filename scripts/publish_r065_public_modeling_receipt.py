#!/usr/bin/env python3
"""Atomically seal one fresh, public-only R065 modeling-session receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

from formal_acceptance_session import AcceptanceSessionError, _snapshot_identity
from formal_runtime_gate_binding import RuntimeGateError, load_binding


class ReceiptError(RuntimeError):
    pass


CHILD_NAMES = ("w1", "w2", "w3_public_audit", "w3_live_dynamic", "w5")
_COMMAND_TOPIC_PUBLISHERS = {
    "/cmd_vel_nav": ["/controller_server"],
    "/cmd_vel_smoothed": ["/velocity_smoother"],
    "/cmd_vel_gate": ["/collision_monitor"],
    "/base_controller/cmd_vel": ["/whole_vehicle_safety_manager"],
}
_GROUND_CONTACT_ARM_LINKS = {
    "ur5e_shoulder_link", "ur5e_upper_arm_link", "ur5e_forearm_link",
    "ur5e_wrist_1_link", "ur5e_wrist_2_link", "ur5e_wrist_3_link",
    "flange", "tool0",
}
_W2_ALLOWED_TRACK_STATES = {"CONFIRMED", "QUEUED", "APPROACHING", "CLEANING"}


def _hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ReceiptError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ReceiptError(f"JSON root is not an object: {path}")
    return value


def _regular_in(path: Path, root: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_file():
        raise ReceiptError(f"{label} must be a regular file")
    resolved = path.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ReceiptError(f"{label} escapes the new run root") from exc
    return resolved


def _fresh_row(path: Path, root: Path, started_ns: int, label: str) -> dict[str, Any]:
    resolved = _regular_in(path, root, label)
    stat = resolved.stat()
    if stat.st_mtime_ns < started_ns:
        raise ReceiptError(f"{label} predates the acceptance session")
    return {
        "path": str(resolved),
        "sha256": _hash(resolved),
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
    }


def _finite_number(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(float(value))


def _strictly_increasing_revisions(value: Any) -> bool:
    if not isinstance(value, dict) or set(value) != {
        "initial",
        "after_ground_restore",
        "after_perceived_cube_add",
        "after_perceived_cube_remove",
    }:
        return False
    ordered = [
        value["initial"], value["after_ground_restore"],
        value["after_perceived_cube_add"], value["after_perceived_cube_remove"],
    ]
    return all(type(item) is int for item in ordered) and all(
        left < right for left, right in zip(ordered, ordered[1:])
    )


def _w2_contacts_valid(value: Any) -> bool:
    if not isinstance(value, list) or not value:
        return False
    for contact in value:
        if not isinstance(contact, dict):
            continue
        bodies = {contact.get("body_1"), contact.get("body_2")}
        if "ground" not in bodies:
            continue
        # Match the formal runtime's configured non-exempt arm contact set;
        # a generic substring could admit a wheel or base contact.
        if _GROUND_CONTACT_ARM_LINKS & bodies:
            return True
    return False


def _w2_passed(payload: dict[str, Any]) -> bool:
    normal_ik = payload.get("normal_collision_checked_ik")
    normal_anchors = payload.get("normal_anchor_state_valid")
    cartesian = payload.get("normal_pick_cartesian_fraction")
    return (
        payload.get("runtime_gate") == "moveit_ground_collision"
        and payload.get("passed") is True
        and payload.get("executor_or_controller_commands_sent") is False
        and payload.get("truth_used_for_control") is False
        and isinstance(normal_ik, dict) and normal_ik and all(
            type(value) is int and value == 1 for value in normal_ik.values()
        )
        and isinstance(normal_anchors, dict) and normal_anchors and all(
            value is True for value in normal_anchors.values()
        )
        and _finite_number(cartesian) and float(cartesian) >= 0.98
        and _w2_contacts_valid(payload.get("below_ground_ground_contacts"))
        and _strictly_increasing_revisions(payload.get("scene_revisions"))
        and payload.get("ground_removal_preserved_non_ground_world_and_acm") is True
        and payload.get("ground_removal_used_robot_state_diff_only") is True
    )


def _w2_target_matches_request(target: Any, request: dict[str, Any]) -> bool:
    if not isinstance(target, dict) or set(target) != {
        "uuid", "frame_id", "source_stamp_ns", "header_stamp_ns",
        "source_backend", "target_type", "track_state", "confidence",
        "pose", "size_m",
    }:
        return False
    if set(request) != {
        "schema_version", "target_id", "frame_id", "pose", "size_m",
        "material", "confidence", "truth_used",
    }:
        return False
    target_pose = target.get("pose")
    request_pose = request.get("pose")
    pose_keys = {"x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"}
    if not isinstance(target_pose, dict) or not isinstance(request_pose, dict):
        return False
    if set(target_pose) != pose_keys or set(request_pose) != pose_keys:
        return False
    target_size = target.get("size_m")
    request_size = request.get("size_m")
    if not isinstance(target_size, list) or not isinstance(request_size, list):
        return False
    return (
        request.get("schema_version") == 2
        and request.get("material") == "unknown"
        and request.get("truth_used") is False
        and target.get("uuid") == request.get("target_id")
        and target.get("frame_id") == request.get("frame_id")
        and target.get("source_backend") == "dosod_edgesam_pc"
        and target.get("target_type") == "discrete"
        and str(target.get("track_state", "")).upper() in _W2_ALLOWED_TRACK_STATES
        and type(target.get("source_stamp_ns")) is int
        and target["source_stamp_ns"] > 0
        and target.get("header_stamp_ns") == target["source_stamp_ns"]
        and _finite_number(target.get("confidence"))
        and _finite_number(request.get("confidence"))
        and math.isclose(
            float(target["confidence"]), float(request["confidence"]), abs_tol=1e-6
        )
        and len(target_size) == 3 and len(request_size) == 3
        and all(
            _finite_number(left) and _finite_number(right)
            and math.isclose(float(left), float(right), abs_tol=1e-6)
            for left, right in zip(target_size, request_size, strict=True)
        )
        and all(
            _finite_number(target_pose[key]) and _finite_number(request_pose[key])
            and math.isclose(
                float(target_pose[key]), float(request_pose[key]), abs_tol=1e-6
            )
            for key in pose_keys
        )
    )


def _w2_request_provenance_rows(
    run_root: Path,
    started_ns: int,
    session_path: Path,
    runtime_binding: dict[str, Any],
) -> dict[str, Any]:
    request_path = run_root / "w2_request.json"
    provenance_path = run_root / "w2_request_provenance.json"
    request_row = _fresh_row(request_path, run_root, started_ns, "w2 live request")
    provenance_row = _fresh_row(
        provenance_path, run_root, started_ns, "w2 live request provenance"
    )
    request = _object(request_path)
    provenance = _object(provenance_path)
    target = provenance.get("target")
    closure_binding = runtime_binding.get("runtime_closure_binding", {})
    closure_manifest_text = closure_binding.get("manifest")
    if not isinstance(closure_manifest_text, str) or not closure_manifest_text:
        raise ReceiptError("runtime binding lacks its closure manifest path")
    closure_manifest_path = Path(closure_manifest_text)
    if closure_manifest_path.is_symlink() or not closure_manifest_path.is_file():
        raise ReceiptError("runtime closure manifest is not a regular file")
    closure_manifest = _object(closure_manifest_path)
    closure = closure_manifest.get("closure")
    if not isinstance(closure, dict):
        raise ReceiptError("runtime closure manifest lacks its closure object")
    frozen_roots: dict[str, str] = {}
    for key in ("perception_artifact_root", "onnx_pythonpath"):
        value = closure.get(key)
        if not isinstance(value, str) or not value:
            raise ReceiptError(f"runtime closure manifest lacks {key}")
        candidate = Path(value)
        if candidate.is_symlink() or not candidate.is_dir():
            raise ReceiptError(f"runtime closure {key} is not a regular directory")
        frozen_roots[key] = str(candidate.resolve(strict=True))
    marker = Path(frozen_roots["onnx_pythonpath"]) / "onnxruntime" / "__init__.py"
    if marker.is_symlink() or not marker.is_file():
        raise ReceiptError("runtime closure ONNX root lacks regular onnxruntime/__init__.py")
    provenance_binding = provenance.get("runtime_binding")
    if not isinstance(provenance_binding, dict) or not isinstance(
        provenance_binding.get("path"), str
    ):
        raise ReceiptError("w2 live request provenance lacks its gate binding")
    w2_binding_path = Path(provenance_binding["path"])
    w2_binding_row = _fresh_row(
        w2_binding_path, run_root, started_ns, "w2 runtime gate binding"
    )
    try:
        w2_binding = load_binding(w2_binding_path)
    except (RuntimeGateError, OSError, ValueError) as exc:
        raise ReceiptError(f"w2 runtime gate binding is invalid: {exc}") from exc
    if not (
        provenance.get("report_id") == "r065_w2_live_grasp_request_provenance"
        and provenance.get("passed") is True
        and isinstance(provenance.get("capture_epoch_ns"), int)
        and provenance["capture_epoch_ns"] >= started_ns
        and isinstance(provenance.get("capture_ros_time_ns"), int)
        and _finite_number(provenance.get("source_age_s"))
        and 0.0 <= float(provenance["source_age_s"]) <= 1.0
        and provenance.get("raw_request_sha256") == request_row["sha256"]
        and provenance.get("request")
        == {"path": str(request_path.resolve()), "size_bytes": request_row["size_bytes"]}
        and provenance.get("product_topics")
        == {
            "targets": {
                "topic": "/perception/garbage/targets",
                "type": "sanitation_perception_interfaces/msg/GarbageTargetArray",
                "publisher": "/pc_open_vocab_product_adapter",
            },
            "wrist_recheck": {
                "topic": "/perception/wrist/grasp_recheck",
                "type": "std_msgs/msg/String",
                "publisher": "/pc_open_vocab_product_adapter",
            },
        }
        and _w2_target_matches_request(target, request)
        and provenance["capture_ros_time_ns"] >= target["source_stamp_ns"]
        and math.isclose(
            float(provenance["source_age_s"]),
            (provenance["capture_ros_time_ns"] - target["source_stamp_ns"]) * 1e-9,
            abs_tol=1e-9,
        )
        and provenance.get("acceptance_session")
        == {"path": str(session_path.resolve()), "sha256": _hash(session_path)}
        and provenance_binding
        == {"path": str(w2_binding_path.resolve()), "sha256": w2_binding_row["sha256"]}
        and w2_binding.get("acceptance_session_binding")
        == runtime_binding.get("acceptance_session_binding")
        and w2_binding.get("runtime_closure_binding") == closure_binding
        and isinstance(provenance.get("closure_manifest"), dict)
        and provenance["closure_manifest"].get("path")
        == closure_binding.get("manifest")
        and provenance["closure_manifest"].get("sha256")
        == closure_binding.get("manifest_sha256")
        and provenance.get("perception_artifact_root")
        == frozen_roots["perception_artifact_root"]
        and provenance.get("onnx_pythonpath") == frozen_roots["onnx_pythonpath"]
    ):
        raise ReceiptError("w2 live request provenance fails its exact product contract")
    return {
        "request": request_row,
        "provenance": provenance_row,
        "runtime_gate_binding": w2_binding_row,
    }


def _matching_gate_binding_row(
    path: Path,
    run_root: Path,
    started_ns: int,
    root_binding: dict[str, Any],
    label: str,
) -> dict[str, Any]:
    row = _fresh_row(path, run_root, started_ns, label)
    try:
        child_binding = load_binding(path)
    except (RuntimeGateError, OSError, ValueError) as exc:
        raise ReceiptError(f"{label} is invalid: {exc}") from exc
    if (
        child_binding.get("acceptance_session_binding")
        != root_binding.get("acceptance_session_binding")
        or child_binding.get("runtime_closure_binding")
        != root_binding.get("runtime_closure_binding")
    ):
        raise ReceiptError(f"{label} differs from the root session/closure binding")
    return row


def _w3_live_passed(
    payload: dict[str, Any], *, runtime_schedule_sha256: str,
    walker_ids: list[str], walker_radii: dict[str, float], world_name: str,
) -> bool:
    metrics = payload.get("metrics")
    if (
        payload.get("status") != "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_PASSED"
        or payload.get("passed") is not True
        or not isinstance(metrics, dict)
    ):
        return False
    collector = metrics.get("environment_truth_collector")
    if not isinstance(collector, dict):
        return False
    minima = collector.get("minimum_walker_center_distance_m_by_pair")
    thresholds = collector.get("walker_pair_clearance_threshold_m_by_pair")
    violations = collector.get("walker_center_distance_violations_lte_0_50_m")
    expected_pairs = {
        "|".join(sorted((left, right)))
        for index, left in enumerate(walker_ids)
        for right in walker_ids[index + 1 :]
    }
    if (
        collector.get("collector_role") != "evaluator_only_no_robot_control"
        or collector.get("control_topics_published") != []
        or collector.get("product_actions_created") != []
        or collector.get("pedestrian_schedule_sha256") != runtime_schedule_sha256
        or collector.get("pose_source_topic") != f"/world/{world_name}/pose/info"
        or collector.get("gazebo_native_pose_topic") != f"/world/{world_name}/pose/info"
        or collector.get("pose_source_native_gazebo_read") is not True
        or collector.get("evaluator_native_gazebo_topics_read")
        != [f"/world/{world_name}/pose/info"]
        or collector.get("pose_source_schedule_bound_walker_ids") != walker_ids
        or collector.get("walker_radius_m_by_id") != walker_radii
        or collector.get("pose_source_is_live_gazebo_truth") is not True
        or collector.get("walker_pose_source_fresh_at_window_end") is not True
        or collector.get("walker_pose_sampling_sufficient") is not True
        or collector.get("native_pose_transport_error_count") != 0
        or collector.get("native_pose_transport_timeout_count") != 0
        or collector.get("native_pose_transport_timeout_policy")
        != "count_and_fail_closed"
        or collector.get("walker_peer_gate_passed") is not True
        or collector.get("walker_center_distance_violation_count") != 0
        or violations != []
        or not isinstance(minima, dict) or set(minima) != expected_pairs
        or not isinstance(thresholds, dict) or set(thresholds) != expected_pairs
    ):
        return False
    for pair in expected_pairs:
        distance = minima[pair]
        threshold = thresholds[pair]
        left, right = pair.split("|", 1)
        expected_threshold = walker_radii[left] + walker_radii[right]
        if (
            not _finite_number(distance)
            or not _finite_number(threshold)
            or not math.isclose(float(threshold), expected_threshold, abs_tol=1e-9)
            or not math.isclose(float(threshold), 0.50, abs_tol=1e-9)
            or float(distance) <= float(threshold)
        ):
            return False
    return True


def _mapping_runtime_passed(payload: dict[str, Any]) -> bool:
    return (
        payload.get("passed") is True
        and payload.get("truth_used_for_control") is False
        and payload.get("command_topic_publishers") == _COMMAND_TOPIC_PUBLISHERS
        and payload.get("command_chain_publishers_attributed") is True
        and payload.get("active_command_chain_command_timeout_count") == 0
        and _finite_number(payload.get("odom_displacement_m"))
        and float(payload["odom_displacement_m"]) >= 0.10
    )


def _runtime_schedule_walker_contract(
    value: dict[str, Any],
) -> tuple[list[str], dict[str, float], str]:
    world_name = value.get("world_name")
    if not isinstance(world_name, str) or not world_name:
        raise ReceiptError("prepared runtime schedule lacks its world name")
    pedestrians = value.get("pedestrians")
    if not isinstance(pedestrians, list) or len(pedestrians) != 8:
        raise ReceiptError("prepared runtime schedule must contain exactly eight walkers")
    ids: list[str] = []
    radii: dict[str, float] = {}
    for row in pedestrians:
        if not isinstance(row, dict):
            raise ReceiptError("prepared runtime schedule walker is invalid")
        identity = row.get("object_id")
        radius = row.get("radius_m")
        if not isinstance(identity, str) or not identity or not _finite_number(radius):
            raise ReceiptError("prepared runtime schedule walker contract is invalid")
        if not math.isclose(float(radius), 0.25, abs_tol=1e-9):
            raise ReceiptError("prepared runtime schedule walker radius is not 0.25 m")
        ids.append(identity)
        radii[identity] = float(radius)
    if len(set(ids)) != 8:
        raise ReceiptError("prepared runtime schedule walker IDs are not unique")
    return ids, radii, world_name


def _w5_passed(payload: dict[str, Any]) -> bool:
    checks = payload.get("checks")
    return (
        payload.get("status") == "FORMAL_FIRST_MAP_THEN_SAVED_MAP_CLEANING_PASSED"
        and payload.get("passed") is True
        and isinstance(checks, dict)
        and checks.get("quality_gated_map_manifest") is True
        and checks.get("mapping_runtime_passed") is True
        and checks.get("mapping_safe_profile_retains_0_45_m_s") is True
        and checks.get("saved_map_cleaning_runtime_passed") is True
    )


def _w5_mapping_evidence_row(
    payload: dict[str, Any], root: Path, started_ns: int
) -> dict[str, Any]:
    evidence = payload.get("evidence")
    mapping_path = evidence.get("mapping_runtime") if isinstance(evidence, dict) else None
    if not isinstance(mapping_path, str) or not mapping_path:
        raise ReceiptError("w5 report lacks mapping runtime evidence path")
    mapping = Path(mapping_path)
    row = _fresh_row(mapping, root, started_ns, "w5 mapping runtime evidence")
    if not _mapping_runtime_passed(_object(mapping)):
        raise ReceiptError("w5 mapping runtime evidence fails its exact contract")
    return row


def _child_passed(
    name: str, payload: dict[str, Any], *, runtime_schedule_sha256: str | None = None,
    walker_ids: list[str] | None = None, walker_radii: dict[str, float] | None = None,
    world_name: str | None = None,
) -> bool:
    if name == "w1":
        return (
            payload.get("result") == "PASS"
            and payload.get("runtime_only") is True
            and payload.get("input_type") == "geometry_msgs/msg/Polygon"
            and payload.get("published_type") == "geometry_msgs/msg/PolygonStamped"
            and payload.get("profiles_read_back")
            == ["transport_stowed", "cleaning_deployed", "arm_deployed"]
            and payload.get("base_motion_inhibit_independent_safety_subscriber") is True
            and payload.get("safety_status_fresh_per_override") is True
            and payload.get("safety_manager_state") == "BASE_COMMAND_STOPPED"
            and payload.get("safety_manager_reason") == "manipulator_base_inhibit"
            and payload.get("test_override_preserves_base_inhibit") is True
            and payload.get("test_override_never_authorizes_motion") is True
            and payload.get("fresh_readback_required_per_override") is True
        )
    if name == "w2":
        return _w2_passed(payload)
    if name == "w3_public_audit":
        return payload.get("scope") == "public_train_val_only" and payload.get("hidden_accessed") is False and payload.get("map_count") == 40 and payload.get("episode_count") == 800 and payload.get("pedestrian_path_count") == 6400 and payload.get("pedestrian_pair_count") == 22400 and payload.get("pedestrian_static_collision_path_count") == 0 and payload.get("pedestrian_cube_collision_path_count") == 0 and payload.get("pedestrian_pair_violation_count") == 0
    if name == "w3_live_dynamic":
        return (
            runtime_schedule_sha256 is not None
            and walker_ids is not None and walker_radii is not None
            and world_name is not None
            and _w3_live_passed(
                payload, runtime_schedule_sha256=runtime_schedule_sha256,
                walker_ids=walker_ids, walker_radii=walker_radii,
                world_name=world_name,
            )
        )
    if name == "w5":
        return _w5_passed(payload)
    return False


def seal_stdout(input_path: Path, output: Path) -> None:
    """Atomically turn a successful W2 stdout JSON object into its receipt."""
    if output.exists() or input_path.is_symlink() or not input_path.is_file():
        raise ReceiptError("stdout receipt input/output is unsafe")
    payload = _object(input_path)
    if not _child_passed("w2", payload):
        raise ReceiptError("stdout receipt has no passing result")
    temporary = output.with_suffix(output.suffix + f".pending.{os.getpid()}")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(output)


def publish(
    *, repository_root: Path, run_root: Path, session: Path, runtime_binding: Path,
    episode_manifest: Path, world: Path, environment_schedule: Path, runtime_schedule: Path,
    children: dict[str, Path], output: Path,
) -> dict[str, Any]:
    repository_root = repository_root.resolve()
    if run_root.is_symlink() or not run_root.is_dir():
        raise ReceiptError("run root must be a non-symlink directory")
    run_root = run_root.resolve()
    if output.exists():
        raise ReceiptError("refusing to overwrite retained R065 receipt")
    if output.parent.resolve() != run_root:
        raise ReceiptError("receipt must be written directly into the new run root")
    if "hidden" in str(run_root).lower():
        raise ReceiptError("hidden paths are forbidden in the public modeling gate")

    session_path = _regular_in(session, run_root, "acceptance session")
    session_value = _object(session_path)
    if session_value.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ReceiptError("formal acceptance session is not RUNNING")
    started_ns = session_value.get("started_epoch_ns")
    if not isinstance(started_ns, int) or started_ns <= 0 or started_ns > time.time_ns():
        raise ReceiptError("acceptance session has an invalid start time")
    if session_path.stat().st_mtime_ns < started_ns:
        raise ReceiptError("acceptance session file predates its own start")

    binding_row = _fresh_row(runtime_binding, run_root, started_ns, "runtime gate binding")
    try:
        binding = load_binding(runtime_binding)
    except (RuntimeGateError, OSError, ValueError) as exc:
        raise ReceiptError(f"runtime gate binding is invalid: {exc}") from exc
    if binding["acceptance_session_binding"].get("session_manifest") != str(session_path):
        raise ReceiptError("runtime binding selects a different acceptance session")
    if binding["acceptance_session_binding"].get("session_started_epoch_ns") != started_ns:
        raise ReceiptError("runtime binding session start differs")
    if binding["acceptance_session_binding"].get("session_manifest_sha256") != _hash(session_path):
        raise ReceiptError("runtime binding session hash differs")

    canonical_snapshot = repository_root / "reports/engineering/formal_vehicle_snapshot_manifest.json"
    snapshot = session_value.get("snapshot")
    if not canonical_snapshot.is_file() or not isinstance(snapshot, dict):
        raise ReceiptError("canonical snapshot is missing or unbound")
    try:
        snapshot_identity = _snapshot_identity(canonical_snapshot)
    except (AcceptanceSessionError, OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ReceiptError(f"canonical snapshot identity is invalid: {exc}") from exc
    if snapshot != snapshot_identity:
        raise ReceiptError("acceptance session snapshot differs from canonical snapshot")
    if binding["acceptance_session_binding"].get("snapshot") != snapshot_identity:
        raise ReceiptError("runtime binding snapshot differs from canonical snapshot")

    episode_path = _regular_in(episode_manifest, run_root, "public episode manifest")
    episode = _object(episode_path)
    if episode.get("split") not in {"train", "val"}:
        raise ReceiptError("only public train/val episodes are admissible")
    if "hidden" in episode_path.name.lower() or "hidden" in json.dumps(episode).lower():
        raise ReceiptError("hidden materialization is forbidden")
    schedule_path = _regular_in(environment_schedule, run_root, "environment schedule")
    schedule = _object(schedule_path)
    if schedule.get("access") != "environment_driver_only_not_robot_control":
        raise ReceiptError("base environment schedule is not driver-only")
    runtime_schedule_path = _regular_in(runtime_schedule, run_root, "prepared runtime schedule")
    runtime_schedule_value = _object(runtime_schedule_path)
    environment = runtime_schedule_value.get("acceptance_environment")
    if not isinstance(environment, dict) or environment.get("product_control_access_prohibited") is not True:
        raise ReceiptError("prepared runtime schedule does not prohibit product control truth access")
    walker_ids, walker_radii, world_name = _runtime_schedule_walker_contract(
        runtime_schedule_value
    )
    world_row = _fresh_row(world, run_root, started_ns, "public world")
    episode_row = _fresh_row(episode_path, run_root, started_ns, "public episode manifest")
    schedule_row = _fresh_row(schedule_path, run_root, started_ns, "environment schedule")
    runtime_schedule_row = _fresh_row(runtime_schedule_path, run_root, started_ns, "prepared runtime schedule")

    child_rows: dict[str, dict[str, Any]] = {}
    for name in CHILD_NAMES:
        path = children.get(name)
        if path is None:
            raise ReceiptError(f"missing required {name} gate")
        row = _fresh_row(path, run_root, started_ns, f"{name} output")
        payload = _object(path)
        if not _child_passed(
            name, payload, runtime_schedule_sha256=runtime_schedule_row["sha256"],
            walker_ids=walker_ids, walker_radii=walker_radii, world_name=world_name,
        ):
            raise ReceiptError(f"{name} output fails its exact R065 semantic contract")
        row["key_pass"] = {
            key: payload[key] for key in ("passed", "result", "status") if key in payload
        }
        if name == "w5":
            row["mapping_runtime_evidence"] = _w5_mapping_evidence_row(
                payload, run_root, started_ns
            )
        if name == "w2":
            row["live_request_evidence"] = _w2_request_provenance_rows(
                run_root, started_ns, session_path, binding
            )
        if name == "w1":
            row["runtime_gate_binding"] = _matching_gate_binding_row(
                run_root / "w1.runtime_binding.json",
                run_root,
                started_ns,
                binding,
                "w1 runtime gate binding",
            )
        if name == "w3_live_dynamic":
            row["runtime_gate_binding"] = _matching_gate_binding_row(
                run_root / "w3.runtime_binding.json",
                run_root,
                started_ns,
                binding,
                "w3 runtime gate binding",
            )
        child_rows[name] = row

    result = {
        "schema_version": 1,
        "report_id": "r065_public_modeling_receipt",
        "status": "R065_PUBLIC_MODELING_PASSED",
        "passed": True,
        "run_root": str(run_root),
        "FORMAL_RUNTIME_GATE_BOUND": binding,
        "runtime_gate_binding_file": binding_row,
        "acceptance_session": _fresh_row(session_path, run_root, started_ns, "acceptance session"),
        "canonical_snapshot": {
            "path": str(canonical_snapshot.resolve()), "sha256": _hash(canonical_snapshot),
            "identity": snapshot_identity,
        },
        "episode_provenance": {
            "split": episode["split"], "world": world_row, "manifest": episode_row,
            "environment_schedule": schedule_row, "prepared_runtime_schedule": runtime_schedule_row,
            "environment_schedule_boundary": "environment_driver_only_not_evaluator_or_control_truth",
        },
        "children": child_rows,
    }
    temporary = output.with_suffix(output.suffix + f".pending.{os.getpid()}")
    temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(output)
    return result


def main() -> int:
    if len(sys.argv) == 4 and sys.argv[1] == "--seal-stdout":
        try:
            seal_stdout(Path(sys.argv[2]), Path(sys.argv[3]))
        except ReceiptError as exc:
            print(json.dumps({"status": "R065_PUBLIC_MODELING_BLOCKED", "error": str(exc)}))
            return 2
        return 0
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--environment-schedule", type=Path, required=True)
    parser.add_argument("--runtime-schedule", type=Path, required=True)
    for name in CHILD_NAMES:
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        value = publish(
            repository_root=args.repository_root, run_root=args.run_root, session=args.session,
            runtime_binding=args.runtime_binding, episode_manifest=args.episode_manifest,
            world=args.world, environment_schedule=args.environment_schedule, runtime_schedule=args.runtime_schedule,
            children={name: getattr(args, name) for name in CHILD_NAMES}, output=args.output,
        )
    except ReceiptError as exc:
        print(json.dumps({"status": "R065_PUBLIC_MODELING_BLOCKED", "error": str(exc)}))
        return 2
    print(json.dumps(value, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
