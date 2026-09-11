#!/usr/bin/env python3
"""Canonical A12/A20 product MCAP replay and metric recalculation producer.

The producer reads the retained MCAP itself, performs a real ``ros2 bag play``
in an isolated ROS domain, recalculates coverage/localization from recorded
samples, and atomically publishes a receipt bound to the current formal
session, snapshot, and runtime closure.  It never starts Gazebo.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import time
import uuid
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = "tzcup.formal_product_mcap_replay.v1"
PRODUCER_ID = "scripts/formal_product_mcap_replay.py"
PRODUCT_TOPICS = {
    "/perception/garbage/detections_2d": "observations",
    "/perception/garbage/targets": "tracks_and_dynamic_trash_map",
    "/spot_clean/state": "scheduler_and_post_clean_state",
    "/garbage/cleaning_events": "clean_decisions",
}


class ProductReplayError(RuntimeError):
    """The MCAP cannot become current formal product replay evidence."""


def _non_link_path(path: Path, label: str) -> Path:
    candidate = path.absolute()
    current = Path(candidate.anchor)
    for part in candidate.parts[1:]:
        current /= part
        if current.exists() and current.is_symlink():
            raise ProductReplayError(f"{label} has a symbolic-link component")
    if candidate.resolve() != candidate:
        raise ProductReplayError(f"{label} is not a canonical path")
    return candidate


def sha256(path: Path) -> str:
    path = _non_link_path(path, "file")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact_sha256(path: Path) -> str:
    """Hash a regular file or a bag directory with names and file digests."""
    path = _non_link_path(path, "artifact")
    if path.is_file() and not path.is_symlink():
        return sha256(path)
    if not path.is_dir() or path.is_symlink():
        raise ProductReplayError(f"artifact is missing, non-regular, or a symlink: {path}")
    digest = hashlib.sha256()
    files = sorted(item for item in path.rglob("*") if item.is_file())
    if not files:
        raise ProductReplayError(f"artifact directory is empty: {path}")
    for item in files:
        if item.is_symlink():
            raise ProductReplayError(f"artifact contains a symlink: {item}")
        relative = item.relative_to(path).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(4, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(sha256(item)))
    return digest.hexdigest()


def read_object(path: Path) -> dict[str, Any]:
    path = _non_link_path(path, "JSON input")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ProductReplayError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ProductReplayError(f"JSON root is not an object: {path}")
    return value


def write_fresh_json(path: Path, value: dict[str, Any]) -> None:
    """Atomically publish a new receipt without replacing retained evidence."""
    path = path.absolute()
    path.parent.mkdir(parents=True, exist_ok=True)
    _non_link_path(path.parent, "output parent")
    if path.exists() or path.is_symlink():
        raise ProductReplayError(f"refusing to overwrite retained evidence: {path}")
    pending = path.with_name(f".{path.name}.pending.{uuid.uuid4().hex}")
    data = (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8")
    descriptor = os.open(
        pending,
        os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0),
        0o600,
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(pending, path)
    finally:
        try:
            pending.unlink()
        except FileNotFoundError:
            pass


def relative_delta(actual: float, expected: float) -> float:
    return abs(actual - expected) / max(abs(expected), 1.0e-12)


def _message_pose(message: Any) -> Any:
    return message.pose.pose if hasattr(message.pose, "pose") else message.pose


def read_mcap_records(bag: Path) -> dict[str, Any]:
    """Read and deserialize the exact topics used for product recalculation."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:  # pragma: no cover - exercised on the ROS runtime
        raise ProductReplayError(f"ROS 2 MCAP reader dependencies are unavailable: {exc}") from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    counts = {topic: 0 for topic in PRODUCT_TOPICS}
    chain = {
        "observation_count": 0,
        "unique_track_ids": set(),
        "track_observation_count": 0,
        "track_states": set(),
        "scheduler_states": set(),
        "clean_event_ids": set(),
        "cleaned_fraction_sum": 0.0,
        "post_clean_success_count": 0,
    }
    fused: list[tuple[float, float, float]] = []
    truth: list[tuple[float, float, float]] = []
    brush_points: list[tuple[float, float, float]] = []
    evaluation_stamps: list[float] = []
    coverage_states: list[str] = []
    brush_enabled = False
    selected = set(PRODUCT_TOPICS) | {
        "/brush_enabled",
        "/coverage/evaluation_sample",
        "/coverage/state",
        "/ground_truth/odom",
        "/localization/fused_pose",
    }
    while reader.has_next():
        topic, data, _received_stamp = reader.read_next()
        if topic not in selected:
            continue
        if topic not in topic_types:
            raise ProductReplayError(f"MCAP has data without a declared topic type: {topic}")
        message = deserialize_message(data, get_message(topic_types[topic]))
        if topic in counts:
            counts[topic] += 1
            if topic == "/perception/garbage/detections_2d":
                chain["observation_count"] += len(message.detections)
            elif topic == "/perception/garbage/targets":
                for target in message.targets:
                    chain["unique_track_ids"].add(str(target.uuid))
                    chain["track_observation_count"] += int(target.observation_count)
                    chain["track_states"].add(str(target.track_state))
            elif topic == "/spot_clean/state":
                chain["scheduler_states"].add(str(message.data))
            elif topic == "/garbage/cleaning_events":
                chain["clean_event_ids"].add(str(message.event_id))
                chain["cleaned_fraction_sum"] += float(message.cleaned_fraction)
                if str(message.result).upper() in {"SUCCESS", "CLEANED", "PASS"} and float(message.cleaned_fraction) > 0.0:
                    chain["post_clean_success_count"] += 1
            continue
        if topic == "/brush_enabled":
            brush_enabled = bool(message.data)
            continue
        if topic == "/coverage/evaluation_sample":
            sample = json.loads(message.data)
            stamp = float(sample["stamp_sec"])
            evaluation_stamps.append(stamp)
            if sample["brush_enabled"]:
                yaw = float(sample["yaw_rad"])
                brush_points.append(
                    (
                        stamp,
                        float(sample["base_x_m"]) + 0.55 * math.cos(yaw),
                        float(sample["base_y_m"]) + 0.55 * math.sin(yaw),
                    )
                )
            continue
        if topic == "/coverage/state":
            coverage_states.append(str(message.data))
            continue
        pose = _message_pose(message)
        stamp = message.header.stamp
        sample = (stamp.sec + stamp.nanosec * 1.0e-9, float(pose.position.x), float(pose.position.y))
        if topic == "/ground_truth/odom" and brush_enabled and "/coverage/evaluation_sample" not in topic_types:
            orientation = pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
            )
            brush_points.append((sample[0], sample[1] + 0.55 * math.cos(yaw), sample[2] + 0.55 * math.sin(yaw)))
        (truth if topic == "/ground_truth/odom" else fused).append(sample)
    return {
        "topic_types": topic_types,
        "product_topic_counts": counts,
        "product_chain_recalculation": {
            **chain,
            "unique_track_ids": sorted(chain["unique_track_ids"]),
            "track_states": sorted(chain["track_states"]),
            "scheduler_states": sorted(chain["scheduler_states"]),
            "clean_event_ids": sorted(chain["clean_event_ids"]),
        },
        "fused": fused,
        "truth": truth,
        "brush_points": brush_points,
        "evaluation_stamps": evaluation_stamps,
        "coverage_states": coverage_states,
    }


def recalculate(records: dict[str, Any], source_metrics: dict[str, Any]) -> dict[str, Any]:
    """Recompute coverage and localization; never trust embedded pass flags."""
    try:
        from sanitation_coverage.metrics import empirical_swept_metrics, summarize_distances, synchronized_xy_errors
    except ImportError as exc:  # pragma: no cover - exercised on the ROS runtime
        raise ProductReplayError(f"coverage metric dependencies are unavailable: {exc}") from exc

    fused = list(records["fused"])
    truth = list(records["truth"])
    stamps = list(records["evaluation_stamps"])
    if stamps:
        start, end = min(stamps), max(stamps)
        fused = [sample for sample in fused if start <= sample[0] <= end]
        truth = [sample for sample in truth if start <= sample[0] <= end]
    else:
        start = end = None
    errors, sync_errors, dropped = synchronized_xy_errors(fused, truth)
    localization = summarize_distances(errors)
    expected_rmse = float(source_metrics["localization_regression_during_coverage"]["rmse_m"])
    actual_rmse = localization["rmse_m"]
    rmse_delta = relative_delta(float(actual_rmse), expected_rmse) if actual_rmse is not None else None
    geometry = source_metrics["mission_geometry"]
    empirical = empirical_swept_metrics(
        geometry["cleanable_outer_polygon"],
        records["brush_points"],
        float(source_metrics["operation_width_m"]),
        resolution=float(source_metrics["empirical_metrics"]["resolution_m"]),
        exclusion_polygons=geometry["cleanable_exclusion_polygons"],
    )
    expected_coverage = float(source_metrics["empirical_metrics"]["coverage_rate"])
    actual_coverage = float(empirical["coverage_rate"])
    return {
        "coverage": {
            "source_rate": expected_coverage,
            "recalculated_rate": actual_coverage,
            "relative_delta": relative_delta(actual_coverage, expected_coverage),
            "brush_on_sample_count": len(records["brush_points"]),
        },
        "localization": {
            "source_rmse_m": expected_rmse,
            "recalculated_rmse_m": actual_rmse,
            "relative_delta": rmse_delta,
            "estimate_sample_count": len(fused),
            "truth_sample_count": len(truth),
            "matched_sample_count": len(errors),
            "dropped_estimate_count": dropped,
            "sync_error_sample_count": len(sync_errors),
            "evaluation_window_start_sec": start,
            "evaluation_window_end_sec": end,
        },
    }


def play_mcap(bag: Path, *, domain_id: int, rate: float, timeout_seconds: float) -> dict[str, Any]:
    if not 1 <= domain_id <= 232:
        raise ProductReplayError("isolated ROS_DOMAIN_ID must be in [1, 232]")
    if rate <= 0.0:
        raise ProductReplayError("playback rate must be positive")
    command = ["ros2", "bag", "play", str(bag), "--storage", "mcap", "--rate", str(rate)]
    environment = os.environ.copy()
    environment.update({"ROS_DOMAIN_ID": str(domain_id), "ROS_LOCALHOST_ONLY": "1"})
    started = time.time_ns()
    try:
        completed = subprocess.run(
            command,
            env=environment,
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout_seconds,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ProductReplayError(f"ros2 bag play failed: {exc}") from exc
    finished = time.time_ns()
    return {
        "command": command,
        "ros_domain_id": domain_id,
        "ros_localhost_only": True,
        "started_epoch_ns": started,
        "finished_epoch_ns": finished,
        "exit_code": completed.returncode,
        "stdout_sha256": hashlib.sha256(completed.stdout.encode()).hexdigest(),
        "stderr_sha256": hashlib.sha256(completed.stderr.encode()).hexdigest(),
    }


def _git_identity(repository_root: Path) -> dict[str, str]:
    def run(*arguments: str) -> str:
        result = subprocess.run(
            ["git", "-C", str(repository_root), *arguments],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode:
            raise ProductReplayError(f"git {' '.join(arguments)} failed: {result.stderr.strip()}")
        return result.stdout.strip()
    return {"source_commit": run("rev-parse", "HEAD"), "source_tree": run("rev-parse", "HEAD^{tree}")}


def bind_formal_context(
    repository_root: Path,
    session_path: Path,
    snapshot_path: Path,
    runtime_binding_path: Path,
) -> dict[str, Any]:
    from formal_acceptance_session import _snapshot_identity
    from formal_runtime_gate_binding import load_binding
    from generate_formal_vehicle_snapshot import verify_snapshot

    for path, label in (
        (session_path, "formal session"),
        (snapshot_path, "formal snapshot"),
        (runtime_binding_path, "runtime binding"),
    ):
        _non_link_path(path, label)
    session = read_object(session_path)
    if session.get("report_id") != "tzcup_formal_final_acceptance_session_v1":
        raise ProductReplayError("wrong formal session identity")
    if session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ProductReplayError("product replay must be produced while the formal session is RUNNING")
    try:
        snapshot = _snapshot_identity(snapshot_path)
        verified_snapshot = verify_snapshot(repository_root, snapshot_path)
        binding = load_binding(runtime_binding_path)
    except RuntimeError as exc:
        raise ProductReplayError(f"formal context verification failed: {exc}") from exc
    if verified_snapshot.get("source_inventory_sha256") != snapshot["source_inventory_sha256"]:
        raise ProductReplayError("current repository sources do not match the frozen snapshot")
    if session.get("snapshot") != snapshot:
        raise ProductReplayError("formal session and current snapshot differ")
    acceptance = binding["acceptance_session_binding"]
    if acceptance.get("snapshot") != snapshot:
        raise ProductReplayError("runtime gate and current snapshot differ")
    if acceptance.get("session_started_epoch_ns") != session.get("started_epoch_ns"):
        raise ProductReplayError("runtime gate and formal session start differ")
    if acceptance.get("session_manifest_sha256") != sha256(session_path):
        raise ProductReplayError("runtime gate does not bind the current session file")
    if binding.get("runtime_closure_binding") != session.get("runtime_closure_binding"):
        raise ProductReplayError("runtime gate and session closure bindings differ")
    closure = binding["runtime_closure_binding"]
    if closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED":
        raise ProductReplayError("runtime closure is not VERIFIED")
    return {
        **_git_identity(repository_root),
        "session": {
            "path": str(session_path.resolve()),
            "sha256_at_replay": sha256(session_path),
            "report_id": session["report_id"],
            "status_at_replay": session["status"],
            "started_epoch_ns": session["started_epoch_ns"],
        },
        "snapshot": snapshot,
        "snapshot_manifest": {"path": str(snapshot_path.resolve()), "sha256": sha256(snapshot_path)},
        "runtime_closure_binding": closure,
        "runtime_gate_binding": {"path": str(runtime_binding_path.resolve()), "sha256": sha256(runtime_binding_path)},
    }


def produce_report(
    *,
    repository_root: Path,
    bag: Path,
    source_metrics_path: Path,
    session_path: Path,
    snapshot_path: Path,
    runtime_binding_path: Path,
    input_hashes: dict[str, str],
    input_artifacts: dict[str, dict[str, str]] | None = None,
    domain_id: int,
    playback_rate: float,
    timeout_seconds: float,
    reader=read_mcap_records,
    player=play_mcap,
) -> dict[str, Any]:
    context = bind_formal_context(repository_root, session_path, snapshot_path, runtime_binding_path)
    started_ns = int(context["session"]["started_epoch_ns"])
    for artifact, label in ((bag, "MCAP"), (source_metrics_path, "source metrics")):
        if artifact.stat().st_mtime_ns < started_ns:
            raise ProductReplayError(f"{label} predates the current formal session")
    if not (bag / "metadata.yaml").is_file():
        raise ProductReplayError("MCAP directory has no metadata.yaml")
    records = reader(bag)
    required_topics = set(PRODUCT_TOPICS) | {
        "/coverage/evaluation_sample", "/coverage/state", "/ground_truth/odom", "/localization/fused_pose"
    }
    missing_topics = sorted(required_topics - set(records["topic_types"]))
    source_metrics = read_object(source_metrics_path)
    recalculated = recalculate(records, source_metrics)
    playback = player(bag, domain_id=domain_id, rate=playback_rate, timeout_seconds=timeout_seconds)
    counts = records["product_topic_counts"]
    checks = {
        "mcap_metadata_readable": True,
        "required_product_and_metric_topics_present": not missing_topics,
        "product_chain_observed": all(counts.get(topic, 0) > 0 for topic in PRODUCT_TOPICS),
        "coverage_terminal_state_observed": any(
            state in {"COMPLETED", "FAILED", "RECOVERY"} for state in records["coverage_states"]
        ),
        "coverage_recalculated_within_1_percent": (
            recalculated["coverage"]["brush_on_sample_count"] > 0
            and recalculated["coverage"]["relative_delta"] <= 0.01
        ),
        "localization_recalculated_within_1_percent": (
            recalculated["localization"]["matched_sample_count"] > 0
            and recalculated["localization"]["relative_delta"] is not None
            and recalculated["localization"]["relative_delta"] <= 0.01
        ),
        "ros2_bag_play_exit_zero": playback["exit_code"] == 0,
        "current_formal_session_snapshot_closure_bound": True,
    }
    return {
        "schema": SCHEMA,
        "status": "FORMAL_PRODUCT_MCAP_REPLAY_PASS" if all(checks.values()) else "FORMAL_PRODUCT_MCAP_REPLAY_BLOCKED",
        "pass": all(checks.values()),
        "producer": {"id": PRODUCER_ID, "sha256": sha256(repository_root / PRODUCER_ID)},
        "produced_epoch_ns": time.time_ns(),
        "bag": {"path": str(bag.resolve()), "sha256": artifact_sha256(bag)},
        "source_metrics": {"path": str(source_metrics_path.resolve()), "sha256": sha256(source_metrics_path)},
        "input_hashes": input_hashes,
        "input_artifacts": input_artifacts or {},
        "formal_context": context,
        "missing_topics": missing_topics,
        "product_chain": {
            "topic_counts": counts,
            "semantic_roles": PRODUCT_TOPICS,
            "recalculated": records.get("product_chain_recalculation", {}),
        },
        "coverage": recalculated["coverage"],
        "localization": recalculated["localization"],
        "playback": playback,
        "checks": checks,
    }


def _parse_input_artifacts(values: Iterable[str], container_digest: str) -> tuple[dict[str, str], dict[str, dict[str, str]]]:
    required = {"model", "config", "dataset", "dependency"}
    references: dict[str, dict[str, str]] = {}
    for value in values:
        name, separator, raw_path = value.partition("=")
        if not separator or name not in required or not raw_path:
            raise ProductReplayError(f"invalid --input-artifact {value!r}; expected name=<path>")
        path = Path(raw_path).absolute()
        references[name] = {"path": str(path), "sha256": artifact_sha256(path)}
    if set(references) != required:
        raise ProductReplayError(f"input artifacts must contain exactly {sorted(required)}")
    digest = container_digest.removeprefix("sha256:")
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise ProductReplayError("--container-digest must be an immutable SHA-256 digest")
    hashes = {name: reference["sha256"] for name, reference in references.items()}
    hashes["container"] = digest
    return hashes, references


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--source-metrics", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--input-artifact", action="append", default=[], help="model|config|dataset|dependency=<file-or-directory>")
    parser.add_argument("--container-digest", required=True, help="sha256:<64 hex> or 64 hex")
    parser.add_argument("--ros-domain-id", type=int, required=True)
    parser.add_argument("--playback-rate", type=float, default=1.0)
    parser.add_argument("--timeout-seconds", type=float, default=7200.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        input_hashes, input_artifacts = _parse_input_artifacts(args.input_artifact, args.container_digest)
        report = produce_report(
            repository_root=args.repository_root.resolve(),
            bag=args.bag.absolute(),
            source_metrics_path=args.source_metrics.absolute(),
            session_path=args.session.absolute(),
            snapshot_path=args.snapshot.absolute(),
            runtime_binding_path=args.runtime_binding.absolute(),
            input_hashes=input_hashes,
            input_artifacts=input_artifacts,
            domain_id=args.ros_domain_id,
            playback_rate=args.playback_rate,
            timeout_seconds=args.timeout_seconds,
        )
        write_fresh_json(args.output.absolute(), report)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        print(json.dumps({"schema": SCHEMA, "status": "FORMAL_PRODUCT_MCAP_REPLAY_BLOCKED", "pass": False, "error": str(exc)}, sort_keys=True))
        return 2
    print(json.dumps(report, sort_keys=True))
    return 0 if report["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
