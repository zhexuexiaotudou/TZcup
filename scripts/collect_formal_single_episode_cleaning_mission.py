#!/usr/bin/env python3
"""Collect one fail-closed cleaning mission from one live ROS / Gazebo run.

The collector is observation-only. It freezes every file and directory passed
to the product launch, records initial and terminal evaluator states, keeps the
complete per-target grasp evidence, queries live product parameters, and audits
the executable ROS graph for subscriptions to evaluator truth.
"""

from __future__ import annotations

import argparse
import array
import contextlib
import heapq
import hashlib
import json
import math
import numbers
import os
import re
import time
import tempfile
from pathlib import Path
from typing import Any


def publish_raw_report(path: Path, report: dict[str, Any]) -> None:
    """Publish the completed collector/recorder handoff without a partial file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, prefix=path.name + ".pending-",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        # link is atomic and fails if a previous run already owns this path.
        os.link(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


PRODUCT_TOPICS = {
    "planner": "/active_cleaning/planner_status",
    "mission_complete": "/active_cleaning/mission_complete",
    "trajectory": "/active_cleaning/trajectory",
    "grasp_result": "/active_cleaning/grasp_result",
    "odometry": "/odom",
}
EVALUATOR_TOPICS = {
    "ground_dirt": "/evaluation/single_episode/ground_dirt/status_json",
    "water": "/evaluation/single_episode/water_recovery/status_json",
    "dry_bin": "/evaluation/single_episode/dry_bin/status_json",
    "pedestrians": "/scenario/environment/pedestrian_driver/status",
    "collision": "/collision_monitor_state",
    "front_bumper": "/formal_vehicle/simulation/raw/front_bumper/contact",
    "rear_bumper": "/formal_vehicle/simulation/raw/rear_bumper/contact",
}
CONTROL_PROHIBITED_TRUTH_TOPICS = tuple(
    EVALUATOR_TOPICS[key] for key in ("ground_dirt", "water", "dry_bin", "pedestrians")
)
REPLAY_METRIC_TOPICS = {
    "post_safety_brush_command": {
        "name": "/brush_controller/commands", "type": "std_msgs/msg/Float64MultiArray",
        "stamp_semantics": "collector_ros_time_at_post_safety_controller_receipt_no_message_header",
    },
    "fused_odom": {
        "name": "/localization/fused_odom", "type": "nav_msgs/msg/Odometry",
        "stamp_semantics": "positive_message_header_stamp_ns_callback_arrival_preserved",
    },
    "ground_truth_odom": {
        "name": "/ground_truth/odom", "type": "nav_msgs/msg/Odometry",
        "stamp_semantics": "positive_message_header_stamp_ns_callback_arrival_preserved",
    },
}
LOCALIZATION_PAIR_TOLERANCE_NS = 50_000_000
PAIRING_SORT_CHUNK_ROWS = 4096
HIGH_FIDELITY_CLEANING_MECHANISM = (
    Path(__file__).resolve().parents[1]
    / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro"
)
RUNTIME_PARAMETER_CONTRACT = {
    "/formal_active_cleaning_policy_planner": (
        "policy_checkpoint", "episode_seed", "maximum_task_distance_m"
    ),
    "/pc_open_vocab_product_adapter": ("artifact_root",),
    "/formal_map_lifecycle_manager": ("mode", "episode_manifest", "artifact_directory"),
}
MULTISITE_INTERFACES = {
    "dosod": {
        "name": "/perception/open_vocab/dosod_boxes",
        "type": "vision_msgs/msg/Detection2DArray",
        "interface_kind": "topic",
        "observed_topic": "/perception/open_vocab/dosod_boxes",
    },
    "edgesam": {
        "name": "/perception/ground_dirt/masks",
        "type": "sensor_msgs/msg/Image",
        "interface_kind": "topic",
        "observed_topic": "/perception/ground_dirt/masks",
    },
    "nav2": {
        "name": "/follow_path",
        "type": "nav2_msgs/action/FollowPath",
        "interface_kind": "action",
        "observed_topic": "/follow_path/_action/status",
    },
    "dynamic_pedestrians": {
        "name": "/scenario/environment/pedestrian_driver/status",
        "type": "std_msgs/msg/String",
        "interface_kind": "topic",
        "observed_topic": "/scenario/environment/pedestrian_driver/status",
    },
    "cleaning_actuator": {
        "name": "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/motor_current_a",
        "type": "std_msgs/msg/Float64MultiArray",
        "interface_kind": "topic",
        "observed_topic": "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/motor_current_a",
    },
}
REQUIRED_RUNTIME_NODES = {
    "/formal_active_cleaning_policy_planner",
    "/formal_physical_grasp_executor",
    "/pc_open_vocab_product_adapter",
    "/formal_map_lifecycle_manager",
    "/formal_product_demo_operator_gate",
    "/collision_monitor",
}
TRUSTED_GT_RECORDER_NODE = "/a12_trusted_gt_recorder"
RAW_GROUND_TRUTH_TOPIC = "/ground_truth/model_odom_raw"
RAW_GROUND_TRUTH_ADAPTER_NODE = "/formal_model_ground_truth_adapter"


class CollectionError(RuntimeError):
    pass


def replay_metric_capture_complete(
    metric_stats: dict[str, dict[str, int]],
    brush_on_record_count: int,
    localization_pairing: dict[str, Any],
    window: dict[str, Any],
) -> bool:
    """Require runtime evidence for every replay metric, never inferred values."""
    return (
        window.get("operator_start_ros_time_ns") is not None
        and window.get("task_complete_ros_time_ns") is not None
        and all(
            metric_stats.get(name, {}).get("window_received", 0) > 0
            and metric_stats.get(name, {}).get("written", 0) > 0
            and metric_stats.get(name, {}).get("invalid_stamp", 0) == 0
            and metric_stats.get(name, {}).get("invalid_sample", 0) == 0
            for name in REPLAY_METRIC_TOPICS
        )
        and brush_on_record_count > 0
        and localization_pairing.get("status") == "COMPLETE"
        and localization_pairing.get("input_streams_complete") is True
        and localization_pairing.get("paired_sample_count", 0) > 0
    )


def positive_stamp_ns(stamp_ns: int) -> bool:
    return type(stamp_ns) is int and stamp_ns > 0


def _validated_brush_command(values: Any) -> list[float] | None:
    """Validate the coordinator's exact ``[left, right, roller]`` contract."""
    if not isinstance(values, (list, tuple, array.array)) or len(values) != 3:
        return None
    if any(isinstance(value, bool) or not isinstance(value, numbers.Real) for value in values):
        return None
    command = [float(value) for value in values]
    return command if all(math.isfinite(value) for value in command) else None


def any_brush_rotating(values: Any) -> bool | None:
    """Safety/energy fact: either signed brush or roller has nonzero speed."""
    command = _validated_brush_command(values)
    return None if command is None else any(abs(value) > 1.0e-9 for value in command)


def full_width_coverage_active(values: Any) -> bool | None:
    """Coverage fact: both signed lateral brushes rotate, independent of roller.

    The formal coordinator publishes exactly ``[left, right, roller]``.
    Coverage is attributed only while *both* lateral brushes are nonzero;
    roller-only or one-sided operation is not silently promoted to a covered
    swath.
    """
    command = _validated_brush_command(values)
    if command is None:
        return None
    return abs(command[0]) > 1.0e-9 and abs(command[1]) > 1.0e-9


class JsonlMetricStream:
    """No-overwrite raw stream with a rolling content hash and bounded RAM."""

    def __init__(self, directory: Path, name: str):
        self.path = directory / f"{name}.jsonl"
        self._stream = self.path.open("x", encoding="utf-8", newline="\n")
        self._digest = hashlib.sha256()
        self.count = 0

    def append(self, sample: dict[str, Any]) -> None:
        line = json.dumps(sample, sort_keys=True, separators=(",", ":")) + "\n"
        self._stream.write(line)
        self._digest.update(line.encode("utf-8"))
        self.count += 1

    def close(self) -> dict[str, Any]:
        if not self._stream.closed:
            self._stream.flush()
            os.fsync(self._stream.fileno())
            self._stream.close()
        return {
            "path": str(self.path.resolve()), "sha256": self._digest.hexdigest(),
            "record_count": self.count, "format": "jsonl",
        }


def windowed_jsonl_metric_stream(
    source: Path, directory: Path, name: str, start_ns: int | None, end_ns: int | None,
) -> dict[str, Any]:
    """Stream a no-overwrite final ``[start,end)`` view by source header stamp."""
    if not positive_stamp_ns(start_ns) or not positive_stamp_ns(end_ns) or start_ns >= end_ns:
        raise CollectionError("cannot build final replay stream without a positive ROS task window")
    output = JsonlMetricStream(directory, f"{name}.windowed")
    excluded = 0
    invalid = 0
    try:
        with source.open("r", encoding="utf-8") as stream:
            for line in stream:
                try:
                    row = json.loads(line)
                    stamp_ns = row.get("stamp_ns") if isinstance(row, dict) else None
                except json.JSONDecodeError:
                    invalid += 1
                    continue
                if not positive_stamp_ns(stamp_ns):
                    invalid += 1
                elif start_ns <= stamp_ns < end_ns:
                    output.append(row)
                else:
                    excluded += 1
    finally:
        descriptor = output.close()
    return {**descriptor, "excluded_by_header_window": excluded, "invalid_input_rows": invalid}


def high_fidelity_brush_geometry() -> dict[str, Any]:
    """Derive side-brush centers from the frozen high-fidelity xacro source."""
    descriptor = file_descriptor(HIGH_FIDELITY_CLEANING_MECHANISM)
    text = HIGH_FIDELITY_CLEANING_MECHANISM.read_text(encoding="utf-8")
    match = re.search(
        r'<origin xyz="(?P<x>-?\d+(?:\.\d+)?) \$\{lateral_sign \* '
        r'(?P<y>\d+(?:\.\d+)?)\} 0\.0244"', text,
    )
    if match is None:
        raise CollectionError("high-fidelity side-brush geometry cannot be derived")
    x_m, y_m = float(match["x"]), float(match["y"])
    return {
        "source": descriptor,
        "centers_base_link_m": (
            {"brush": "left", "x_m": x_m, "y_m": y_m},
            {"brush": "right", "x_m": x_m, "y_m": -y_m},
        ),
    }


def canonical_localization_pairing(
    fused_odom_samples: list[dict[str, Any]],
    ground_truth_odom_samples: list[dict[str, Any]],
) -> dict[str, Any]:
    """Use the replay metric's one-to-one pairer after stable stamp ordering."""
    try:
        from sanitation_coverage.metrics import synchronized_xy_errors
    except ImportError as exc:  # pragma: no cover - available in the frozen ROS overlay
        raise CollectionError("canonical synchronized_xy_errors is unavailable") from exc
    fused = sorted(
        fused_odom_samples, key=lambda row: (row["stamp_ns"], row["arrival_sequence"])
    )
    truth = sorted(
        ground_truth_odom_samples, key=lambda row: (row["stamp_ns"], row["arrival_sequence"])
    )
    origin_ns = min(row["stamp_ns"] for row in [*fused, *truth])
    errors, sync_errors, dropped = synchronized_xy_errors(
        [((row["stamp_ns"] - origin_ns) / 1_000_000_000.0, row["x_m"], row["y_m"]) for row in fused],
        [((row["stamp_ns"] - origin_ns) / 1_000_000_000.0, row["x_m"], row["y_m"]) for row in truth],
        tolerance_sec=LOCALIZATION_PAIR_TOLERANCE_NS / 1_000_000_000.0,
    )
    paired = len(errors)
    return {
        "pairer": "sanitation_coverage.metrics.synchronized_xy_errors",
        "tolerance_ns": LOCALIZATION_PAIR_TOLERANCE_NS,
        "input_order": "callback_arrival_sequence",
        "offline_order": "stable_ascending_ros_stamp_ns_then_arrival_sequence",
        "normalization": "subtract_minimum_ros_stamp_ns_before_canonical_float_pairing",
        "paired_sample_count": paired,
        "dropped_estimate_count": dropped,
        "unpaired_truth_count": len(truth) - paired,
        "sync_error_sample_count": len(sync_errors),
    }


def _pairing_row(row: Any, source: Path) -> dict[str, Any]:
    """Validate the compact localization row before it reaches an external sort."""
    if not isinstance(row, dict):
        raise CollectionError(f"non-object localization row in {source}")
    stamp_ns = row.get("stamp_ns")
    arrival_sequence = row.get("arrival_sequence")
    if not positive_stamp_ns(stamp_ns) or type(arrival_sequence) is not int or arrival_sequence < 0:
        raise CollectionError(f"invalid localization stamp or arrival sequence in {source}")
    try:
        x_m, y_m = float(row["x_m"]), float(row["y_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise CollectionError(f"invalid localization coordinates in {source}") from exc
    if not math.isfinite(x_m) or not math.isfinite(y_m):
        raise CollectionError(f"non-finite localization coordinates in {source}")
    return {
        "stamp_ns": stamp_ns,
        "arrival_sequence": arrival_sequence,
        "x_m": x_m,
        "y_m": y_m,
    }


def _sorted_pairing_chunks(source: Path, directory: Path, name: str) -> dict[str, Any]:
    """Externally sort one stream without retaining a long mission in RAM."""
    chunks: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    count = 0

    def flush() -> None:
        if not rows:
            return
        stream = JsonlMetricStream(directory, f"{name}.pairing_sort.{len(chunks):06d}")
        for item in sorted(rows, key=lambda value: (value["stamp_ns"], value["arrival_sequence"])):
            stream.append(item)
        chunks.append(stream.close())
        rows.clear()

    with source.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise CollectionError(f"invalid localization JSONL in {source}") from exc
            rows.append(_pairing_row(row, source))
            count += 1
            if len(rows) >= PAIRING_SORT_CHUNK_ROWS:
                flush()
    flush()
    return {
        "record_count": count,
        "sort_chunk_row_limit": PAIRING_SORT_CHUNK_ROWS,
        "chunks": chunks,
    }


def _merged_pairing_rows(chunks: list[dict[str, Any]]):
    """Yield an externally sorted stream, keeping only one row per chunk."""
    with contextlib.ExitStack() as stack:
        streams = [
            stack.enter_context(Path(chunk["path"]).open("r", encoding="utf-8"))
            for chunk in chunks
        ]

        def rows(stream: Any):
            for line in stream:
                yield _pairing_row(json.loads(line), Path(stream.name))

        yield from heapq.merge(
            *(rows(stream) for stream in streams),
            key=lambda value: (value["stamp_ns"], value["arrival_sequence"]),
        )


def canonical_localization_pairing_from_streams(
    fused_path: Path, ground_truth_path: Path, directory: Path,
) -> dict[str, Any]:
    """Run the canonical 50 ms pairer over external-sort JSONL streams.

    The underlying metric routine is intentionally called for every estimate
    with its only possible unused predecessor/successor truth candidates.  For
    monotonically sorted inputs this is equivalent to its bisect-based
    one-to-one selection, while preserving bounded memory for a six-hour run.
    """
    try:
        from sanitation_coverage.metrics import synchronized_xy_errors
    except ImportError as exc:  # pragma: no cover - frozen ROS overlay supplies it
        raise CollectionError("canonical synchronized_xy_errors is unavailable") from exc

    fused_sort = _sorted_pairing_chunks(fused_path, directory, "fused_odom")
    truth_sort = _sorted_pairing_chunks(ground_truth_path, directory, "ground_truth_odom")
    if not fused_sort["record_count"] or not truth_sort["record_count"]:
        return {
            "pairer": "sanitation_coverage.metrics.synchronized_xy_errors",
            "tolerance_ns": LOCALIZATION_PAIR_TOLERANCE_NS,
            "input_streams_complete": False,
            "status": "FAILED_MISSING_LOCALIZATION_STREAM",
            "paired_sample_count": 0,
            "sorts": {"fused_odom": fused_sort, "ground_truth_odom": truth_sort},
        }

    truth_iterator = iter(_merged_pairing_rows(truth_sort["chunks"]))
    current_truth = next(truth_iterator, None)
    previous_truth: dict[str, Any] | None = None
    previous_used = False
    current_used = False
    paired = 0
    dropped_estimates = 0
    used_truth = 0
    invocations = 0
    previous_estimate_key: tuple[int, int] | None = None
    for estimate in _merged_pairing_rows(fused_sort["chunks"]):
        estimate_key = (estimate["stamp_ns"], estimate["arrival_sequence"])
        if previous_estimate_key is not None and estimate_key < previous_estimate_key:
            raise CollectionError("external localization sort is not monotonic")
        previous_estimate_key = estimate_key
        while current_truth is not None and current_truth["stamp_ns"] < estimate["stamp_ns"]:
            previous_truth, previous_used = current_truth, current_used
            current_truth, current_used = next(truth_iterator, None), False
        candidates = [
            row for row, used in ((previous_truth, previous_used), (current_truth, current_used))
            if row is not None and not used
        ]
        origin_ns = min(
            [estimate["stamp_ns"]] + [row["stamp_ns"] for row in candidates]
        )
        errors, _sync_errors, dropped = synchronized_xy_errors(
            [((estimate["stamp_ns"] - origin_ns) / 1_000_000_000.0, estimate["x_m"], estimate["y_m"])],
            [((row["stamp_ns"] - origin_ns) / 1_000_000_000.0, row["x_m"], row["y_m"]) for row in candidates],
            tolerance_sec=LOCALIZATION_PAIR_TOLERANCE_NS / 1_000_000_000.0,
        )
        invocations += 1
        if errors:
            chosen = min(candidates, key=lambda row: abs(row["stamp_ns"] - estimate["stamp_ns"]))
            if chosen is previous_truth:
                previous_used = True
            else:
                current_used = True
            paired += 1
            used_truth += 1
        else:
            dropped_estimates += dropped
    return {
        "pairer": "sanitation_coverage.metrics.synchronized_xy_errors",
        "tolerance_ns": LOCALIZATION_PAIR_TOLERANCE_NS,
        "input_order": "callback_arrival_sequence",
        "offline_order": "external_stable_ascending_ros_stamp_ns_then_arrival_sequence",
        "normalization": "subtract_local_minimum_ros_stamp_ns_before_canonical_float_pairing",
        "canonical_invocation_count": invocations,
        "paired_sample_count": paired,
        "dropped_estimate_count": dropped_estimates,
        "unpaired_truth_count": truth_sort["record_count"] - used_truth,
        "input_streams_complete": True,
        "status": "COMPLETE",
        "sorts": {"fused_odom": fused_sort, "ground_truth_odom": truth_sort},
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_descriptor(path: Path) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    if resolved.is_symlink() or not resolved.is_file():
        raise CollectionError(f"input is not a regular file: {path}")
    return {
        "kind": "file",
        "path": str(resolved),
        "size_bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
    }


def directory_descriptor(path: Path) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    if resolved.is_symlink() or not resolved.is_dir():
        raise CollectionError(f"input is not a regular directory: {path}")
    rows: list[dict[str, Any]] = []
    for item in sorted(
        resolved.rglob("*"), key=lambda value: value.relative_to(resolved).as_posix()
    ):
        if item.is_symlink():
            raise CollectionError(f"symlink prohibited in frozen input directory: {item}")
        if item.is_dir():
            continue
        if not item.is_file():
            raise CollectionError(f"non-regular entry in frozen input directory: {item}")
        rows.append({
            "relative_path": item.relative_to(resolved).as_posix(),
            "size_bytes": item.stat().st_size,
            "sha256": sha256_file(item),
        })
    if not rows:
        raise CollectionError(f"frozen input directory is empty: {path}")
    canonical = json.dumps(rows, ensure_ascii=False, separators=(",", ":"), sort_keys=True)
    return {
        "kind": "directory",
        "path": str(resolved),
        "file_count": len(rows),
        "total_bytes": sum(row["size_bytes"] for row in rows),
        "tree_sha256": hashlib.sha256(canonical.encode("utf-8")).hexdigest(),
        "files": rows,
    }


def build_input_binding(args: argparse.Namespace) -> dict[str, Any]:
    files = {
        "episode_manifest": args.episode_manifest,
        "evaluator_episode_manifest": args.evaluator_episode_manifest,
        "evaluator_ground_truth": args.evaluator_ground_truth,
        "world": args.world,
        "pedestrian_schedule": args.pedestrian_schedule,
        "session_status": args.session_status,
        "same_map_baseline": args.same_map_baseline,
        "policy_checkpoint": args.policy_checkpoint,
        "runtime_binding": args.runtime_binding,
    }
    directories = {
        "saved_map": args.saved_map,
        "perception_artifacts": args.perception_artifacts,
    }
    return {
        "schema_version": 1,
        "artifact_kind": "single_episode_immutable_input_binding",
        "artifacts": {
            **{name: file_descriptor(Path(path)) for name, path in files.items()},
            **{name: directory_descriptor(Path(path)) for name, path in directories.items()},
        },
    }


def verify_input_binding(binding: dict[str, Any]) -> None:
    if binding.get("artifact_kind") != "single_episode_immutable_input_binding":
        raise CollectionError("invalid immutable input binding")
    artifacts = binding.get("artifacts")
    if not isinstance(artifacts, dict) or not artifacts:
        raise CollectionError("immutable input artifact ledger missing")
    for name, expected in artifacts.items():
        if not isinstance(expected, dict):
            raise CollectionError(f"invalid input descriptor: {name}")
        path = Path(str(expected.get("path", "")))
        if expected.get("kind") == "file":
            actual = file_descriptor(path)
        elif expected.get("kind") == "directory":
            actual = directory_descriptor(path)
        else:
            raise CollectionError(f"invalid input descriptor kind: {name}")
        if actual != expected:
            raise CollectionError(f"input changed after pre-launch freeze: {name}")


def parse_diagnostic(message: Any, *, expected_name: str) -> dict[str, Any] | None:
    for row in message.status:
        if row.name == expected_name:
            values: dict[str, Any] = {item.key: item.value for item in row.values}
            values.update({
                "diagnostic_name": row.name,
                "hardware_id": row.hardware_id,
                "level": int(row.level),
                "state": row.message,
            })
            return values
    return None


def source_row(
    metric: str, topic: str, source_class: str,
    identity: dict[str, Any], count: int,
) -> dict[str, Any]:
    return {
        "metric": metric,
        "topic": topic,
        "source_class": source_class,
        **{key: identity[key] for key in (
            "session_id", "episode_id", "episode_seed", "gazebo_process_id",
            "runtime_id", "ros_domain_id", "gz_partition",
        )},
        "sample_count": count,
    }


def _parameter_value(value: Any) -> Any:
    return {
        1: value.bool_value,
        2: value.integer_value,
        3: value.double_value,
        4: value.string_value,
        5: list(value.byte_array_value),
        6: list(value.bool_array_value),
        7: list(value.integer_array_value),
        8: list(value.double_array_value),
        9: list(value.string_array_value),
    }.get(int(value.type))


def _full_node_name(name: str, namespace: str) -> str:
    namespace = namespace.rstrip("/")
    return f"{namespace}/{name}" if namespace else f"/{name}"


def truth_subscriber_names(endpoints: Any) -> list[str]:
    # Duplicate node names are not a second trusted endpoint. Preserve their
    # multiplicity so the exact allowed-subscription comparison fails closed.
    return sorted(_full_node_name(info.node_name, info.node_namespace) for info in endpoints)


def _add_input_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--episode-manifest", required=True, type=Path)
    parser.add_argument("--evaluator-episode-manifest", required=True, type=Path)
    parser.add_argument("--evaluator-ground-truth", required=True, type=Path)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--pedestrian-schedule", required=True, type=Path)
    parser.add_argument("--session-status", required=True, type=Path)
    parser.add_argument("--same-map-baseline", required=True, type=Path)
    parser.add_argument("--policy-checkpoint", required=True, type=Path)
    parser.add_argument("--runtime-binding", required=True, type=Path)
    parser.add_argument("--saved-map", required=True, type=Path)
    parser.add_argument("--perception-artifacts", required=True, type=Path)


def validate_trusted_gt_recorder_args(node: str | None, pid: int | None, pgid: int | None) -> None:
    """Allow only the supervisor-owned fixed recorder, or no recorder."""
    values = (node, pid, pgid)
    if any(value is not None for value in values) and not all(value is not None for value in values):
        raise SystemExit("trusted GT recorder node, PID, and PGID are required together")
    if node is None:
        return
    if node != TRUSTED_GT_RECORDER_NODE or pid <= 1 or pgid <= 1:
        raise SystemExit("trusted GT recorder identity is invalid or not the fixed recorder node")
    try:
        observed_pgid = os.getpgid(pid)
    except (AttributeError, OSError) as exc:
        raise SystemExit("trusted GT recorder PID is not live in this POSIX runtime") from exc
    if observed_pgid != pgid:
        raise SystemExit("trusted GT recorder PID does not belong to declared PGID")


def main() -> int:
    if "--prepare-input-binding" in __import__("sys").argv:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--prepare-input-binding", required=True, type=Path)
        _add_input_arguments(parser)
        args = parser.parse_args()
        result = build_input_binding(args)
        args.prepare_input_binding.parent.mkdir(parents=True, exist_ok=True)
        args.prepare_input_binding.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return 0

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--episode-id", required=True)
    parser.add_argument("--episode-seed", required=True, type=int)
    parser.add_argument("--runtime-id", required=True)
    parser.add_argument("--gazebo-process-id", required=True, type=int)
    parser.add_argument("--session-start-epoch-ns", required=True, type=int)
    parser.add_argument("--input-binding", required=True, type=Path)
    parser.add_argument("--ready-file", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=21600.0)
    parser.add_argument(
        "--require-replay-metric-capture", action="store_true",
        help="fail closed unless current-episode coverage and localization samples are retained",
    )
    parser.add_argument("--replay-metric-stream-dir", required=True, type=Path)
    parser.add_argument(
        "--trusted-gt-recorder-node",
        help="capture producer's evaluation-only second /ground_truth/odom subscriber",
    )
    parser.add_argument("--trusted-gt-recorder-pid", type=int)
    parser.add_argument("--trusted-gt-recorder-pgid", type=int)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--multisite-topic-observations", type=Path,
        help="optional canonical read-only product-interface observation record",
    )
    _add_input_arguments(parser)
    args = parser.parse_args()

    validate_trusted_gt_recorder_args(
        args.trusted_gt_recorder_node,
        args.trusted_gt_recorder_pid,
        args.trusted_gt_recorder_pgid,
    )

    binding = json.loads(args.input_binding.read_text(encoding="utf-8"))
    if not isinstance(binding, dict):
        raise SystemExit("input binding root must be an object")
    verify_input_binding(binding)
    if args.replay_metric_stream_dir.exists():
        raise SystemExit("refusing to overwrite replay metric stream directory")
    args.replay_metric_stream_dir.mkdir(parents=True, exist_ok=False)
    saved_map_descriptor = binding.get("artifacts", {}).get("saved_map")
    if not isinstance(saved_map_descriptor, dict):
        raise SystemExit("input binding has no saved-map descriptor")
    mission_geometry = file_descriptor(args.saved_map / "mission_geometry.yaml")
    bound_geometry = next(
        (
            row for row in saved_map_descriptor.get("files", [])
            if isinstance(row, dict) and row.get("relative_path") == "mission_geometry.yaml"
        ),
        None,
    )
    if (
        not isinstance(bound_geometry, dict)
        or bound_geometry.get("sha256") != mission_geometry["sha256"]
    ):
        raise SystemExit("mission geometry is absent from or differs from the frozen saved-map binding")
    brush_geometry = high_fidelity_brush_geometry()
    truth = json.loads(args.evaluator_ground_truth.read_text(encoding="utf-8"))
    cubes = truth.get("discrete_cubes") if isinstance(truth, dict) else None
    if not isinstance(cubes, list) or not cubes:
        raise SystemExit("evaluator truth has no discrete cube identity ledger")
    expected_cube_ids = {
        str(row.get("object_id", "")) for row in cubes if isinstance(row, dict)
    }
    if len(expected_cube_ids) != len(cubes) or "" in expected_cube_ids:
        raise SystemExit("evaluator truth cube IDs are missing or duplicated")

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import Odometry, Path as NavPath
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
    )
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, Float64MultiArray, String
    from vision_msgs.msg import Detection2DArray

    identity = {
        "session_id": args.session_id,
        "episode_id": args.episode_id,
        "episode_seed": args.episode_seed,
        "runtime_id": args.runtime_id,
        "gazebo_process_id": args.gazebo_process_id,
        "session_start_epoch_ns": args.session_start_epoch_ns,
        "ros_domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")),
        "gz_partition": os.environ.get("GZ_PARTITION", ""),
    }
    if not identity["gz_partition"]:
        raise SystemExit("GZ_PARTITION is required for a uniquely bound live episode")

    class Collector(Node):
        def __init__(self) -> None:
            super().__init__("formal_single_episode_cleaning_collector")
            latched = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.started = time.monotonic()
            self.done = False
            self.complete = False
            self.planner: dict[str, Any] = {}
            self.first: dict[str, dict[str, Any]] = {}
            self.latest: dict[str, dict[str, Any]] = {}
            self.counts = {key: 0 for key in (*PRODUCT_TOPICS, *EVALUATOR_TOPICS)}
            self.path_count = 0
            self.trajectory_evidence: list[dict[str, Any]] = []
            self.planner_status_samples: list[dict[str, Any]] = []
            self.grasp_results: list[dict[str, Any]] = []
            self.collision_count = 0
            self.intervention_count = 0
            self.return_started_seen = False
            self.return_start_state: dict[str, Any] | None = None
            self.odom_points: list[tuple[float, float]] = []
            self.replay_metric_stats = {
                name: {
                    "received_total": 0, "window_received": 0, "written": 0,
                    "invalid_stamp": 0, "invalid_sample": 0,
                    "out_of_order_stamp": 0, "outside_window_header_stamp": 0,
                }
                for name in REPLAY_METRIC_TOPICS
            }
            self.replay_streams = {
                name: JsonlMetricStream(args.replay_metric_stream_dir, name)
                for name in REPLAY_METRIC_TOPICS
            }
            self.brush_on_stream = JsonlMetricStream(
                args.replay_metric_stream_dir, "brush_on_ground_truth"
            )
            self._last_stamp_by_metric: dict[str, int] = {}
            self._arrival_sequence = 0
            self.brush_active = False
            self.operator_start_received = False
            self.capture_window = {
                "operator_start_ros_time_ns": None,
                "operator_start_received_epoch_ns": None,
                "task_complete_ros_time_ns": None,
                "task_complete_received_epoch_ns": None,
                "raw_sample_window": "operator_start_callback_inclusive_to_first_true_mission_complete_callback_exclusive",
            }
            self.mission_start_odom_index: int | None = None
            self.return_start_odom_index: int | None = None
            self.runtime_parameters: dict[str, dict[str, Any]] = {}
            self.parameter_futures: dict[str, Any] = {}
            self.parameter_clients: dict[str, Any] = {}
            self.runtime_graph: dict[str, Any] = {}
            self.ready_written = False
            self.multisite_counts = {role: 0 for role in MULTISITE_INTERFACES}
            self.multisite_nav2_goal_succeeded = False
            self.create_subscription(
                DiagnosticArray, PRODUCT_TOPICS["planner"], self.on_planner, latched
            )
            self.create_subscription(
                Bool, PRODUCT_TOPICS["mission_complete"], self.on_complete, latched
            )
            self.create_subscription(NavPath, PRODUCT_TOPICS["trajectory"], self.on_path, 20)
            self.create_subscription(String, PRODUCT_TOPICS["grasp_result"], self.on_grasp, 50)
            self.create_subscription(Odometry, PRODUCT_TOPICS["odometry"], self.on_odom, 50)
            metric_qos = QoSProfile(
                depth=20, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                Float64MultiArray, REPLAY_METRIC_TOPICS["post_safety_brush_command"]["name"],
                self.on_brush_command, metric_qos,
            )
            self.create_subscription(
                Odometry, REPLAY_METRIC_TOPICS["fused_odom"]["name"],
                self.on_fused_odom, metric_qos,
            )
            self.create_subscription(
                Odometry, REPLAY_METRIC_TOPICS["ground_truth_odom"]["name"],
                self.on_ground_truth_odom, metric_qos,
            )
            self.create_subscription(
                Bool, "/product_demo/operator_start", self.on_operator_start, 10
            )
            for key in ("ground_dirt", "water", "dry_bin", "pedestrians"):
                self.create_subscription(
                    String, EVALUATOR_TOPICS[key],
                    lambda msg, k=key: self.on_json(k, msg), 20,
                )
            self.create_subscription(
                CollisionMonitorState, EVALUATOR_TOPICS["collision"], self.on_collision, 20
            )
            self.create_subscription(
                Contacts, EVALUATOR_TOPICS["front_bumper"],
                lambda msg: self.on_contact("front_bumper", msg), qos_profile_sensor_data,
            )
            self.create_subscription(
                Contacts, EVALUATOR_TOPICS["rear_bumper"],
                lambda msg: self.on_contact("rear_bumper", msg), qos_profile_sensor_data,
            )
            if args.multisite_topic_observations is not None:
                self.create_subscription(Detection2DArray, MULTISITE_INTERFACES["dosod"]["observed_topic"], lambda msg: self.on_multisite("dosod"), 20)
                self.create_subscription(Image, MULTISITE_INTERFACES["edgesam"]["observed_topic"], lambda msg: self.on_multisite("edgesam"), 20)
                self.create_subscription(GoalStatusArray, MULTISITE_INTERFACES["nav2"]["observed_topic"], self.on_nav2_status, 20)
                self.create_subscription(String, MULTISITE_INTERFACES["dynamic_pedestrians"]["observed_topic"], lambda msg: self.on_multisite("dynamic_pedestrians"), 20)
                self.create_subscription(Float64MultiArray, MULTISITE_INTERFACES["cleaning_actuator"]["observed_topic"], lambda msg: self.on_multisite("cleaning_actuator"), 20)
            for node_name, names in RUNTIME_PARAMETER_CONTRACT.items():
                client = self.create_client(GetParameters, f"{node_name}/get_parameters")
                self.parameter_clients[node_name] = (client, names)
            self.create_timer(0.2, self.tick)
            self.create_timer(1.0, self.audit_runtime)

        def on_planner(self, msg: Any) -> None:
            self.counts["planner"] += 1
            row = parse_diagnostic(
                msg, expected_name="formal_active_cleaning_policy_planner"
            )
            if row is not None:
                self.planner = row
                self.planner_status_samples.append(
                    {**row, "collector_received_epoch_ns": time.time_ns()}
                )
                if row.get("returning_home") == "true" and not self.return_started_seen:
                    self.return_started_seen = True
                    self.return_start_odom_index = max(0, len(self.odom_points) - 1)
                    self.return_start_state = {
                        "planner_status": dict(row),
                        "evaluator": {
                            key: dict(self.latest.get(key, {}))
                            for key in ("ground_dirt", "water", "dry_bin")
                        },
                        "successful_grasp_target_ids": sorted({
                            str(item.get("target_id")) for item in self.grasp_results
                            if item.get("verified_in_bin") is True
                        }),
                    }

        def on_complete(self, msg: Any) -> None:
            self.counts["mission_complete"] += 1
            if bool(msg.data) and not self.complete:
                self.capture_window["task_complete_ros_time_ns"] = self.get_clock().now().nanoseconds
                self.capture_window["task_complete_received_epoch_ns"] = time.time_ns()
            self.complete = self.complete or bool(msg.data)

        def on_path(self, msg: Any) -> None:
            self.counts["trajectory"] += 1
            self.path_count += 1
            points = [
                [float(pose.pose.position.x), float(pose.pose.position.y)]
                for pose in msg.poses
            ]
            self.trajectory_evidence.append(
                {
                    "collector_received_epoch_ns": time.time_ns(),
                    "frame_id": str(msg.header.frame_id),
                    "pose_count": len(points),
                    "trajectory_xy_m": points,
                }
            )

        def on_operator_start(self, msg: Any) -> None:
            if bool(msg.data) and not self.operator_start_received:
                self.operator_start_received = True
                self.mission_start_odom_index = max(0, len(self.odom_points) - 1)
                self.capture_window["operator_start_ros_time_ns"] = self.get_clock().now().nanoseconds
                self.capture_window["operator_start_received_epoch_ns"] = time.time_ns()

        def on_grasp(self, msg: Any) -> None:
            self.counts["grasp_result"] += 1
            try:
                row = json.loads(msg.data)
                if isinstance(row, dict):
                    row["collector_received_epoch_ns"] = time.time_ns()
                    self.grasp_results.append(row)
            except (json.JSONDecodeError, TypeError):
                pass

        def on_odom(self, msg: Any) -> None:
            self.counts["odometry"] += 1
            self.odom_points.append(
                (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
            )

        @staticmethod
        def _stamp_ns(message: Any) -> int:
            stamp = message.header.stamp
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

        def _record_metric(self, name: str, sample: dict[str, Any]) -> bool:
            stats = self.replay_metric_stats[name]
            stats["received_total"] += 1
            if not self.operator_start_received or self.complete:
                return False
            stats["window_received"] += 1
            stamp_ns = sample.get("stamp_ns")
            if not positive_stamp_ns(stamp_ns):
                stats["invalid_stamp"] += 1
                return False
            # The ROS task window is authoritative.  Callbacks can arrive out
            # of order, so admission is based on the source header stamp, not
            # callback arrival time.  The terminal upper bound is additionally
            # recorded for the downstream streaming source-metrics filter.
            start_ns = self.capture_window["operator_start_ros_time_ns"]
            if stamp_ns < start_ns:
                stats["outside_window_header_stamp"] += 1
                return False
            if not all(math.isfinite(float(sample[key])) for key in ("x_m", "y_m")):
                stats["invalid_sample"] += 1
                return False
            previous = self._last_stamp_by_metric.get(name)
            if previous is not None and stamp_ns < previous:
                stats["out_of_order_stamp"] += 1
            self._last_stamp_by_metric[name] = stamp_ns
            self._arrival_sequence += 1
            sample["arrival_sequence"] = self._arrival_sequence
            self.replay_streams[name].append(sample)
            stats["written"] += 1
            return True

        @staticmethod
        def _yaw_from_quaternion(orientation: Any) -> float | None:
            values = (orientation.x, orientation.y, orientation.z, orientation.w)
            if not all(math.isfinite(float(value)) for value in values):
                return None
            norm = sum(float(value) * float(value) for value in values)
            if norm <= 1.0e-12:
                return None
            x, y, z, w = (float(value) for value in values)
            return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        def on_brush_command(self, msg: Any) -> None:
            stats = self.replay_metric_stats["post_safety_brush_command"]
            stats["received_total"] += 1
            if not self.operator_start_received or self.complete:
                return
            stats["window_received"] += 1
            active = full_width_coverage_active(msg.data)
            any_rotating = any_brush_rotating(msg.data)
            stamp_ns = self.get_clock().now().nanoseconds
            if active is None or any_rotating is None or not positive_stamp_ns(stamp_ns):
                stats["invalid_sample"] += 1
                return
            self.brush_active = active
            self._arrival_sequence += 1
            self.replay_streams["post_safety_brush_command"].append({
                "stamp_ns": stamp_ns, "arrival_sequence": self._arrival_sequence,
                "active": active, "command_rad_s": [float(value) for value in msg.data],
                "any_brush_rotating": any_rotating,
                "full_width_coverage_active": active,
                "association": "post_safety_controller_receipt_to_brush_on_ground_truth_pose",
            })
            stats["written"] += 1

        def on_fused_odom(self, msg: Any) -> None:
            pose = msg.pose.pose.position
            self._record_metric("fused_odom", {
                "stamp_ns": self._stamp_ns(msg), "x_m": float(pose.x), "y_m": float(pose.y),
            })

        def on_ground_truth_odom(self, msg: Any) -> None:
            pose = msg.pose.pose.position
            yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
            sample = {
                "stamp_ns": self._stamp_ns(msg), "x_m": float(pose.x), "y_m": float(pose.y),
                "yaw_rad": yaw,
            }
            if yaw is None:
                self.replay_metric_stats["ground_truth_odom"]["received_total"] += 1
                if self.operator_start_received and not self.complete:
                    self.replay_metric_stats["ground_truth_odom"]["window_received"] += 1
                    self.replay_metric_stats["ground_truth_odom"]["invalid_sample"] += 1
                return
            written = self._record_metric("ground_truth_odom", sample)
            if self.brush_active and written:
                self.brush_on_stream.append(dict(sample))

        def on_json(self, key: str, msg: Any) -> None:
            self.counts[key] += 1
            try:
                row = json.loads(msg.data)
                if isinstance(row, dict):
                    self.first.setdefault(key, dict(row))
                    self.latest[key] = row
                    if key == "pedestrians":
                        self.collision_count = max(
                            self.collision_count, int(row.get("collision_count", 0))
                        )
            except (json.JSONDecodeError, TypeError, ValueError):
                pass

        def on_collision(self, msg: Any) -> None:
            self.counts["collision"] += 1
            if int(msg.action_type) != 0:
                self.intervention_count += 1

        def on_contact(self, key: str, msg: Any) -> None:
            self.counts[key] += 1
            if msg.contacts:
                self.collision_count += 1

        def on_multisite(self, role: str) -> None:
            self.multisite_counts[role] += 1

        def on_nav2_status(self, msg: Any) -> None:
            self.on_multisite("nav2")
            self.multisite_nav2_goal_succeeded = self.multisite_nav2_goal_succeeded or any(
                int(row.status) == GoalStatus.STATUS_SUCCEEDED for row in msg.status_list
            )

        def _parameter_done(
            self, node_name: str, names: tuple[str, ...], future: Any
        ) -> None:
            try:
                response = future.result()
                if len(response.values) != len(names):
                    return
                self.runtime_parameters[node_name] = {
                    name: _parameter_value(value)
                    for name, value in zip(names, response.values)
                }
            except Exception:
                return

        def audit_runtime(self) -> None:
            for node_name, (client, names) in self.parameter_clients.items():
                if node_name in self.runtime_parameters or node_name in self.parameter_futures:
                    continue
                if client.service_is_ready():
                    request = GetParameters.Request(names=list(names))
                    future = client.call_async(request)
                    self.parameter_futures[node_name] = future
                    future.add_done_callback(
                        lambda done, n=node_name, p=names:
                        self._parameter_done(n, p, done)
                    )
            nodes = sorted({
                _full_node_name(name, namespace)
                for name, namespace in self.get_node_names_and_namespaces()
            })
            truth_topics = CONTROL_PROHIBITED_TRUTH_TOPICS + (RAW_GROUND_TRUTH_TOPIC,) + (
                (REPLAY_METRIC_TOPICS["ground_truth_odom"]["name"],)
                if args.require_replay_metric_capture else ()
            )
            subscribers: dict[str, list[str]] = {}
            for topic in truth_topics:
                subscribers[topic] = truth_subscriber_names(
                    self.get_subscriptions_info_by_topic(topic)
                )
            self.runtime_graph = {
                "observed_epoch_ns": time.time_ns(),
                "nodes": nodes,
                "required_nodes": sorted(REQUIRED_RUNTIME_NODES),
                "required_nodes_present": REQUIRED_RUNTIME_NODES.issubset(nodes),
                "truth_subscription_audit_enabled": args.require_replay_metric_capture,
                "control_prohibited_truth_topic_subscribers": subscribers,
                "ground_truth_odom_allowed_subscribers": self._allowed_gt_subscribers(),
                "ground_truth_odom_trusted_recorder": self._trusted_gt_recorder_identity(),
                "ground_truth_model_odom_raw_allowed_subscribers": [RAW_GROUND_TRUTH_ADAPTER_NODE],
            }

        def _allowed_gt_subscribers(self) -> list[str]:
            allowed = ["/formal_single_episode_cleaning_collector"]
            if args.trusted_gt_recorder_node is not None:
                allowed.append(args.trusted_gt_recorder_node)
            return sorted(allowed)

        def _trusted_gt_recorder_identity(self) -> dict[str, Any] | None:
            if args.trusted_gt_recorder_node is None:
                return None
            live_pgid: int | None = None
            try:
                live_pgid = os.getpgid(args.trusted_gt_recorder_pid)
            except (AttributeError, OSError):
                pass
            return {
                "node": args.trusted_gt_recorder_node,
                "pid": args.trusted_gt_recorder_pid,
                "pgid": args.trusted_gt_recorder_pgid,
                "pid_pgid_match": live_pgid == args.trusted_gt_recorder_pgid,
            }

        def multisite_observations(self) -> dict[str, Any]:
            action_servers = {
                name: sorted(types)
                for name, types in self.get_action_server_names_and_types()
            }
            interfaces: dict[str, dict[str, Any]] = {}
            for role, contract in MULTISITE_INTERFACES.items():
                topic = contract["observed_topic"]
                publishers = sorted({
                    _full_node_name(info.node_name, info.node_namespace)
                    for info in self.get_publishers_info_by_topic(topic)
                })
                action_ready = (
                    role != "nav2"
                    or action_servers.get(contract["name"]) == [contract["type"]]
                )
                interfaces[role] = {
                    "name": contract["name"], "type": contract["type"],
                    "interface_kind": contract["interface_kind"],
                    "live_observed": self.multisite_counts[role] > 0 and bool(publishers) and action_ready,
                    "message_count": self.multisite_counts[role],
                    "publisher_nodes": publishers,
                    "goal_succeeded": self.multisite_nav2_goal_succeeded if role == "nav2" else None,
                }
            return {
                "schema_version": 1,
                "artifact_kind": "formal_multisite_live_product_interface_observations",
                "collected_epoch_ns": time.time_ns(),
                "interfaces": interfaces,
            }

        def is_ready(self) -> bool:
            initial_ready = all(
                key in self.first for key in ("ground_dirt", "water", "dry_bin", "pedestrians")
            )
            parameters_ready = set(self.runtime_parameters) == set(RUNTIME_PARAMETER_CONTRACT)
            graph_ready = self.runtime_graph.get("required_nodes_present") is True
            subscribers = self.runtime_graph.get(
                "control_prohibited_truth_topic_subscribers", {}
            )
            # At readiness a capture sidecar may not have subscribed yet;
            # allow only the collector or its pre-authorized exact second
            # subscriber.  Terminal report tightens this to the configured
            # set and rechecks PID/PGID ownership.
            allowed_gt = self._allowed_gt_subscribers()
            truth_boundary_ready = (
                len(subscribers) == len(CONTROL_PROHIBITED_TRUTH_TOPICS) + int(
                    args.require_replay_metric_capture
                ) + 1
                and all(
                    rows == ["/formal_single_episode_cleaning_collector"]
                    for topic, rows in subscribers.items()
                    if topic not in {REPLAY_METRIC_TOPICS["ground_truth_odom"]["name"], RAW_GROUND_TRUTH_TOPIC}
                )
                and subscribers.get(RAW_GROUND_TRUTH_TOPIC, []) == [RAW_GROUND_TRUTH_ADAPTER_NODE]
                and (
                    not args.require_replay_metric_capture
                    or subscribers.get(REPLAY_METRIC_TOPICS["ground_truth_odom"]["name"], [])
                    in (["/formal_single_episode_cleaning_collector"], allowed_gt)
                )
            )
            return (
                initial_ready and parameters_ready and graph_ready
                and truth_boundary_ready and bool(self.planner) and bool(self.odom_points)
            )

        def tick(self) -> None:
            if self.is_ready() and not self.ready_written:
                ready_payload = json.dumps({
                    "schema_version": 1,
                    "artifact_kind": "single_episode_collector_ready",
                    "run_identity": identity,
                    "created_epoch_ns": time.time_ns(),
                }, indent=2, sort_keys=True) + "\n"
                temporary = args.ready_file.with_suffix(args.ready_file.suffix + ".tmp")
                temporary.write_text(ready_payload, encoding="utf-8")
                temporary.replace(args.ready_file)
                self.ready_written = True
            if self.complete and self.planner.get("state") == "COMPLETE":
                self.done = True
            elif time.monotonic() - self.started >= args.timeout:
                self.done = True

        def report(self) -> dict[str, Any]:
            verify_input_binding(binding)
            # Re-sample the graph at the terminal boundary; readiness alone
            # must not hide a product node that exited or subscribed later.
            self.audit_runtime()
            if args.multisite_topic_observations is not None:
                observations = self.multisite_observations()
                args.multisite_topic_observations.parent.mkdir(parents=True, exist_ok=True)
                if args.multisite_topic_observations.exists():
                    raise CollectionError("refusing to overwrite multisite topic observations")
                args.multisite_topic_observations.write_text(
                    json.dumps(observations, indent=2, sort_keys=True) + "\n", encoding="utf-8"
                )
            sources = [
                source_row(k, v, "product", identity, self.counts[k])
                for k, v in PRODUCT_TOPICS.items()
            ]
            sources += [
                source_row(k, v, "evaluator_truth", identity, self.counts[k])
                for k, v in EVALUATOR_TOPICS.items()
            ]
            successful = sorted({
                str(row.get("target_id")) for row in self.grasp_results
                if row.get("verified_in_bin") is True and row.get("target_id")
            })
            mission_start = self.mission_start_odom_index
            return_start = self.return_start_odom_index
            stream_descriptors = {
                name: stream.close() for name, stream in self.replay_streams.items()
            }
            brush_on_descriptor = self.brush_on_stream.close()
            start_ns = self.capture_window["operator_start_ros_time_ns"]
            end_ns = self.capture_window["task_complete_ros_time_ns"]
            windowed_stream_descriptors: dict[str, dict[str, Any]] = {}
            windowed_brush_on_descriptor: dict[str, Any] | None = None
            if positive_stamp_ns(start_ns) and positive_stamp_ns(end_ns) and start_ns < end_ns:
                windowed_stream_descriptors = {
                    name: windowed_jsonl_metric_stream(
                        Path(descriptor["path"]), args.replay_metric_stream_dir,
                        name, start_ns, end_ns,
                    )
                    for name, descriptor in stream_descriptors.items()
                }
                windowed_brush_on_descriptor = windowed_jsonl_metric_stream(
                    Path(brush_on_descriptor["path"]), args.replay_metric_stream_dir,
                    "brush_on_ground_truth", start_ns, end_ns,
                )
            pairing_inputs = {
                "fused_odom": windowed_stream_descriptors.get("fused_odom"),
                "ground_truth_odom": windowed_stream_descriptors.get("ground_truth_odom"),
            }
            if all(
                descriptor is not None and descriptor.get("record_count", 0) > 0
                for descriptor in pairing_inputs.values()
            ):
                try:
                    localization_pairing = canonical_localization_pairing_from_streams(
                        Path(pairing_inputs["fused_odom"]["path"]),
                        Path(pairing_inputs["ground_truth_odom"]["path"]),
                        args.replay_metric_stream_dir,
                    )
                except CollectionError as exc:
                    localization_pairing = {
                        "pairer": "sanitation_coverage.metrics.synchronized_xy_errors",
                        "tolerance_ns": LOCALIZATION_PAIR_TOLERANCE_NS,
                        "input_streams_complete": True,
                        "status": "FAILED_PAIRING_INPUT",
                        "failure": str(exc),
                        "paired_sample_count": 0,
                    }
            else:
                localization_pairing = {
                    "pairer": "sanitation_coverage.metrics.synchronized_xy_errors",
                    "tolerance_ns": LOCALIZATION_PAIR_TOLERANCE_NS,
                    "input_streams_complete": False,
                    "status": "FAILED_MISSING_LOCALIZATION_STREAM",
                    "paired_sample_count": 0,
                }
            localization_pairing["input_streams"] = pairing_inputs
            localization_pairing["header_stamp_window_filter"] = {
                "start_inclusive_ros_time_ns": self.capture_window["operator_start_ros_time_ns"],
                "end_exclusive_ros_time_ns": self.capture_window["task_complete_ros_time_ns"],
            }
            gt_topic = REPLAY_METRIC_TOPICS["ground_truth_odom"]["name"]
            terminal_gt_subscribers = self.runtime_graph.get(
                "control_prohibited_truth_topic_subscribers", {}
            ).get(gt_topic, [])
            terminal_raw_gt_subscribers = self.runtime_graph.get(
                "control_prohibited_truth_topic_subscribers", {}
            ).get(RAW_GROUND_TRUTH_TOPIC, [])
            trusted_identity = self._trusted_gt_recorder_identity()
            terminal_gt_boundary_ok = (
                terminal_gt_subscribers == self._allowed_gt_subscribers()
                and (
                    trusted_identity is None
                    or trusted_identity["pid_pgid_match"] is True
                )
            )
            terminal_raw_gt_boundary_ok = terminal_raw_gt_subscribers == [RAW_GROUND_TRUTH_ADAPTER_NODE]
            replay_capture_complete = replay_metric_capture_complete(
                self.replay_metric_stats,
                (windowed_brush_on_descriptor or {}).get("record_count", 0),
                localization_pairing,
                self.capture_window,
            ) and terminal_gt_boundary_ok and terminal_raw_gt_boundary_ok
            task_points = (
                self.odom_points[mission_start : return_start + 1]
                if mission_start is not None and return_start is not None
                else []
            )
            return_points = (
                self.odom_points[return_start:]
                if return_start is not None
                else []
            )
            return {
                "schema_version": 2,
                "artifact_kind": "single_live_episode_raw_collection",
                "created_epoch_ns": time.time_ns(),
                "run_identity": identity,
                "input_binding": {
                    "path": str(args.input_binding.resolve()),
                    "sha256": sha256_file(args.input_binding),
                    "artifacts": binding["artifacts"],
                },
                "metric_sources": sources,
                "runtime_graph": self.runtime_graph,
                "runtime_parameters": self.runtime_parameters,
                "product": {
                    "planner_status": self.planner,
                    "mission_complete": self.complete,
                    "trajectory_publish_count": self.path_count,
                    "trajectory_evidence": self.trajectory_evidence,
                    "planner_status_samples": self.planner_status_samples,
                    "grasp_results": self.grasp_results,
                    "successful_grasp_target_ids": successful,
                    "odom_sample_count": len(self.odom_points),
                    "operator_start_received": self.operator_start_received,
                    "task_odom_trajectory_xy_m": [list(point) for point in task_points],
                    "return_odom_trajectory_xy_m": [list(point) for point in return_points],
                    "return_started_seen": self.return_started_seen,
                    "return_start_state": self.return_start_state,
                },
                "evaluator": {
                    "initial": self.first,
                    "terminal": self.latest,
                    "collision_count": self.collision_count,
                    "collision_monitor_intervention_count": self.intervention_count,
                },
                "collector_ready_before_operator_start": self.ready_written,
                "replay_metric_capture": {
                    "schema_version": 2,
                    "evaluation_only_truth": True,
                    "capture_window": self.capture_window,
                    "storage": "no_overwrite_jsonl_streams_with_rolling_sha256",
                    "complete": replay_capture_complete,
                    "terminal_ground_truth_subscription_boundary_ok": terminal_gt_boundary_ok,
                    "terminal_raw_ground_truth_subscription_boundary_ok": terminal_raw_gt_boundary_ok,
                    "topics": {
                        name: {**contract, **self.replay_metric_stats[name]}
                        for name, contract in REPLAY_METRIC_TOPICS.items()
                    },
                    "streams": stream_descriptors,
                    "windowed_streams": windowed_stream_descriptors,
                    "brush_on_ground_truth_stream": brush_on_descriptor,
                    "windowed_brush_on_ground_truth_stream": windowed_brush_on_descriptor,
                    "brush_geometry": brush_geometry,
                    "mission_geometry": mission_geometry,
                    "localization_pairing": localization_pairing,
                },
                "timed_out": not self.complete,
            }

    args.ready_file.parent.mkdir(parents=True, exist_ok=True)
    if args.ready_file.exists():
        raise SystemExit(f"refusing to reuse collector readiness evidence: {args.ready_file}")
    rclpy.init()
    node = Collector()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    publish_raw_report(args.output, report)
    return 0 if (
        report["product"]["mission_complete"]
        and report["collector_ready_before_operator_start"]
        and (not args.require_replay_metric_capture
             or report["replay_metric_capture"]["complete"])
    ) else 3


if __name__ == "__main__":
    raise SystemExit(main())
