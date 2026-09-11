#!/usr/bin/env python3
"""Seal a read-only localization rosbag2 diagnostic from first-map creation."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml


REQUIRED_TOPICS = (
    "/odom",
    "/odom/unfiltered",
    "/odometry/gps",
    "/gnss/fix",
    "/formal_mapping/lifecycle_status",
)
OPTIONAL_TOPICS = {"/ground_truth/odom"}
EXPECTED_TOPIC_TYPES = {
    "/odom": "nav_msgs/msg/Odometry",
    "/odom/unfiltered": "nav_msgs/msg/Odometry",
    "/odometry/gps": "nav_msgs/msg/Odometry",
    "/gnss/fix": "sensor_msgs/msg/NavSatFix",
    "/formal_mapping/lifecycle_status": "std_msgs/msg/String",
    "/ground_truth/odom": "nav_msgs/msg/Odometry",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _has_mcap_magic(path: Path) -> bool:
    """Check the MCAP envelope without loading a multi-hour recording in RAM."""
    magic = b"\x89MCAP0\r\n"
    if path.stat().st_size < 2 * len(magic):
        return False
    with path.open("rb") as stream:
        if stream.read(len(magic)) != magic:
            return False
        stream.seek(-len(magic), 2)
        return stream.read(len(magic)) == magic


def _direct_child(root: Path, candidate: Path, label: str) -> Path:
    root = root.resolve()
    resolved = candidate.resolve(strict=False)
    if candidate.is_symlink() or resolved.parent != root:
        raise ValueError(f"{label} must be a non-symlink direct child of run root")
    return resolved


def _metadata_topics(metadata: dict[str, Any], requested_topics: tuple[str, ...]) -> dict[str, int]:
    info = metadata.get("rosbag2_bagfile_information")
    rows = info.get("topics_with_message_count") if isinstance(info, dict) else None
    if not isinstance(rows, list) or not rows:
        raise ValueError("rosbag metadata topic set does not match requested topics")
    counts: dict[str, int] = {}
    for row in rows:
        topic_metadata = row.get("topic_metadata") if isinstance(row, dict) else None
        topic = topic_metadata.get("name") if isinstance(topic_metadata, dict) else None
        count = row.get("message_count") if isinstance(row, dict) else None
        if (
            not isinstance(topic, str)
            or type(count) is not int
            or count < 0
            or topic in counts
            or topic not in EXPECTED_TOPIC_TYPES
            or not isinstance(topic_metadata, dict)
            or topic_metadata.get("type") != EXPECTED_TOPIC_TYPES[topic]
            or topic_metadata.get("serialization_format") != "cdr"
        ):
            raise ValueError("rosbag metadata topic contract is invalid")
        counts[topic] = count
    if not set(REQUIRED_TOPICS).issubset(counts) or not set(counts).issubset(requested_topics):
        raise ValueError("rosbag metadata topic set differs from requested topics")
    return counts


def _metadata_window(info: dict[str, Any], started_epoch_ns: int, binding_epoch_ns: int) -> tuple[int, int]:
    start = info.get("starting_time")
    duration = info.get("duration")
    start_ns = start.get("nanoseconds_since_epoch") if isinstance(start, dict) else None
    duration_ns = duration.get("nanoseconds") if isinstance(duration, dict) else None
    if (
        type(start_ns) is not int or type(duration_ns) is not int
        or start_ns <= 0 or duration_ns <= 0
        or started_epoch_ns < binding_epoch_ns
        or start_ns + duration_ns < started_epoch_ns
        # Allow only normal process scheduling/clock-read skew before the
        # runner's recorded start marker; never admit an old bag.
        or start_ns < started_epoch_ns - 5_000_000_000
    ):
        raise ValueError("rosbag metadata time window is not fresh for this run")
    return start_ns, start_ns + duration_ns


def finalize(
    *,
    run_root: Path,
    bag_dir: Path,
    topic_manifest: Path,
    output: Path,
    recorder_stop_rc: int,
    optional_topics: tuple[str, ...],
    runtime_binding: Path,
    started_epoch_ns: int,
) -> dict[str, Any]:
    if run_root.is_symlink() or not run_root.is_dir():
        # Do not resolve or write through a non-fresh/symlinked run root.
        return {
            "schema_version": 1,
            "status": "FORMAL_FIRST_MAP_LOCALIZATION_DIAGNOSTIC_BLOCKED",
            "passed": False,
            "diagnostic_only": True,
            "world_truth_used_for_control": False,
            "control_chain_modified": False,
            "recorder_stop_rc": recorder_stop_rc,
            "blockers": ["run root must be an existing non-symlink directory"],
        }
    root = run_root.resolve()
    blockers: list[str] = []
    try:
        receipt = _direct_child(root, output, "diagnostic receipt")
    except (OSError, ValueError) as exc:
        # Do not create a failure receipt outside the fresh run root either.
        return {
            "schema_version": 1,
            "status": "FORMAL_FIRST_MAP_LOCALIZATION_DIAGNOSTIC_BLOCKED",
            "passed": False,
            "diagnostic_only": True,
            "world_truth_used_for_control": False,
            "control_chain_modified": False,
            "recorder_stop_rc": recorder_stop_rc,
            "blockers": [str(exc)],
        }
    if recorder_stop_rc != 0:
        report = {
            "schema_version": 1,
            "status": "FORMAL_FIRST_MAP_LOCALIZATION_DIAGNOSTIC_BLOCKED",
            "passed": False,
            "diagnostic_only": True,
            "world_truth_used_for_control": False,
            "control_chain_modified": False,
            "recorder_stop_rc": recorder_stop_rc,
            "blockers": [f"diagnostic recorder stop failed: rc={recorder_stop_rc}"],
        }
        receipt.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return report
    try:
        bag = _direct_child(root, bag_dir, "diagnostic bag")
        topics_path = _direct_child(root, topic_manifest, "diagnostic topic manifest")
        binding_path = _direct_child(root, runtime_binding, "runtime gate binding")
        if receipt.exists():
            raise ValueError("diagnostic receipt already exists")
        requested_topics = tuple(
            line.strip() for line in topics_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        )
        if (
            len(set(requested_topics)) != len(requested_topics)
            or requested_topics[:len(REQUIRED_TOPICS)] != REQUIRED_TOPICS
            or any(topic not in {*REQUIRED_TOPICS, *OPTIONAL_TOPICS} for topic in requested_topics)
        ):
            raise ValueError("diagnostic topic manifest is not an exact unique formal set")
        if tuple(topic for topic in requested_topics if topic not in REQUIRED_TOPICS) != optional_topics:
            raise ValueError("diagnostic topic manifest optional topics differ from receipt arguments")
        binding = json.loads(binding_path.read_text(encoding="utf-8"))
        binding_epoch_ns = binding.get("verified_epoch_ns") if isinstance(binding, dict) else None
        if type(binding_epoch_ns) is not int or binding_epoch_ns <= 0:
            raise ValueError("runtime gate binding has no verified epoch")
        metadata_path = bag / "metadata.yaml"
        if not bag.is_dir() or bag.is_symlink() or not metadata_path.is_file() or metadata_path.is_symlink():
            raise ValueError("rosbag metadata is missing")
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        info = metadata.get("rosbag2_bagfile_information") if isinstance(metadata, dict) else None
        if not isinstance(info, dict) or info.get("storage_identifier") != "mcap":
            raise ValueError("diagnostic rosbag is not MCAP")
        relative_files = info.get("relative_file_paths")
        if (
            not isinstance(relative_files, list)
            or not relative_files
            or len(set(relative_files)) != len(relative_files)
        ):
            raise ValueError("diagnostic rosbag has no MCAP data file")
        files: dict[str, str] = {"metadata.yaml": _sha256(metadata_path)}
        for name in relative_files:
            if not isinstance(name, str) or Path(name).name != name or not name.endswith(".mcap"):
                raise ValueError("diagnostic rosbag has an unsafe non-MCAP data filename")
            path = bag / name
            if path.is_symlink() or not path.is_file() or path.resolve().parent != bag.resolve() or path.stat().st_size == 0:
                raise ValueError("diagnostic rosbag MCAP data file is missing or empty")
            if not _has_mcap_magic(path):
                raise ValueError("diagnostic rosbag data file lacks MCAP magic")
            files[name] = _sha256(path)
        topic_counts = _metadata_topics(metadata, requested_topics)
        total_message_count = info.get("message_count")
        if (
            type(total_message_count) is not int
            or total_message_count <= 0
            or total_message_count != sum(topic_counts.values())
        ):
            raise ValueError("rosbag metadata total message count is inconsistent")
        missing = [topic for topic in REQUIRED_TOPICS if topic_counts.get(topic, 0) <= 0]
        if missing:
            raise ValueError("diagnostic rosbag has no messages for: " + ",".join(missing))
        started_ns, ended_ns = _metadata_window(info, started_epoch_ns, binding_epoch_ns)
        report: dict[str, Any] = {
            "schema_version": 1,
            "status": "FORMAL_FIRST_MAP_LOCALIZATION_DIAGNOSTIC_CAPTURED",
            "passed": True,
            "diagnostic_only": True,
            "world_truth_used_for_control": False,
            "control_chain_modified": False,
            "storage_identifier": "mcap",
            "required_topics": list(REQUIRED_TOPICS),
            "optional_topics_requested": list(optional_topics),
            "ground_truth_odom_available": topic_counts.get("/ground_truth/odom", 0) > 0,
            "requested_topics_sha256": _sha256(topics_path),
            "runtime_gate_binding_sha256": _sha256(binding_path),
            "runtime_gate_binding_verified_epoch_ns": binding_epoch_ns,
            "recorder_started_epoch_ns": started_epoch_ns,
            "bag_started_epoch_ns": started_ns,
            "bag_ended_epoch_ns": ended_ns,
            "topics_with_message_count": topic_counts,
            "recorder_stop_rc": recorder_stop_rc,
            "bag_files_sha256": files,
        }
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        report = {
            "schema_version": 1,
            "status": "FORMAL_FIRST_MAP_LOCALIZATION_DIAGNOSTIC_BLOCKED",
            "passed": False,
            "diagnostic_only": True,
            "world_truth_used_for_control": False,
            "control_chain_modified": False,
            "recorder_stop_rc": recorder_stop_rc,
            "blockers": [str(exc)],
        }
    receipt.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--bag-dir", type=Path, required=True)
    parser.add_argument("--topic-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--recorder-stop-rc", type=int, required=True)
    parser.add_argument("--optional-topic", action="append", default=[])
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--started-epoch-ns", type=int, required=True)
    args = parser.parse_args()
    report = finalize(
        run_root=args.run_root,
        bag_dir=args.bag_dir,
        topic_manifest=args.topic_manifest,
        output=args.output,
        recorder_stop_rc=args.recorder_stop_rc,
        optional_topics=tuple(args.optional_topic),
        runtime_binding=args.runtime_binding,
        started_epoch_ns=args.started_epoch_ns,
    )
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
