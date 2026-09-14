#!/usr/bin/env python3
"""Fail-closed admission audit for offline SLAM replay inputs.

This command does not run SLAM and does not measure map area.  It verifies
that a closed rosbag2/MCAP input set actually contains the observations and
transforms needed by the project's configured ``slam_toolbox`` mapping mode.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Iterable

import yaml


MCAP_MAGIC = b"\x89MCAP0\r\n"
REQUIRED_TOPICS = {
    "/scan": "sensor_msgs/msg/LaserScan",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/clock": "rosgraph_msgs/msg/Clock",
    "/odom": "nav_msgs/msg/Odometry",
}


class ReplayInputError(ValueError):
    """Raised when replay input cannot be admitted without guessing."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _regular_file(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise ReplayInputError(f"{label} must be a regular file: {path}")


def _relative_path(root: Path, path: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _metadata_topics(info: dict[str, Any]) -> dict[str, tuple[str, int]]:
    rows = info.get("topics_with_message_count")
    if not isinstance(rows, list) or not rows:
        raise ReplayInputError("rosbag metadata has no topics")
    topics: dict[str, tuple[str, int]] = {}
    for row in rows:
        topic_metadata = row.get("topic_metadata") if isinstance(row, dict) else None
        count = row.get("message_count") if isinstance(row, dict) else None
        if not isinstance(topic_metadata, dict):
            raise ReplayInputError("rosbag topic metadata is malformed")
        topic = topic_metadata.get("name")
        type_name = topic_metadata.get("type")
        serialization = topic_metadata.get("serialization_format")
        if (
            not isinstance(topic, str)
            or not topic.startswith("/")
            or not isinstance(type_name, str)
            or not type_name
            or serialization != "cdr"
            or isinstance(count, bool)
            or not isinstance(count, int)
            or count <= 0
        ):
            raise ReplayInputError("rosbag topic contract is malformed")
        if topic in topics:
            raise ReplayInputError(f"duplicate rosbag topic: {topic}")
        topics[topic] = (type_name, count)
    return topics


def inspect_bag(bag_dir: Path) -> dict[str, Any]:
    """Inspect one closed rosbag2 directory without reading message payloads."""

    if bag_dir.is_symlink() or not bag_dir.is_dir():
        raise ReplayInputError(f"bag directory must be a real directory: {bag_dir}")
    metadata_path = bag_dir / "metadata.yaml"
    _regular_file(metadata_path, "rosbag metadata")
    try:
        raw = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    except Exception as error:
        raise ReplayInputError(f"cannot parse rosbag metadata: {error}") from error
    info = raw.get("rosbag2_bagfile_information") if isinstance(raw, dict) else None
    if not isinstance(info, dict):
        raise ReplayInputError("metadata lacks rosbag2_bagfile_information")
    if info.get("storage_identifier") != "mcap":
        raise ReplayInputError("only closed MCAP bags are accepted")
    topics = _metadata_topics(info)
    total_count = info.get("message_count")
    if (
        isinstance(total_count, bool)
        or not isinstance(total_count, int)
        or total_count <= 0
        or total_count != sum(count for _, count in topics.values())
    ):
        raise ReplayInputError("rosbag total message count does not match topic counts")

    names = info.get("relative_file_paths")
    if not isinstance(names, list) or not names or not all(
        isinstance(name, str) and name for name in names
    ):
        raise ReplayInputError("rosbag metadata lacks data file paths")
    data_files: list[dict[str, Any]] = []
    for name in names:
        data_path = bag_dir / name
        if Path(name).is_absolute() or not data_path.resolve().is_relative_to(
            bag_dir.resolve()
        ):
            raise ReplayInputError(f"rosbag data path escapes the bag: {name}")
        _regular_file(data_path, "rosbag data file")
        if data_path.stat().st_size < 2 * len(MCAP_MAGIC):
            raise ReplayInputError(f"rosbag data file is too short: {name}")
        with data_path.open("rb") as stream:
            if stream.read(len(MCAP_MAGIC)) != MCAP_MAGIC:
                raise ReplayInputError(f"rosbag data file lacks MCAP header: {name}")
            stream.seek(-len(MCAP_MAGIC), os.SEEK_END)
            if stream.read(len(MCAP_MAGIC)) != MCAP_MAGIC:
                raise ReplayInputError(f"rosbag data file lacks MCAP footer: {name}")
        data_files.append(
            {
                "path": name,
                "size_bytes": data_path.stat().st_size,
                "sha256": _sha256(data_path),
            }
        )

    return {
        "bag_dir": bag_dir.resolve().as_posix(),
        "metadata_path": metadata_path.resolve().as_posix(),
        "metadata_sha256": _sha256(metadata_path),
        "message_count": total_count,
        "duration_ns": (info.get("duration") or {}).get("nanoseconds"),
        "topics": {
            topic: {"type": type_name, "message_count": count}
            for topic, (type_name, count) in sorted(topics.items())
        },
        "data_files": sorted(data_files, key=lambda row: row["path"]),
    }


def _topic_contract(
    inspected_bags: Iterable[dict[str, Any]],
) -> tuple[dict[str, dict[str, Any]], list[str]]:
    observed: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    for bag in inspected_bags:
        for topic, row in bag["topics"].items():
            current = observed.get(topic)
            if current is not None and current["type"] != row["type"]:
                errors.append(
                    f"topic {topic} has conflicting types "
                    f"{current['type']} and {row['type']}"
                )
                continue
            if current is None:
                current = {"type": row["type"], "message_count": 0, "bags": []}
                observed[topic] = current
            current["message_count"] += row["message_count"]
            current["bags"].append(bag["bag_dir"])
    return observed, errors


def audit_replay_inputs(
    bag_dirs: Iterable[Path],
    *,
    run_root: str | None = None,
    source_revision: str | None = None,
) -> dict[str, Any]:
    """Return a deterministic admission receipt for an offline replay set."""

    directories = [Path(path) for path in bag_dirs]
    if not directories:
        raise ReplayInputError("at least one bag directory is required")
    bags: list[dict[str, Any]] = []
    errors: list[str] = []
    for bag_dir in directories:
        try:
            bags.append(inspect_bag(bag_dir))
        except ReplayInputError as error:
            errors.append(f"{bag_dir}: {error}")

    observed, contract_errors = _topic_contract(bags)
    errors.extend(contract_errors)
    missing: list[dict[str, str]] = []
    for topic, expected_type in REQUIRED_TOPICS.items():
        row = observed.get(topic)
        if row is None:
            missing.append(
                {
                    "topic": topic,
                    "expected_type": expected_type,
                    "reason": "topic absent",
                }
            )
        elif row["type"] != expected_type:
            missing.append(
                {
                    "topic": topic,
                    "expected_type": expected_type,
                    "reason": f"type is {row['type']}",
                }
            )
    eligible = not errors and not missing
    status = (
        "LIVE_SLAM_REPLAY_INPUT_READY"
        if eligible
        else "LIVE_SLAM_REPLAY_INPUT_BLOCKED"
    )
    return {
        "schema_version": 1,
        "report_id": "tzcup_live_slam_replay_input_audit_v1",
        "generated_utc": dt.datetime.now(dt.timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z"),
        "run_root": run_root,
        "source_revision": source_revision,
        "auditor_sha256": _sha256(Path(__file__).resolve()),
        "status": status,
        "replay_eligible": eligible,
        "slam_replay_claimed": False,
        "area_gate": "NOT_MEASURED",
        "required_topics": REQUIRED_TOPICS,
        "observed_topics": observed,
        "missing_requirements": missing,
        "errors": errors,
        "bags": bags,
        "claim_boundary": (
            "Admission audit only. A ready status permits a separate offline "
            "SLAM replay attempt; it does not prove replay execution, map "
            "quality, or the 20,000 m2 area gate."
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Fail-closed audit of closed MCAP inputs for the configured "
            "slam_toolbox mapping mode. This command never starts ROS."
        )
    )
    parser.add_argument("--bag-dir", action="append", required=True, type=Path)
    parser.add_argument("--run-root")
    parser.add_argument("--source-revision")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args(argv)

    try:
        report = audit_replay_inputs(
            args.bag_dir,
            run_root=args.run_root,
            source_revision=args.source_revision,
        )
    except ReplayInputError as error:
        print(json.dumps({"status": "INPUT_ERROR", "error": str(error)}))
        return 1

    serialized = json.dumps(report, ensure_ascii=True, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    else:
        print(serialized, end="")
    if report["errors"]:
        return 1
    if args.require_pass and not report["replay_eligible"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
