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
import signal
import subprocess
import time
import uuid
from pathlib import Path
from typing import Any, Iterable

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = "tzcup.formal_product_mcap_replay.v1"
PRODUCER_ID = "scripts/formal_product_mcap_replay.py"
PRODUCT_TOPICS = {
    "/perception/open_vocab/dosod_boxes": "observations",
    "/perception/garbage/targets": "tracks_and_dynamic_trash_map",
    "/active_cleaning/planner_status": "scheduler_and_post_clean_state",
    "/active_cleaning/grasp_result": "clean_decisions",
}
GROUND_DIRT_STATUS_TOPIC = "/evaluation/single_episode/ground_dirt/status_json"
RAW_CAPTURE_SCHEMA = "tzcup.a12.raw_capture_receipt.v1"
# A raw-capture receipt is evidence only when this exact, independently
# reviewed producer created it.  Do not relax this to an arbitrary repository
# file: that would let a replay source self-attest a capture it never ran.
RAW_CAPTURE_PRODUCER_ID = "scripts/formal_a12_single_execution_capture.py"
FORMAL_SAFE_CAPTURE_DOMAINS = frozenset((*range(0, 102), *range(215, 232)))
FORMAL_SAFE_REPLAY_DOMAINS = frozenset(range(215, 232))


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


def _mcap_metadata(bag: Path) -> tuple[dict[str, Any], list[Path]]:
    """Return the rosbag-declared storage files and reject every extra file.

    rosbag metadata is the authority for the MCAP tree.  Hashing a recursive
    directory alone is insufficient: a copied/augmented directory can carry
    unrelated files while still looking like a bag to the ROS reader.
    """
    bag = _non_link_path(bag, "MCAP")
    if not bag.is_dir() or bag.is_symlink():
        raise ProductReplayError("MCAP is not a canonical directory")
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file() or metadata_path.is_symlink():
        raise ProductReplayError("MCAP directory has no canonical metadata.yaml")
    try:
        document = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ProductReplayError(f"cannot read MCAP metadata: {exc}") from exc
    info = document.get("rosbag2_bagfile_information") if isinstance(document, dict) else None
    if not isinstance(info, dict) or info.get("storage_identifier") != "mcap":
        raise ProductReplayError("MCAP metadata does not declare mcap storage")
    declared = info.get("relative_file_paths")
    files = info.get("files")
    if not isinstance(declared, list) or not declared or not isinstance(files, list):
        raise ProductReplayError("MCAP metadata has no declared storage files")
    declared_paths: list[Path] = []
    for value in declared:
        relative = Path(value) if isinstance(value, str) else Path()
        if not isinstance(value, str) or relative.is_absolute() or ".." in relative.parts or str(relative) in {"", "."}:
            raise ProductReplayError("MCAP metadata has an unsafe storage path")
        declared_paths.append(relative)
    if len(set(declared_paths)) != len(declared_paths):
        raise ProductReplayError("MCAP metadata repeats a storage file")
    if any(not isinstance(item, dict) or set(item) - {"path", "starting_time", "duration", "message_count"} or not isinstance(item.get("path"), str) for item in files):
        raise ProductReplayError("MCAP metadata has an illegal files entry")
    file_paths = [Path(item["path"]) for item in files]
    if len(file_paths) != len(set(file_paths)) or set(file_paths) != set(declared_paths):
        raise ProductReplayError("MCAP metadata files disagree with relative_file_paths")
    storage_files: list[Path] = []
    for relative in declared_paths:
        path = _inside(bag, bag / relative, "MCAP storage file")
        if not path.is_file() or path.is_symlink():
            raise ProductReplayError("MCAP metadata declares a missing or linked storage file")
        storage_files.append(path)
    actual = {item.relative_to(bag) for item in bag.rglob("*") if item.is_file()}
    expected = {Path("metadata.yaml"), *declared_paths}
    if actual != expected:
        raise ProductReplayError("MCAP tree contains an undeclared or missing file")
    if any(item.is_symlink() for item in bag.rglob("*")):
        raise ProductReplayError("MCAP tree contains a symbolic link")
    return info, storage_files


def mcap_sha256(bag: Path) -> str:
    """Hash only a strict metadata-declared MCAP tree."""
    _info, storage_files = _mcap_metadata(bag)
    bag = _non_link_path(bag, "MCAP")
    digest = hashlib.sha256()
    for path in [bag / "metadata.yaml", *sorted(storage_files)]:
        relative = path.relative_to(bag).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(4, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(sha256(path)))
    return digest.hexdigest()


def mcap_semantic_sha256(bag: Path) -> str:
    """Hash the canonical ROS message stream, independent of MCAP packing/order."""
    _mcap_metadata(bag)
    try:
        import rosbag2_py
    except ImportError as exc:  # pragma: no cover - formal runtime only
        raise ProductReplayError(f"cannot canonicalize MCAP message stream: {exc}") from exc
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
        topic_types = sorted((item.name, item.type) for item in reader.get_all_topics_and_types())
        digest = hashlib.sha256(json.dumps(topic_types, separators=(",", ":")).encode("utf-8"))
        while reader.has_next():
            topic, data, received_ns = reader.read_next()
            row = (int(received_ns), topic, hashlib.sha256(bytes(data)).hexdigest())
            digest.update(json.dumps(row, separators=(",", ":")).encode("utf-8"))
    except Exception as exc:  # ROS binding errors must not turn into uniqueness claims
        raise ProductReplayError(f"cannot read canonical MCAP message stream: {exc}") from exc
    return digest.hexdigest()


def _inside(root: Path, path: Path, label: str) -> Path:
    root = _non_link_path(root, "run root")
    candidate = _non_link_path(path, label)
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise ProductReplayError(f"{label} escapes raw capture run root") from exc
    return candidate


def _sha256_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise ProductReplayError(f"{label} must be a lowercase SHA-256 digest")
    return value


def _capture_descriptor(raw: dict[str, Any], root: Path, field: str, *, directory: bool) -> dict[str, str]:
    descriptor = raw.get(field)
    if not isinstance(descriptor, dict) or set(descriptor) != {"path", "sha256"}:
        raise ProductReplayError(f"raw capture {field} descriptor is incomplete")
    path_value = descriptor["path"]
    if not isinstance(path_value, str):
        raise ProductReplayError(f"raw capture {field} path is invalid")
    path = _inside(root, Path(path_value), f"raw capture {field}")
    if (directory and (not path.is_dir() or path.is_symlink())) or (not directory and (not path.is_file() or path.is_symlink())):
        raise ProductReplayError(f"raw capture {field} artifact is missing or linked")
    digest = mcap_sha256(path) if field == "mcap" else (artifact_sha256(path) if directory else sha256(path))
    if digest != _sha256_text(descriptor["sha256"], f"raw capture {field} hash"):
        raise ProductReplayError(f"raw capture {field} content hash mismatch")
    result = {"path": str(path), "sha256": digest}
    if field == "mcap":
        result["semantic_sha256"] = mcap_semantic_sha256(path)
    return result


def _bound_file(binding: dict[str, Any], root: Path, name: str) -> tuple[Path, dict[str, Any]]:
    artifacts = binding.get("artifacts")
    reference = artifacts.get(name) if isinstance(artifacts, dict) else None
    if not isinstance(reference, dict) or reference.get("kind") != "file" or not isinstance(reference.get("path"), str):
        raise ProductReplayError(f"capture input binding lacks immutable {name}")
    path = _inside(root, Path(reference["path"]), f"capture bound {name}")
    if not path.is_file() or path.is_symlink() or sha256(path) != _sha256_text(reference.get("sha256"), f"capture bound {name} hash"):
        raise ProductReplayError(f"capture bound {name} changed or is not a regular file")
    return path, reference


def _capture_identity_from_bound_episode(raw: dict[str, Any], root: Path) -> dict[str, Any]:
    """Derive labels only from immutable episode input, never receipt/CLI labels."""
    descriptor = raw.get("episode_input_binding")
    if not isinstance(descriptor, dict) or set(descriptor) != {"path", "sha256"} or not isinstance(descriptor["path"], str):
        raise ProductReplayError("raw capture has no immutable episode input binding")
    binding_path = _inside(root, Path(descriptor["path"]), "capture input binding")
    if not binding_path.is_file() or binding_path.is_symlink() or sha256(binding_path) != _sha256_text(descriptor["sha256"], "capture input binding hash"):
        raise ProductReplayError("raw capture input binding changed or is not regular")
    binding = read_object(binding_path)
    episode_path, _episode_reference = _bound_file(binding, root, "episode_manifest")
    evaluator_path, _evaluator_reference = _bound_file(binding, root, "evaluator_manifest")
    episode = read_object(episode_path)
    evaluator = read_object(evaluator_path)
    identity = episode.get("a12_execution")
    if (
        not isinstance(identity, dict)
        or set(identity) != {"scenario_id", "seed", "mission_id", "mission_group_id"}
        or not isinstance(identity["scenario_id"], str) or not identity["scenario_id"]
        or type(identity["seed"]) is not int
        or not isinstance(identity["mission_id"], str) or not identity["mission_id"]
        or not isinstance(identity["mission_group_id"], str) or not identity["mission_group_id"]
    ):
        raise ProductReplayError("bound episode manifest has no complete A12 execution identity")
    seeds = evaluator.get("seeds")
    if not isinstance(seeds, dict) or seeds.get("dirt") != identity["seed"]:
        raise ProductReplayError("bound evaluator seed does not match the bound A12 episode")
    return {
        **identity,
        "episode_manifest": {"path": str(episode_path), "sha256": sha256(episode_path)},
        "evaluator_manifest": {"path": str(evaluator_path), "sha256": sha256(evaluator_path)},
        "input_binding": {"path": str(binding_path), "sha256": sha256(binding_path)},
    }


def validate_raw_capture_receipt(
    *,
    repository_root: Path,
    raw_capture_receipt_path: Path,
    formal_context: dict[str, Any],
    scenario_id: str,
    seed: int,
    mission_id: str,
    run_root: Path | None = None,
) -> dict[str, Any]:
    """Validate one immutable capture tuple before replay or receipt sealing."""
    receipt_path = _non_link_path(raw_capture_receipt_path, "raw capture receipt")
    raw = read_object(receipt_path)
    if raw.get("schema") != RAW_CAPTURE_SCHEMA or raw.get("status") != "A12_RAW_CAPTURE_COMPLETE" or raw.get("capture_complete") is not True:
        raise ProductReplayError("raw capture receipt is not a completed canonical capture")
    publication = raw.get("publication")
    if not isinstance(publication, dict) or publication.get("method") != "atomic_link_no_replace" or publication.get("overwrote_existing") is not False:
        raise ProductReplayError("raw capture receipt lacks no-replace publication proof")
    producer = raw.get("producer")
    if not isinstance(producer, dict) or set(producer) != {"id", "sha256"} or producer.get("id") != RAW_CAPTURE_PRODUCER_ID:
        raise ProductReplayError("raw capture receipt producer identity is missing")
    producer_path = _non_link_path(repository_root / RAW_CAPTURE_PRODUCER_ID, "raw capture producer")
    if not producer_path.is_file() or producer_path.is_symlink():
        raise ProductReplayError("raw capture receipt producer is not a current regular source file")
    if sha256(producer_path) != _sha256_text(producer.get("sha256"), "raw capture producer hash"):
        raise ProductReplayError("raw capture receipt producer source hash mismatch")
    capture_id = raw.get("capture_id")
    if not isinstance(capture_id, str) or len(capture_id) < 32 or any(character not in "0123456789abcdef" for character in capture_id):
        raise ProductReplayError("raw capture receipt has an invalid capture identity")
    if raw.get("formal_context") != formal_context:
        raise ProductReplayError("raw capture receipt session/snapshot/runtime closure differs from replay")
    started_ns, finished_ns = raw.get("started_epoch_ns"), raw.get("finished_epoch_ns")
    session_started_ns = formal_context["session"]["started_epoch_ns"]
    now_ns = time.time_ns()
    if (
        type(started_ns) is not int or type(finished_ns) is not int
        or started_ns < session_started_ns or finished_ns < started_ns or finished_ns > now_ns
    ):
        raise ProductReplayError("raw capture receipt has invalid current-session timing")
    dds = raw.get("dds")
    if (
        not isinstance(dds, dict)
        or dds.get("ros_domain_id") not in FORMAL_SAFE_CAPTURE_DOMAINS
        or dds.get("ros_localhost_only") is not True
        or dds.get("rmw_implementation") != "rmw_cyclonedds_cpp"
        or dds.get("automatic_discovery_range") != "LOCALHOST"
    ):
        raise ProductReplayError("raw capture DDS isolation is outside the formal-safe contract")
    cleanup = raw.get("process_group_cleanup")
    if (
        not isinstance(cleanup, dict)
        or type(cleanup.get("process_group_id")) is not int
        or cleanup["process_group_id"] <= 1
        or cleanup.get("process_group_isolated") is not True
        or cleanup.get("zero_survivor") is not True
        or cleanup.get("surviving_group_processes") != 0
        or type(cleanup.get("cleanup_completed_epoch_ns")) is not int
        or cleanup["cleanup_completed_epoch_ns"] < finished_ns or cleanup["cleanup_completed_epoch_ns"] > now_ns
    ):
        raise ProductReplayError("raw capture process-group cleanup is not proven")
    census = raw.get("post_run_pgid_census")
    if (
        not isinstance(census, dict)
        or census.get("method") != "procfs_post_cleanup_pgid_census_v1"
        or census.get("process_group_id") != cleanup["process_group_id"]
        or census.get("surviving_process_ids") != []
        or type(census.get("census_epoch_ns")) is not int
        or census["census_epoch_ns"] < cleanup["cleanup_completed_epoch_ns"] or census["census_epoch_ns"] > now_ns
    ):
        raise ProductReplayError("raw capture lacks a post-run empty PGID census")
    root_value = raw.get("run_root")
    if not isinstance(root_value, str):
        raise ProductReplayError("raw capture receipt has no run root")
    root = _non_link_path(Path(root_value), "raw capture run root")
    if run_root is None:
        raise ProductReplayError("expected execution run root is required")
    if root != _non_link_path(run_root, "execution run root"):
        raise ProductReplayError("raw capture receipt belongs to another run root")
    _inside(root, receipt_path, "raw capture receipt")
    bound_identity = _capture_identity_from_bound_episode(raw, root)
    if (
        raw.get("scenario_id") != bound_identity["scenario_id"]
        or raw.get("seed") != bound_identity["seed"]
        or raw.get("mission_id") != bound_identity["mission_id"]
        or (scenario_id, seed, mission_id) != (
            bound_identity["scenario_id"], bound_identity["seed"], bound_identity["mission_id"]
        )
    ):
        raise ProductReplayError("scenario, seed, or mission is not derived from the immutable bound episode")
    descriptors = {
        "video": _capture_descriptor(raw, root, "video", directory=False),
        "mcap": _capture_descriptor(raw, root, "mcap", directory=True),
        "source_metrics": _capture_descriptor(raw, root, "source_metrics", directory=False),
    }
    for field, descriptor in descriptors.items():
        modified_ns = Path(descriptor["path"]).stat().st_mtime_ns
        if modified_ns < started_ns or modified_ns > finished_ns:
            raise ProductReplayError(f"raw capture {field} falls outside its capture timing window")
    # This is capture-time evidence, not a replay-time /proc lookup: a later
    # PGID can be recycled, so replay must not mistake it for the old capture.
    return {
        "path": str(receipt_path),
        "sha256": sha256(receipt_path),
        "run_root": str(root),
        "producer": producer,
        "publication": publication,
        "capture_id": capture_id,
        "scenario_id": bound_identity["scenario_id"],
        "seed": bound_identity["seed"],
        "mission_id": bound_identity["mission_id"],
        "episode_identity": bound_identity,
        "started_epoch_ns": started_ns,
        "finished_epoch_ns": finished_ns,
        "video": descriptors["video"],
        "mcap": descriptors["mcap"],
        "source_metrics": descriptors["source_metrics"],
        "dds": dds,
        "process_group_cleanup": cleanup,
        "post_run_pgid_census": census,
    }


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


def update_product_chain(chain: dict[str, Any], topic: str, message: Any) -> None:
    """Decode the formal product interfaces, never the legacy small-car ones."""
    if topic == "/perception/open_vocab/dosod_boxes":
        chain["observation_count"] += len(message.detections)
    elif topic == "/perception/garbage/targets":
        for target in message.targets:
            chain["unique_track_ids"].add(str(target.uuid))
            chain["track_observation_count"] += int(target.observation_count)
            chain["track_states"].add(str(target.track_state))
    elif topic == "/active_cleaning/planner_status":
        for row in message.status:
            if row.name == "formal_active_cleaning_policy_planner":
                chain["scheduler_states"].add(str(row.message))
    elif topic == "/active_cleaning/grasp_result":
        try:
            result = json.loads(message.data)
            if (not isinstance(result, dict) or result.get("schema_version") != 2
                    or not isinstance(result.get("target_id"), str) or not result["target_id"]
                    or type(result.get("verified_in_bin")) is not bool):
                raise ValueError("invalid physical grasp result")
        except (TypeError, ValueError) as exc:
            raise ProductReplayError("malformed formal physical grasp result") from exc
        chain["grasp_target_ids"].add(result["target_id"])
        if result["verified_in_bin"]:
            chain["verified_grasp_target_ids"].add(result["target_id"])
        chain["post_clean_success_count"] = len(chain["verified_grasp_target_ids"])


def read_mcap_records(bag: Path) -> dict[str, Any]:
    """Read and deserialize the exact topics used for product recalculation."""
    try:
        from formal_cleaning_geometry import load_cleaning_geometry, sample_from_ground_dirt_status
    except ImportError as exc:
        raise ProductReplayError(f"formal cleaning geometry dependencies are unavailable: {exc}") from exc
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
    if topic_types.get(GROUND_DIRT_STATUS_TOPIC) != "std_msgs/msg/String":
        raise ProductReplayError(
            f"ground-dirt status topic must be std_msgs/msg/String: {GROUND_DIRT_STATUS_TOPIC}"
        )
    cleaning_geometry = load_cleaning_geometry()
    boundary_topics = {"/product_demo/operator_start", "/active_cleaning/mission_complete"}
    boundaries: dict[str, list[int]] = {topic: [] for topic in boundary_topics}
    while reader.has_next():
        topic, data, received_stamp = reader.read_next()
        if topic not in boundary_topics:
            continue
        if topic not in topic_types:
            raise ProductReplayError(f"MCAP boundary has no declared topic type: {topic}")
        if bool(deserialize_message(data, get_message(topic_types[topic])).data):
            boundaries[topic].append(int(received_stamp))
    starts, completes = boundaries["/product_demo/operator_start"], boundaries["/active_cleaning/mission_complete"]
    if len(starts) != 1 or len(completes) != 1 or starts[0] >= completes[0]:
        raise ProductReplayError("MCAP must contain exactly one ordered operator start and mission complete")
    window_start, window_complete = starts[0], completes[0]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    counts = {topic: 0 for topic in PRODUCT_TOPICS}
    chain = {
        "observation_count": 0,
        "unique_track_ids": set(),
        "track_observation_count": 0,
        "track_states": set(),
        "scheduler_states": set(),
        "grasp_target_ids": set(),
        "verified_grasp_target_ids": set(),
        "post_clean_success_count": 0,
    }
    fused: list[tuple[float, float, float]] = []
    truth: list[tuple[float, float, float]] = []
    cleaning_samples: list[dict[str, Any]] = []
    evaluation_stamps: list[float] = []
    coverage_states: list[str] = []
    selected = set(PRODUCT_TOPICS) | {
        GROUND_DIRT_STATUS_TOPIC,
        "/ground_truth/odom",
        "/localization/fused_odom",
    }
    selected_stamps: list[tuple[str, int]] = []
    while reader.has_next():
        topic, data, received_stamp = reader.read_next()
        if topic not in selected:
            continue
        if int(received_stamp) < window_start or int(received_stamp) >= window_complete:
            continue  # recorder pre-roll/post-roll is deliberately ignored.
        if topic not in topic_types:
            raise ProductReplayError(f"MCAP has data without a declared topic type: {topic}")
        message = deserialize_message(data, get_message(topic_types[topic]))
        selected_stamps.append((topic, int(received_stamp)))
        if topic in counts:
            counts[topic] += 1
            update_product_chain(chain, topic, message)
            continue
        if topic == GROUND_DIRT_STATUS_TOPIC:
            if not isinstance(message.data, str):
                raise ProductReplayError("ground-dirt status_json is not a string")
            try:
                status = json.loads(message.data)
            except (TypeError, json.JSONDecodeError) as exc:
                raise ProductReplayError("ground-dirt status_json is malformed") from exc
            if not isinstance(status, dict):
                raise ProductReplayError("ground-dirt status_json must be an object")
            try:
                cleaning_samples.append(
                    sample_from_ground_dirt_status(
                        status, int(received_stamp) * 1.0e-9, cleaning_geometry
                    )
                )
            except (KeyError, TypeError, ValueError) as exc:
                raise ProductReplayError(f"ground-dirt status_json is invalid: {exc}") from exc
            continue
        pose = _message_pose(message)
        stamp = message.header.stamp
        sample = (stamp.sec + stamp.nanosec * 1.0e-9, float(pose.position.x), float(pose.position.y))
        (truth if topic == "/ground_truth/odom" else fused).append(sample)
    return {
        "topic_types": topic_types,
        "product_topic_counts": counts,
        "product_chain_recalculation": {
            **chain,
            "unique_track_ids": sorted(chain["unique_track_ids"]),
            "track_states": sorted(chain["track_states"]),
            "scheduler_states": sorted(chain["scheduler_states"]),
            "grasp_target_ids": sorted(chain["grasp_target_ids"]),
            "verified_grasp_target_ids": sorted(chain["verified_grasp_target_ids"]),
        },
        "fused": fused,
        "truth": truth,
        "cleaning_samples": cleaning_samples,
        "brush_geometry": {
            "basis": "formal_observed_bristle_sweep_proxy",
            "status_topic": GROUND_DIRT_STATUS_TOPIC,
            "timestamp_basis": "mcap_received_stamp_ns",
        },
        "evaluation_stamps": evaluation_stamps,
        "coverage_states": coverage_states,
        "capture_window": {"start_ns": window_start, "complete_ns": window_complete, "selected_message_count": len(selected_stamps)},
    }


def recalculate(records: dict[str, Any], source_metrics: dict[str, Any]) -> dict[str, Any]:
    """Recompute coverage and localization; never trust embedded pass flags."""
    try:
        from sanitation_coverage.metrics import summarize_distances, synchronized_xy_errors
        from formal_cleaning_geometry import empirical_cleaning_metrics, samples_in_mission_frame
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
    try:
        empirical = empirical_cleaning_metrics(
            geometry["cleanable_outer_polygon"],
            samples_in_mission_frame(records["cleaning_samples"], geometry),
            resolution=float(source_metrics["empirical_metrics"]["resolution_m"]),
            exclusion_polygons=geometry["cleanable_exclusion_polygons"],
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ProductReplayError(f"ground-dirt geometry cannot be recalculated: {exc}") from exc
    if not empirical["valid"]:
        raise ProductReplayError(
            "ground-dirt geometry evidence is invalid: "
            f"invalid_sample_count={empirical['invalid_sample_count']}, "
            f"continuity_break_count={empirical['continuity_break_count']}"
        )
    source_empirical = source_metrics["empirical_metrics"]
    if source_empirical.get("valid") is not True:
        raise ProductReplayError("source ground-dirt geometry metric is not valid")
    if source_empirical.get("metric_basis") != empirical["metric_basis"]:
        raise ProductReplayError("source metrics use a different cleaning geometry basis")
    if source_empirical.get("source_hashes") != empirical["source_hashes"]:
        raise ProductReplayError("source metrics geometry provenance does not match replay")
    if source_empirical.get("sampling") != empirical["sampling"]:
        raise ProductReplayError("source metrics sampling safeguards do not match replay")
    try:
        expected_coverage = float(source_empirical["coverage_rate"])
    except (TypeError, ValueError) as exc:
        raise ProductReplayError("source coverage rate is not numeric") from exc
    if not math.isfinite(expected_coverage) or not 0.0 <= expected_coverage <= 1.0:
        raise ProductReplayError("source coverage rate is outside [0, 1]")
    actual_coverage = float(empirical["coverage_rate"])
    return {
        "coverage": {
            "source_rate": expected_coverage,
            "recalculated_rate": actual_coverage,
            "relative_delta": relative_delta(actual_coverage, expected_coverage),
            "brush_on_sample_count": int(empirical["active_tool_sample_count"]),
            "geometry_metric_basis": empirical["metric_basis"],
            "geometry_source_hashes": empirical["source_hashes"],
            "geometry_metrics": empirical,
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
    if domain_id not in FORMAL_SAFE_REPLAY_DOMAINS:
        raise ProductReplayError("isolated ROS_DOMAIN_ID must be in the formal-safe 215..231 range")
    if rate <= 0.0:
        raise ProductReplayError("playback rate must be positive")
    if timeout_seconds <= 0.0:
        raise ProductReplayError("playback timeout must be positive")
    if os.name != "posix":
        raise ProductReplayError("formal MCAP replay requires POSIX process-group supervision")
    executable = os.environ.get("FORMAL_ROS2_EXECUTABLE")
    if not executable or not Path(executable).is_absolute():
        raise ProductReplayError("FORMAL_ROS2_EXECUTABLE must be a trusted absolute ros2 path")
    ros2 = _non_link_path(Path(executable), "ros2 executable")
    if not ros2.is_file() or ros2.is_symlink():
        raise ProductReplayError("trusted ros2 executable is missing or linked")
    command = [str(ros2), "bag", "play", str(bag), "--storage", "mcap", "--rate", str(rate)]
    environment = os.environ.copy()
    environment.update({
        "ROS_DOMAIN_ID": str(domain_id),
        "ROS_LOCALHOST_ONLY": "1",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    })
    started = time.time_ns()
    signals_sent: list[str] = []
    process_group_id: int | None = None
    timed_out = False
    try:
        process = subprocess.Popen(
            command, env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            text=True, start_new_session=True,
        )
        process_group_id = process.pid
        try:
            stdout, stderr = process.communicate(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            timed_out = True
            _terminate_process_group(process_group_id, signals_sent)
            try:
                stdout, stderr = process.communicate(timeout=5.0)
            except subprocess.TimeoutExpired as exc:
                raise ProductReplayError("ros2 bag play did not exit after timeout cleanup") from exc
        cleanup = _terminate_process_group(process_group_id, signals_sent)
    except (OSError, subprocess.SubprocessError) as exc:
        raise ProductReplayError(f"ros2 bag play failed: {exc}") from exc
    if cleanup["zero_survivor"] is not True:
        raise ProductReplayError("ros2 bag play process group retains survivors")
    if timed_out:
        # A process killed after exceeding the contract may still report 0
        # during SIGTERM handling; that is never a successful replay.
        raise ProductReplayError("ros2 bag play exceeded its timeout")
    finished = time.time_ns()
    return {
        "command": command,
        "ros_domain_id": domain_id,
        "ros_localhost_only": True,
        "rmw_implementation": "rmw_cyclonedds_cpp",
        "automatic_discovery_range": "LOCALHOST",
        "started_epoch_ns": started,
        "finished_epoch_ns": finished,
        "exit_code": process.returncode,
        "timed_out": timed_out,
        "ros2_executable": {"path": str(ros2), "sha256": sha256(ros2)},
        "stdout_sha256": hashlib.sha256((stdout or "").encode()).hexdigest(),
        "stderr_sha256": hashlib.sha256((stderr or "").encode()).hexdigest(),
        "process_group_cleanup": cleanup,
    }


def _parse_proc_stat_process_group(payload: str) -> int:
    """Extract field 5 (pgrp) despite spaces/parentheses in ``comm``."""
    closing = payload.rfind(")")
    if closing <= 0 or closing + 2 >= len(payload) or payload[closing + 1] != " ":
        raise ProductReplayError("malformed /proc stat payload")
    fields = payload[closing + 2:].split()
    # Fields after the final ')' begin with state (field 3); pgrp is field 5.
    if len(fields) < 3:
        raise ProductReplayError("incomplete /proc stat payload")
    try:
        return int(fields[2])
    except ValueError as exc:
        raise ProductReplayError("invalid process group in /proc stat payload") from exc


def _process_group_survivors(process_group_id: int) -> list[int]:
    proc = Path("/proc")
    if os.name != "posix" or not proc.is_dir():
        raise ProductReplayError("cannot attest formal replay process-group cleanup")
    survivors: list[int] = []
    for item in proc.iterdir():
        if not item.name.isdigit():
            continue
        try:
            process_group = _parse_proc_stat_process_group((item / "stat").read_text(encoding="utf-8"))
        except FileNotFoundError:
            # The PID exited between /proc enumeration and stat open (ENOENT).
            continue
        except (OSError, UnicodeError, ProductReplayError) as exc:
            raise ProductReplayError(f"cannot attest process group {process_group_id}: {exc}") from exc
        if process_group == process_group_id:
            survivors.append(int(item.name))
    return sorted(survivors)


def _terminate_process_group(process_group_id: int, signals_sent: list[str]) -> dict[str, Any]:
    survivors = _process_group_survivors(process_group_id)
    for signal_value, label, grace_seconds in (
        (signal.SIGTERM, "SIGTERM", 10.0),
        (signal.SIGKILL, "SIGKILL", 2.0),
    ):
        if not survivors:
            break
        try:
            os.killpg(process_group_id, signal_value)
            signals_sent.append(label)
        except ProcessLookupError:
            survivors = []
            break
        deadline = time.monotonic() + grace_seconds
        while survivors and time.monotonic() < deadline:
            time.sleep(0.05)
            survivors = _process_group_survivors(process_group_id)
    return {
        "process_group_id": process_group_id,
        "process_group_isolated": True,
        "cleanup_attempted": True,
        "sigterm_attempted": "SIGTERM" in signals_sent,
        "sigkill_attempted": "SIGKILL" in signals_sent,
        "signals_sent": signals_sent,
        "surviving_group_processes": len(survivors),
        "zero_survivor": not survivors,
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
    raw_capture_receipt_path: Path,
    scenario_id: str,
    seed: int,
    mission_id: str,
    run_root: Path,
    input_hashes: dict[str, str],
    input_artifacts: dict[str, dict[str, str]] | None = None,
    domain_id: int,
    playback_rate: float,
    timeout_seconds: float,
    reader=read_mcap_records,
    player=play_mcap,
) -> dict[str, Any]:
    expected_run_root = _non_link_path(run_root, "execution run root")
    context = bind_formal_context(repository_root, session_path, snapshot_path, runtime_binding_path)
    raw_capture = validate_raw_capture_receipt(
        repository_root=repository_root,
        raw_capture_receipt_path=raw_capture_receipt_path,
        formal_context=context,
        scenario_id=scenario_id,
        seed=seed,
        mission_id=mission_id,
        run_root=run_root,
    )
    if Path(raw_capture["mcap"]["path"]) != _non_link_path(bag, "MCAP"):
        raise ProductReplayError("requested MCAP differs from the raw capture receipt")
    if Path(raw_capture["source_metrics"]["path"]) != _non_link_path(source_metrics_path, "source metrics"):
        raise ProductReplayError("requested source metrics differs from the raw capture receipt")
    if raw_capture["run_root"] != str(expected_run_root):
        raise ProductReplayError("raw capture run root differs from replay run root")
    def assert_raw_artifacts_unchanged() -> None:
        if mcap_sha256(bag) != raw_capture["mcap"]["sha256"]:
            raise ProductReplayError("MCAP changed after raw capture receipt validation")
        if sha256(source_metrics_path) != raw_capture["source_metrics"]["sha256"]:
            raise ProductReplayError("source metrics changed after raw capture receipt validation")
    started_ns = int(context["session"]["started_epoch_ns"])
    for artifact, label in ((bag, "MCAP"), (source_metrics_path, "source metrics")):
        if artifact.stat().st_mtime_ns < started_ns:
            raise ProductReplayError(f"{label} predates the current formal session")
    # Validate the exact declared storage tree before a ROS reader can select
    # a subset of it and silently ignore injected/copied files.
    assert_raw_artifacts_unchanged()
    records = reader(bag)
    required_topics = set(PRODUCT_TOPICS) | {
        "/brush_controller/commands", GROUND_DIRT_STATUS_TOPIC,
        "/ground_truth/odom", "/localization/fused_odom",
    }
    missing_topics = sorted(required_topics - set(records["topic_types"]))
    source_metrics = read_object(source_metrics_path)
    recalculated = recalculate(records, source_metrics)
    playback = player(bag, domain_id=domain_id, rate=playback_rate, timeout_seconds=timeout_seconds)
    # Reader/player are untrusted execution boundaries; seal only the exact
    # raw-capture bytes that were validated before replay.
    assert_raw_artifacts_unchanged()
    counts = records["product_topic_counts"]
    checks = {
        "mcap_metadata_readable": True,
        "required_product_and_metric_topics_present": not missing_topics,
        "product_chain_observed": all(counts.get(topic, 0) > 0 for topic in PRODUCT_TOPICS),
        "coverage_recalculated_within_1_percent": (
            recalculated["coverage"]["brush_on_sample_count"] > 0
            and recalculated["coverage"]["relative_delta"] <= 0.01
        ),
        "localization_recalculated_within_1_percent": (
            recalculated["localization"]["matched_sample_count"] > 0
            and recalculated["localization"]["relative_delta"] is not None
            and recalculated["localization"]["relative_delta"] <= 0.01
        ),
        "ros2_bag_play_exit_zero": playback["exit_code"] == 0 and playback.get("timed_out") is False,
        "replay_uses_formal_safe_dds_domain": (
            playback.get("ros_domain_id") in FORMAL_SAFE_REPLAY_DOMAINS
            and playback.get("ros_localhost_only") is True
            and playback.get("rmw_implementation") == "rmw_cyclonedds_cpp"
            and playback.get("automatic_discovery_range") == "LOCALHOST"
        ),
        "replay_process_group_cleanup_proven": (
            isinstance(playback.get("process_group_cleanup"), dict)
            and playback["process_group_cleanup"].get("process_group_isolated") is True
            and playback["process_group_cleanup"].get("zero_survivor") is True
            and playback["process_group_cleanup"].get("surviving_group_processes") == 0
        ),
        "ros2_executable_matches_runtime_closure": (
            isinstance(context["runtime_closure_binding"].get("ros2_executable"), dict)
            and playback.get("ros2_executable") == context["runtime_closure_binding"].get("ros2_executable")
        ),
        "current_formal_session_snapshot_closure_bound": True,
    }
    return {
        "schema": SCHEMA,
        "status": "FORMAL_PRODUCT_MCAP_REPLAY_PASS" if all(checks.values()) else "FORMAL_PRODUCT_MCAP_REPLAY_BLOCKED",
        "pass": all(checks.values()),
        "producer": {"id": PRODUCER_ID, "sha256": sha256(repository_root / PRODUCER_ID)},
        "produced_epoch_ns": time.time_ns(),
        "bag": {
            "path": str(bag.resolve()),
            "sha256": mcap_sha256(bag),
            "semantic_sha256": mcap_semantic_sha256(bag),
        },
        "source_metrics": {"path": str(source_metrics_path.resolve()), "sha256": sha256(source_metrics_path)},
        "run_root": str(expected_run_root),
        "raw_capture": raw_capture,
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
    parser.add_argument("--raw-capture-receipt", type=Path, required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--mission-id", required=True)
    parser.add_argument("--run-root", type=Path, required=True)
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
            raw_capture_receipt_path=args.raw_capture_receipt.absolute(),
            scenario_id=args.scenario,
            seed=args.seed,
            mission_id=args.mission_id,
            run_root=args.run_root.absolute(),
            input_hashes=input_hashes,
            input_artifacts=input_artifacts,
            domain_id=args.ros_domain_id,
            playback_rate=args.playback_rate,
            timeout_seconds=args.timeout_seconds,
        )
        write_fresh_json(args.output.absolute(), report)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError):
        # Keep command-line failures structural and path/credential-free.
        print(json.dumps({"schema": SCHEMA, "status": "FORMAL_PRODUCT_MCAP_REPLAY_BLOCKED", "pass": False, "error": "product replay validation failed"}, sort_keys=True))
        return 2
    print(json.dumps(report, sort_keys=True))
    return 0 if report["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
