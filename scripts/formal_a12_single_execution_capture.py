#!/usr/bin/env python3
"""Observe one real A12 product execution as a bounded, fail-closed MP4.

This sidecar never starts a mission, publishes a command, or writes a product
receipt.  It observes an already-running ROS graph.  A video is published only
after the real RGB topic and both task-window signals have been verified at
runtime, an actual start-to-complete window was observed, and ffprobe accepts
the resulting MP4.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import queue
import re
import shutil
import signal
import stat
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PRODUCER_ID = "scripts/formal_a12_single_execution_capture.py"
RGB_TOPIC = "/sensors/front_rgbd/depth/image_rect_raw/image"
OPERATOR_START_TOPIC = "/product_demo/operator_start"
MISSION_COMPLETE_TOPIC = "/active_cleaning/mission_complete"
MINIMUM_VIDEO_BYTES = 100_000
RGB_SOURCE_CONTRACT = ROOT / "config/high_fidelity_vehicle/pre_urdf_contract.yaml"
RGB_SOURCE_CONTRACT_ID = "front_d435_depth"
RGB_SOURCE_CONTRACT_TOPIC = "/sensors/front_rgbd/depth/image_rect_raw"
# The formal forward D435 contract is 30 Hz.  Video output is 15 fps, but a
# source stream below 10 Hz is not sufficient evidence for that resampling.
RGB_SOURCE_NOMINAL_HZ = 30.0
MINIMUM_SOURCE_FRAME_RATE_HZ = 10.0
MAX_SOURCE_FRAME_GAP_SECONDS = 0.5
MAX_WINDOW_EDGE_FRESHNESS_SECONDS = 0.5
MINIMUM_UNIQUE_SOURCE_FRAMES = 20
EXPECTED_BAG_TOPICS = {
    "/brush_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/localization/fused_odom": "nav_msgs/msg/Odometry",
    "/ground_truth/odom": "nav_msgs/msg/Odometry",
    "/perception/open_vocab/dosod_boxes": "vision_msgs/msg/Detection2DArray",
    "/perception/garbage/targets": "sanitation_perception_interfaces/msg/GarbageTargetArray",
    "/active_cleaning/planner_status": "diagnostic_msgs/msg/DiagnosticArray",
    "/active_cleaning/grasp_result": "std_msgs/msg/String",
    OPERATOR_START_TOPIC: "std_msgs/msg/Bool",
    MISSION_COMPLETE_TOPIC: "std_msgs/msg/Bool",
}


class CaptureError(RuntimeError):
    """The observer could not establish or preserve a truthful video window."""


@dataclass(frozen=True)
class TopicRequirement:
    name: str
    message_type: str
    reliability: str
    durability: str
    require_publisher: bool = False


TOPIC_REQUIREMENTS = (
    TopicRequirement(RGB_TOPIC, "sensor_msgs/msg/Image", "BEST_EFFORT", "VOLATILE", True),
    TopicRequirement(OPERATOR_START_TOPIC, "std_msgs/msg/Bool", "RELIABLE", "TRANSIENT_LOCAL"),
    TopicRequirement(MISSION_COMPLETE_TOPIC, "std_msgs/msg/Bool", "RELIABLE", "TRANSIENT_LOCAL"),
)


def _utc_now() -> str:
    return datetime.now(UTC).isoformat().replace("+00:00", "Z")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _regular_non_link(path: Path, label: str) -> None:
    try:
        details = path.lstat()
    except FileNotFoundError as exc:
        raise CaptureError(f"{label} is missing: {path}") from exc
    if stat.S_ISLNK(details.st_mode) or not stat.S_ISREG(details.st_mode):
        raise CaptureError(f"{label} must be a regular non-link file: {path}")


def _existing_non_link_directory(path: Path, label: str) -> None:
    try:
        details = path.lstat()
    except FileNotFoundError as exc:
        raise CaptureError(f"{label} is missing: {path}") from exc
    if stat.S_ISLNK(details.st_mode) or not stat.S_ISDIR(details.st_mode):
        raise CaptureError(f"{label} must be a directory, not a link: {path}")


def _safe_outputs(run_root: Path, output: Path) -> tuple[Path, Path]:
    if not run_root.is_absolute():
        raise CaptureError("run root must be an absolute path")
    _existing_non_link_directory(run_root, "run root")
    run_root = run_root.resolve()
    if output.is_absolute() or ".." in output.parts or not output.parts:
        raise CaptureError("output must be a relative path below the run root")
    video = run_root.joinpath(output)
    if video.suffix.lower() != ".mp4":
        raise CaptureError("output must have an .mp4 extension")
    try:
        video.relative_to(run_root)
    except ValueError as exc:
        raise CaptureError("output escapes the run root") from exc
    parent = video.parent
    while parent != run_root:
        _existing_non_link_directory(parent, "output parent")
        parent = parent.parent
    if video.exists() or video.is_symlink():
        raise CaptureError(f"video output already exists: {video}")
    manifest = video.with_suffix(video.suffix + ".json")
    if manifest.exists() or manifest.is_symlink():
        raise CaptureError(f"video manifest output already exists: {manifest}")
    return video, manifest


def _safe_fresh_relative_file(run_root: Path, relative: Path, label: str) -> Path:
    if relative.is_absolute() or ".." in relative.parts or not relative.parts:
        raise CaptureError(f"{label} must be a relative path below the run root")
    path = run_root.resolve().joinpath(relative)
    try:
        path.relative_to(run_root.resolve())
    except ValueError as exc:
        raise CaptureError(f"{label} escapes the run root") from exc
    parent = path.parent
    while parent != run_root.resolve():
        _existing_non_link_directory(parent, f"{label} parent")
        parent = parent.parent
    if path.exists() or path.is_symlink():
        raise CaptureError(f"{label} already exists: {path}")
    return path


def _session_binding(path: Path) -> dict[str, Any]:
    _regular_non_link(path, "formal session")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        started = int(payload["started_epoch_ns"])
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise CaptureError("formal session is malformed") from exc
    if payload.get("report_id") != "tzcup_formal_final_acceptance_session_v1":
        raise CaptureError("wrong formal session identity")
    if payload.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING" or started <= 0:
        raise CaptureError("capture may start only while the formal session is RUNNING")
    return {
        "path": str(path.resolve()),
        "sha256_at_capture_start": _sha256(path),
        "report_id": payload["report_id"],
        "status_at_capture_start": payload["status"],
        "started_epoch_ns": started,
    }


def _normal_qos(value: str) -> str:
    return value.strip().upper().replace("RMW_QOS_POLICY_RELIABILITY_", "").replace(
        "RMW_QOS_POLICY_DURABILITY_", ""
    )


def _topic_info_contract(info: str, requirement: TopicRequirement) -> dict[str, Any]:
    type_match = re.search(r"^Type:\s*(\S+)\s*$", info, flags=re.MULTILINE)
    if not type_match or type_match.group(1) != requirement.message_type:
        actual = type_match.group(1) if type_match else None
        raise CaptureError(f"topic type mismatch for {requirement.name}: expected {requirement.message_type}, got {actual}")
    count_matches = dict(re.findall(r"^(Publisher|Subscription) count:\s*(\d+)\s*$", info, flags=re.MULTILINE))
    publisher_count = int(count_matches.get("Publisher", "0"))
    subscription_count = int(count_matches.get("Subscription", "0"))
    if publisher_count + subscription_count == 0:
        raise CaptureError(f"topic has no live endpoint: {requirement.name}")
    if requirement.require_publisher and publisher_count == 0:
        raise CaptureError(f"topic has no live publisher: {requirement.name}")
    profiles = [
        (_normal_qos(reliability), _normal_qos(durability))
        for reliability, durability in re.findall(
            r"Reliability:\s*([^\r\n]+).*?Durability:\s*([^\r\n]+)", info, flags=re.DOTALL
        )
    ]
    expected = (requirement.reliability, requirement.durability)
    if expected not in profiles:
        raise CaptureError(f"topic QoS mismatch for {requirement.name}: expected {expected}, observed {profiles}")
    return {
        "topic": requirement.name,
        "type": requirement.message_type,
        "publisher_count": publisher_count,
        "subscription_count": subscription_count,
        "matching_qos": {"reliability": expected[0], "durability": expected[1]},
    }


def _run_checked(command: list[str], timeout_seconds: float) -> subprocess.CompletedProcess[str]:
    try:
        completed = subprocess.run(command, capture_output=True, text=True, check=False, timeout=timeout_seconds)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise CaptureError(f"command failed: {command[0]}: {exc}") from exc
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise CaptureError(f"command rejected ({command[0]}): {detail[:512]}")
    return completed


def verify_runtime_topics(ros2: str, timeout_seconds: float = 15.0) -> list[dict[str, Any]]:
    """Read the live ROS graph; static configuration is intentionally insufficient."""
    if not shutil.which(ros2):
        raise CaptureError(f"ros2 executable is unavailable: {ros2}")
    verified: list[dict[str, Any]] = []
    for requirement in TOPIC_REQUIREMENTS:
        completed = _run_checked([ros2, "topic", "info", requirement.name, "--verbose"], timeout_seconds)
        verified.append(_topic_info_contract(completed.stdout, requirement))
    return verified


def _ffprobe_video(video: Path, ffprobe: str, timeout_seconds: float = 30.0) -> dict[str, Any]:
    if not shutil.which(ffprobe):
        raise CaptureError(f"ffprobe executable is unavailable: {ffprobe}")
    completed = _run_checked(
        [
            ffprobe, "-v", "error", "-count_frames", "-select_streams", "v:0",
            "-show_entries", "stream=codec_name,width,height,nb_frames,nb_read_frames:format=duration",
            "-of", "json", str(video),
        ],
        timeout_seconds,
    )
    try:
        payload = json.loads(completed.stdout)
        stream = payload["streams"][0]
        duration = float(payload["format"]["duration"])
        frame_text = stream.get("nb_read_frames") or stream.get("nb_frames")
        frame_count = int(frame_text)
        width, height = int(stream["width"]), int(stream["height"])
    except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise CaptureError("ffprobe returned incomplete MP4 metadata") from exc
    if duration <= 0 or frame_count <= 0 or width <= 0 or height <= 0:
        raise CaptureError("ffprobe reported a non-positive MP4 duration, frame count, or dimensions")
    return {
        "tool": ffprobe,
        "codec_name": stream.get("codec_name"),
        "width": width,
        "height": height,
        "duration_seconds": duration,
        "frame_count": frame_count,
    }


def _bag_storage_snapshot(path: Path) -> dict[str, int]:
    """Return only real MCAP storage files; metadata alone proves no samples."""
    _existing_non_link_directory(path, "MCAP output directory")
    rows: dict[str, int] = {}
    for item in path.iterdir():
        if item.is_symlink() or not item.is_file() or item.suffix.lower() != ".mcap":
            continue
        size = item.stat().st_size
        if size > 0:
            rows[item.name] = size
    return rows


def _recorder_subscription_contract(info: str) -> dict[str, str]:
    """Parse only the recorder's Subscribers section, never a name-only hit."""
    match = re.search(
        r"^Subscribers:\s*$\r?\n(?P<body>(?:^[ \t]+.*(?:\r?\n|$))*)",
        info,
        flags=re.MULTILINE,
    )
    if match is None:
        raise CaptureError("trusted recorder node info has no Subscribers section")
    observed: dict[str, str] = {}
    for line in match["body"].splitlines():
        topic_match = re.match(r"^\s*(/\S+):\s*(\S+)\s*$", line)
        if topic_match is not None:
            observed[topic_match.group(1)] = topic_match.group(2)
    return observed


def _wait_trusted_recorder_ready(ros2: str, bag_path: Path, timeout_seconds: float) -> dict[str, Any]:
    """Require the fixed recorder node and every expected subscription live."""
    if not shutil.which(ros2):
        raise CaptureError(f"ros2 executable is unavailable: {ros2}")
    deadline = time.monotonic() + timeout_seconds
    last_error = "recorder node was not observed"
    while time.monotonic() < deadline:
        try:
            completed = _run_checked([ros2, "node", "info", "/a12_trusted_gt_recorder"], min(10.0, timeout_seconds))
            observed_subscriptions = _recorder_subscription_contract(completed.stdout)
            wrong = {
                topic: observed_subscriptions.get(topic)
                for topic, expected in EXPECTED_BAG_TOPICS.items()
                if observed_subscriptions.get(topic) != expected
            }
            if wrong:
                last_error = f"trusted recorder subscription type contract failed: {wrong}"
            else:
                before = _bag_storage_snapshot(bag_path)
                if not before:
                    last_error = "trusted recorder has no non-empty MCAP storage file"
                    time.sleep(0.1)
                    continue
                # ``metadata.yaml`` is normally finalized only after SIGINT.
                # During READY, require an actual storage file to gain bytes,
                # which proves the subscribed recorder is receiving messages.
                time.sleep(0.25)
                after = _bag_storage_snapshot(bag_path)
                grew = {
                    name: {"before_bytes": size, "after_bytes": after.get(name, 0)}
                    for name, size in before.items() if after.get(name, 0) > size
                }
                if not grew:
                    last_error = "trusted recorder MCAP storage did not grow while READY was checked"
                    continue
                return {
                    "node": "/a12_trusted_gt_recorder",
                    "expected_topics": EXPECTED_BAG_TOPICS,
                    "observed_subscriptions": {
                        topic: observed_subscriptions[topic] for topic in EXPECTED_BAG_TOPICS
                    },
                    "storage_growth": grew,
                    "metadata_present_at_ready": (bag_path / "metadata.yaml").is_file(),
                    "readiness_semantics": (
                        "all recorder subscriptions/type contracts plus non-empty MCAP storage "
                        "growth; per-topic message counts are verified only in final metadata"
                    ),
                }
        except CaptureError as exc:
            last_error = str(exc)
        time.sleep(0.1)
    raise CaptureError(f"trusted recorder did not become ready: {last_error}")


def _inspect_bag_metadata(bag_path: Path) -> dict[str, Any]:
    metadata = bag_path / "metadata.yaml"
    _regular_non_link(metadata, "MCAP metadata")
    try:
        import yaml
    except ImportError as exc:
        raise CaptureError("MCAP metadata parser is unavailable") from exc
    try:
        root = yaml.safe_load(metadata.read_text(encoding="utf-8"))
        rows = root["rosbag2_bagfile_information"]["topics_with_message_count"]
    except (KeyError, TypeError, ValueError, OSError, yaml.YAMLError) as exc:
        raise CaptureError("MCAP metadata is unreadable") from exc
    observed: dict[str, dict[str, Any]] = {}
    for row in rows:
        if not isinstance(row, dict):
            continue
        topic = row.get("topic_metadata", {})
        if isinstance(topic, dict) and isinstance(topic.get("name"), str):
            observed[topic["name"]] = {"type": topic.get("type"), "count": row.get("message_count")}
    missing = sorted(topic for topic in EXPECTED_BAG_TOPICS if topic not in observed)
    wrong = {
        topic: observed.get(topic) for topic, expected in EXPECTED_BAG_TOPICS.items()
        if topic in observed and (observed[topic].get("type") != expected or not isinstance(observed[topic].get("count"), int) or observed[topic]["count"] <= 0)
    }
    if missing or wrong:
        raise CaptureError(f"MCAP topic/type/count validation failed: missing={missing}, wrong={wrong}")
    return {"metadata_path": str(metadata.resolve()), "topics": {topic: observed[topic] for topic in EXPECTED_BAG_TOPICS}}


def verify_encoder(run_root: Path, ffprobe: str) -> dict[str, Any]:
    """Prove this runtime can encode and read an MP4 before observing a task."""
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise CaptureError(f"OpenCV encoder dependencies are unavailable: {exc}") from exc
    probe = run_root / f".a12-encoder-probe-{uuid.uuid4().hex}.mp4"
    try:
        writer = cv2.VideoWriter(str(probe), cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (320, 240))
        if not writer.isOpened():
            raise CaptureError("OpenCV could not open the mp4v encoder")
        for index in range(12):
            frame = np.full((240, 320, 3), index * 19, dtype=np.uint8)
            cv2.putText(frame, f"a12 encoder probe {index}", (8, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            writer.write(frame)
        writer.release()
        _regular_non_link(probe, "encoder probe")
        return _ffprobe_video(probe, ffprobe)
    finally:
        if "writer" in locals():
            writer.release()
        if probe.exists() and not probe.is_symlink():
            probe.unlink()


def _header_timestamp_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    sec, nanosec = getattr(stamp, "sec", None), getattr(stamp, "nanosec", None)
    if isinstance(sec, int) and isinstance(nanosec, int) and sec >= 0 and nanosec >= 0:
        return sec * 1_000_000_000 + nanosec
    return None


@dataclass
class FrameItem:
    message: Any
    received_epoch_ns: int
    header_timestamp_ns: int | None


@dataclass
class WindowState:
    queue_size: int
    started_epoch_ns: int | None = None
    completed_epoch_ns: int | None = None
    start_signal_count: int = 0
    complete_signal_count: int = 0
    received_frame_count: int = 0
    accepted_frame_count: int = 0
    dropped_frame_count: int = 0
    resample_skipped_frame_count: int = 0
    encoded_video_frame_count: int = 0
    source_receive_gap_max_ns: int = 0
    source_stamp_gap_max_ns: int = 0
    duplicate_or_reversed_stamp_count: int = 0
    ignored_before_start_count: int = 0
    ignored_after_complete_count: int = 0
    first_frame_epoch_ns: int | None = None
    last_frame_epoch_ns: int | None = None
    first_header_timestamp_ns: int | None = None
    last_header_timestamp_ns: int | None = None
    frames: queue.Queue[FrameItem] = field(init=False)

    def __post_init__(self) -> None:
        if self.queue_size < 1:
            raise CaptureError("frame queue size must be at least one")
        self.frames = queue.Queue(maxsize=self.queue_size)

    @property
    def started(self) -> bool:
        return self.started_epoch_ns is not None

    @property
    def completed(self) -> bool:
        return self.completed_epoch_ns is not None

    def on_operator_start(self, value: bool, now_ns: int) -> None:
        if value:
            self.start_signal_count += 1
            if not self.started:
                self.started_epoch_ns = now_ns

    def on_mission_complete(self, value: bool, now_ns: int) -> None:
        if value:
            self.complete_signal_count += 1
            if not self.started:
                raise CaptureError("mission_complete arrived before operator_start")
            if not self.completed:
                self.completed_epoch_ns = now_ns

    def on_image(self, message: Any, now_ns: int) -> None:
        self.received_frame_count += 1
        if not self.started:
            self.ignored_before_start_count += 1
            return
        if self.completed:
            self.ignored_after_complete_count += 1
            return
        item = FrameItem(message, now_ns, _header_timestamp_ns(message))
        try:
            self.frames.put_nowait(item)
        except queue.Full:
            self.dropped_frame_count += 1

    def accepted(self, item: FrameItem, max_gap_ns: int) -> None:
        if item.header_timestamp_ns is None or item.header_timestamp_ns <= 0:
            raise CaptureError("RGB source frame has no positive header timestamp")
        if self.last_frame_epoch_ns is not None:
            receive_gap = item.received_epoch_ns - self.last_frame_epoch_ns
            assert self.last_header_timestamp_ns is not None
            stamp_gap = item.header_timestamp_ns - self.last_header_timestamp_ns
            if receive_gap <= 0 or stamp_gap <= 0:
                self.duplicate_or_reversed_stamp_count += 1
                raise CaptureError("RGB source header timestamp repeated or moved backwards")
            self.source_receive_gap_max_ns = max(self.source_receive_gap_max_ns, receive_gap)
            self.source_stamp_gap_max_ns = max(self.source_stamp_gap_max_ns, stamp_gap)
            if receive_gap > max_gap_ns or stamp_gap > max_gap_ns:
                raise CaptureError("RGB source frame gap exceeds the formal freshness bound")
        self.accepted_frame_count += 1
        self.first_frame_epoch_ns = self.first_frame_epoch_ns or item.received_epoch_ns
        self.last_frame_epoch_ns = item.received_epoch_ns
        self.first_header_timestamp_ns = self.first_header_timestamp_ns or item.header_timestamp_ns
        self.last_header_timestamp_ns = item.header_timestamp_ns


def _resample_index(start_ns: int, sample_ns: int, fps: float) -> int:
    if sample_ns < start_ns:
        raise CaptureError("RGB frame timestamp predates operator_start")
    return int((sample_ns - start_ns) * fps / 1_000_000_000)


def _load_formal_rgb_source_contract(path: Path = RGB_SOURCE_CONTRACT) -> dict[str, Any]:
    """Parse the frozen source contract; constants alone are not authority."""
    _regular_non_link(path, "formal RGB source contract")
    try:
        import yaml
    except ImportError as exc:
        raise CaptureError("formal RGB source contract parser is unavailable") from exc
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
        rows = payload["sensor_contracts"]
    except (OSError, TypeError, KeyError, yaml.YAMLError) as exc:
        raise CaptureError("formal RGB source contract is malformed") from exc
    if not isinstance(rows, list):
        raise CaptureError("formal RGB source contract has no sensor_contracts list")
    matches = [
        row for row in rows
        if isinstance(row, dict) and row.get("id") == RGB_SOURCE_CONTRACT_ID
    ]
    if len(matches) != 1:
        raise CaptureError("formal RGB source contract must contain exactly one front_d435_depth sensor")
    row = matches[0]
    topic, rate = row.get("topic"), row.get("update_rate_hz")
    if topic != RGB_SOURCE_CONTRACT_TOPIC:
        raise CaptureError("formal RGB source contract topic differs from front D435 binding")
    if type(rate) not in (int, float) or float(rate) != RGB_SOURCE_NOMINAL_HZ:
        raise CaptureError("formal RGB source contract update rate differs from 30.0 Hz")
    return {
        "path": str(path.resolve()), "sha256": _sha256(path),
        "id": RGB_SOURCE_CONTRACT_ID, "topic": topic, "update_rate_hz": float(rate),
    }


def _validate_source_frame_quality(state: WindowState, args: argparse.Namespace) -> dict[str, Any]:
    """Prove an MP4 cadence is backed by timely, distinct physical RGB frames."""
    formal_contract = _load_formal_rgb_source_contract()
    if state.started_epoch_ns is None or state.completed_epoch_ns is None or state.first_frame_epoch_ns is None or state.last_frame_epoch_ns is None:
        raise CaptureError("RGB source window has incomplete timing")
    duration_ns = state.completed_epoch_ns - state.started_epoch_ns
    if duration_ns <= 0:
        raise CaptureError("task window duration must be positive")
    edge_ns = int(args.max_window_edge_freshness_seconds * 1_000_000_000)
    start_lag_ns = state.first_frame_epoch_ns - state.started_epoch_ns
    terminal_lag_ns = state.completed_epoch_ns - state.last_frame_epoch_ns
    if start_lag_ns < 0 or start_lag_ns > edge_ns:
        raise CaptureError("first RGB frame is not fresh at operator_start")
    if terminal_lag_ns < 0 or terminal_lag_ns > edge_ns:
        raise CaptureError("last RGB frame is not fresh at mission_complete")
    duration_seconds = duration_ns / 1_000_000_000
    required_count = max(
        args.minimum_unique_source_frames,
        int(duration_seconds * args.minimum_source_frame_rate_hz + 0.999999),
    )
    observed_rate_hz = state.accepted_frame_count / duration_seconds
    if state.accepted_frame_count < required_count or observed_rate_hz < args.minimum_source_frame_rate_hz:
        raise CaptureError(
            "RGB source frame count/rate is below the formal minimum "
            f"(observed={state.accepted_frame_count}/{observed_rate_hz:.6f}Hz, "
            f"required={required_count}/{args.minimum_source_frame_rate_hz:.6f}Hz)"
        )
    return {
        "formal_contract": formal_contract,
        "nominal_rate_hz": RGB_SOURCE_NOMINAL_HZ,
        "minimum_rate_hz": args.minimum_source_frame_rate_hz,
        "observed_rate_hz": observed_rate_hz,
        "required_unique_source_frames": required_count,
        "accepted_unique_source_frames": state.accepted_frame_count,
        "max_receive_gap_seconds": state.source_receive_gap_max_ns / 1_000_000_000,
        "max_header_stamp_gap_seconds": state.source_stamp_gap_max_ns / 1_000_000_000,
        "allowed_gap_seconds": args.max_source_frame_gap_seconds,
        "operator_start_to_first_frame_seconds": start_lag_ns / 1_000_000_000,
        "last_frame_to_mission_complete_seconds": terminal_lag_ns / 1_000_000_000,
        "edge_freshness_limit_seconds": args.max_window_edge_freshness_seconds,
        "duplicate_or_reversed_stamp_count": state.duplicate_or_reversed_stamp_count,
    }


def _atomic_no_replace(source: Path, destination: Path) -> None:
    _regular_non_link(source, "temporary output")
    if destination.exists() or destination.is_symlink():
        raise CaptureError(f"output was concurrently created: {destination}")
    try:
        os.link(source, destination)
    except FileExistsError as exc:
        raise CaptureError(f"output was concurrently created: {destination}") from exc
    except OSError as exc:
        raise CaptureError(f"could not atomically publish no-replace output: {exc}") from exc


def _write_manifest_no_replace(path: Path, manifest: dict[str, Any]) -> None:
    temporary = path.parent / f".{path.name}.{uuid.uuid4().hex}.part"
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as handle:
            json.dump(manifest, handle, sort_keys=True, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        _atomic_no_replace(temporary, path)
    finally:
        if temporary.exists() and not temporary.is_symlink():
            temporary.unlink()


def _source_identity() -> dict[str, str]:
    source = ROOT / PRODUCER_ID
    return {"id": PRODUCER_ID, "sha256": _sha256(source)}


def capture(args: argparse.Namespace) -> dict[str, Any]:
    video, manifest_path = _safe_outputs(args.run_root, args.output)
    ready_path = _safe_fresh_relative_file(args.run_root, args.video_ready_file, "video-worker readiness")
    session = _session_binding(args.session_status)
    topic_observations = verify_runtime_topics(args.ros2, args.command_timeout_seconds)
    encoder_probe = verify_encoder(args.run_root.resolve(), args.ffprobe)
    try:
        import cv2
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool
    except ImportError as exc:
        raise CaptureError(f"ROS image capture dependencies are unavailable: {exc}") from exc

    state = WindowState(args.queue_size)
    temporary_video = video.parent / f".{video.name}.{uuid.uuid4().hex}.part.mp4"
    writer: Any | None = None
    writer_size: tuple[int, int] | None = None
    last_image: Any | None = None
    last_resample_index = -1
    bridge = CvBridge()
    node: Any | None = None
    rclpy.init(args=None)
    try:
        from rclpy.node import Node

        node = Node("formal_a12_single_execution_capture_observer")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Image, RGB_TOPIC, lambda message: state.on_image(message, time.time_ns()), qos_profile_sensor_data)
        node.create_subscription(Bool, OPERATOR_START_TOPIC, lambda message: state.on_operator_start(bool(message.data), time.time_ns()), latched)
        node.create_subscription(Bool, MISSION_COMPLETE_TOPIC, lambda message: state.on_mission_complete(bool(message.data), time.time_ns()), latched)
        _write_manifest_no_replace(ready_path, {
            "schema": "tzcup.a12.video_capture_ready.v1",
            "status": "A12_VIDEO_CAPTURE_READY",
            "producer": _source_identity(),
            "created_utc": _utc_now(),
            "run_root": str(args.run_root.resolve()),
            "runtime_id": args.runtime_id,
            "formal_session": session,
            "supervision": {"pid": os.getpid(), "pgid": os.getpgrp(), "sid": os.getsid(0)},
            "video_output": str(video),
            "runtime_topic_observations": topic_observations,
            "encoder_probe": encoder_probe,
        })

        def encode(item: FrameItem) -> None:
            """Resample host-receipt timestamps onto the fixed MP4 cadence."""
            nonlocal writer, writer_size, last_image, last_resample_index
            image = bridge.imgmsg_to_cv2(item.message, desired_encoding="bgr8")
            if getattr(image, "ndim", 0) != 3 or image.shape[2] != 3:
                raise CaptureError("RGB topic did not yield a three-channel BGR frame")
            height, width = int(image.shape[0]), int(image.shape[1])
            if height <= 0 or width <= 0:
                raise CaptureError("RGB topic yielded an empty frame")
            if writer is None:
                writer = cv2.VideoWriter(str(temporary_video), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (width, height))
                if not writer.isOpened():
                    raise CaptureError("OpenCV could not open the execution MP4 encoder")
                writer_size = (width, height)
            elif writer_size != (width, height):
                raise CaptureError("RGB frame dimensions changed during the task window")
            state.accepted(item, int(args.max_source_frame_gap_seconds * 1_000_000_000))
            assert state.started_epoch_ns is not None
            index = _resample_index(state.started_epoch_ns, item.received_epoch_ns, args.fps)
            if last_image is None:
                # The first image may cover only the short independently
                # checked operator-start freshness interval; no synthetic
                # scene or unbounded frozen prefix is introduced.
                if item.received_epoch_ns - state.started_epoch_ns > int(args.max_window_edge_freshness_seconds * 1_000_000_000):
                    raise CaptureError("first RGB frame is not fresh at operator_start")
                for _ in range(index + 1):
                    writer.write(image)
                    state.encoded_video_frame_count += 1
                last_image, last_resample_index = image, index
                return
            if index <= last_resample_index:
                state.resample_skipped_frame_count += 1
                return
            while last_resample_index + 1 < index:
                writer.write(last_image)
                state.encoded_video_frame_count += 1
                last_resample_index += 1
            writer.write(image)
            state.encoded_video_frame_count += 1
            last_image, last_resample_index = image, index

        deadline = time.monotonic() + args.window_timeout_seconds
        while not state.completed:
            if time.monotonic() >= deadline:
                raise CaptureError("timed out before a complete operator_start-to-mission_complete window")
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            while not state.frames.empty():
                encode(state.frames.get_nowait())
        while not state.frames.empty():
            encode(state.frames.get_nowait())
        if writer is None or last_image is None or state.started_epoch_ns is None or state.completed_epoch_ns is None:
            raise CaptureError("no RGB frame arrived during the task window")
        source_quality = _validate_source_frame_quality(state, args)
        terminal_index = _resample_index(state.started_epoch_ns, state.completed_epoch_ns, args.fps)
        if state.completed_epoch_ns - state.last_frame_epoch_ns > int(args.max_window_edge_freshness_seconds * 1_000_000_000):
            raise CaptureError("last RGB frame is not fresh at mission_complete")
        while last_resample_index < terminal_index:
            # The terminal extension is bounded by the same independently
            # verified freshness gate above, never a single-frame freeze.
            writer.write(last_image)
            state.encoded_video_frame_count += 1
            last_resample_index += 1
    finally:
        if writer is not None:
            writer.release()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    try:
        if not state.started or not state.completed or state.accepted_frame_count < 1:
            raise CaptureError("task window did not contain an accepted RGB frame")
        _regular_non_link(temporary_video, "temporary execution video")
        if temporary_video.stat().st_size < args.minimum_video_bytes:
            raise CaptureError(f"execution MP4 is below {args.minimum_video_bytes} bytes")
        video_audit = _ffprobe_video(temporary_video, args.ffprobe)
        if video_audit["frame_count"] < 1:
            raise CaptureError("ffprobe did not find a video frame")
        if abs(video_audit["frame_count"] - state.encoded_video_frame_count) > 1:
            raise CaptureError("ffprobe frame count differs from the resampled encoder count")
        task_duration = (state.completed_epoch_ns - state.started_epoch_ns) / 1_000_000_000
        if abs(video_audit["duration_seconds"] - task_duration) > max(2.0 / args.fps, 0.5):
            raise CaptureError("MP4 duration does not match the observed task window")
        source_window_count = state.accepted_frame_count + state.dropped_frame_count
        drop_ratio = state.dropped_frame_count / source_window_count if source_window_count else 1.0
        if drop_ratio > args.max_drop_ratio:
            raise CaptureError(f"RGB queue drop ratio {drop_ratio:.6f} exceeds {args.max_drop_ratio:.6f}")
        manifest = {
            "schema": "tzcup.a12.video_observation.v1",
            "status": "A12_VIDEO_OBSERVED",
            "producer": _source_identity(),
            "created_utc": _utc_now(),
            "run_root": str(args.run_root.resolve()),
            "runtime_id": args.runtime_id,
            "formal_session": session,
            "supervision": {
                "pid": os.getpid(), "pgid": os.getpgrp(), "sid": os.getsid(0),
                "ready_file": str(ready_path),
            },
            "video": {"path": str(video), "sha256": _sha256(temporary_video), "size_bytes": temporary_video.stat().st_size, "ffprobe": video_audit},
            "window": {
                "operator_start_epoch_ns": state.started_epoch_ns,
                "mission_complete_epoch_ns": state.completed_epoch_ns,
                "first_frame_epoch_ns": state.first_frame_epoch_ns,
                "last_frame_epoch_ns": state.last_frame_epoch_ns,
                "first_header_timestamp_ns": state.first_header_timestamp_ns,
                "last_header_timestamp_ns": state.last_header_timestamp_ns,
                "duration_seconds": task_duration,
            },
            "frames": {
                "received": state.received_frame_count,
                "accepted": state.accepted_frame_count,
                "dropped_queue_full": state.dropped_frame_count,
                "resample_skipped": state.resample_skipped_frame_count,
                "encoded_video_frames": state.encoded_video_frame_count,
                "queue_drop_ratio": drop_ratio,
                "ignored_before_start": state.ignored_before_start_count,
                "ignored_after_complete": state.ignored_after_complete_count,
                "queue_bound": args.queue_size,
            },
            "source_frame_quality": source_quality,
            "runtime_topic_observations": topic_observations,
            "encoder_probe": encoder_probe,
        }
        published_video = False
        try:
            _atomic_no_replace(temporary_video, video)
            published_video = True
            _write_manifest_no_replace(manifest_path, manifest)
        except Exception:
            # Do not leave an unbound MP4 that could be mistaken for a sealed
            # observation if its hash sidecar could not be published.  The
            # inode check prevents deleting a concurrently replaced pathname.
            if published_video:
                try:
                    if os.path.samestat(video.stat(), temporary_video.stat()):
                        video.unlink()
                except OSError:
                    pass
            raise
        return manifest
    finally:
        if temporary_video.exists() and not temporary_video.is_symlink():
            temporary_video.unlink()


def _child_snapshot(
    process: subprocess.Popen[Any], role: str, command: list[str], timeout_seconds: float,
) -> dict[str, Any]:
    """Snapshot OS-derived child identity; callers never nominate a PGID."""
    try:
        return {
            "role": role, "pid": process.pid, "pgid": os.getpgid(process.pid),
            "argv": command, "timeout_seconds": timeout_seconds, "returncode": process.poll(),
        }
    except ProcessLookupError:
        return {
            "role": role, "pid": process.pid, "pgid": None,
            "argv": command, "timeout_seconds": timeout_seconds, "returncode": process.poll(),
        }


def _stop_child(
    process: subprocess.Popen[Any], role: str, command: list[str], timeout_seconds: float,
) -> dict[str, Any]:
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=10)
    row = _child_snapshot(process, role, command, timeout_seconds)
    if row["returncode"] is None:
        raise CaptureError(f"{role} survived bounded supervisor cleanup")
    return row


def _same_pgid_survivors(pgid: int) -> list[int]:
    """Return live Linux processes still sharing the private supervisor PGID.

    A ``Z`` entry is already dead but waits for its direct parent to reap it;
    it is not a survivor.  Supervisor-owned ``Popen`` children are always
    signalled and waited before this census, so their zombies are reaped.
    """
    proc = Path("/proc")
    if not proc.is_dir():
        raise CaptureError("A12 supervisor requires Linux /proc for zero-survivor census")
    survivors: list[int] = []
    for entry in proc.iterdir():
        if not entry.name.isdecimal() or int(entry.name) == os.getpid():
            continue
        try:
            # ``comm`` is parenthesized and may itself contain spaces or `)`;
            # parse from its final `)` before reading state/ppid/pgrp fields.
            raw = (entry / "stat").read_text(encoding="utf-8")
            closing = raw.rfind(")")
            fields = raw[closing + 2:].split() if closing >= 0 else []
            if len(fields) > 2 and fields[0] != "Z" and int(fields[2]) == pgid:
                survivors.append(int(entry.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
    return sorted(survivors)


def _cleanup_private_pgid(pgid: int) -> list[int]:
    """Terminate every descendant in this private supervisor group, never self."""
    if pgid != os.getpgrp() or pgid != os.getpid():
        raise CaptureError("refusing to clean a non-private A12 process group")
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for pid in _same_pgid_survivors(pgid):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                continue
        deadline = time.monotonic() + 5.0
        while _same_pgid_survivors(pgid) and time.monotonic() < deadline:
            time.sleep(0.05)
    return _same_pgid_survivors(pgid)


def _wait_for_fresh_inputs(paths: dict[str, Path], session_started_ns: int, timeout_seconds: float) -> dict[str, dict[str, str]]:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        ready = True
        for path in paths.values():
            if not path.exists() or path.is_symlink():
                ready = False
                break
        if ready:
            rows: dict[str, dict[str, str]] = {}
            for name, path in paths.items():
                _regular_non_link(path, name)
                if path.stat().st_mtime_ns < session_started_ns:
                    raise CaptureError(f"{name} predates the current formal session")
                rows[name] = {"path": str(path.resolve()), "sha256": _sha256(path)}
            return rows
        time.sleep(0.25)
    missing = sorted(name for name, path in paths.items() if not path.exists())
    raise CaptureError(f"timed out waiting for required A12 finalization inputs: {missing}")


def _bag_command(args: argparse.Namespace) -> list[str]:
    return [
        args.ros2, "bag", "record", "--storage", "mcap", "--node-name", "a12_trusted_gt_recorder",
        "--output", str(args.run_root.resolve() / args.bag_output),
        *EXPECTED_BAG_TOPICS.keys(),
    ]


def _worker_command(args: argparse.Namespace) -> list[str]:
    return [
        sys.executable, str(Path(__file__).resolve()), "--worker-video",
        "--run-root", str(args.run_root), "--output", str(args.output),
        "--video-ready-file", str(args.video_ready_file), "--session-status", str(args.session_status),
        "--runtime-id", args.runtime_id, "--ros2", args.ros2, "--ffprobe", args.ffprobe,
        "--fps", str(args.fps), "--queue-size", str(args.queue_size),
        "--max-drop-ratio", str(args.max_drop_ratio),
        "--minimum-source-frame-rate-hz", str(args.minimum_source_frame_rate_hz),
        "--max-source-frame-gap-seconds", str(args.max_source_frame_gap_seconds),
        "--max-window-edge-freshness-seconds", str(args.max_window_edge_freshness_seconds),
        "--minimum-unique-source-frames", str(args.minimum_unique_source_frames),
        "--minimum-video-bytes", str(args.minimum_video_bytes),
        "--window-timeout-seconds", str(args.window_timeout_seconds),
        "--command-timeout-seconds", str(args.command_timeout_seconds),
    ]


def _runtime_context() -> dict[str, Any]:
    """Bind the supervisor to the live DDS/Gazebo isolation, not a caller claim."""
    ros_domain = os.environ.get("ROS_DOMAIN_ID")
    gz_partition = os.environ.get("GZ_PARTITION")
    if ros_domain is None or not ros_domain.isdecimal() or gz_partition is None or not gz_partition.startswith("tzcup-single-episode-"):
        raise CaptureError("A12 supervisor requires live ROS_DOMAIN_ID and isolated GZ_PARTITION")
    return {
        "ros_domain_id": int(ros_domain),
        "gz_partition": gz_partition,
        "cwd": str(Path.cwd().resolve()),
    }


def _supervisor_readiness(args: argparse.Namespace, session: dict[str, Any], bag: subprocess.Popen[Any], worker: subprocess.Popen[Any]) -> dict[str, Any]:
    pgid, sid = os.getpgrp(), os.getsid(0)
    bag_command = _bag_command(args)
    worker_command = _worker_command(args)
    bag_row = _child_snapshot(bag, "trusted_gt_recorder", bag_command, args.window_timeout_seconds)
    worker_row = _child_snapshot(worker, "video_worker", worker_command, args.window_timeout_seconds)
    if pgid != os.getpid() or sid != os.getpid() or bag_row["pgid"] != pgid or worker_row["pgid"] != pgid:
        raise CaptureError("A12 bag recorder and video worker must share the private supervisor PGID/SID")
    return {
        "schema": "tzcup.a12.capture_supervisor_ready.v1",
        "status": "A12_CAPTURE_SUPERVISOR_READY",
        "producer": _source_identity(),
        "created_utc": _utc_now(),
        "run_root": str(args.run_root.resolve()),
        "runtime_id": args.runtime_id,
        "runtime_context": _runtime_context(),
        "formal_session": session,
        "supervisor": {"pid": os.getpid(), "pgid": pgid, "sid": sid},
        "trusted_gt_recorder": {"node": "/a12_trusted_gt_recorder", "pid": bag_row["pid"], "pgid": pgid},
        "video_worker": {"pid": worker_row["pid"], "pgid": pgid, "ready_file": str(args.run_root.resolve() / args.video_ready_file)},
        "expected_outputs": {
            "video": str(args.run_root.resolve() / args.output),
            "mcap": str(args.run_root.resolve() / args.bag_output),
            "collector_raw": str(args.run_root.resolve() / args.collector_raw),
            "source_metrics": str(args.run_root.resolve() / args.source_metrics),
            "execution_receipt": str(args.run_root.resolve() / args.execution_receipt),
        },
    }


def supervisor(args: argparse.Namespace) -> dict[str, Any]:
    if os.getpid() != os.getpgrp() or os.getpid() != os.getsid(0):
        if not shutil.which("setsid"):
            raise CaptureError("setsid is unavailable; refusing an uncontained A12 supervisor")
        os.execvp("setsid", ["setsid", sys.executable, str(Path(__file__).resolve()), "--supervisor-child", *sys.argv[1:]])
    _safe_outputs(args.run_root, args.output)
    for path, label in ((args.ready_file, "supervisor readiness"), (args.video_ready_file, "video-worker readiness"), (args.supervisor_status_file, "supervisor terminal status"), (args.execution_receipt, "execution receipt")):
        _safe_fresh_relative_file(args.run_root, path, label)
    bag_path = _safe_fresh_relative_file(args.run_root, args.bag_output, "MCAP output")
    if bag_path.suffix:
        raise CaptureError("MCAP output must be a fresh directory path")
    session = _session_binding(args.session_status)
    worker_log = _safe_fresh_relative_file(args.run_root, Path("a12_video_worker.log"), "video worker log")
    bag_log = _safe_fresh_relative_file(args.run_root, Path("a12_trusted_gt_recorder.log"), "trusted GT recorder log")
    worker: subprocess.Popen[Any] | None = None
    bag: subprocess.Popen[Any] | None = None
    bag_command = _bag_command(args)
    worker_command = _worker_command(args)
    child_lifecycles: list[dict[str, Any]] = []
    status: dict[str, Any] = {
        "schema": "tzcup.a12.capture_supervisor_status.v1", "producer": _source_identity(),
        "run_root": str(args.run_root.resolve()), "runtime_id": args.runtime_id,
        "runtime_context": _runtime_context(), "formal_session": session,
        "supervisor": {"pid": os.getpid(), "pgid": os.getpgrp(), "sid": os.getsid(0)},
        "source_metrics_child": {
            "status": "BLOCKED_INTERFACE_UNAVAILABLE",
            "reason": "canonical source-metrics child command is not installed in this revision",
        },
    }
    try:
        with worker_log.open("x", encoding="utf-8") as worker_stream, bag_log.open("x", encoding="utf-8") as bag_stream:
            bag = subprocess.Popen(
                bag_command,
                stdin=subprocess.DEVNULL, stdout=bag_stream, stderr=subprocess.STDOUT, start_new_session=False,
            )
            worker = subprocess.Popen(worker_command, stdin=subprocess.DEVNULL, stdout=worker_stream, stderr=subprocess.STDOUT, start_new_session=False)
            recorder_observation = _wait_trusted_recorder_ready(args.ros2, bag_path, args.ready_timeout_seconds)
            ready = _supervisor_readiness(args, session, bag, worker)
            ready["trusted_gt_recorder"]["runtime_observation"] = recorder_observation
            _write_manifest_no_replace(args.run_root.resolve() / args.ready_file, ready)
            deadline = time.monotonic() + args.ready_timeout_seconds
            worker_ready = args.run_root.resolve() / args.video_ready_file
            while time.monotonic() < deadline and not worker_ready.exists():
                if worker.poll() is not None or bag.poll() is not None:
                    raise CaptureError("A12 child exited before readiness")
                time.sleep(0.1)
            if not worker_ready.is_file() or worker_ready.is_symlink():
                raise CaptureError("A12 video worker did not publish readiness")
            if worker.wait() != 0:
                raise CaptureError("A12 video worker failed; no receipt may be emitted")
            # The worker exits only after the first true mission_complete. Stop
            # the recorder at that same terminal boundary so later ROS traffic
            # cannot extend the evidence window.
            recorder_exit = _stop_child(bag, "trusted_gt_recorder", bag_command, args.window_timeout_seconds)
            child_lifecycles.append(recorder_exit)
            bag = None
            status["mcap"] = _inspect_bag_metadata(bag_path)
            status["mcap"]["recorder_exit"] = recorder_exit
            input_paths = {
                "collector_raw": args.run_root.resolve() / args.collector_raw,
                "source_metrics": args.run_root.resolve() / args.source_metrics,
            }
            missing_inputs = sorted(name for name, path in input_paths.items() if not path.exists())
            if missing_inputs:
                status.update({
                    "status": "A12_CAPTURE_SUPERVISOR_BLOCKED",
                    "completed": False,
                    "nonfatal_sidecar_blocked": True,
                    "error": f"canonical finalizer inputs are not ready: {missing_inputs}",
                })
                return status
            status["finalizer_inputs"] = _wait_for_fresh_inputs(
                input_paths, int(session["started_epoch_ns"]), timeout_seconds=0.001,
            )
            # The current repository intentionally has no canonical source-metrics
            # finalizer for the three runtime topics above. Retain the verified
            # inputs and return a nonfatal BLOCKED sidecar state; the runner's
            # pre-existing collector/aggregate path remains authoritative.
            status.update({
                "status": "A12_CAPTURE_SUPERVISOR_BLOCKED",
                "completed": False,
                "nonfatal_sidecar_blocked": True,
                "error": "A12 source-metrics and execution-receipt finalizer is not installed",
            })
            return status
    except CaptureError as exc:
        status.update({"status": "A12_CAPTURE_SUPERVISOR_BLOCKED", "error": str(exc), "completed": False})
        return status
    finally:
        children: list[dict[str, Any]] = []
        for process, role in ((worker, "video_worker"), (bag, "trusted_gt_recorder")):
            if process is not None:
                try:
                    command = worker_command if role == "video_worker" else bag_command
                    row = _stop_child(process, role, command, args.window_timeout_seconds)
                    children.append(row)
                    child_lifecycles.append(row)
                except CaptureError as exc:
                    status.setdefault("cleanup_errors", []).append(str(exc))
        try:
            survivors = _cleanup_private_pgid(os.getpgrp())
        except CaptureError as exc:
            survivors = [-1]
            status.setdefault("cleanup_errors", []).append(str(exc))
        status["children"] = child_lifecycles
        status["same_pgid_survivors"] = survivors
        if survivors:
            status.update({"status": "A12_CAPTURE_SUPERVISOR_BLOCKED", "completed": False, "error": "same-PGID child survivors after cleanup"})
        if status.get("status") != "A12_CAPTURE_SUPERVISOR_BLOCKED":
            status.update({"status": "A12_CAPTURE_SUPERVISOR_BLOCKED", "completed": False, "error": "supervisor did not reach a canonical finalizer"})
        try:
            _write_manifest_no_replace(args.run_root.resolve() / args.supervisor_status_file, status)
        except CaptureError:
            pass


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="relative .mp4 path below --run-root")
    parser.add_argument("--ready-file", required=True, type=Path, help="fresh relative supervisor-ready JSON path below --run-root")
    parser.add_argument("--video-ready-file", type=Path, default=Path("a12_video_worker_ready.json"))
    parser.add_argument("--session-status", required=True, type=Path)
    parser.add_argument("--runtime-id", required=True)
    parser.add_argument("--ros2", default="ros2")
    parser.add_argument("--ffprobe", default="ffprobe")
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--queue-size", type=int, default=32)
    parser.add_argument("--max-drop-ratio", type=float, default=0.10)
    parser.add_argument("--minimum-source-frame-rate-hz", type=float, default=MINIMUM_SOURCE_FRAME_RATE_HZ)
    parser.add_argument("--max-source-frame-gap-seconds", type=float, default=MAX_SOURCE_FRAME_GAP_SECONDS)
    parser.add_argument("--max-window-edge-freshness-seconds", type=float, default=MAX_WINDOW_EDGE_FRESHNESS_SECONDS)
    parser.add_argument("--minimum-unique-source-frames", type=int, default=MINIMUM_UNIQUE_SOURCE_FRAMES)
    parser.add_argument("--minimum-video-bytes", type=int, default=MINIMUM_VIDEO_BYTES)
    parser.add_argument("--window-timeout-seconds", type=float, default=21600.0)
    parser.add_argument("--command-timeout-seconds", type=float, default=15.0)
    parser.add_argument("--bag-output", type=Path, default=Path("a12_execution.mcap"))
    parser.add_argument("--collector-raw", type=Path, default=Path("raw_collection.json"))
    parser.add_argument("--source-metrics", type=Path, default=Path("a12_source_metrics.json"))
    parser.add_argument("--execution-receipt", type=Path, default=Path("a12_execution_receipt.json"))
    parser.add_argument("--supervisor-status-file", type=Path, default=Path("a12_capture_supervisor_status.json"))
    parser.add_argument("--ready-timeout-seconds", type=float, default=240.0)
    parser.add_argument("--finalize-timeout-seconds", type=float, default=900.0)
    parser.add_argument("--worker-video", action="store_true")
    parser.add_argument("--supervisor-child", action="store_true")
    args = parser.parse_args(argv)
    if (
        args.fps <= 0 or not 0.0 <= args.max_drop_ratio <= 1.0
        or args.minimum_video_bytes < MINIMUM_VIDEO_BYTES or args.window_timeout_seconds < 21600.0
        or args.command_timeout_seconds <= 0 or args.ready_timeout_seconds <= 0 or args.finalize_timeout_seconds <= 0
        or args.minimum_source_frame_rate_hz < MINIMUM_SOURCE_FRAME_RATE_HZ
        or args.minimum_source_frame_rate_hz > RGB_SOURCE_NOMINAL_HZ
        or args.max_source_frame_gap_seconds <= 0 or args.max_source_frame_gap_seconds > MAX_SOURCE_FRAME_GAP_SECONDS
        or args.max_window_edge_freshness_seconds <= 0 or args.max_window_edge_freshness_seconds > MAX_WINDOW_EDGE_FRESHNESS_SECONDS
        or args.minimum_unique_source_frames < MINIMUM_UNIQUE_SOURCE_FRAMES
    ):
        parser.error("capture freshness/source-rate gates may only be strengthened from the formal 30Hz contract")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = capture(args) if args.worker_video else supervisor(args)
        print(json.dumps(result, sort_keys=True))
        if result.get("status") == "A12_CAPTURE_SUPERVISOR_BLOCKED" and not result.get("nonfatal_sidecar_blocked"):
            return 2
    except (CaptureError, OSError, ValueError, queue.Empty) as exc:
        print(json.dumps({"status": "A12_VIDEO_OBSERVATION_BLOCKED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
