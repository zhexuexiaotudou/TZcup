#!/usr/bin/env python3
"""Collect fail-closed, read-only S100P G1/G3 interface evidence.

The collector never opens CAN/TTY/video device nodes, publishes ROS messages,
calls ROS services/actions, starts product nodes, installs packages, or writes
outside one newly-created JSON evidence path.  A ROS subscriber is used only to
observe explicitly requested sensor topics from an already-running graph.
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import platform
import stat
import time
from pathlib import Path
from typing import Any, Callable, Iterable


DEVICE_PATTERNS: dict[str, tuple[str, ...]] = {
    "camera": ("/dev/video*", "/dev/media*"),
    "can": ("/dev/can*",),
    "tty": ("/dev/ttyUSB*", "/dev/ttyACM*"),
    "i2c": ("/dev/i2c-*",),
    "spi": ("/dev/spidev*",),
}
FORBIDDEN_TOPIC_TOKENS = ("sim", "gazebo", "truth", "evaluator", "replay", "bag")
MAX_DURATION_SEC = 120.0


class ReadOnlyEvidenceError(ValueError):
    """A requested observation cannot become trustworthy G1/G3 evidence."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _path_identity(path: Path) -> dict[str, Any]:
    try:
        metadata = path.lstat()
    except OSError as exc:
        return {"path": str(path), "status": "ABSENT", "error": type(exc).__name__}
    if path.is_symlink() or not stat.S_ISREG(metadata.st_mode):
        return {
            "path": str(path),
            "status": "UNSAFE",
            "is_symlink": path.is_symlink(),
            "is_regular_file": stat.S_ISREG(metadata.st_mode),
        }
    return {
        "path": str(path),
        "status": "PRESENT",
        "byte_size": metadata.st_size,
        "sha256": _sha256(path),
    }


def _safe_fresh_output(path: Path) -> None:
    if not path.is_absolute():
        raise ReadOnlyEvidenceError("output_path_must_be_absolute")
    if path.exists() or path.is_symlink():
        raise ReadOnlyEvidenceError("output_path_must_be_fresh_nonlink")
    parent = path.parent
    if not parent.is_dir() or parent.is_symlink():
        raise ReadOnlyEvidenceError("output_parent_must_be_existing_nonlink_directory")
    for ancestor in (parent, *parent.parents):
        if ancestor.is_symlink():
            raise ReadOnlyEvidenceError(f"output_symlink_ancestor:{ancestor}")


def _device_row(path: Path) -> dict[str, Any]:
    try:
        metadata = path.lstat()
    except OSError as exc:
        return {"path": str(path), "status": "UNAVAILABLE", "error": type(exc).__name__}
    return {
        "path": str(path),
        "status": "PRESENT",
        "is_symlink": path.is_symlink(),
        "is_character_device": stat.S_ISCHR(metadata.st_mode),
        "mode": stat.S_IMODE(metadata.st_mode),
    }


def _network_interfaces(root: Path = Path("/sys/class/net")) -> dict[str, Any]:
    """Read network-interface metadata only; SocketCAN uses ARPHRD_CAN (280)."""
    try:
        names = sorted(path.name for path in root.iterdir())
    except OSError as exc:
        return {"status": "UNAVAILABLE", "error": type(exc).__name__, "names": [], "socketcan_interfaces": []}
    socketcan_interfaces: list[str] = []
    for name in names:
        try:
            interface_type = (root / name / "type").read_text(encoding="ascii").strip()
        except OSError:
            continue
        if interface_type == "280":
            socketcan_interfaces.append(name)
    return {"status": "PRESENT", "names": names, "socketcan_interfaces": socketcan_interfaces}


def collect_devices() -> dict[str, Any]:
    """Enumerate paths with metadata only; never open a device node or CAN socket."""
    result: dict[str, Any] = {}
    for category, patterns in DEVICE_PATTERNS.items():
        paths = sorted({candidate for pattern in patterns for candidate in glob.glob(pattern)})
        result[category] = {
            "patterns": list(patterns),
            "status": "PRESENT" if paths else "ABSENT",
            "nodes": [_device_row(Path(candidate)) for candidate in paths],
        }
    result["network_interfaces"] = _network_interfaces()
    socketcan_interfaces = result["network_interfaces"]["socketcan_interfaces"]
    result["can"]["socketcan_interfaces"] = socketcan_interfaces
    if socketcan_interfaces:
        result["can"]["status"] = "PRESENT"
    return result


def _validate_topics(topics: Iterable[str]) -> list[str]:
    normalized: list[str] = []
    for raw in topics:
        topic = raw.strip()
        lowered = topic.lower()
        if not topic.startswith("/") or any(token in lowered for token in FORBIDDEN_TOPIC_TOKENS):
            raise ReadOnlyEvidenceError(f"unsafe_or_invalid_topic:{raw}")
        if topic in normalized:
            raise ReadOnlyEvidenceError(f"duplicate_topic:{topic}")
        normalized.append(topic)
    return normalized


def _stamp_and_frame(message: Any) -> tuple[int | None, str | None]:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    sec, nanosec = getattr(stamp, "sec", None), getattr(stamp, "nanosec", None)
    if not isinstance(sec, int) or not isinstance(nanosec, int) or sec < 0 or nanosec < 0:
        return None, None
    frame_id = getattr(header, "frame_id", None)
    return sec * 1_000_000_000 + nanosec, str(frame_id) if isinstance(frame_id, str) else None


def _publisher_rows(node: Any, topic: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for endpoint in node.get_publishers_info_by_topic(topic):
        name = str(getattr(endpoint, "node_name", ""))
        namespace = str(getattr(endpoint, "node_namespace", ""))
        type_name = str(getattr(endpoint, "topic_type", ""))
        if any(token in f"{name} {namespace}".lower() for token in FORBIDDEN_TOPIC_TOKENS):
            raise ReadOnlyEvidenceError(f"untrusted_topic_publisher:{topic}:{namespace}/{name}")
        rows.append({"node_name": name, "node_namespace": namespace, "topic_type": type_name})
    return rows


def observe_ros_topics(topics: list[str], duration_sec: float) -> dict[str, Any]:
    """Observe requested ROS topics through subscriptions only; never publish."""
    try:
        import rclpy
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:  # Keep a machine-readable blocked receipt on ABI failures.
        raise ReadOnlyEvidenceError(f"ros_python_import_failed:{type(exc).__name__}") from exc

    rclpy.init(args=None)
    node = None
    subscriptions: list[Any] = []
    try:
        node = rclpy.create_node("s100p_g1_g3_read_only_evidence_collector")
        graph = {name: list(types) for name, types in node.get_topic_names_and_types()}
        records: dict[str, dict[str, Any]] = {}
        for topic in topics:
            type_names = graph.get(topic, [])
            if len(type_names) != 1:
                raise ReadOnlyEvidenceError(f"topic_type_not_unique:{topic}:{type_names}")
            message_type = get_message(type_names[0])
            records[topic] = {
                "requested_type": type_names[0],
                "graph_type": type_names[0],
                "publishers": _publisher_rows(node, topic),
                "arrival_monotonic_ns": [],
                "header_stamp_ns": [],
                "frame_ids": [],
            }

            def callback(message: Any, *, key: str = topic) -> None:
                row = records[key]
                row["arrival_monotonic_ns"].append(time.monotonic_ns())
                stamp_ns, frame_id = _stamp_and_frame(message)
                if stamp_ns is not None:
                    row["header_stamp_ns"].append(stamp_ns)
                if frame_id:
                    row["frame_ids"].append(frame_id)

            subscriptions.append(node.create_subscription(message_type, topic, callback, 20))
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))

        for topic, row in records.items():
            arrivals = row.pop("arrival_monotonic_ns")
            stamps = row["header_stamp_ns"]
            unique_frames = sorted(set(row["frame_ids"]))
            periods = [later - earlier for earlier, later in zip(arrivals, arrivals[1:]) if later > earlier]
            row["messages_received"] = len(arrivals)
            row["frame_ids"] = unique_frames
            row["header_stamp_monotonic"] = len(stamps) >= 2 and all(later > earlier for earlier, later in zip(stamps, stamps[1:]))
            row["observed_rate_hz"] = (len(periods) / (sum(periods) / 1_000_000_000)) if periods else 0.0
            row["status"] = "OBSERVED" if len(arrivals) >= 2 and len(stamps) >= 2 and bool(unique_frames) else "BLOCKED"
        return {
            "collector_node": "s100p_g1_g3_read_only_evidence_collector",
            "use_sim_time": bool(node.get_parameter("use_sim_time").value),
            "topic_graph": graph,
            "topics": records,
        }
    finally:
        for subscription in subscriptions:
            if node is not None:
                node.destroy_subscription(subscription)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def collect(
    *,
    tros_setup: Path,
    overlay_setup: Path,
    topics: list[str],
    duration_sec: float,
    required_devices: list[str],
    ros_observer: Callable[[list[str], float], dict[str, Any]] = observe_ros_topics,
) -> dict[str, Any]:
    if duration_sec <= 0 or duration_sec > MAX_DURATION_SEC:
        raise ReadOnlyEvidenceError("duration_sec_out_of_bounds")
    if any(name not in DEVICE_PATTERNS for name in required_devices):
        raise ReadOnlyEvidenceError("unknown_required_device_category")
    selected_topics = _validate_topics(topics)
    devices = collect_devices()
    blockers: list[str] = []
    identities = {"tros_setup": _path_identity(tros_setup), "overlay_setup": _path_identity(overlay_setup)}
    for name, row in identities.items():
        if row["status"] != "PRESENT":
            blockers.append(f"{name}_{row['status'].lower()}")
    for category in required_devices:
        if devices[category]["status"] != "PRESENT":
            blockers.append(f"required_device_category_absent:{category}")
    ros: dict[str, Any]
    try:
        if not selected_topics:
            raise ReadOnlyEvidenceError("no_topics_requested_g3_observation_incomplete")
        ros = ros_observer(selected_topics, duration_sec)
        if ros.get("use_sim_time") is True:
            blockers.append("collector_use_sim_time_true")
        for topic, row in ros.get("topics", {}).items():
            if row.get("status") != "OBSERVED":
                blockers.append(f"topic_not_fresh_or_headerless:{topic}")
    except Exception as exc:
        ros = {"status": "BLOCKED", "error": f"{type(exc).__name__}:{exc}", "topics": {}}
        blockers.append(
            "no_topics_requested_g3_observation_incomplete"
            if isinstance(exc, ReadOnlyEvidenceError) and str(exc) == "no_topics_requested_g3_observation_incomplete"
            else "ros_topic_observation_failed"
        )
    return {
        "schema_version": 1,
        "report_id": "tzcup_s100p_g1_g3_read_only_evidence_v1",
        "status": "BLOCKED" if blockers else "READ_ONLY_EVIDENCE_COLLECTED_NOT_ACCEPTED",
        "acceptance": {"g1_accepted": False, "g2_accepted": False, "g3_accepted": False, "ready_for_g4": False},
        "operation_boundary": "read_only_no_device_open_can_write_ros_publish_service_action_product_node_start_dependency_install_or_actuator_command",
        "collected_epoch_ns": time.time_ns(),
        "platform": {"architecture": platform.machine(), "hostname": platform.node()},
        "sourced_identities": identities,
        "devices": devices,
        "ros": ros,
        "required_devices": required_devices,
        "requested_topics": selected_topics,
        "blockers": blockers,
    }


def _atomic_write_new(path: Path, payload: dict[str, Any]) -> None:
    _safe_fresh_output(path)
    encoded = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    pending = path.with_name(f".{path.name}.pending.{os.getpid()}")
    try:
        pending.write_text(encoded, encoding="utf-8")
        try:
            # ``link`` is a no-replace finalization primitive: unlike rename/replace,
            # it cannot overwrite an evidence file created by a concurrent process.
            os.link(pending, path)
        except FileExistsError as exc:
            raise ReadOnlyEvidenceError("output_path_became_nonfresh") from exc
    finally:
        if pending.exists():
            pending.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tros-setup", type=Path, required=True)
    parser.add_argument("--overlay-setup", type=Path, required=True)
    parser.add_argument("--topic", action="append", default=[])
    parser.add_argument("--require-device", action="append", default=[])
    parser.add_argument("--duration-sec", type=float, default=10.0)
    args = parser.parse_args()
    try:
        _safe_fresh_output(args.output)
        report = collect(
            tros_setup=args.tros_setup,
            overlay_setup=args.overlay_setup,
            topics=args.topic,
            duration_sec=args.duration_sec,
            required_devices=args.require_device,
        )
    except Exception as exc:
        report = {
            "schema_version": 1,
            "report_id": "tzcup_s100p_g1_g3_read_only_evidence_v1",
            "status": "BLOCKED",
            "acceptance": {"g1_accepted": False, "g2_accepted": False, "g3_accepted": False, "ready_for_g4": False},
            "operation_boundary": "read_only_no_device_open_can_write_ros_publish_service_action_product_node_start_dependency_install_or_actuator_command",
            "blockers": [f"collector_failed:{type(exc).__name__}:{exc}"],
        }
    try:
        _atomic_write_new(args.output, report)
    except ReadOnlyEvidenceError as exc:
        print(f"s100p_g1_g3_evidence_refused:{exc}")
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "READ_ONLY_EVIDENCE_COLLECTED_NOT_ACCEPTED" else 4


if __name__ == "__main__":
    raise SystemExit(main())
