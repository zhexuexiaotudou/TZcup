#!/usr/bin/env python3
"""One-shot, read-only TF readiness probe for the R065 W2 live runner.

The probe intentionally owns no publisher, client, action, command, truth, or
control interface.  It only consumes /clock and /tf through a tf2 listener,
then atomically writes a bounded PASS/BLOCKED report for the wrapper.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
import time
from pathlib import Path
from typing import Any


PARENT_FRAME = "map"
CHILD_FRAME = "base_footprint"


class ReadinessError(RuntimeError):
    """A fail-closed readiness condition with a stable report reason."""


def atomic_write_json(path: Path, value: dict[str, Any]) -> None:
    """Write a regular, non-link JSON report without leaving partial evidence."""
    # Check the lexical path before resolve(): resolve would silently erase the
    # very leaf/ancestor symlink evidence this fail-closed writer must reject.
    path = Path(os.path.abspath(os.fspath(path)))
    cursor = path
    while True:
        if cursor.is_symlink():
            raise ReadinessError("output_path_or_ancestor_is_symlink")
        if cursor.parent == cursor:
            break
        cursor = cursor.parent
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n"
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent, text=True)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def validate_transform(transform: Any, now_ns: int, max_age_ns: int) -> dict[str, Any]:
    """Check the exact TF and simulated-clock freshness contract."""
    header = transform.header
    parent = str(header.frame_id).lstrip("/")
    child = str(transform.child_frame_id).lstrip("/")
    stamp_ns = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
    if parent != PARENT_FRAME:
        raise ReadinessError(f"tf_parent_frame_mismatch:{parent or '<empty>'}")
    if child != CHILD_FRAME:
        raise ReadinessError(f"tf_child_frame_mismatch:{child or '<empty>'}")
    if now_ns <= 0:
        raise ReadinessError("ros_clock_not_ready")
    if stamp_ns <= 0:
        raise ReadinessError("map_to_base_footprint_tf_zero_stamp")
    if stamp_ns > now_ns:
        raise ReadinessError("map_to_base_footprint_tf_future")
    age_ns = now_ns - stamp_ns
    if age_ns > max_age_ns:
        raise ReadinessError("map_to_base_footprint_tf_stale")
    return {
        "parent_frame": parent,
        "child_frame": child,
        "stamp_ns": stamp_ns,
        "clock_ns": now_ns,
        "age_ns": age_ns,
    }


def run_probe(timeout_sec: float, max_age_sec: float) -> dict[str, Any]:
    """Use one Buffer/TransformListener and bounded sim-time polling only."""
    import rclpy
    from rclpy.parameter import Parameter
    from tf2_ros import Buffer, TransformException, TransformListener

    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise ReadinessError("timeout_sec_invalid")
    if not math.isfinite(max_age_sec) or max_age_sec < 0.0:
        raise ReadinessError("max_age_sec_invalid")

    rclpy.init()
    context_domain_id = rclpy.get_default_context().get_domain_id()
    node = rclpy.create_node(
        "r065_w2_tf_readiness",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    buffer = Buffer()
    TransformListener(buffer, node, spin_thread=False)
    deadline = time.monotonic() + timeout_sec
    first_clock_ns: int | None = None
    last_reason = "ros_clock_not_ready"
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
            now_ns = node.get_clock().now().nanoseconds
            if now_ns <= 0:
                last_reason = "ros_clock_not_ready"
                continue
            if first_clock_ns is None:
                first_clock_ns = now_ns
                last_reason = "ros_clock_not_advanced"
                continue
            if now_ns <= first_clock_ns:
                last_reason = "ros_clock_not_advanced"
                continue
            try:
                transform = buffer.lookup_transform(PARENT_FRAME, CHILD_FRAME, rclpy.time.Time())
                result = validate_transform(transform, now_ns, int(max_age_sec * 1_000_000_000))
                result["clock_advanced_from_ns"] = first_clock_ns
                result["rclpy_context_domain_id"] = context_domain_id
                return result
            except TransformException:
                last_reason = "map_to_base_footprint_tf_unavailable"
            except ReadinessError as exc:
                last_reason = str(exc)
        raise ReadinessError(last_reason)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--max-age-sec", type=float, default=0.50)
    args = parser.parse_args()
    report: dict[str, Any] = {
        "report_id": "r065_w2_one_shot_tf_readiness_v1",
        "status": "BLOCKED",
        "passed": False,
        "parent_frame": PARENT_FRAME,
        "child_frame": CHILD_FRAME,
        "use_sim_time": True,
        "tf_or_actuator_control_interfaces_created": False,
        "runtime_environment": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "ros_automatic_discovery_range": os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", ""),
            "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", ""),
            "cyclonedds_uri": os.environ.get("CYCLONEDDS_URI", ""),
        },
    }
    exit_code = 4
    try:
        report["observation"] = run_probe(args.timeout_sec, args.max_age_sec)
        report["status"] = "PASS"
        report["passed"] = True
        exit_code = 0
    except Exception as exc:  # The report is the evidence, never a traceback-only failure.
        report["reason"] = str(exc) or type(exc).__name__
    try:
        atomic_write_json(args.output, report)
    except Exception as exc:
        print(f"R065 W2 TF readiness cannot write report: {exc}", file=os.sys.stderr)
        return 5
    print(json.dumps(report, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
