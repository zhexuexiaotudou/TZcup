#!/usr/bin/env python3
"""Capture one fresh, product-only W2 grasp request and its provenance.

The collector is evaluator-only: it has exactly two subscriptions and no
publishers, actions, services, or simulation-control interfaces.  The raw
request remains the strict v2 product payload accepted by ``GraspRequest``;
all capture metadata lives in the separate provenance receipt.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import time
from pathlib import Path
from typing import Any, Mapping, Sequence


TARGET_TOPIC = "/perception/garbage/targets"
RECHECK_TOPIC = "/perception/wrist/grasp_recheck"
TARGET_TYPE = "sanitation_perception_interfaces/msg/GarbageTargetArray"
RECHECK_TYPE = "std_msgs/msg/String"
PRODUCT_NODE = "pc_open_vocab_product_adapter"
PRODUCT_NAMESPACE = "/"
ALLOWED_TRACK_STATES = frozenset(("CONFIRMED", "QUEUED", "APPROACHING", "CLEANING"))
MAX_SOURCE_AGE_S = 1.0


class CaptureError(RuntimeError):
    pass


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _regular(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_file():
        raise CaptureError(f"{label} must be a regular file: {path}")
    return path.resolve(strict=True)


def _inside(path: Path, root: Path, label: str) -> Path:
    resolved = path.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise CaptureError(f"{label} escapes run root") from exc
    return resolved


def closure_perception_roots(value: Mapping[str, Any]) -> tuple[Path, Path]:
    """Resolve both frozen perception roots without accepting symlink aliases."""
    closure = value.get("closure")
    if not isinstance(closure, Mapping):
        raise CaptureError("closure manifest has no closure object")
    resolved: list[Path] = []
    for key in ("perception_artifact_root", "onnx_pythonpath"):
        raw = closure.get(key)
        if not isinstance(raw, str) or not raw:
            raise CaptureError(f"closure has no {key}")
        candidate = Path(raw)
        if candidate.is_symlink() or not candidate.is_dir():
            raise CaptureError(f"closure {key} is not a regular directory")
        resolved.append(candidate.resolve(strict=True))
    onnx_marker = resolved[1] / "onnxruntime" / "__init__.py"
    if onnx_marker.is_symlink() or not onnx_marker.is_file():
        raise CaptureError("closure onnx_pythonpath lacks regular onnxruntime/__init__.py")
    return resolved[0], resolved[1]


def _write_new(path: Path, value: bytes) -> None:
    if path.exists() or path.is_symlink():
        raise CaptureError(f"refusing retained output: {path}")
    pending = path.with_name(f"{path.name}.pending.{os.getpid()}")
    pending.write_bytes(value)
    pending.replace(path)


def _finite_number(value: Any, label: str) -> float:
    converted = float(value)
    if converted != converted or converted in (float("inf"), float("-inf")):
        raise CaptureError(f"{label} is not finite")
    return converted


def publisher_contract(rows: Mapping[str, Sequence[Mapping[str, str]]]) -> bool:
    """Require one exact product publisher for each capture topic."""
    expected = {TARGET_TOPIC: TARGET_TYPE, RECHECK_TOPIC: RECHECK_TYPE}
    if set(rows) != set(expected):
        return False
    for topic, expected_type in expected.items():
        endpoints = list(rows[topic])
        if len(endpoints) != 1:
            return False
        endpoint = endpoints[0]
        if endpoint != {
            "node_name": PRODUCT_NODE,
            "node_namespace": PRODUCT_NAMESPACE,
            "topic_type": expected_type,
        }:
            return False
    return True


def request_from_pair(target: Mapping[str, Any], raw_recheck: str) -> tuple[dict[str, Any], int]:
    """Validate one same-observation target/recheck pair without provenance gaps."""
    try:
        request = json.loads(raw_recheck)
    except json.JSONDecodeError as exc:
        raise CaptureError("wrist recheck is not JSON") from exc
    if not isinstance(request, dict):
        raise CaptureError("wrist recheck must be a JSON object")
    required = {
        "schema_version", "target_id", "frame_id", "pose", "size_m",
        "material", "confidence", "truth_used",
    }
    if set(request) != required or request.get("schema_version") != 2:
        raise CaptureError("wrist recheck is not the strict v2 grasp request")
    if request.get("truth_used") is not False or request.get("material") != "unknown":
        raise CaptureError("wrist recheck violates the truth/material boundary")
    if target.get("source_backend") != "dosod_edgesam_pc":
        raise CaptureError("target does not come from the PC product adapter")
    if target.get("target_type") != "discrete":
        raise CaptureError("target is not a discrete grasp target")
    if str(target.get("track_state", "")).upper() not in ALLOWED_TRACK_STATES:
        raise CaptureError("target track state is not eligible")
    source_stamp_ns = int(target.get("source_stamp_ns", 0))
    if source_stamp_ns <= 0:
        raise CaptureError("target source stamp is absent")
    if request.get("target_id") != target.get("uuid"):
        raise CaptureError("recheck target UUID differs from product target")
    if request.get("frame_id") != target.get("frame_id"):
        raise CaptureError("recheck frame differs from product target")
    pose = request.get("pose")
    if not isinstance(pose, dict) or set(pose) != {"x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"}:
        raise CaptureError("recheck pose schema is invalid")
    for key in ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"):
        if abs(_finite_number(pose[key], f"recheck.pose.{key}") - _finite_number(target["pose"][key], f"target.pose.{key}")) > 1e-6:
            raise CaptureError("recheck geometry differs from same-source target")
    sizes = request.get("size_m")
    target_sizes = target.get("size_m")
    if not isinstance(sizes, list) or not isinstance(target_sizes, list) or len(sizes) != 3 or len(target_sizes) != 3:
        raise CaptureError("recheck size schema is invalid")
    if any(abs(_finite_number(left, "recheck.size") - _finite_number(right, "target.size")) > 1e-6 for left, right in zip(sizes, target_sizes, strict=True)):
        raise CaptureError("recheck size differs from same-source target")
    if abs(_finite_number(request.get("confidence"), "recheck.confidence") - _finite_number(target.get("confidence"), "target.confidence")) > 1e-6:
        raise CaptureError("recheck confidence differs from same-source target")
    return request, source_stamp_ns


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def source_age_s(*, now_ros_ns: int, source_stamp_ns: int) -> float:
    """Return a bounded live simulation-time age or fail closed."""
    if now_ros_ns <= 0 or source_stamp_ns <= 0:
        raise CaptureError("simulation clock or target source stamp is absent")
    if source_stamp_ns > now_ros_ns:
        raise CaptureError("target source stamp is future-dated against simulation clock")
    age = (now_ros_ns - source_stamp_ns) * 1e-9
    if age > MAX_SOURCE_AGE_S:
        raise CaptureError("target source stamp is stale against simulation clock")
    return age


def _target_row(item: Any) -> dict[str, Any]:
    pose = item.map_pose.pose
    return {
        "uuid": str(item.uuid),
        "frame_id": str(item.header.frame_id),
        "source_stamp_ns": _stamp_ns(item.source_stamp),
        "header_stamp_ns": _stamp_ns(item.header.stamp),
        "source_backend": str(item.source_backend),
        "target_type": str(item.target_type),
        "track_state": str(item.track_state),
        "confidence": float(item.confidence),
        "pose": {
            "x_m": float(pose.position.x), "y_m": float(pose.position.y),
            "z_m": float(pose.position.z), "qx": float(pose.orientation.x),
            "qy": float(pose.orientation.y), "qz": float(pose.orientation.z),
            "qw": float(pose.orientation.w),
        },
        "size_m": [float(item.size.x), float(item.size.y), float(item.size.z)],
    }


def capture(
    *, run_root: Path, request_output: Path, provenance_output: Path,
    session: Path, binding: Path, closure_manifest: Path, timeout_sec: float,
) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sanitation_perception_interfaces.msg import GarbageTargetArray
    from std_msgs.msg import String

    root = run_root.resolve(strict=True)
    if run_root.is_symlink() or not root.is_dir() or "hidden" in str(root).lower():
        raise CaptureError("run root must be a non-symlink public directory")
    request_output = _inside(request_output, root, "request output")
    provenance_output = _inside(provenance_output, root, "provenance output")
    session = _regular(session, "acceptance session")
    binding = _regular(binding, "runtime binding")
    closure_manifest = _regular(closure_manifest, "closure manifest")
    for path, label in ((session, "acceptance session"), (binding, "runtime binding")):
        _inside(path, root, label)
    closure = json.loads(closure_manifest.read_text(encoding="utf-8"))
    artifact, onnx_pythonpath = closure_perception_roots(closure)

    class Collector(Node):
        def __init__(self) -> None:
            super().__init__(
                "r065_w2_live_grasp_request_collector",
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            self.started_monotonic = time.monotonic()
            self.targets: dict[str, tuple[dict[str, Any], float, int, float]] = {}
            self.result: tuple[dict[str, Any], str, dict[str, Any], float, int, float] | None = None
            self.last_error = "no_matching_product_target_and_wrist_recheck"
            self.create_subscription(GarbageTargetArray, TARGET_TOPIC, self.on_targets, 10)
            self.create_subscription(String, RECHECK_TOPIC, self.on_recheck, 10)

        def graph_rows(self) -> dict[str, list[dict[str, str]]]:
            return {
                topic: [
                    {
                        "node_name": str(endpoint.node_name),
                        "node_namespace": str(endpoint.node_namespace),
                        "topic_type": str(endpoint.topic_type),
                    }
                    for endpoint in self.get_publishers_info_by_topic(topic)
                ]
                for topic in (TARGET_TOPIC, RECHECK_TOPIC)
            }

        def on_targets(self, message: Any) -> None:
            if not publisher_contract(self.graph_rows()):
                self.last_error = "product_topic_publishers_are_not_exclusive"
                return
            received = time.monotonic()
            now_ros_ns = self.get_clock().now().nanoseconds
            for item in message.targets:
                row = _target_row(item)
                if row["header_stamp_ns"] != row["source_stamp_ns"]:
                    continue
                try:
                    age = source_age_s(
                        now_ros_ns=now_ros_ns,
                        source_stamp_ns=row["source_stamp_ns"],
                    )
                except CaptureError as exc:
                    self.last_error = str(exc)
                    continue
                self.targets[row["uuid"]] = (row, received, now_ros_ns, age)

        def on_recheck(self, message: Any) -> None:
            if not publisher_contract(self.graph_rows()):
                self.last_error = "product_topic_publishers_are_not_exclusive"
                return
            received = time.monotonic()
            try:
                value = json.loads(str(message.data))
                target_id = str(value["target_id"])
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                self.last_error = "wrist_recheck_is_not_a_targeted_json_request"
                return
            candidate = self.targets.get(target_id)
            if candidate is None:
                self.last_error = "wrist_recheck_has_no_fresh_product_target"
                return
            target, target_received, _, _ = candidate
            if target_received < self.started_monotonic or received < target_received or received - target_received > 1.0:
                self.last_error = "target_and_wrist_recheck_are_not_one_fresh_observation"
                return
            try:
                request, source_stamp_ns = request_from_pair(target, str(message.data))
                capture_ros_time_ns = self.get_clock().now().nanoseconds
                source_age = source_age_s(
                    now_ros_ns=capture_ros_time_ns,
                    source_stamp_ns=source_stamp_ns,
                )
            except CaptureError as exc:
                self.last_error = str(exc)
                return
            self.result = (
                request, str(message.data), target, received,
                capture_ros_time_ns, source_age,
            )

    rclpy.init()
    node = Collector()
    try:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result is None:
            raise CaptureError(f"timed out: {node.last_error}")
        request, raw_request, target, received, capture_ros_time_ns, source_age = node.result
        if not publisher_contract(node.graph_rows()):
            raise CaptureError("product publisher contract changed before capture")
        # Preserve the exact product message bytes; strict request parsing was
        # already performed above, while the provenance receipt binds this raw
        # payload by hash rather than silently canonicalizing it.
        raw = raw_request.encode("utf-8")
        capture_epoch_ns = time.time_ns()
        provenance = {
            "schema_version": 1,
            "report_id": "r065_w2_live_grasp_request_provenance",
            "passed": True,
            "capture_epoch_ns": capture_epoch_ns,
            "capture_monotonic_s": received,
            "capture_ros_time_ns": capture_ros_time_ns,
            "source_age_s": source_age,
            "raw_request_sha256": _sha256_bytes(raw),
            "request": {"path": str(request_output), "size_bytes": len(raw)},
            "product_topics": {
                "targets": {"topic": TARGET_TOPIC, "type": TARGET_TYPE, "publisher": f"{PRODUCT_NAMESPACE}{PRODUCT_NODE}"},
                "wrist_recheck": {"topic": RECHECK_TOPIC, "type": RECHECK_TYPE, "publisher": f"{PRODUCT_NAMESPACE}{PRODUCT_NODE}"},
            },
            "target": target,
            "acceptance_session": {"path": str(session), "sha256": _sha256_bytes(session.read_bytes())},
            "runtime_binding": {"path": str(binding), "sha256": _sha256_bytes(binding.read_bytes())},
            "closure_manifest": {"path": str(closure_manifest), "sha256": _sha256_bytes(closure_manifest.read_bytes())},
            "perception_artifact_root": str(artifact),
            "onnx_pythonpath": str(onnx_pythonpath),
        }
        _write_new(request_output, raw)
        _write_new(provenance_output, (json.dumps(provenance, indent=2, sort_keys=True) + "\n").encode("utf-8"))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--request-output", type=Path, required=True)
    parser.add_argument("--provenance-output", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--closure-manifest", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    args = parser.parse_args()
    if args.timeout_sec <= 0.0:
        raise CaptureError("timeout-sec must be positive")
    try:
        capture(
            run_root=args.run_root, request_output=args.request_output,
            provenance_output=args.provenance_output, session=args.session,
            binding=args.runtime_binding, closure_manifest=args.closure_manifest,
            timeout_sec=args.timeout_sec,
        )
    except (CaptureError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"R065 W2 grasp-request capture blocked: {exc}", file=os.sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
