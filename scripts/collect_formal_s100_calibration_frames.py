#!/usr/bin/env python3
"""Freeze DOSOD S100 calibration tensors from one real ROS Image stream.

The collector intentionally starts only with an empty local output directory.
It never accepts simulated/truth/evaluator topics, ROS simulated time, a
non-``sensor_msgs/msg/Image`` graph type, duplicate sources/tensors, or a
holdout source hash.  A manifest exists only after the requested contract
minimum is reached and is atomically frozen.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

from formal_s100_live_acceptance_core import probe_hardware


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "dosod_s100p_hbm_compile_contract.json"
IMAGE_TYPE = "sensor_msgs/msg/Image"
FORBIDDEN_TOPIC_TOKENS = ("sim", "gazebo", "truth", "evaluator", "replay", "bag")
FORBIDDEN_PUBLISHER_TOKENS = FORBIDDEN_TOPIC_TOKENS
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
RECEIPT_NAME = "calibration_collection_receipt.json"


class CalibrationRejected(ValueError):
    """A source frame or collection request that cannot become calibration."""


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return sha256_bytes(encoded.encode("utf-8"))


def atomic_write_bytes(path: Path, payload: bytes) -> None:
    pending = path.with_name(f".{path.name}.pending.{os.getpid()}")
    pending.write_bytes(payload)
    pending.replace(path)


def atomic_write_json(path: Path, value: Any) -> None:
    atomic_write_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def reject_unsafe_output(output: Path) -> None:
    """Require a new/empty normal directory and no symbolic-link parent."""

    for candidate in (output, *output.parents):
        if candidate.is_symlink():
            raise CalibrationRejected(f"output_symlink_forbidden:{candidate}")
    if output.exists() and not output.is_dir():
        raise CalibrationRejected("output_not_directory")
    if output.exists() and any(output.iterdir()):
        raise CalibrationRejected("output_must_be_empty")


def write_receipt(output: Path, payload: dict[str, Any], *, allow_partial: bool) -> None:
    """Write only inside a collector-owned safe output root."""

    for candidate in (output, *output.parents):
        if candidate.is_symlink():
            return
    if output.exists() and not output.is_dir():
        return
    if output.exists() and any(output.iterdir()) and not allow_partial:
        return
    output.mkdir(parents=True, exist_ok=True)
    atomic_write_json(output / RECEIPT_NAME, payload)


def write_blocked_receipt_if_manifest_absent(
    output: Path, contract: dict[str, Any] | None, payload: dict[str, Any], *, allow_partial: bool
) -> None:
    """A committed manifest is the only successful terminal state."""

    manifest_name = (contract or {}).get("calibration", {}).get("manifest_name", "calibration_manifest.json")
    if isinstance(manifest_name, str) and (output / manifest_name).is_file():
        return
    write_receipt(output, payload, allow_partial=allow_partial)


def authorization_record(
    path: Path | None, *, accepted: bool, output: Path, topic: str,
    contract: dict[str, Any], target_count: int, now: datetime | None = None,
) -> dict[str, Any]:
    if not accepted:
        raise CalibrationRejected("operator_authorization_flag_required")
    if path is None:
        raise CalibrationRejected("authorization_record_required")
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationRejected(f"authorization_record_unreadable:{type(exc).__name__}") from exc
    if not isinstance(record, dict):
        raise CalibrationRejected("authorization_record_not_object")
    required_text = ("operator", "scope", "authorized_action", "timestamp", "private_output_root", "retention_policy", "privacy_policy", "contract_id", "topic", "board")
    if any(not isinstance(record.get(key), str) or not record[key].strip() for key in required_text):
        raise CalibrationRejected("authorization_record_required_text_missing")
    try:
        parsed_timestamp = datetime.fromisoformat(record["timestamp"].replace("Z", "+00:00"))
    except ValueError as exc:
        raise CalibrationRejected("authorization_timestamp_invalid") from exc
    if parsed_timestamp.tzinfo is None:
        raise CalibrationRejected("authorization_timestamp_timezone_required")
    current = now or datetime.now(timezone.utc)
    if not current.tzinfo:
        raise CalibrationRejected("authorization_now_timezone_required")
    if parsed_timestamp < current - timedelta(hours=24) or parsed_timestamp > current + timedelta(minutes=5):
        raise CalibrationRejected("authorization_timestamp_outside_window")
    consent = record.get("consent")
    privacy = record.get("privacy")
    consent_ok = consent is True or (isinstance(consent, dict) and consent.get("approved") is True)
    if not consent_ok or not privacy or record.get("processed_tensor_retention_approved") is not True:
        raise CalibrationRejected("authorization_consent_or_privacy_missing")
    privacy_policy = record["privacy_policy"].lower()
    if "npy" not in privacy_policy or not any(token in privacy_policy for token in ("retain", "retention")):
        raise CalibrationRejected("authorization_privacy_policy_does_not_disclose_npy_retention")
    if record["scope"] != "tzcup_s100p_dosod_calibration_capture_v1":
        raise CalibrationRejected("authorization_scope_invalid")
    if record["authorized_action"] != "capture_real_s100p_calibration":
        raise CalibrationRejected("authorization_action_invalid")
    if record.get("override_collection_status") != "PAUSED_NO_NEW_CAPTURE":
        raise CalibrationRejected("authorization_collection_status_override_invalid")
    if record["contract_id"] != contract.get("contract_id"):
        raise CalibrationRejected("authorization_contract_id_mismatch")
    if record.get("source_kind") != "physical_camera" or record.get("no_replay") is not True:
        raise CalibrationRejected("authorization_source_kind_or_replay_invalid")
    if record.get("topic") != topic or record.get("board") != "RDK S100P":
        raise CalibrationRejected("authorization_topic_or_board_mismatch")
    if record.get("target_count") != target_count:
        raise CalibrationRejected("authorization_target_count_mismatch")
    private_root = Path(record["private_output_root"])
    if not private_root.is_absolute() or not output.resolve().is_relative_to(private_root.resolve()):
        raise CalibrationRejected("authorization_private_output_root_mismatch")
    return record


def require_capture_status(contract: dict[str, Any], authorization: dict[str, Any]) -> None:
    status = contract["calibration"].get("collection_status")
    if status == "PAUSED_NO_NEW_CAPTURE" and not authorization:
        raise CalibrationRejected("collection_paused_no_new_capture")
    if not isinstance(status, str) or not status:
        raise CalibrationRejected("collection_status_invalid")


def require_s100p_hardware(probe: Any = probe_hardware) -> dict[str, Any]:
    hardware = probe()
    if not isinstance(hardware, dict) or hardware.get("attested") is not True:
        raise CalibrationRejected("s100p_hardware_identity_not_attested")
    if hardware.get("architecture") not in {"aarch64", "arm64"}:
        raise CalibrationRejected("s100p_hardware_architecture_invalid")
    if hardware.get("board") != "RDK S100P" or hardware.get("soc") != "Journey 6P":
        raise CalibrationRejected("s100p_hardware_board_or_soc_invalid")
    return hardware


def load_contract(path: Path, *, allow_paused_capture: bool = False) -> dict[str, Any]:
    try:
        contract = json.loads(path.read_text(encoding="utf-8"))
        preprocessing = contract["preprocessing"]
        calibration = contract["calibration"]
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        raise CalibrationRejected(f"contract_unreadable:{type(exc).__name__}") from exc
    required_preprocessing = {
        "source_color_space": "RGB",
        "square_padding": "symmetric_black_zero_short_axis_floor_leading_remainder_trailing",
        "tensor_dtype": "float32",
        "tensor_layout": "NCHW",
        "tensor_shape": [1, 3, 640, 640],
        "value_range": [0.0, 1.0],
        "mean": [],
        "standard_deviation": [],
    }
    if preprocessing.get("resize") != {"height": 640, "width": 640, "interpolation": "bilinear"}:
        raise CalibrationRejected("contract_resize_not_dosod_640")
    if any(preprocessing.get(key) != expected for key, expected in required_preprocessing.items()):
        raise CalibrationRejected("contract_preprocessing_not_dosod_nchw_float32")
    if preprocessing.get("value_scale") != 0.003921568627451:
        raise CalibrationRejected("contract_value_scale_not_dosod_rgb_255")
    if calibration.get("sample_suffix") != ".npy" or calibration.get("dtype") != "float32":
        raise CalibrationRejected("contract_calibration_dtype_or_suffix_invalid")
    if calibration.get("shape") != preprocessing["tensor_shape"]:
        raise CalibrationRejected("contract_calibration_shape_invalid")
    if calibration.get("value_range") != preprocessing["value_range"]:
        raise CalibrationRejected("contract_calibration_range_invalid")
    if not isinstance(calibration.get("minimum_sample_count"), int) or calibration["minimum_sample_count"] < 500:
        raise CalibrationRejected("contract_minimum_sample_count_below_500")
    if calibration.get("collection_status") == "PAUSED_NO_NEW_CAPTURE" and not allow_paused_capture:
        raise CalibrationRejected("collection_paused_no_new_capture")
    if not isinstance(calibration.get("collection_status"), str) or not calibration["collection_status"]:
        raise CalibrationRejected("collection_status_invalid")
    return contract


def load_holdout_hashes(paths: Iterable[Path], values: Iterable[str]) -> set[str]:
    hashes = set(values)
    for path in paths:
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise CalibrationRejected(f"holdout_unreadable:{type(exc).__name__}") from exc
        if isinstance(loaded, dict):
            loaded = loaded.get("evaluation_holdout_source_sha256")
        if not isinstance(loaded, list):
            raise CalibrationRejected("holdout_not_hash_list")
        hashes.update(loaded)
    if not all(isinstance(item, str) and SHA256_RE.fullmatch(item) for item in hashes):
        raise CalibrationRejected("holdout_hash_invalid")
    return hashes


def validate_topic(topic: str, graph_types: Iterable[str]) -> None:
    normalized = topic.strip().lower()
    if not normalized.startswith("/"):
        raise CalibrationRejected("topic_must_be_absolute")
    if any(token in normalized for token in FORBIDDEN_TOPIC_TOKENS):
        raise CalibrationRejected("topic_forbidden_simulator_truth_or_evaluator")
    if set(graph_types) != {IMAGE_TYPE}:
        raise CalibrationRejected("topic_not_exact_sensor_msgs_image")


def rgb_from_ros_image(
    *, data: bytes, width: int, height: int, step: int, encoding: str
) -> np.ndarray:
    normalized = encoding.strip().lower()
    if normalized not in {"rgb8", "bgr8"}:
        raise CalibrationRejected(f"image_encoding_not_rgb8_or_bgr8:{encoding}")
    if width <= 0 or height <= 0:
        raise CalibrationRejected("image_dimensions_invalid")
    row_step = int(step) if int(step) > 0 else width * 3
    if row_step < width * 3 or len(data) != row_step * height:
        raise CalibrationRejected("image_payload_or_step_invalid")
    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, row_step)
    image = np.ascontiguousarray(rows[:, : width * 3].reshape(height, width, 3))
    return image if normalized == "rgb8" else np.ascontiguousarray(image[:, :, ::-1])


def preprocess_dosod_rgb(image: np.ndarray) -> np.ndarray:
    """DOSOD RGB -> symmetric-black-square -> bilinear 640 -> float32 NCHW."""

    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise CalibrationRejected("rgb_image_shape_or_dtype_invalid")
    height, width = image.shape[:2]
    if not height or not width:
        raise CalibrationRejected("rgb_image_empty")
    side = max(height, width)
    padded = np.zeros((side, side, 3), dtype=np.uint8)
    top = (side - height) // 2
    left = (side - width) // 2
    padded[top : top + height, left : left + width] = image
    resized = cv2.resize(padded, (640, 640), interpolation=cv2.INTER_LINEAR)
    tensor = np.ascontiguousarray(resized.transpose(2, 0, 1)[None], dtype=np.float32)
    tensor *= np.float32(1.0 / 255.0)
    if tensor.shape != (1, 3, 640, 640) or not np.isfinite(tensor).all():
        raise CalibrationRejected("preprocessed_tensor_invalid")
    if float(tensor.min()) < 0.0 or float(tensor.max()) > 1.0:
        raise CalibrationRejected("preprocessed_tensor_out_of_range")
    return tensor


@dataclass(frozen=True)
class ImageFrame:
    data: bytes
    width: int
    height: int
    step: int
    encoding: str
    stamp_ns: int
    frame_id: str


class CalibrationStore:
    def __init__(self, output: Path, contract: dict[str, Any], topic: str, holdout: set[str]) -> None:
        reject_unsafe_output(output)
        self.output = output
        self.contract = contract
        self.topic = topic
        self.holdout = holdout
        self.records: list[dict[str, Any]] = []
        self.source_hashes: set[str] = set()
        self.tensor_hashes: set[str] = set()
        if not self.output.exists():
            self.output.mkdir(parents=True)
        for name in ("provenance", "samples"):
            (self.output / name).mkdir()

    def add(self, frame: ImageFrame) -> bool:
        if not frame.frame_id.strip():
            raise CalibrationRejected("camera_frame_id_missing")
        source_sha = sha256_bytes(frame.data)
        if source_sha in self.holdout:
            raise CalibrationRejected("evaluation_holdout_overlap")
        if source_sha in self.source_hashes:
            return False
        rgb = rgb_from_ros_image(
            data=frame.data, width=frame.width, height=frame.height,
            step=frame.step, encoding=frame.encoding,
        )
        tensor = preprocess_dosod_rgb(rgb)
        index = len(self.records)
        name = f"frame_{index:06d}"
        sample = self.output / "samples" / f"{name}.npy"
        pending = sample.with_name(f".{sample.name}.pending.{os.getpid()}")
        with pending.open("wb") as stream:
            np.save(stream, tensor, allow_pickle=False)
        pending.replace(sample)
        tensor_sha = sha256_file(sample)
        if tensor_sha in self.tensor_hashes:
            sample.unlink()
            return False
        provenance_relative = f"provenance/{name}.json"
        atomic_write_json(
            self.output / provenance_relative,
            {
                "topic": self.topic,
                "header_stamp_ns": frame.stamp_ns,
                "frame_id": frame.frame_id,
                "encoding": frame.encoding,
                "width": frame.width,
                "height": frame.height,
                "step": frame.step,
                "source_role": "calibration_only",
            },
        )
        relative = f"samples/{name}.npy"
        self.records.append(
            {
                "relative_path": relative,
                "byte_size": sample.stat().st_size,
                "sha256": tensor_sha,
                "source_sha256": source_sha,
                "source_role": "calibration_only",
                "provenance_relative_path": provenance_relative,
            }
        )
        self.source_hashes.add(source_sha)
        self.tensor_hashes.add(tensor_sha)
        return True

    def freeze(
        self, *, authorization: dict[str, Any], hardware: dict[str, Any],
        publisher_identity: list[dict[str, str]], camera_frame_id: str,
    ) -> Path:
        calibration = self.contract["calibration"]
        if len(self.records) < calibration["minimum_sample_count"]:
            raise CalibrationRejected("minimum_sample_count_not_reached")
        manifest = {
            "schema_version": 1,
            "dataset_id": "tzcup_formal_s100_real_ros_image_calibration_v1",
            "status": "FROZEN",
            "model_sha256": self.contract["model"]["sha256"],
            "vocabulary_sha256": self.contract["vocabulary"]["sha256"],
            "preprocessing_sha256": canonical_sha256(self.contract["preprocessing"]),
            "evaluation_holdout_source_sha256": sorted(self.holdout),
            "authorization_record": authorization,
            "hardware": hardware,
            "publisher_identity": publisher_identity,
            "camera_frame_id": camera_frame_id,
            "records": self.records,
            "records_sha256": canonical_sha256(self.records),
        }
        manifest_path = self.output / calibration["manifest_name"]
        atomic_write_json(manifest_path, manifest)
        return manifest_path


def graph_types(node: Any, topic: str) -> list[str]:
    return [item_type for name, types in node.get_topic_names_and_types() if name == topic for item_type in types]


def publisher_identity(publishers: Iterable[Any]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for item in publishers:
        if isinstance(item, dict):
            row = {key: str(item.get(key, "")) for key in ("node_name", "node_namespace", "topic_type", "endpoint_gid")}
        else:
            gid = getattr(item, "endpoint_gid", b"")
            row = {
                "node_name": str(getattr(item, "node_name", "")),
                "node_namespace": str(getattr(item, "node_namespace", "")),
                "topic_type": str(getattr(item, "topic_type", "")),
                "endpoint_gid": bytes(gid).hex() if isinstance(gid, (bytes, bytearray)) else str(gid),
            }
        rows.append(row)
    return rows


def validate_publishers(rows: list[dict[str, str]]) -> None:
    if not rows:
        raise CalibrationRejected("image_publisher_missing")
    for row in rows:
        identity = " ".join(row.values()).lower()
        if not row["node_name"] or any(token in identity for token in FORBIDDEN_PUBLISHER_TOKENS):
            raise CalibrationRejected("image_publisher_replay_or_untrusted")
        if row["topic_type"] and row["topic_type"] != IMAGE_TYPE:
            raise CalibrationRejected("image_publisher_type_invalid")


def collect(
    *, output: Path, topic: str, target_count: int, contract_path: Path,
    holdout_paths: Iterable[Path], holdout_values: Iterable[str], timeout_s: float,
    accept_operator_authorized_real_capture: bool, authorization_path: Path | None,
    hardware_probe: Any = probe_hardware,
) -> Path:
    store: CalibrationStore | None = None
    contract: dict[str, Any] | None = None
    authorization: dict[str, Any] | None = None
    hardware: dict[str, Any] | None = None
    publishers: list[dict[str, str]] = []
    camera_frame_id: str | None = None
    node: Any = None
    rclpy: Any = None
    try:
        reject_unsafe_output(output)
        contract = load_contract(
            contract_path, allow_paused_capture=accept_operator_authorized_real_capture
        )
        authorization = authorization_record(
            authorization_path, accepted=accept_operator_authorized_real_capture,
            output=output, topic=topic, contract=contract, target_count=target_count,
        )
        require_capture_status(contract, authorization)
        hardware = require_s100p_hardware(hardware_probe)
        if target_count < contract["calibration"]["minimum_sample_count"]:
            raise CalibrationRejected("target_count_below_contract_minimum")
        holdout = load_holdout_hashes(holdout_paths, holdout_values)
        import rclpy as loaded_rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        rclpy = loaded_rclpy
        rclpy.init()
        node = rclpy.create_node("formal_s100_calibration_collector")
        deadline = time.monotonic() + timeout_s
        if bool(node.get_parameter("use_sim_time").value):
            raise CalibrationRejected("use_sim_time_forbidden")
        while time.monotonic() < deadline and (not graph_types(node, topic) or not node.get_publishers_info_by_topic(topic)):
            rclpy.spin_once(node, timeout_sec=0.1)
        validate_topic(topic, graph_types(node, topic))
        publishers = publisher_identity(node.get_publishers_info_by_topic(topic))
        validate_publishers(publishers)
        store = CalibrationStore(output, contract, topic, holdout)

        def on_image(message: Image) -> None:
            nonlocal camera_frame_id
            stamp = message.header.stamp
            frame = ImageFrame(
                data=bytes(message.data), width=int(message.width), height=int(message.height),
                step=int(message.step), encoding=str(message.encoding),
                stamp_ns=int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
                frame_id=str(message.header.frame_id),
            )
            try:
                accepted = store.add(frame)
            except CalibrationRejected as exc:
                node.get_logger().error(f"calibration_frame_rejected:{exc}")
                return
            if accepted:
                camera_frame_id = frame.frame_id
                node.get_logger().info(f"calibration_frame_accepted:{len(store.records)}/{target_count}")

        subscription = node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
        while time.monotonic() < deadline and len(store.records) < target_count:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(subscription)
        if len(store.records) < target_count:
            raise CalibrationRejected("target_count_not_reached_before_timeout")
        if camera_frame_id is None:
            raise CalibrationRejected("camera_frame_id_missing")
        manifest = store.freeze(
            authorization=authorization, hardware=hardware, publisher_identity=publishers,
            camera_frame_id=camera_frame_id,
        )
        return manifest
    except BaseException as exc:
        write_blocked_receipt_if_manifest_absent(output, contract, {
            "schema_version": 1, "status": "BLOCKED", "reason": f"{type(exc).__name__}:{exc}",
            "collection_complete": False, "partial_data_not_reusable": True,
            "frozen_manifest_written": False, "authorization_record": authorization,
            "hardware": hardware, "publisher_identity": publishers,
            "camera_frame_id": camera_frame_id,
            "accepted_frame_count": len(store.records) if store else 0,
        }, allow_partial=store is not None)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None:
            rclpy.try_shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--target-count", type=int, default=500)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--evaluation-holdout-file", type=Path, action="append", default=[])
    parser.add_argument("--evaluation-holdout-source-sha256", action="append", default=[])
    parser.add_argument("--timeout", type=float, default=1800.0)
    parser.add_argument("--accept-operator-authorized-real-capture", action="store_true")
    parser.add_argument("--authorization-record", type=Path)
    args = parser.parse_args()
    try:
        manifest = collect(
            output=args.output, topic=args.topic, target_count=args.target_count,
            contract_path=args.contract, holdout_paths=args.evaluation_holdout_file,
            holdout_values=args.evaluation_holdout_source_sha256, timeout_s=args.timeout,
            accept_operator_authorized_real_capture=args.accept_operator_authorized_real_capture,
            authorization_path=args.authorization_record,
        )
    except Exception as exc:
        print(f"calibration_collection_blocked:{exc}")
        return 2
    print(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
