#!/usr/bin/env python3
"""Prepare a NON_FORMAL public-Gazebo DOSOD calibration set.

This is deliberately not the real-S100 collector: it accepts only public
Gazebo camera frames and writes no receipt, PASS claim, or board evidence.
The live caller must first hold the shared Gazebo lock; this module never
starts Gazebo, stages entities, or subscribes to truth/evaluator topics.
"""
from __future__ import annotations

import hashlib
import argparse
from collections import OrderedDict
import json
import math
import os
import re
import secrets
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from collect_formal_s100_calibration_frames import (
    CalibrationRejected, atomic_write_json, canonical_sha256, preprocess_dosod_rgb,
    rgb_from_ros_image, sha256_file,
)

CLASS_IDS = ("litter_cube", "fallen_leaves", "dust_or_soil", "puddle")
MIN_HOLDOUT_SAMPLES = 100
FORBIDDEN = ("hidden", "truth", "evaluator", "replay", "bag")
SCENE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]*$")
FORMAL_CAMPUS_CONFIG = Path(__file__).resolve().parents[1] / "starter_ws/src/sanitation_formal_campus_integration/config/formal_campus_integration.yaml"
FORMAL_RGB_TOPIC = "/camera/color/image_raw"
FORMAL_CAMERA_INFO_TOPIC = "/camera/color/camera_info"
PAIR_CACHE_LIMIT = 64


def _unsafe(path: Path) -> bool:
    return any(item.is_symlink() for item in (path, *path.parents))


def require_empty_output(output: Path) -> None:
    if _unsafe(output) or (output.exists() and (not output.is_dir() or any(output.iterdir()))):
        raise CalibrationRejected("output_must_be_fresh_empty_nonlink_directory")


def load_scene_plan(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("source_domain") != "public_gazebo_sensor":
        raise CalibrationRejected("scene_plan_not_public_gazebo_sensor")
    if value.get("class_ids") != list(CLASS_IDS):
        raise CalibrationRejected("scene_plan_class_ids_not_current_dosod_four")
    groups = value.get("scene_groups")
    if not isinstance(groups, dict):
        raise CalibrationRejected("scene_plan_groups_invalid")
    calibration, holdout = groups.get("calibration"), groups.get("holdout")
    if not all(
        isinstance(rows, list)
        and rows
        and all(isinstance(x, str) and SCENE_ID_RE.fullmatch(x) for x in rows)
        for rows in (calibration, holdout)
    ):
        raise CalibrationRejected("scene_plan_groups_invalid")
    if len(set(calibration)) != len(calibration) or len(set(holdout)) != len(holdout) or set(calibration) & set(holdout):
        raise CalibrationRejected("scene_plan_not_scene_disjoint")
    return value

def load_scene_selector(path: Path) -> dict[str, str]:
    if path.is_symlink() or not path.is_file(): raise CalibrationRejected("scene_selector_not_regular_nonlink")
    value=json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value,dict): raise CalibrationRejected("scene_selector_invalid")
    if value.get("state") not in {"ACTIVE", "INACTIVE"} or not re.fullmatch(r"[0-9a-f]{32}", str(value.get("generation_nonce", ""))):
        raise CalibrationRejected("scene_selector_invalid")
    if value["state"] == "INACTIVE":
        return {"state": "INACTIVE", "generation_nonce": value["generation_nonce"]}
    required=("scene_id", "episode_manifest_sha256", "generation_nonce")
    if any(not isinstance(value.get(k), str) or not value[k] for k in required) or not SCENE_ID_RE.fullmatch(value["scene_id"]) or not re.fullmatch(r"[0-9a-f]{64}", value["episode_manifest_sha256"]):
        raise CalibrationRejected("scene_selector_invalid")
    return {"state": "ACTIVE", **{k:value[k] for k in required}}

def atomic_scene_selector(path: Path, value: dict[str,str]) -> None:
    if path.is_symlink(): raise CalibrationRejected("scene_selector_not_regular_nonlink")
    pending=path.with_name(f".{path.name}.pending.{os.getpid()}")
    with pending.open("x",encoding="utf-8") as handle: json.dump(value,handle,sort_keys=True); handle.write("\n")
    os.replace(pending,path)


def select_scene_from_manifest(*, selector: Path, scene_id: str, episode_manifest: Path) -> dict[str, str]:
    """Atomically bind one public scene selector to its exact manifest bytes."""

    if not SCENE_ID_RE.fullmatch(scene_id):
        raise CalibrationRejected("scene_selector_scene_id_invalid")
    if episode_manifest.is_symlink() or not episode_manifest.is_file():
        raise CalibrationRejected("episode_manifest_not_regular_nonlink")
    value = {
        "state": "ACTIVE",
        "scene_id": scene_id,
        "episode_manifest_sha256": sha256_file(episode_manifest),
        "generation_nonce": secrets.token_hex(16),
    }
    atomic_scene_selector(selector, value)
    return value


def deactivate_scene_selector(selector: Path) -> dict[str, str]:
    value = {"state": "INACTIVE", "generation_nonce": secrets.token_hex(16)}
    atomic_scene_selector(selector, value)
    return value


def load_contract(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    prep, cal = value.get("preprocessing"), value.get("calibration")
    if not isinstance(prep, dict) or not isinstance(cal, dict) or value.get("vocabulary", {}).get("semantic_class_ids") != list(CLASS_IDS):
        raise CalibrationRejected("contract_four_class_or_schema_invalid")
    expected = {"source_color_space": "RGB", "resize": {"height": 640, "width": 640, "interpolation": "bilinear"}, "tensor_dtype": "float32", "tensor_layout": "NCHW", "tensor_shape": [1, 3, 640, 640], "value_range": [0.0, 1.0]}
    if any(prep.get(k) != v for k, v in expected.items()) or cal.get("minimum_sample_count", 0) < 500:
        raise CalibrationRejected("contract_preprocessing_or_minimum_invalid")
    return value


def require_collection_capacity(*, plan: dict[str, Any], contract: dict[str, Any], per_scene_quota: int) -> None:
    """Reject an undersized scene plan before creating a ROS participant."""

    if len(plan["scene_groups"]["calibration"]) * per_scene_quota < contract["calibration"]["minimum_sample_count"]:
        raise CalibrationRejected("calibration_scene_plan_capacity_below_minimum")
    if len(plan["scene_groups"]["holdout"]) * per_scene_quota < MIN_HOLDOUT_SAMPLES:
        raise CalibrationRejected("holdout_scene_plan_capacity_below_minimum")


def require_formal_camera_topic_pair(*, image_topic: str, camera_info_topic: str) -> None:
    """Bind this collector to the explicit formal-campus alias pair."""

    expected = (
        "/sensors/front_rgbd/depth/image_rect_raw/image: /camera/color/image_raw",
        "/sensors/front_rgbd/depth/image_rect_raw/camera_info: /camera/color/camera_info",
    )
    try:
        source = FORMAL_CAMPUS_CONFIG.read_text(encoding="utf-8")
    except OSError as exc:
        raise CalibrationRejected(f"formal_camera_alias_config_unreadable:{type(exc).__name__}") from exc
    if any(line not in source for line in expected):
        raise CalibrationRejected("formal_camera_alias_config_drift")
    if (image_topic, camera_info_topic) != (FORMAL_RGB_TOPIC, FORMAL_CAMERA_INFO_TOPIC):
        raise CalibrationRejected("formal_camera_topic_pair_not_authorized")


def validate_camera(*, image_frame: str, image_stamp_ns: int, width: int, height: int, camera: dict[str, Any]) -> None:
    if image_stamp_ns <= 0 or not image_frame or camera.get("frame_id") != image_frame or camera.get("stamp_ns") != image_stamp_ns:
        raise CalibrationRejected("camera_info_frame_or_stamp_mismatch")
    if camera.get("width") != width or camera.get("height") != height:
        raise CalibrationRejected("camera_info_dimensions_mismatch")
    k = camera.get("k")
    if not isinstance(k, list) or len(k) != 9 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in k) or k[0] <= 0 or k[4] <= 0:
        raise CalibrationRejected("camera_info_intrinsics_invalid")


@dataclass(frozen=True)
class Frame:
    scene_id: str; topic: str; frame_id: str; stamp_ns: int; data: bytes; width: int; height: int; step: int; encoding: str; camera: dict[str, Any]
    generation_nonce: str = ""; episode_manifest_sha256: str = ""


class FreshPairCache:
    """Bounded exact-stamp Image/CameraInfo pairing within one selector nonce."""

    def __init__(self, limit: int = PAIR_CACHE_LIMIT) -> None:
        if not isinstance(limit, int) or isinstance(limit, bool) or limit < 1:
            raise CalibrationRejected("pair_cache_limit_invalid")
        self.limit = limit
        self.images: OrderedDict[tuple[str, int], tuple[dict[str, str], Any]] = OrderedDict()
        self.infos: OrderedDict[tuple[str, int], tuple[dict[str, str], Any]] = OrderedDict()

    def reset(self) -> None:
        self.images.clear()
        self.infos.clear()

    def _put(self, cache: OrderedDict[tuple[str, int], tuple[dict[str, str], Any]], key: tuple[str, int], value: tuple[dict[str, str], Any]) -> None:
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > self.limit:
            cache.popitem(last=False)

    @staticmethod
    def _same_selector(left: dict[str, str], right: dict[str, str]) -> bool:
        return left.get("state") == right.get("state") == "ACTIVE" and all(left[key] == right[key] for key in ("generation_nonce", "scene_id", "episode_manifest_sha256"))

    def put_image(self, key: tuple[str, int], selector: dict[str, str], image: Any) -> tuple[dict[str, str], Any, Any] | None:
        self._put(self.images, key, (selector, image))
        return self._take(key)

    def put_info(self, key: tuple[str, int], selector: dict[str, str], info: Any) -> tuple[dict[str, str], Any, Any] | None:
        self._put(self.infos, key, (selector, info))
        return self._take(key)

    def _take(self, key: tuple[str, int]) -> tuple[dict[str, str], Any, Any] | None:
        image = self.images.get(key)
        info = self.infos.get(key)
        if image is None or info is None:
            return None
        self.images.pop(key)
        self.infos.pop(key)
        if not self._same_selector(image[0], info[0]):
            return None
        return image[0], image[1], info[1]


class PublicGazeboStore:
    def __init__(self, output: Path, contract: dict[str, Any], plan: dict[str, Any], *, per_scene_quota: int = 1) -> None:
        if not isinstance(per_scene_quota, int) or isinstance(per_scene_quota, bool) or per_scene_quota < 1:
            raise CalibrationRejected("per_scene_quota_invalid")
        require_empty_output(output); self.output, self.contract, self.plan = output, contract, plan
        self.per_scene_quota = per_scene_quota
        self.calibration_scenes = set(plan["scene_groups"]["calibration"]); self.holdout_scenes = set(plan["scene_groups"]["holdout"])
        self.records: list[dict[str, Any]] = []
        self.holdout_sources: set[str] = set()
        self.holdout_records: list[dict[str, Any]] = []
        self.holdout_scene_counts: dict[str, int] = {}
        self.calibration_scene_counts: dict[str, int] = {}
        self.sources: set[str] = set(); self.tensors: set[str] = set(); self.tensor_contents: set[str] = set()
        self.duplicate_source_count = 0; self.duplicate_tensor_count = 0
        output.mkdir(parents=True); (output / "samples").mkdir(); (output / "holdout_samples").mkdir(); (output / "provenance").mkdir()

    def add(self, frame: Frame) -> bool:
        if any(token in frame.topic.lower() for token in FORBIDDEN) or frame.scene_id not in self.calibration_scenes | self.holdout_scenes:
            raise CalibrationRejected("frame_topic_or_scene_not_public_plan")
        validate_camera(image_frame=frame.frame_id, image_stamp_ns=frame.stamp_ns, width=frame.width, height=frame.height, camera=frame.camera)
        counts = self.holdout_scene_counts if frame.scene_id in self.holdout_scenes else self.calibration_scene_counts
        if counts.get(frame.scene_id, 0) >= self.per_scene_quota:
            return False
        source = hashlib.sha256(frame.data).hexdigest()
        if source in self.sources:
            self.duplicate_source_count += 1
            return False
        self.sources.add(source)
        rgb = rgb_from_ros_image(data=frame.data, width=frame.width, height=frame.height, step=frame.step, encoding=frame.encoding)
        tensor = preprocess_dosod_rgb(rgb)
        tensor_bytes = tensor.tobytes()
        tensor_content_sha = hashlib.sha256(tensor_bytes).hexdigest()
        if tensor_content_sha in self.tensor_contents:
            self.duplicate_tensor_count += 1
            return False
        self.tensor_contents.add(tensor_content_sha)
        if frame.scene_id in self.holdout_scenes:
            name = f"holdout_{len(self.holdout_records):06d}"
            sample = self.output / "holdout_samples" / f"{name}.npy"
            pending = sample.with_name(f".{sample.name}.pending.{os.getpid()}")
            with pending.open("xb") as handle:
                np.save(handle, tensor, allow_pickle=False)
            os.replace(pending, sample)
            tensor_sha = sha256_file(sample)
            self.holdout_sources.add(source)
            self.holdout_records.append({
                "scene_id": frame.scene_id,
                "source_sha256": source,
                "generation_nonce": frame.generation_nonce,
                "episode_manifest_sha256": frame.episode_manifest_sha256,
                "relative_path": f"holdout_samples/{name}.npy",
                "byte_size": sample.stat().st_size,
                "sha256": tensor_sha,
            })
            counts[frame.scene_id] = counts.get(frame.scene_id, 0) + 1
            return False
        name = f"{len(self.records):06d}"; sample = self.output / "samples" / f"{name}.npy"
        pending = sample.with_name(f".{sample.name}.pending.{os.getpid()}")
        with pending.open("xb") as handle: np.save(handle, tensor, allow_pickle=False)
        os.replace(pending, sample); tensor_sha = sha256_file(sample)
        if tensor_sha in self.tensors: sample.unlink(); self.duplicate_tensor_count += 1; return False
        self.tensors.add(tensor_sha)
        provenance = {"source_domain": "public_gazebo_sensor", "scene_id": frame.scene_id, "topic": frame.topic, "frame_id": frame.frame_id, "stamp_ns": frame.stamp_ns, "encoding": frame.encoding, "camera": frame.camera, "source_sha256": source, "generation_nonce": frame.generation_nonce, "episode_manifest_sha256": frame.episode_manifest_sha256}
        atomic_write_json(self.output / "provenance" / f"{name}.json", provenance)
        self.records.append({"relative_path": f"samples/{name}.npy", "byte_size": sample.stat().st_size, "sha256": tensor_sha, "source_sha256": source, "source_role": "calibration_only", "scene_id": frame.scene_id, "generation_nonce": frame.generation_nonce, "episode_manifest_sha256": frame.episode_manifest_sha256, "provenance": f"provenance/{name}.json"})
        counts[frame.scene_id] = counts.get(frame.scene_id, 0) + 1
        return True

    def collection_complete(self) -> bool:
        return (
            all(self.calibration_scene_counts.get(scene, 0) >= self.per_scene_quota for scene in self.calibration_scenes)
            and all(self.holdout_scene_counts.get(scene, 0) >= self.per_scene_quota for scene in self.holdout_scenes)
        )

    def progress(self) -> dict[str, Any]:
        return {
            "status": "COLLECTING",
            "formal_passed": False,
            "per_scene_quota": self.per_scene_quota,
            "calibration_records": len(self.records),
            "holdout_source_count": len(self.holdout_sources),
            "duplicate_source_count": self.duplicate_source_count,
            "duplicate_tensor_count": self.duplicate_tensor_count,
            "calibration_scene_counts": dict(sorted(self.calibration_scene_counts.items())),
            "holdout_scene_counts": dict(sorted(self.holdout_scene_counts.items())),
            "collection_complete": self.collection_complete(),
        }

    def freeze(self) -> Path:
        minimum = self.contract["calibration"]["minimum_sample_count"]
        bindings = [*self.records, *self.holdout_records]
        bindings_valid = all(
            re.fullmatch(r"[0-9a-f]{32}", str(row.get("generation_nonce", "")))
            and re.fullmatch(r"[0-9a-f]{64}", str(row.get("episode_manifest_sha256", "")))
            for row in bindings
        )
        if (
            not self.collection_complete()
            or len(self.records) < minimum
            or len(self.holdout_records) < MIN_HOLDOUT_SAMPLES
            or not bindings_valid
        ):
            raise CalibrationRejected("calibration_or_scene_disjoint_holdout_below_minimum")
        value = {"schema_version": 1, "status": "FROZEN", "dataset_id": "tzcup_public_gazebo_dosod_calibration_v1", "source_domain": "public_gazebo_sensor", "formal_passed": False, "class_ids": list(CLASS_IDS), "preprocessing_sha256": canonical_sha256(self.contract["preprocessing"]), "calibration_sample_count": len(self.records), "calibration_scene_count": len(self.calibration_scene_counts), "evaluation_holdout_sample_count": len(self.holdout_records), "evaluation_holdout_scene_count": len(self.holdout_scene_counts), "evaluation_holdout_source_sha256": sorted(self.holdout_sources), "evaluation_holdout_scene_ids": sorted(self.holdout_scene_counts), "duplicate_source_count": self.duplicate_source_count, "duplicate_tensor_count": self.duplicate_tensor_count, "holdout_records": self.holdout_records, "per_scene_quota": self.per_scene_quota, "records": self.records}
        target = self.output / self.contract["calibration"]["manifest_name"]
        atomic_write_json(target, value); return target


def frame_from_ros(*, scene_id: str, topic: str, image: Any, camera_info: Any, generation_nonce: str = "", episode_manifest_sha256: str = "") -> Frame:
    """Convert only an exactly paired ROS Image/CameraInfo pair to ``Frame``."""
    stamp = int(image.header.stamp.sec) * 1_000_000_000 + int(image.header.stamp.nanosec)
    camera_stamp = int(camera_info.header.stamp.sec) * 1_000_000_000 + int(camera_info.header.stamp.nanosec)
    camera = {"frame_id": str(camera_info.header.frame_id), "stamp_ns": camera_stamp, "width": int(camera_info.width), "height": int(camera_info.height), "k": [float(x) for x in camera_info.k]}
    return Frame(scene_id, topic, str(image.header.frame_id), stamp, bytes(image.data), int(image.width), int(image.height), int(image.step), str(image.encoding), camera, generation_nonce, episode_manifest_sha256)


def collect_live(*, output: Path, plan: dict[str, Any], contract: dict[str, Any], topic: str, camera_info_topic: str, timeout_s: float, selector_path: Path, per_scene_quota: int, progress_path: Path) -> Path:
    """Attach read-only to one already-running public Gazebo camera scene.

    A lock-owning runner is responsible for advancing between public scene IDs;
    this function has no publisher, service client, Gazebo command, or truth
    subscription.  It fails rather than infer a CameraInfo pairing.
    """
    if timeout_s <= 0 or not topic.startswith("/") or any(token in topic.lower() for token in FORBIDDEN):
        raise CalibrationRejected("live_topic_or_timeout_invalid")
    if selector_path.is_symlink() or (selector_path.exists() and not selector_path.is_file()):
        raise CalibrationRejected("scene_selector_not_regular_nonlink")
    require_collection_capacity(plan=plan, contract=contract, per_scene_quota=per_scene_quota)
    require_formal_camera_topic_pair(image_topic=topic, camera_info_topic=camera_info_topic)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init(); node = rclpy.create_node("public_gazebo_dosod_calibration_collector")
    pairs = FreshPairCache()
    store = PublicGazeboStore(output, contract, plan, per_scene_quota=per_scene_quota)
    selector_nonce: str | None = None
    errors: list[str] = []
    def selection() -> tuple[dict[str, str], bool]:
        nonlocal selector_nonce
        selected = load_scene_selector(selector_path)
        changed = selected["generation_nonce"] != selector_nonce
        if changed:
            selector_nonce = selected["generation_nonce"]
            pairs.reset()
        return selected, changed
    def save_progress() -> None:
        atomic_write_json(progress_path, store.progress())
    def consume_pair(pair: tuple[dict[str, str], Any, Any] | None) -> None:
        if pair is None:
            return
        selected, image_message, info_message = pair
        try:
            store.add(frame_from_ros(scene_id=selected["scene_id"], topic=topic, image=image_message, camera_info=info_message, generation_nonce=selected["generation_nonce"], episode_manifest_sha256=selected["episode_manifest_sha256"]))
            save_progress()
        except CalibrationRejected as exc:
            errors.append(str(exc))
    def on_info(message: CameraInfo) -> None:
        try:
            selected, _ = selection()
        except CalibrationRejected as exc:
            errors.append(str(exc))
            return
        if selected["state"] != "ACTIVE":
            return
        key = (str(message.header.frame_id), int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec))
        consume_pair(pairs.put_info(key, selected, message))
    def on_image(message: Image) -> None:
        try:
            selected, changed = selection()
        except CalibrationRejected as exc:
            errors.append(str(exc))
            return
        if selected["state"] != "ACTIVE":
            return
        key = (str(message.header.frame_id), int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec))
        consume_pair(pairs.put_image(key, selected, message))
    try:
        node.create_subscription(CameraInfo, topic.rsplit("/", 1)[0] + "/camera_info", on_info, qos_profile_sensor_data)
        node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
        save_progress()
        deadline = __import__("time").monotonic() + timeout_s
        while __import__("time").monotonic() < deadline and not store.collection_complete():
            rclpy.spin_once(node, timeout_sec=0.2)
            if errors:
                raise CalibrationRejected(errors[0])
        if not store.collection_complete():
            raise CalibrationRejected("scene_quota_or_collection_timeout")
        return store.freeze()
    finally:
        node.destroy_node(); rclpy.shutdown()


def main() -> int:
    """Validate public scene partition and the frozen preprocessing contract.

    This intentionally has no ``--launch`` or Gazebo-control option.  The
    later lock-owning public runner may call this preflight before it attaches
    to a live Image/CameraInfo pair.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-plan", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--live-output", type=Path)
    parser.add_argument("--scene-id")
    parser.add_argument("--image-topic")
    parser.add_argument("--camera-info-topic")
    parser.add_argument("--timeout-sec", type=float)
    parser.add_argument("--scene-selector", type=Path)
    parser.add_argument("--per-scene-quota", type=int)
    parser.add_argument("--progress-output", type=Path)
    parser.add_argument("--write-scene-selector", action="store_true")
    parser.add_argument("--deactivate-scene-selector", action="store_true")
    parser.add_argument("--episode-manifest", type=Path)
    args = parser.parse_args()
    plan, contract = load_scene_plan(args.scene_plan), load_contract(args.contract)
    if args.write_scene_selector and args.deactivate_scene_selector:
        parser.error("selector cannot be activated and deactivated together")
    if args.deactivate_scene_selector:
        if args.scene_selector is None:
            parser.error("selector deactivation requires --scene-selector")
        print(json.dumps(deactivate_scene_selector(args.scene_selector), sort_keys=True))
        return 0
    if args.write_scene_selector:
        if not all((args.scene_selector, args.scene_id, args.episode_manifest)):
            parser.error("selector writing requires --scene-selector --scene-id --episode-manifest")
        print(json.dumps(select_scene_from_manifest(selector=args.scene_selector, scene_id=args.scene_id, episode_manifest=args.episode_manifest), sort_keys=True))
        return 0
    live = (args.live_output, args.image_topic, args.camera_info_topic, args.timeout_sec, args.scene_selector, args.per_scene_quota, args.progress_output)
    if any(value is not None for value in live):
        if any(value is None for value in live): parser.error("all live arguments are required together")
        print(collect_live(output=args.live_output, plan=plan, contract=contract, topic=args.image_topic, camera_info_topic=args.camera_info_topic, timeout_s=args.timeout_sec, selector_path=args.scene_selector, per_scene_quota=args.per_scene_quota, progress_path=args.progress_output))
        return 0
    print(json.dumps({"report_id": "tzcup_public_gazebo_dosod_calibration_preflight", "status": "NON_FORMAL_PREPARED", "formal_passed": False, "calibration_scenes": len(plan["scene_groups"]["calibration"]), "holdout_scenes": len(plan["scene_groups"]["holdout"]), "preprocessing_sha256": canonical_sha256(contract["preprocessing"])}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
