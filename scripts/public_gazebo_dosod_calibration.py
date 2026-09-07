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
from dataclasses import asdict, dataclass
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
MOBILE_EVIDENCE_MAX_AGE_NS = 2_000_000_000
POSE_SEPARATION_M = 0.5
YAW_SEPARATION_RAD = math.radians(15.0)
RETRYABLE_MOBILE_REJECTIONS = frozenset({
    "mobile_nav2_action_server_missing",
    "mobile_nav2_goal_not_bt_navigator_executing",
    "mobile_odom_source_missing",
    "mobile_image_source_missing",
    "mobile_camera_info_source_missing",
    "mobile_odom_or_camera_tf_missing",
    "mobile_odom_or_camera_tf_not_fresh",
    "mobile_camera_tf_source_missing",
    "mobile_pose_not_materially_distinct",
})


def _unsafe(path: Path) -> bool:
    return any(item.is_symlink() for item in (path, *path.parents))


def require_empty_output(output: Path) -> None:
    if _unsafe(output) or (output.exists() and (not output.is_dir() or any(output.iterdir()))):
        raise CalibrationRejected("output_must_be_fresh_empty_nonlink_directory")


def validate_pilot_manifest(pilot_manifest: Path, *, plan: dict[str, Any], contract: dict[str, Any]) -> dict[str, Any]:
    if _unsafe(pilot_manifest) or not pilot_manifest.is_file():
        raise CalibrationRejected("pilot_review_receipt_or_manifest_missing")
    try: value = json.loads(pilot_manifest.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc: raise CalibrationRejected("pilot_manifest_invalid") from exc
    if not isinstance(value, dict) or value.get("status") != "NON_FORMAL_PILOT_CAPTURED" or value.get("formal_passed") is not False or value.get("pilot_scene") != "map-0-mission-0" or value.get("record_count") != 25 or value.get("per_scene_quota") != 25 or value.get("plan_sha256") != canonical_sha256(plan) or value.get("preprocessing_sha256") != canonical_sha256(contract["preprocessing"]):
        raise CalibrationRejected("pilot_manifest_invalid")
    records = value.get("records")
    if not isinstance(records, list) or len(records) != 25 or value.get("record_sha256") != canonical_sha256(records):
        raise CalibrationRejected("pilot_manifest_invalid")
    seen_source: set[str] = set(); seen_tensor: set[str] = set(); accepted_poses: list[tuple[float, float, float]] = []
    for row in records:
        relative = row.get("relative_path") if isinstance(row, dict) else None
        if not isinstance(relative, str) or relative != f"samples/{len(seen_source):06d}.npy" or "\\" in relative or row.get("scene_id") != "map-0-mission-0" or row.get("source_role") != "calibration_only" or not re.fullmatch(r"[0-9a-f]{64}", str(row.get("source_sha256", ""))) or not re.fullmatch(r"[0-9a-f]{64}", str(row.get("sha256", ""))) or not re.fullmatch(r"[0-9a-f]{32}", str(row.get("generation_nonce", ""))) or not re.fullmatch(r"[0-9a-f]{64}", str(row.get("episode_manifest_sha256", ""))):
            raise CalibrationRejected("pilot_manifest_invalid")
        sample = pilot_manifest.parent / relative
        if _unsafe(sample) or not sample.is_file() or sample.stat().st_size != row.get("byte_size") or sha256_file(sample) != row["sha256"] or row["source_sha256"] in seen_source or row["sha256"] in seen_tensor:
            raise CalibrationRejected("pilot_manifest_invalid")
        try: tensor = np.load(sample, allow_pickle=False)
        except (OSError, ValueError) as exc: raise CalibrationRejected("pilot_manifest_invalid") from exc
        if tensor.dtype != np.float32 or tensor.shape != (1, 3, 640, 640) or not np.isfinite(tensor).all() or np.any(tensor < 0.0) or np.any(tensor > 1.0):
            raise CalibrationRejected("pilot_manifest_invalid")
        proof = pilot_manifest.parent / f"provenance/{len(seen_source):06d}.json"
        if row.get("provenance") != f"provenance/{len(seen_source):06d}.json" or _unsafe(proof) or not proof.is_file() or proof.stat().st_size != row.get("provenance_byte_size") or sha256_file(proof) != row.get("provenance_sha256"):
            raise CalibrationRejected("pilot_manifest_invalid")
        try: proof_value = json.loads(proof.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc: raise CalibrationRejected("pilot_manifest_invalid") from exc
        stamp = proof_value.get("stamp_ns") if isinstance(proof_value, dict) else None
        if not isinstance(proof_value, dict) or any(proof_value.get(key) != row.get(key) for key in ("scene_id", "source_sha256", "generation_nonce", "episode_manifest_sha256")) or proof_value.get("source_domain") != "public_gazebo_sensor" or proof_value.get("topic") != FORMAL_RGB_TOPIC or not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
            raise CalibrationRejected("pilot_manifest_invalid")
        evidence = proof_value.get("mobile_evidence")
        if not isinstance(evidence, dict): raise CalibrationRejected("pilot_manifest_invalid")
        try:
            mobile = MobileEvidence(**evidence)
            require_mobile_evidence(image_stamp_ns=proof_value["stamp_ns"], image_frame=str(proof_value.get("frame_id", "")), evidence=mobile, accepted_poses=accepted_poses)
        except (TypeError, CalibrationRejected) as exc: raise CalibrationRejected("pilot_manifest_invalid") from exc
        accepted_poses.append((mobile.odom_x, mobile.odom_y, mobile.odom_yaw))
        seen_source.add(row["source_sha256"]); seen_tensor.add(row["sha256"])
    sheet = value.get("contact_sheet")
    if not isinstance(sheet, dict) or sheet.get("relative_path") != "pilot_contact_sheet.png" or sheet.get("record_sha256") != value["record_sha256"] or sheet.get("scene_id") != "map-0-mission-0" or sheet.get("record_count") != 25:
        raise CalibrationRejected("pilot_manifest_invalid")
    contact = pilot_manifest.parent / sheet["relative_path"]
    if _unsafe(contact) or not contact.is_file() or contact.stat().st_size != sheet.get("byte_size") or sha256_file(contact) != sheet.get("sha256"):
        raise CalibrationRejected("pilot_manifest_invalid")
    try:
        from PIL import Image
        with Image.open(contact) as image: image.verify()
        with Image.open(contact) as image:
            if image.size != (800, 800): raise ValueError("contact_sheet_size")
    except (ImportError, OSError, ValueError) as exc: raise CalibrationRejected("pilot_manifest_invalid") from exc
    return value


def require_full_review_receipt(receipt: Path | None, pilot_manifest: Path | None, *, plan: dict[str, Any], contract: dict[str, Any]) -> dict[str, str]:
    """Do not expand a pilot into the full public set without visible-RGB review."""
    if receipt is None or pilot_manifest is None or any(_unsafe(path) or not path.is_file() for path in (receipt, pilot_manifest)):
        raise CalibrationRejected("pilot_review_receipt_or_manifest_missing")
    try:
        review = json.loads(receipt.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationRejected("pilot_review_receipt_invalid") from exc
    if not isinstance(review, dict): raise CalibrationRejected("pilot_review_receipt_invalid")
    pilot = validate_pilot_manifest(pilot_manifest, plan=plan, contract=contract)
    visible = review.get("visible_classes")
    sheet = pilot["contact_sheet"]
    if review.get("status") != "APPROVED" or review.get("pilot_manifest_sha256") != sha256_file(pilot_manifest) or review.get("contact_sheet_sha256") != sheet["sha256"] or review.get("record_sha256") != pilot["record_sha256"]:
        raise CalibrationRejected("pilot_review_receipt_invalid")
    if not isinstance(visible, dict) or set(visible) != set(CLASS_IDS) or not all(isinstance(visible[name], int) and not isinstance(visible[name], bool) and 0 < visible[name] <= 25 for name in CLASS_IDS):
        raise CalibrationRejected("pilot_review_four_class_visibility_not_approved")
    if not all(review.get(key) is True for key in ("background_review_passed", "material_view_review_passed")):
        raise CalibrationRejected("pilot_review_background_or_view_not_approved")
    reviews = review.get("reviews")
    if not isinstance(reviews, list) or not any(isinstance(item, dict) and item.get("type") in {"manual", "agent"} and item.get("passed") is True and isinstance(item.get("reviewer"), str) and item["reviewer"].strip() for item in reviews):
        raise CalibrationRejected("pilot_review_reviewer_not_explicit")
    return {"review_receipt_sha256": sha256_file(receipt), "pilot_manifest_sha256": sha256_file(pilot_manifest), "contact_sheet_sha256": sheet["sha256"], "record_sha256": pilot["record_sha256"]}


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
    mobile_evidence: MobileEvidence | None = None


@dataclass(frozen=True)
class MobileEvidence:
    """Read-only proof that a production Nav2 goal moved this camera view."""

    goal_uuid: str
    action_status: int
    action_server: str
    odom_stamp_ns: int
    odom_x: float
    odom_y: float
    odom_yaw: float
    tf_stamp_ns: int
    camera_frame: str
    tf_translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tf_quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    tf_static_source_node: str = ""
    tf_static_source_gid: str = ""
    odom_source_node: str = ""
    odom_source_gid: str = ""
    image_source_node: str = ""
    image_source_gid: str = ""
    camera_info_source_node: str = ""
    camera_info_source_gid: str = ""


def _valid_endpoint_gid(value: str) -> bool:
    return bool(re.fullmatch(r"[0-9a-f]{32}", value)) and value != "0" * 32


def require_sole_publisher_identity(
    infos: list[Any], *, topic: str, node_name: str, topic_type: str, missing: str
) -> str:
    """Return the sole expected ROS publisher GID, or fail closed."""

    if not infos:
        raise CalibrationRejected(missing)
    if len(infos) != 1:
        raise CalibrationRejected("mobile_sensor_source_identity_invalid")
    source = infos[0]
    if (
        source.node_name != node_name
        or source.node_namespace != "/"
        or source.topic_type != topic_type
        or not topic.startswith("/")
    ):
        raise CalibrationRejected("mobile_sensor_source_identity_invalid")
    try:
        gid = bytes(source.endpoint_gid).hex()
    except (AttributeError, TypeError, ValueError) as exc:
        raise CalibrationRejected("mobile_sensor_source_identity_invalid") from exc
    if not _valid_endpoint_gid(gid):
        raise CalibrationRejected("mobile_sensor_source_identity_invalid")
    return gid


def _angle_delta(left: float, right: float) -> float:
    return abs((left - right + math.pi) % (2.0 * math.pi) - math.pi)


def require_mobile_evidence(*, image_stamp_ns: int, image_frame: str, evidence: MobileEvidence, accepted_poses: list[tuple[float, float, float]]) -> None:
    """Fail closed unless an active bt_navigator goal and a new physical view exist."""

    action_prefix = "bt_navigator:"
    if (
        not evidence.action_server.startswith(action_prefix)
        or not _valid_endpoint_gid(evidence.action_server[len(action_prefix):])
        or not re.fullmatch(r"[0-9a-f]{32}", evidence.goal_uuid)
    ):
        raise CalibrationRejected("mobile_nav2_action_identity_invalid")
    if evidence.action_status != 2:
        raise CalibrationRejected("mobile_nav2_goal_not_bt_navigator_executing")
    if (
        evidence.odom_source_node != "local_ekf"
        or evidence.image_source_node != "formal_legacy_topic_adapter"
        or evidence.camera_info_source_node != "formal_legacy_topic_adapter"
        or not all(_valid_endpoint_gid(value) for value in (
            evidence.odom_source_gid,
            evidence.image_source_gid,
            evidence.camera_info_source_gid,
        ))
    ):
        raise CalibrationRejected("mobile_sensor_source_identity_invalid")
    if evidence.camera_frame != image_frame or evidence.odom_stamp_ns <= 0:
        raise CalibrationRejected("mobile_odom_or_camera_tf_missing")
    if image_stamp_ns - evidence.odom_stamp_ns > MOBILE_EVIDENCE_MAX_AGE_NS or evidence.odom_stamp_ns > image_stamp_ns:
        raise CalibrationRejected("mobile_odom_or_camera_tf_not_fresh")
    if evidence.tf_stamp_ns and (image_stamp_ns - evidence.tf_stamp_ns > MOBILE_EVIDENCE_MAX_AGE_NS or evidence.tf_stamp_ns > image_stamp_ns):
        raise CalibrationRejected("mobile_odom_or_camera_tf_not_fresh")
    if evidence.tf_static_source_node != "robot_state_publisher" or not _valid_endpoint_gid(evidence.tf_static_source_gid) or not all(math.isfinite(value) for value in (*evidence.tf_translation, *evidence.tf_quaternion)):
        raise CalibrationRejected("mobile_camera_tf_invalid")
    if abs(math.sqrt(sum(value * value for value in evidence.tf_quaternion)) - 1.0) > 1e-3:
        raise CalibrationRejected("mobile_camera_tf_invalid")
    if not all(math.isfinite(value) for value in (evidence.odom_x, evidence.odom_y, evidence.odom_yaw)):
        raise CalibrationRejected("mobile_odom_pose_invalid")
    if any(math.hypot(evidence.odom_x - x, evidence.odom_y - y) < POSE_SEPARATION_M and _angle_delta(evidence.odom_yaw, yaw) < YAW_SEPARATION_RAD for x, y, yaw in accepted_poses):
        raise CalibrationRejected("mobile_pose_not_materially_distinct")


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
    def __init__(self, output: Path, contract: dict[str, Any], plan: dict[str, Any], *, per_scene_quota: int = 1, pilot_scene: str | None = None) -> None:
        if not isinstance(per_scene_quota, int) or isinstance(per_scene_quota, bool) or per_scene_quota < 1:
            raise CalibrationRejected("per_scene_quota_invalid")
        require_empty_output(output); self.output, self.contract, self.plan = output, contract, plan
        self.per_scene_quota = per_scene_quota
        self.calibration_scenes = set(plan["scene_groups"]["calibration"]); self.holdout_scenes = set(plan["scene_groups"]["holdout"])
        if pilot_scene is not None and (pilot_scene != plan["scene_groups"]["calibration"][0] or pilot_scene != "map-0-mission-0" or per_scene_quota != 25):
            raise CalibrationRejected("pilot_scene_or_quota_not_canonical")
        self.pilot_scene = pilot_scene
        self.records: list[dict[str, Any]] = []
        self.holdout_sources: set[str] = set()
        self.holdout_records: list[dict[str, Any]] = []
        self.holdout_scene_counts: dict[str, int] = {}
        self.calibration_scene_counts: dict[str, int] = {}
        self.sources: set[str] = set(); self.tensors: set[str] = set(); self.tensor_contents: set[str] = set()
        self.accepted_poses: dict[str, list[tuple[float, float, float]]] = {}
        self.duplicate_source_count = 0; self.duplicate_tensor_count = 0
        self.mobile_rejection_count = 0
        self.mobile_rejection_reasons: dict[str, int] = {}
        output.mkdir(parents=True); (output / "samples").mkdir(); (output / "holdout_samples").mkdir(); (output / "provenance").mkdir()

    def add(self, frame: Frame) -> bool:
        if any(token in frame.topic.lower() for token in FORBIDDEN) or frame.scene_id not in self.calibration_scenes | self.holdout_scenes:
            raise CalibrationRejected("frame_topic_or_scene_not_public_plan")
        if self.pilot_scene is not None and frame.scene_id != self.pilot_scene:
            raise CalibrationRejected("pilot_frame_not_canonical_scene")
        if self.pilot_scene is not None and frame.mobile_evidence is None:
            raise CalibrationRejected("pilot_mobile_evidence_required")
        validate_camera(image_frame=frame.frame_id, image_stamp_ns=frame.stamp_ns, width=frame.width, height=frame.height, camera=frame.camera)
        if frame.mobile_evidence is not None:
            require_mobile_evidence(image_stamp_ns=frame.stamp_ns, image_frame=frame.frame_id, evidence=frame.mobile_evidence, accepted_poses=self.accepted_poses.get(frame.scene_id, []))
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
            provenance = {"source_domain": "public_gazebo_sensor", "scene_id": frame.scene_id, "topic": frame.topic, "frame_id": frame.frame_id, "stamp_ns": frame.stamp_ns, "encoding": frame.encoding, "camera": frame.camera, "source_sha256": source, "generation_nonce": frame.generation_nonce, "episode_manifest_sha256": frame.episode_manifest_sha256, "mobile_evidence": asdict(frame.mobile_evidence) if frame.mobile_evidence else None}
            provenance_path = f"provenance/{name}.json"
            atomic_write_json(self.output / provenance_path, provenance)
            self.holdout_records.append({
                "scene_id": frame.scene_id,
                "source_sha256": source,
                "source_role": "evaluation_holdout_only",
                "generation_nonce": frame.generation_nonce,
                "episode_manifest_sha256": frame.episode_manifest_sha256,
                "relative_path": f"holdout_samples/{name}.npy",
                "byte_size": sample.stat().st_size,
                "sha256": tensor_sha,
                "provenance": provenance_path,
            })
            counts[frame.scene_id] = counts.get(frame.scene_id, 0) + 1
            if frame.mobile_evidence is not None:
                self.accepted_poses.setdefault(frame.scene_id, []).append((frame.mobile_evidence.odom_x, frame.mobile_evidence.odom_y, frame.mobile_evidence.odom_yaw))
            return False
        name = f"{len(self.records):06d}"; sample = self.output / "samples" / f"{name}.npy"
        pending = sample.with_name(f".{sample.name}.pending.{os.getpid()}")
        with pending.open("xb") as handle: np.save(handle, tensor, allow_pickle=False)
        os.replace(pending, sample); tensor_sha = sha256_file(sample)
        if tensor_sha in self.tensors: sample.unlink(); self.duplicate_tensor_count += 1; return False
        self.tensors.add(tensor_sha)
        provenance = {"source_domain": "public_gazebo_sensor", "scene_id": frame.scene_id, "topic": frame.topic, "frame_id": frame.frame_id, "stamp_ns": frame.stamp_ns, "encoding": frame.encoding, "camera": frame.camera, "source_sha256": source, "generation_nonce": frame.generation_nonce, "episode_manifest_sha256": frame.episode_manifest_sha256, "mobile_evidence": asdict(frame.mobile_evidence) if frame.mobile_evidence else None}
        provenance_file = self.output / "provenance" / f"{name}.json"
        atomic_write_json(provenance_file, provenance)
        self.records.append({"relative_path": f"samples/{name}.npy", "byte_size": sample.stat().st_size, "sha256": tensor_sha, "source_sha256": source, "source_role": "calibration_only", "scene_id": frame.scene_id, "generation_nonce": frame.generation_nonce, "episode_manifest_sha256": frame.episode_manifest_sha256, "provenance": f"provenance/{name}.json", "provenance_sha256": sha256_file(provenance_file), "provenance_byte_size": provenance_file.stat().st_size})
        counts[frame.scene_id] = counts.get(frame.scene_id, 0) + 1
        if frame.mobile_evidence is not None:
            self.accepted_poses.setdefault(frame.scene_id, []).append((frame.mobile_evidence.odom_x, frame.mobile_evidence.odom_y, frame.mobile_evidence.odom_yaw))
        return True

    def collection_complete(self) -> bool:
        if self.pilot_scene is not None:
            return len(self.records) == self.per_scene_quota and not self.holdout_records
        return (
            all(self.calibration_scene_counts.get(scene, 0) >= self.per_scene_quota for scene in self.calibration_scenes)
            and all(self.holdout_scene_counts.get(scene, 0) >= self.per_scene_quota for scene in self.holdout_scenes)
        )

    def reject_mobile(self, reason: str) -> None:
        self.mobile_rejection_count += 1
        self.mobile_rejection_reasons[reason] = self.mobile_rejection_reasons.get(reason, 0) + 1

    def progress(self) -> dict[str, Any]:
        return {
            "status": "NON_FORMAL_PILOT_COLLECTING" if self.pilot_scene else "COLLECTING",
            "formal_passed": False,
            "per_scene_quota": self.per_scene_quota,
            "pilot_scene": self.pilot_scene,
            "calibration_records": len(self.records),
            "holdout_source_count": len(self.holdout_sources),
            "duplicate_source_count": self.duplicate_source_count,
            "duplicate_tensor_count": self.duplicate_tensor_count,
            "mobile_rejection_count": self.mobile_rejection_count,
            "mobile_rejection_reasons": dict(sorted(self.mobile_rejection_reasons.items())),
            "calibration_scene_counts": dict(sorted(self.calibration_scene_counts.items())),
            "holdout_scene_counts": dict(sorted(self.holdout_scene_counts.items())),
            "collection_complete": self.collection_complete(),
        }

    def freeze(self) -> Path:
        if self.pilot_scene is not None:
            bindings_valid = all(re.fullmatch(r"[0-9a-f]{32}", str(row.get("generation_nonce", ""))) and re.fullmatch(r"[0-9a-f]{64}", str(row.get("episode_manifest_sha256", ""))) for row in self.records)
            if not self.collection_complete() or not bindings_valid or any(row["scene_id"] != self.pilot_scene for row in self.records):
                raise CalibrationRejected("pilot_capture_below_canonical_quota")
            record_sha256 = canonical_sha256(self.records)
            try:
                from PIL import Image
            except ImportError as exc:
                raise CalibrationRejected("pilot_contact_sheet_renderer_unavailable") from exc
            tiles = []
            for row in self.records:
                sample = self.output / row["relative_path"]
                if sample.is_symlink() or not sample.is_file() or sample.stat().st_size != row["byte_size"] or sha256_file(sample) != row["sha256"]:
                    raise CalibrationRejected("pilot_sample_drift")
                tensor = np.load(sample, allow_pickle=False)
                if tensor.dtype != np.float32 or tensor.shape != (1, 3, 640, 640) or not np.isfinite(tensor).all() or np.any(tensor < 0.0) or np.any(tensor > 1.0):
                    raise CalibrationRejected("pilot_tensor_invalid")
                rgb = np.clip(np.transpose(tensor[0], (1, 2, 0)) * 255.0, 0, 255).astype(np.uint8)
                tile = Image.fromarray(rgb, "RGB"); tile.thumbnail((160, 160)); tiles.append(tile)
            sheet = Image.new("RGB", (160 * 5, 160 * 5))
            for index, tile in enumerate(tiles): sheet.paste(tile, ((index % 5) * 160, (index // 5) * 160))
            contact_sheet = self.output / "pilot_contact_sheet.png"
            pending = contact_sheet.with_name(f".{contact_sheet.name}.pending.{os.getpid()}")
            sheet.save(pending, format="PNG"); os.replace(pending, contact_sheet)
            value = {"schema_version": 1, "status": "NON_FORMAL_PILOT_CAPTURED", "formal_passed": False,
                     "source_domain": "public_gazebo_sensor", "pilot_scene": self.pilot_scene,
                     "plan_sha256": canonical_sha256(self.plan), "preprocessing_sha256": canonical_sha256(self.contract["preprocessing"]),
                     "record_count": len(self.records), "per_scene_quota": self.per_scene_quota,
                     "record_sha256": record_sha256, "duplicate_source_count": self.duplicate_source_count,
                     "duplicate_tensor_count": self.duplicate_tensor_count, "records": self.records,
                     "contact_sheet": {"relative_path": contact_sheet.name, "sha256": sha256_file(contact_sheet),
                                       "byte_size": contact_sheet.stat().st_size, "record_sha256": record_sha256,
                                       "scene_id": self.pilot_scene, "record_count": len(self.records)}}
            target = self.output / "pilot_manifest.json"; atomic_write_json(target, value); return target
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


def frame_from_ros(*, scene_id: str, topic: str, image: Any, camera_info: Any, generation_nonce: str = "", episode_manifest_sha256: str = "", mobile_evidence: MobileEvidence | None = None) -> Frame:
    """Convert only an exactly paired ROS Image/CameraInfo pair to ``Frame``."""
    stamp = int(image.header.stamp.sec) * 1_000_000_000 + int(image.header.stamp.nanosec)
    camera_stamp = int(camera_info.header.stamp.sec) * 1_000_000_000 + int(camera_info.header.stamp.nanosec)
    camera = {"frame_id": str(camera_info.header.frame_id), "stamp_ns": camera_stamp, "width": int(camera_info.width), "height": int(camera_info.height), "k": [float(x) for x in camera_info.k]}
    return Frame(scene_id, topic, str(image.header.frame_id), stamp, bytes(image.data), int(image.width), int(image.height), int(image.step), str(image.encoding), camera, generation_nonce, episode_manifest_sha256, mobile_evidence)


def collect_live(*, output: Path, plan: dict[str, Any], contract: dict[str, Any], topic: str, camera_info_topic: str, timeout_s: float, selector_path: Path, per_scene_quota: int, progress_path: Path, pilot_scene: str | None = None, review_receipt: Path | None = None, pilot_manifest: Path | None = None, review_validated: bool = False) -> Path:
    """Attach read-only to one already-running public Gazebo camera scene.

    A lock-owning runner is responsible for advancing between public scene IDs;
    this function has no publisher, service client, Gazebo command, or truth
    subscription.  It fails rather than infer a CameraInfo pairing.
    """
    if timeout_s <= 0 or not topic.startswith("/") or any(token in topic.lower() for token in FORBIDDEN):
        raise CalibrationRejected("live_topic_or_timeout_invalid")
    if selector_path.is_symlink() or (selector_path.exists() and not selector_path.is_file()):
        raise CalibrationRejected("scene_selector_not_regular_nonlink")
    if pilot_scene is None:
        require_collection_capacity(plan=plan, contract=contract, per_scene_quota=per_scene_quota)
        if not review_validated:
            require_full_review_receipt(review_receipt, pilot_manifest, plan=plan, contract=contract)
    require_formal_camera_topic_pair(image_topic=topic, camera_info_topic=camera_info_topic)
    import rclpy
    from rclpy.parameter import Parameter
    from action_msgs.msg import GoalStatusArray
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformException, TransformListener
    rclpy.init(); node = rclpy.create_node("public_gazebo_dosod_calibration_collector", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    pairs = FreshPairCache()
    store = PublicGazeboStore(output, contract, plan, per_scene_quota=per_scene_quota, pilot_scene=pilot_scene)
    selector_nonce: str | None = None
    errors: list[str] = []
    latest_odom: Any | None = None
    action_statuses: dict[str, tuple[int, str]] = {}
    tf_buffer = Buffer(); tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
    action_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def selection() -> tuple[dict[str, str], bool]:
        nonlocal selector_nonce, latest_odom
        selected = load_scene_selector(selector_path)
        changed = selected["generation_nonce"] != selector_nonce
        if changed:
            selector_nonce = selected["generation_nonce"]
            pairs.reset()
            action_statuses.clear()
            latest_odom = None
        return selected, changed
    def save_progress() -> None:
        atomic_write_json(progress_path, store.progress())
    def action_server_gid() -> str:
        servers = node.get_publishers_info_by_topic("/navigate_to_pose/_action/status")
        if not servers:
            raise CalibrationRejected("mobile_nav2_action_server_missing")
        if len(servers) != 1 or servers[0].node_name != "bt_navigator" or servers[0].node_namespace != "/" or servers[0].topic_type != "action_msgs/msg/GoalStatusArray":
            raise CalibrationRejected("mobile_nav2_action_server_not_bt_navigator")
        try:
            gid = bytes(servers[0].endpoint_gid).hex()
        except (AttributeError, TypeError, ValueError) as exc:
            raise CalibrationRejected("mobile_nav2_action_server_gid_invalid") from exc
        if not _valid_endpoint_gid(gid):
            raise CalibrationRejected("mobile_nav2_action_server_gid_invalid")
        return gid
    def mobile_evidence(image: Image) -> MobileEvidence:
        if latest_odom is None:
            raise CalibrationRejected("mobile_odom_or_camera_tf_missing")
        odom_gid = require_sole_publisher_identity(
            node.get_publishers_info_by_topic("/odom"), topic="/odom",
            node_name="local_ekf", topic_type="nav_msgs/msg/Odometry",
            missing="mobile_odom_source_missing",
        )
        image_gid = require_sole_publisher_identity(
            node.get_publishers_info_by_topic(topic), topic=topic,
            node_name="formal_legacy_topic_adapter", topic_type="sensor_msgs/msg/Image",
            missing="mobile_image_source_missing",
        )
        camera_gid = require_sole_publisher_identity(
            node.get_publishers_info_by_topic(camera_info_topic), topic=camera_info_topic,
            node_name="formal_legacy_topic_adapter", topic_type="sensor_msgs/msg/CameraInfo",
            missing="mobile_camera_info_source_missing",
        )
        gid = action_server_gid()
        active = [goal for goal, status in action_statuses.items() if status == (2, gid)]
        if len(active) != 1:
            raise CalibrationRejected("mobile_nav2_goal_not_bt_navigator_executing")
        try:
            from rclpy.time import Time
            transform = tf_buffer.lookup_transform("base_footprint", str(image.header.frame_id), Time.from_msg(image.header.stamp))
        except TransformException as exc:
            raise CalibrationRejected("mobile_odom_or_camera_tf_missing") from exc
        if latest_odom.header.frame_id != "odom" or latest_odom.child_frame_id != "base_footprint":
            raise CalibrationRejected("mobile_odom_frame_invalid")
        pose = latest_odom.pose.pose
        q = pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not all(math.isfinite(value) for value in (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)) or norm == 0.0 or abs(norm - 1.0) > 1e-3:
            raise CalibrationRejected("mobile_odom_pose_invalid")
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        odom_stamp = int(latest_odom.header.stamp.sec) * 1_000_000_000 + int(latest_odom.header.stamp.nanosec)
        tf_stamp = int(transform.header.stamp.sec) * 1_000_000_000 + int(transform.header.stamp.nanosec)
        try:
            static_gid = require_sole_publisher_identity(
                node.get_publishers_info_by_topic("/tf_static"), topic="/tf_static",
                node_name="robot_state_publisher", topic_type="tf2_msgs/msg/TFMessage",
                missing="mobile_camera_tf_source_missing",
            )
        except CalibrationRejected as exc:
            if str(exc) == "mobile_sensor_source_identity_invalid":
                raise CalibrationRejected("mobile_camera_tf_source_invalid") from exc
            raise
        translation, rotation = transform.transform.translation, transform.transform.rotation
        return MobileEvidence(active[0], 2, "bt_navigator:" + gid, odom_stamp, float(pose.position.x), float(pose.position.y), yaw, tf_stamp, str(image.header.frame_id), (float(translation.x), float(translation.y), float(translation.z)), (float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)), "robot_state_publisher", static_gid, "local_ekf", odom_gid, "formal_legacy_topic_adapter", image_gid, "formal_legacy_topic_adapter", camera_gid)
    def consume_pair(pair: tuple[dict[str, str], Any, Any] | None) -> None:
        if pair is None:
            return
        selected, image_message, info_message = pair
        try:
            store.add(frame_from_ros(scene_id=selected["scene_id"], topic=topic, image=image_message, camera_info=info_message, generation_nonce=selected["generation_nonce"], episode_manifest_sha256=selected["episode_manifest_sha256"], mobile_evidence=mobile_evidence(image_message)))
            save_progress()
        except CalibrationRejected as exc:
            if str(exc) in RETRYABLE_MOBILE_REJECTIONS:
                store.reject_mobile(str(exc))
                save_progress()
            else:
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
    def on_odom(message: Odometry) -> None:
        nonlocal latest_odom
        try:
            selected, _ = selection()
        except CalibrationRejected as exc:
            errors.append(str(exc)); return
        if selected["state"] != "ACTIVE": return
        latest_odom = message
    def on_action_status(message: GoalStatusArray) -> None:
        try:
            selected, _ = selection()
        except CalibrationRejected as exc:
            errors.append(str(exc)); return
        if selected["state"] != "ACTIVE": return
        try:
            gid = action_server_gid()
        except CalibrationRejected:
            return
        action_statuses.clear()
        for item in message.status_list:
            goal = bytes(item.goal_info.goal_id.uuid).hex()
            action_statuses[goal] = (int(item.status), gid)
    try:
        node.create_subscription(CameraInfo, camera_info_topic, on_info, qos_profile_sensor_data)
        node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
        node.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data)
        node.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", on_action_status, action_qos)
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
    parser.add_argument("--pilot-scene")
    parser.add_argument("--review-receipt", type=Path)
    parser.add_argument("--pilot-manifest", type=Path)
    parser.add_argument("--validate-pilot-manifest", type=Path)
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
    if args.review_receipt is not None or args.pilot_manifest is not None:
        if args.review_receipt is None or args.pilot_manifest is None:
            parser.error("review preflight requires --review-receipt and --pilot-manifest")
        if not any(value is not None for value in live):
            value = require_full_review_receipt(args.review_receipt, args.pilot_manifest, plan=plan, contract=contract)
            print(json.dumps({"status": "NON_FORMAL_REVIEW_APPROVED", "formal_passed": False, **value}, sort_keys=True))
            return 0
    if args.validate_pilot_manifest is not None:
        value = validate_pilot_manifest(args.validate_pilot_manifest, plan=plan, contract=contract)
        print(json.dumps({"status": value["status"], "pilot_manifest_sha256": sha256_file(args.validate_pilot_manifest), "record_sha256": value["record_sha256"]}, sort_keys=True))
        return 0
    if any(value is not None for value in live):
        if any(value is None for value in live): parser.error("all live arguments are required together")
        review_validated = False
        if args.pilot_scene is None:
            require_full_review_receipt(args.review_receipt, args.pilot_manifest, plan=plan, contract=contract)
            review_validated = True
        print(collect_live(output=args.live_output, plan=plan, contract=contract, topic=args.image_topic, camera_info_topic=args.camera_info_topic, timeout_s=args.timeout_sec, selector_path=args.scene_selector, per_scene_quota=args.per_scene_quota, progress_path=args.progress_output, pilot_scene=args.pilot_scene, review_receipt=args.review_receipt, pilot_manifest=args.pilot_manifest, review_validated=review_validated))
        return 0
    print(json.dumps({"report_id": "tzcup_public_gazebo_dosod_calibration_preflight", "status": "NON_FORMAL_PREPARED", "formal_passed": False, "calibration_scenes": len(plan["scene_groups"]["calibration"]), "holdout_scenes": len(plan["scene_groups"]["holdout"]), "preprocessing_sha256": canonical_sha256(contract["preprocessing"])}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
