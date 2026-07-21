from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import random
import secrets
import shutil
import uuid
import zipfile
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


CLASSES = (
    "plastic_bottle",
    "metal_can",
    "paper_litter",
    "leaf_pile",
    "puddle",
    "no_target",
)
NEGATIVE_CATEGORIES = (
    "same_color_non_garbage",
    "bottle_or_can_shaped_obstacle",
    "wet_ground_non_puddle",
    "shadow",
    "leaf_background_non_target",
    "vehicle_self_structure",
    "crop_boundary_artifact",
)
MODEL_CATEGORY = {
    "negative_blue_obstacle": "same_color_non_garbage",
    "negative_green_obstacle": "same_color_non_garbage",
    "negative_red_obstacle": "same_color_non_garbage",
    "negative_bottle_like_reusable_cone": "bottle_or_can_shaped_obstacle",
    "negative_can_like_bollard": "bottle_or_can_shaped_obstacle",
    "negative_fixed_bin": "bottle_or_can_shaped_obstacle",
    "negative_reflective_patch": "wet_ground_non_puddle",
    "negative_wet_asphalt": "wet_ground_non_puddle",
    "negative_shadow": "shadow",
    "negative_leaves_outside_target_region": "leaf_background_non_target",
    "negative_robot_self_pixels": "vehicle_self_structure",
}


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _png_bytes(image: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    if not ok:
        raise RuntimeError("PNG encoding failed")
    return encoded.tobytes()


def _project_background_crop(scene: Path, obj: dict, record: dict, camera_pitch_rad: float = 0.0) -> tuple[np.ndarray, dict] | None:
    camera = _read_json(scene / record["paths"]["camera"])
    tf = _read_json(scene / record["paths"]["tf"])
    rgb = cv2.imread(str(scene / record["paths"]["rgb"]), cv2.IMREAD_COLOR)
    semantic = np.load(scene / record["paths"]["semantic"], allow_pickle=False)
    if rgb is None or rgb.shape[:2] != semantic.shape:
        return None

    fx, fy, cx, cy = float(camera["k"][0]), float(camera["k"][4]), float(camera["k"][2]), float(camera["k"][5])
    vehicle_x, vehicle_y = (float(v) for v in tf["world_to_base_xy"])
    cam_x, cam_y, cam_z = (float(v) for v in tf["base_to_camera_xyz_m"])
    object_x, object_y, object_z = (float(v) for v in obj["xyz_m"])
    delta_x = object_x - (vehicle_x + cam_x)
    delta_y = object_y - (vehicle_y + cam_y)
    delta_z = object_z - cam_z
    # Camera link uses X forward, Y left, Z up. Positive URDF pitch points X downward.
    forward = math.cos(camera_pitch_rad) * delta_x - math.sin(camera_pitch_rad) * delta_z
    camera_y = delta_y
    camera_z = math.sin(camera_pitch_rad) * delta_x + math.cos(camera_pitch_rad) * delta_z
    if forward <= 0.25:
        return None
    u = cx - fx * camera_y / forward
    v = cy - fy * camera_z / forward
    dims = [float(value) for value in obj.get("physical_geometry_values_m", (0.16, 0.12, 0.08))]
    projected = max(fx * max(dims[:2]) / forward, fy * dims[-1] / forward)
    side = int(np.clip(max(40.0, projected * 4.0), 40, 160))
    x0, y0 = int(round(u - side / 2)), int(round(v - side / 2))
    x1, y1 = x0 + side, y0 + side
    if x0 < 0 or y0 < 0 or x1 > rgb.shape[1] or y1 > rgb.shape[0]:
        return None
    semantic_crop = semantic[y0:y1, x0:x1]
    if semantic_crop.size == 0 or np.count_nonzero(semantic_crop) != 0:
        return None
    crop = rgb[y0:y1, x0:x1]
    if float(crop.std()) < 1.0:
        return None
    return crop, {
        "bbox_xyxy": [x0, y0, x1, y1],
        "projected_center_px": [float(u), float(v)],
        "forward_distance_m": forward,
    }


def collect_hard_negatives(dataset_root: str | Path, per_category: int = 10) -> list[dict]:
    root = Path(dataset_root)
    pools: dict[str, list[dict]] = {category: [] for category in NEGATIVE_CATEGORIES}
    seen: dict[str, set[str]] = {category: set() for category in NEGATIVE_CATEGORIES}
    if (root / "scenes").is_dir():
        scene_sources = [(scene, scene / "scene_manifest.json", scene / "capture_report.json", 0.0) for scene in sorted((root / "scenes").glob("scene_*"))]
    else:
        scene_sources = []
        for world_root in sorted(root.glob("world_*")):
            scene_sources.append((
                world_root / "verification",
                world_root / "scene_manifest_verification.json",
                world_root / "verification" / "capture_report.json",
                math.radians(50.0),
            ))
    for scene, manifest_path, capture_path, camera_pitch_rad in scene_sources:
        if not manifest_path.is_file() or not capture_path.is_file():
            continue
        manifest, capture = _read_json(manifest_path), _read_json(capture_path)
        objects = [obj for obj in manifest["objects"] if obj.get("class_id") == "background"]
        for obj in objects:
            category = MODEL_CATEGORY.get(obj["model_name"])
            if category is None or len(pools[category]) >= per_category:
                continue
            for record in capture["records"]:
                result = _project_background_crop(scene, obj, record, camera_pitch_rad)
                if result is None:
                    continue
                crop, projection = result
                encoded = _png_bytes(crop)
                digest = sha256_bytes(encoded)
                if digest in seen[category]:
                    continue
                seen[category].add(digest)
                pools[category].append({
                    "png": encoded,
                    "truth_class": "no_target",
                    "negative_category": category,
                    "source_scene": scene.name,
                    "world_id": manifest["world_id"],
                    "source_frame": int(record["frame_index"]),
                    "source_rgb_sha256": record["rgb_sha256"],
                    "source_model": obj["model_name"],
                    "camera_contract": "V4" if camera_pitch_rad else "production_discovery_camera",
                    "semantic_target_pixel_count": 0,
                    **projection,
                })
                if len(pools[category]) >= per_category:
                    break

        # Boundary crops are real rendered pixels deliberately clipped at the image edge.
        if len(pools["crop_boundary_artifact"]) < per_category:
            for record in capture["records"]:
                rgb = cv2.imread(str(scene / record["paths"]["rgb"]), cv2.IMREAD_COLOR)
                semantic = np.load(scene / record["paths"]["semantic"], allow_pickle=False)
                if rgb is None:
                    continue
                for edge in ("left", "right"):
                    strip_width = 48
                    x0 = 0 if edge == "left" else rgb.shape[1] - strip_width
                    y0, x1, y1 = int(rgb.shape[0] * 0.55), x0 + strip_width, int(rgb.shape[0] * 0.55) + 64
                    if np.count_nonzero(semantic[y0:y1, x0:x1]) != 0:
                        continue
                    source_strip = rgb[y0:y1, x0:x1]
                    if source_strip.size == 0 or float(source_strip.std()) < 1.0:
                        continue
                    crop = np.zeros((64, 64, 3), dtype=np.uint8)
                    if edge == "left":
                        crop[:, 16:] = source_strip
                    else:
                        crop[:, :48] = source_strip
                    encoded = _png_bytes(crop)
                    digest = sha256_bytes(encoded)
                    if digest in seen["crop_boundary_artifact"]:
                        continue
                    seen["crop_boundary_artifact"].add(digest)
                    pools["crop_boundary_artifact"].append({
                        "png": encoded,
                        "truth_class": "no_target",
                        "negative_category": "crop_boundary_artifact",
                        "source_scene": scene.name,
                        "world_id": manifest["world_id"],
                        "source_frame": int(record["frame_index"]),
                        "source_rgb_sha256": record["rgb_sha256"],
                        "source_model": None,
                        "boundary_padding": edge,
                        "semantic_target_pixel_count": 0,
                        "bbox_xyxy": [x0, y0, x1, y1],
                    })
                    if len(pools["crop_boundary_artifact"]) >= per_category:
                        break
                if len(pools["crop_boundary_artifact"]) >= per_category:
                    break

    short = {category: len(items) for category, items in pools.items() if len(items) < per_category}
    if short:
        raise RuntimeError(f"insufficient target-free hard negatives: {short}")
    return [item for category in NEGATIVE_CATEGORIES for item in pools[category][:per_category]]


def load_positive_samples(positive_root: str | Path) -> list[dict]:
    root = Path(positive_root)
    truth = _read_json(root / "truth_mapping_not_for_reviewers.json")
    if len(truth) != 200 or Counter(row["class_id"] for row in truth) != Counter({name: 40 for name in CLASSES[:-1]}):
        raise RuntimeError("Stage5BR5 positive audit set is not five classes x 40")
    samples = []
    for row in truth:
        path = root / "crops" / f"{row['sample_id']}.png"
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"missing positive crop: {path}")
        encoded = _png_bytes(image)
        samples.append({
            "png": encoded,
            "truth_class": row["class_id"],
            "target_present": True,
            "suitable_for_recognition": bool(row["ready_v2"]),
            "self_occluded": False,
            "source_stage5br5_id": row["sample_id"],
            "world_id": row["world_id"],
            "distance_bucket": row["distance_bucket"],
            "source_rgb_sha256": row["source_rgb_sha256"],
            "source_crop_sha256": row["crop_sha256"],
        })
    return samples


def load_prepared_negatives(root: str | Path) -> list[dict]:
    root = Path(root)
    report = _read_json(root / "capture_report.json")
    records = report.get("records", [])
    counts = Counter(row.get("negative_category") for row in records)
    if len(records) < 60 or set(counts) != set(NEGATIVE_CATEGORIES) or min(counts.values()) < 1:
        raise RuntimeError(f"prepared V4 negative category gate failed: {counts}")
    samples = []
    for row in records:
        if row.get("camera_contract") != "V4" or row.get("exact_four_sensor_timestamp") is not True:
            raise RuntimeError("prepared negative is not an exact synchronized V4 capture")
        if int(row.get("semantic_target_pixel_count", -1)) != 0:
            raise RuntimeError("prepared negative contains target semantic pixels")
        crop_path = root / row["crop_path"]
        data = crop_path.read_bytes()
        if sha256_bytes(data) != row["crop_sha256"]:
            raise RuntimeError(f"prepared negative crop SHA mismatch: {crop_path}")
        samples.append({
            "png": data,
            "truth_class": "no_target",
            "negative_category": row["negative_category"],
            "source_scene": report["world_id"],
            "world_id": report["world_id"],
            "source_frame": row["sample_index"],
            "source_rgb_sha256": row["rgb_sha256"],
            "source_model": row.get("model_name"),
            "semantic_target_pixel_count": 0,
            "bbox_xyxy": row["crop_bbox_xyxy"],
            "camera_contract": "V4",
            "camera_xyz_m": report["camera_xyz_m"],
            "camera_pitch_deg": report["camera_pitch_deg"],
        })
    return samples


def _instructions(package_id: str, sample_count: int) -> str:
    classes = "\n".join(f"- `{name}`" for name in CLASSES)
    return f"""# Stage5BR6 独立盲审说明

包编号：`{package_id}`；样本数：`{sample_count}`。

请在不与另一位评审交流、不访问项目仓库或答案映射的情况下独立完成。不要根据类别数量猜测答案。逐张查看 `images/`，在 JSON 或 CSV 模板中填写同名 sample ID。允许类别：

{classes}

字段含义：`target_present` 表示是否确有目标；`class` 为上列类别；`suitable_for_recognition` 表示仅凭该裁剪是否足以可靠辨认；`self_occluded` 表示是否被车辆自身结构遮挡；`confidence_1_to_5` 为 1–5 整数。

开始和完成时填写 ISO-8601 时间、匿名代号，并确认独立性与未访问答案。`package_sha256` 从交付方提供的 `handoff_manifest.json` 复制。不要填写真实姓名。
"""


def _response_template(package_id: str, sample_ids: list[str]) -> dict:
    return {
        "schema_version": 1,
        "reviewer_pseudonym": "",
        "package_id": package_id,
        "package_sha256": "FILL_FROM_HANDOFF_MANIFEST",
        "started_at": "",
        "completed_at": "",
        "independence_attestation": None,
        "truth_mapping_not_accessed": None,
        "responses": [{
            "sample_id": sample_id,
            "target_present": None,
            "class": None,
            "suitable_for_recognition": None,
            "self_occluded": None,
            "confidence_1_to_5": None,
        } for sample_id in sample_ids],
    }


def _write_reviewer_package(output: Path, reviewer: str, samples: list[dict], rng: random.Random) -> tuple[dict, list[dict]]:
    package_id = f"stage5br6-{reviewer.lower()}-{uuid.UUID(int=rng.getrandbits(128)).hex}"
    order = list(range(len(samples)))
    rng.shuffle(order)
    opaque_ids = [f"r{reviewer.lower()}_{uuid.UUID(int=rng.getrandbits(128)).hex[:20]}" for _ in order]
    mapping, files = [], {}
    for index, sample_index in enumerate(order):
        sample_id = opaque_ids[index]
        name = f"images/{sample_id}.png"
        files[name] = samples[sample_index]["png"]
        mapping.append({"review_sample_id": sample_id, "master_index": sample_index})

    instructions = _instructions(package_id, len(samples)).encode("utf-8")
    response = _response_template(package_id, opaque_ids)
    response_json = (json.dumps(response, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    csv_buffer = io.StringIO(newline="")
    writer = csv.writer(csv_buffer)
    writer.writerow(("sample_id", "target_present", "class", "suitable_for_recognition", "self_occluded", "confidence_1_to_5"))
    writer.writerows((sample_id, "", "", "", "", "") for sample_id in opaque_ids)
    response_csv = csv_buffer.getvalue().encode("utf-8-sig")
    files.update({"INSTRUCTIONS.md": instructions, "response_template.json": response_json, "response_template.csv": response_csv})
    sample_ids_sha256 = sha256_bytes(("\n".join(sorted(opaque_ids)) + "\n").encode("utf-8"))
    manifest = {
        "schema_version": 1,
        "package_id": package_id,
        "reviewer_slot": reviewer,
        "sample_count": len(samples),
        "sample_ids_sha256": sample_ids_sha256,
        "outer_zip_sha256": "recorded_in_external_handoff_manifest",
        "files": [{"path": name, "bytes": len(data), "sha256": sha256_bytes(data)} for name, data in sorted(files.items())],
    }
    files["package_manifest.json"] = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    zip_path = output / f"reviewer_{reviewer}_package.zip"
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for name, data in sorted(files.items()):
            info = zipfile.ZipInfo(name, date_time=(2026, 7, 21, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100444 << 16
            archive.writestr(info, data, compresslevel=9)
    with zipfile.ZipFile(zip_path) as archive:
        if archive.testzip() is not None or set(archive.namelist()) != set(files):
            raise RuntimeError(f"package ZIP integrity failure: {zip_path}")
        for name, data in files.items():
            if archive.read(name) != data:
                raise RuntimeError(f"package content mismatch: {name}")
    return {
        "reviewer_slot": reviewer,
        "package_id": package_id,
        "path": zip_path.name,
        "bytes": zip_path.stat().st_size,
        "sha256": sha256_file(zip_path),
        "sample_count": len(samples),
        "sample_ids_sha256": sample_ids_sha256,
        "zip_crc_pass": True,
    }, mapping


def prepare_handoff(
    positive_root: str | Path,
    dataset_root: str | Path | None,
    output_root: str | Path,
    seed: int | None = None,
    prepared_negative_root: str | Path | None = None,
) -> dict:
    output = Path(output_root)
    if output.exists():
        raise RuntimeError(f"handoff output already exists: {output}")
    (output / "sealed_truth").mkdir(parents=True)
    positives = load_positive_samples(positive_root)
    negatives = load_prepared_negatives(prepared_negative_root) if prepared_negative_root else collect_hard_negatives(dataset_root, per_category=10)
    samples = positives + [{
        **item,
        "target_present": False,
        "suitable_for_recognition": False,
        "self_occluded": False,
    } for item in negatives]
    if seed is None:
        seed = secrets.randbits(128)
    seed_a = int.from_bytes(hashlib.sha256(f"A:{seed}".encode()).digest()[:16], "big")
    seed_b = int.from_bytes(hashlib.sha256(f"B:{seed}".encode()).digest()[:16], "big")
    package_a, map_a = _write_reviewer_package(output, "A", samples, random.Random(seed_a))
    package_b, map_b = _write_reviewer_package(output, "B", samples, random.Random(seed_b))
    if [row["master_index"] for row in map_a] == [row["master_index"] for row in map_b]:
        raise RuntimeError("reviewer orders unexpectedly match")
    if set(row["review_sample_id"] for row in map_a) & set(row["review_sample_id"] for row in map_b):
        raise RuntimeError("opaque reviewer IDs overlap")

    master_truth = []
    for index, sample in enumerate(samples):
        truth = {key: value for key, value in sample.items() if key != "png"}
        truth.update({"master_index": index, "image_sha256": sha256_bytes(sample["png"])})
        master_truth.append(truth)
    sealed = {
        "schema_version": 1,
        "stage": "Stage5BR6",
        "pre_registered_camera_candidate": "V4",
        "camera_selected": False,
        "samples": master_truth,
        "reviewer_mappings": {"A": map_a, "B": map_b},
    }
    sealed_path = output / "sealed_truth" / "master_truth.json"
    sealed_path.write_text(json.dumps(sealed, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "stage": "Stage5BR6-A",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "pre_registered_camera_candidate": "V4",
        "camera_selected": False,
        "positive_sample_count": len(positives),
        "negative_sample_count": len(negatives),
        "total_sample_count": len(samples),
        "negative_category_counts": dict(Counter(item["negative_category"] for item in negatives)),
        "negative_camera_contracts": sorted({item.get("camera_contract") for item in negatives}),
        "semantic_target_pixels_in_negative_crops": sum(item["semantic_target_pixel_count"] for item in negatives),
        "reviewer_packages": [package_a, package_b],
        "sealed_truth": {"path": "sealed_truth/master_truth.json", "sha256": sha256_file(sealed_path), "not_for_reviewers": True},
        "AWAITING_HUMAN_REVIEW": True,
        "READY_FOR_STAGE5BR6_ORACLE": False,
    }
    (output / "handoff_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def audit_handoff(output_root: str | Path) -> dict:
    output = Path(output_root)
    manifest = _read_json(output / "handoff_manifest.json")
    if manifest["positive_sample_count"] != 200 or manifest["negative_sample_count"] < 60:
        raise ValueError("positive/negative sample count gate failed")
    if set(manifest["negative_category_counts"]) != set(NEGATIVE_CATEGORIES):
        raise ValueError("hard-negative category coverage is incomplete")
    if min(manifest["negative_category_counts"].values()) < 1:
        raise ValueError("empty hard-negative category")
    if manifest["semantic_target_pixels_in_negative_crops"] != 0:
        raise ValueError("a negative crop contains target semantic pixels")
    if not manifest["AWAITING_HUMAN_REVIEW"] or manifest["READY_FOR_STAGE5BR6_ORACLE"]:
        raise ValueError("Stage5BR6-A fail-closed state changed")

    reviewer_ids, package_ids = [], []
    forbidden = (b'"world_id"', b'"source_scene"', b'"source_model"', b'"source_rgb_sha256"', b'"master_index"', b'"camera_id"', b'"pre_registered_camera_candidate"')
    package_results = []
    for package in manifest["reviewer_packages"]:
        path = output / package["path"]
        if sha256_file(path) != package["sha256"]:
            raise ValueError(f"package SHA mismatch: {path}")
        with zipfile.ZipFile(path) as archive:
            if archive.testzip() is not None:
                raise ValueError(f"package CRC failed: {path}")
            names = archive.namelist()
            image_names = sorted(name for name in names if name.startswith("images/") and name.endswith(".png"))
            if len(image_names) != package["sample_count"] or len(image_names) != len(set(image_names)):
                raise ValueError("reviewer image count/uniqueness failure")
            package_manifest = json.loads(archive.read("package_manifest.json"))
            response = json.loads(archive.read("response_template.json"))
            ids = [Path(name).stem for name in image_names]
            if sorted(row["sample_id"] for row in response["responses"]) != ids:
                raise ValueError("response template IDs do not match package images")
            ids_digest = sha256_bytes(("\n".join(ids) + "\n").encode("utf-8"))
            if ids_digest != package["sample_ids_sha256"] or ids_digest != package_manifest["sample_ids_sha256"]:
                raise ValueError("package sample ID digest mismatch")
            for row in response["responses"]:
                if any(row[field] is not None for field in ("target_present", "class", "suitable_for_recognition", "self_occluded", "confidence_1_to_5")):
                    raise ValueError("review response template was prefilled")
            file_index = {row["path"]: row for row in package_manifest["files"]}
            for name in names:
                data = archive.read(name)
                if name != "package_manifest.json":
                    expected = file_index[name]
                    if len(data) != expected["bytes"] or sha256_bytes(data) != expected["sha256"]:
                        raise ValueError(f"inner manifest mismatch: {name}")
                if name.endswith((".json", ".md", ".csv")) and any(token in data for token in forbidden):
                    raise ValueError(f"reviewer metadata leakage: {name}")
                if name.endswith(".png"):
                    if any(chunk in data for chunk in (b"tEXt", b"zTXt", b"iTXt", b"eXIf")):
                        raise ValueError(f"PNG metadata leakage: {name}")
                    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    if image is None or image.size == 0:
                        raise ValueError(f"invalid reviewer image: {name}")
        reviewer_ids.append(set(ids))
        package_ids.append(package["package_id"])
        package_results.append({"reviewer_slot": package["reviewer_slot"], "zip_crc_pass": True, "sample_count": len(ids), "metadata_leakage_count": 0})
    if reviewer_ids[0] & reviewer_ids[1] or package_ids[0] == package_ids[1]:
        raise ValueError("reviewer package independence failure")
    sealed_path = output / manifest["sealed_truth"]["path"]
    if sha256_file(sealed_path) != manifest["sealed_truth"]["sha256"]:
        raise ValueError("sealed truth SHA mismatch")
    return {
        "stage": "Stage5BR6-A",
        "handoff_integrity_pass": True,
        "reviewer_packages": package_results,
        "reviewer_ids_disjoint": True,
        "reviewer_orders_independent": True,
        "truth_mapping_absent_from_reviewer_packages": True,
        "png_metadata_leakage_count": 0,
        "semantic_target_pixels_in_negative_crops": 0,
        "AWAITING_HUMAN_REVIEW": True,
        "READY_FOR_STAGE5BR6_ORACLE": False,
    }


def validate_completed_response(response: dict, package: dict) -> None:
    if response.get("package_id") != package["package_id"] or response.get("package_sha256") != package["sha256"]:
        raise ValueError("package identity or SHA mismatch")
    if not response.get("reviewer_pseudonym") or response.get("independence_attestation") is not True or response.get("truth_mapping_not_accessed") is not True:
        raise ValueError("reviewer identity/attestation is incomplete")
    try:
        started = datetime.fromisoformat(response["started_at"].replace("Z", "+00:00"))
        completed = datetime.fromisoformat(response["completed_at"].replace("Z", "+00:00"))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("invalid review timestamps") from exc
    if completed <= started:
        raise ValueError("completed_at must be later than started_at")
    rows = response.get("responses")
    if not isinstance(rows, list) or len(rows) != package["sample_count"]:
        raise ValueError("response count mismatch")
    ids = [row.get("sample_id") for row in rows]
    if len(ids) != len(set(ids)):
        raise ValueError("sample IDs must appear exactly once")
    ids_sha256 = sha256_bytes(("\n".join(sorted(ids)) + "\n").encode("utf-8"))
    if ids_sha256 != package.get("sample_ids_sha256"):
        raise ValueError("response sample ID set does not match the package")
    for row in rows:
        if row.get("class") not in CLASSES or not isinstance(row.get("target_present"), bool):
            raise ValueError("illegal target/class response")
        if not isinstance(row.get("suitable_for_recognition"), bool) or not isinstance(row.get("self_occluded"), bool):
            raise ValueError("illegal recognition/occlusion response")
        confidence = row.get("confidence_1_to_5")
        if not isinstance(confidence, int) or isinstance(confidence, bool) or not 1 <= confidence <= 5:
            raise ValueError("confidence must be an integer from 1 to 5")
