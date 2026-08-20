"""Generate and audit a sealed, development-inaccessible G5 final dataset."""

from __future__ import annotations

from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .g2_contract import read_production_camera_contract
from .g4_assets import (
    CLASS_ORDER,
    GEOMETRY_PARAMS,
    REQUIRED_PAPER_TAXONOMIES,
    TEXTURE_FAMILIES,
    _class_variants,
    _model_config,
    _model_sdf,
    _negative_sha256,
    _seed_for,
    _texture_png,
    _variant_sha256,
    load_g4_asset_registry,
)
from .g4_manifest import config_hash
from .g4_qa import (
    MIN_DECLARED_TARGET_VISIBLE_FRAMES,
    finite_pose,
    perceptual_near_duplicate,
    phash,
)
from .gazebo_g4 import (
    GAZEBO_SENSOR_TOPICS,
    GROUND_COLORS,
    GROUND_TEXTURE_SPECS,
    PRODUCTION_SENSOR_TOPICS,
    _ground_texture_png,
    _layout_models,
)
from .rendered import CLASS_INDEX


SEALED_SPLIT = "G5_SEALED_FINAL"
SCENES_PER_WORLD = 25
FRAMES_PER_SCENE = 10
G5_WORLD_PROFILES = (
    (
        "world_g5_01_covered_wet_walkway", "wet_dark_asphalt",
        "covered_walkway_zigzag", "ground_covered_zigzag",
        "side_lit_after_rain", "asphalt_wet_dark",
        "0.70 0.76 0.82 1", "0.48 -0.18 -0.86", 0.22,
    ),
    (
        "world_g5_02_drainage_alley", "concrete_light",
        "drainage_alley_offset", "ground_offset_drainage",
        "cool_overcast_alley", "concrete_fine_light",
        "0.73 0.78 0.84 1", "-0.32 0.42 -0.85", 0.80,
    ),
    (
        "world_g5_03_asymmetric_tree_plaza", "mixed_curb_soil",
        "tree_plaza_asymmetric", "ground_tree_islands",
        "low_dappled_backlight", "soil_mixed_vegetation",
        "0.66 0.70 0.58 1", "0.58 0.28 -0.76", 0.74,
    ),
    (
        "world_g5_04_warehouse_ramp", "smooth_industrial",
        "warehouse_ramp_crossing", "ground_ramp_crossing",
        "mixed_high_bay_glare", "industrial_smooth",
        "0.86 0.82 0.76 1", "-0.52 -0.22 -0.83", 0.66,
    ),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _dataset_tree_digest(root: Path) -> dict:
    """Bind every immutable G5 dataset payload to one reproducible digest."""
    digest = hashlib.sha256()
    file_count = 0
    total_bytes = 0
    for subtree in ("models", "worlds", "scenes"):
        subtree_path = root / subtree
        for path in sorted(item for item in subtree_path.rglob("*") if item.is_file()):
            relative = path.relative_to(root).as_posix()
            file_sha = _sha256(path)
            size = path.stat().st_size
            digest.update(relative.encode("utf-8"))
            digest.update(b"\0")
            digest.update(file_sha.encode("ascii"))
            digest.update(b"\0")
            digest.update(str(size).encode("ascii"))
            digest.update(b"\n")
            file_count += 1
            total_bytes += size
    return {
        "algorithm": "sha256(relative_path\\0file_sha256\\0size\\n)",
        "sha256": digest.hexdigest(),
        "file_count": file_count,
        "total_bytes": total_bytes,
        "included_subtrees": ["models", "worlds", "scenes"],
    }


def derive_g5_registry(development_registry: dict) -> dict:
    """Create new procedural artifact variants without reusing any G4 ID."""
    palette_names = sorted(development_registry["palette_rgb"])
    texture_names = sorted(TEXTURE_FAMILIES)
    classes: dict[str, list[dict]] = {}
    for class_index, class_id in enumerate(CLASS_ORDER):
        source = _class_variants(development_registry, class_id)
        variants = []
        for index, original in enumerate(source[:8]):
            variant = copy.deepcopy(original)
            variant["id"] = f"g5_{class_id}_{index + 1:03d}"
            variant["split_eligibility"] = [SEALED_SPLIT]
            offset = class_index * 5 + index * 3
            variant["palette"] = [
                palette_names[offset % len(palette_names)],
                palette_names[(offset + 7) % len(palette_names)],
            ]
            variant["texture_family"] = texture_names[
                (texture_names.index(original["texture_family"]) + 5 + index)
                % len(texture_names)
            ]
            variant["source"] = (
                "project-authored procedural G5 sealed variant; generated from "
                "an unseen material/texture/palette combination"
            )
            variant["sha256"] = _variant_sha256(class_id, variant)
            variants.append(variant)
        classes[class_id] = variants

    source_negatives = development_registry["hard_negatives"]
    selected = []
    for taxonomy in REQUIRED_PAPER_TAXONOMIES:
        selected.append(next(item for item in source_negatives if item["taxonomy"] == taxonomy))
    for item in source_negatives:
        if item not in selected and len(selected) < 36:
            selected.append(item)
    negatives = []
    for index, original in enumerate(selected):
        negative = copy.deepcopy(original)
        negative["id"] = f"g5_hard_negative_{index + 1:03d}"
        negative["split_eligibility"] = [SEALED_SPLIT]
        negative["texture_family"] = texture_names[
            (texture_names.index(original["texture_family"]) + 9 + index)
            % len(texture_names)
        ]
        negative["source"] = (
            "project-authored procedural G5 sealed hard-negative variant"
        )
        negative["sha256"] = _negative_sha256(negative)
        negatives.append(negative)
    return {
        "schema_version": 1,
        "registry_version": "2026.08-g5-sealed-v1",
        "dataset_domain": SEALED_SPLIT,
        "palette_rgb": copy.deepcopy(development_registry["palette_rgb"]),
        "classes": classes,
        "hard_negatives": negatives,
        "development_asset_ids_reused": False,
    }


def _write_assets(registry: dict, registry_path: Path, output_dir: Path) -> dict:
    generated = []
    for class_id in CLASS_ORDER:
        for variant in registry["classes"][class_id]:
            generated.append((class_id, CLASS_INDEX[class_id], variant))
    generated.extend(("background", 0, item) for item in registry["hard_negatives"])
    records = []
    for class_id, label, item in generated:
        kind, values = GEOMETRY_PARAMS[item["geometry_family"]]
        model_dir = output_dir / item["id"]
        texture_dir = model_dir / "textures"
        texture_dir.mkdir(parents=True, exist_ok=True)
        texture_name = f"texture_{item['id']}.png"
        texture_path = texture_dir / texture_name
        palettes = item.get("palette", ["gray", "stone", "dark_gray"])
        texture = _texture_png(
            item["texture_family"], registry["palette_rgb"], palettes,
            _seed_for(registry_path, item["id"]),
        )
        cv2.imwrite(str(texture_path), cv2.cvtColor(texture, cv2.COLOR_RGB2BGR))
        color = registry["palette_rgb"][palettes[0]]
        rgba = " ".join(f"{channel / 255.0:.4f}" for channel in color) + " 1"
        (model_dir / "model.sdf").write_text(
            _model_sdf(
                item["id"], kind, values, texture_name,
                item["material_family"], label, rgba=rgba,
            ),
            encoding="utf-8",
        )
        (model_dir / "model.config").write_text(
            _model_config(item["id"], f"G5 sealed {class_id} variant"),
            encoding="utf-8",
        )
        records.append(
            {
                "asset_id": item["id"], "class_id": class_id,
                "semantic_label": label,
                "texture_png_sha256": _sha256(texture_path),
                "model_sdf_sha256": _sha256(model_dir / "model.sdf"),
                "registry_sha256": item["sha256"],
            }
        )
    result = {"schema_version": 1, "dataset_id": SEALED_SPLIT, "assets": records}
    (output_dir / "g5_generated_asset_manifest.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    return result


def generate_g5_worlds(
    development_registry_path: str | Path,
    assets_dir: str | Path,
    xacro_path: str | Path,
    output_dir: str | Path,
    *,
    camera_overrides: dict[str, float] | None = None,
    camera_profile_id: str | None = None,
) -> dict:
    development_registry_path = Path(development_registry_path)
    assets_dir, output_dir = Path(assets_dir), Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    assets_dir.mkdir(parents=True, exist_ok=True)
    registry = derive_g5_registry(load_g4_asset_registry(development_registry_path))
    registry_path = assets_dir / "g5_sealed_asset_registry.json"
    registry_path.write_text(json.dumps(registry, indent=2) + "\n", encoding="utf-8")
    _write_assets(registry, registry_path, assets_dir)
    (output_dir / "ground_textures").mkdir(parents=True, exist_ok=True)
    prop_path = output_dir / "ground_textures" / "prop_gray.png"
    prop = _ground_texture_png(
        "clean_stone", ["gray", "dark_gray", "stone"],
        registry["palette_rgb"], 5501,
    )
    cv2.imwrite(str(prop_path), cv2.cvtColor(prop, cv2.COLOR_RGB2BGR))
    assets = []
    for class_id in CLASS_ORDER:
        for index, item in enumerate(registry["classes"][class_id]):
            kind, values = GEOMETRY_PARAMS[item["geometry_family"]]
            assets.append(
                {
                    "model_name": item["id"], "class_id": class_id,
                    "semantic_label": CLASS_INDEX[class_id], "variant_index": index,
                    "geometry_family": item["geometry_family"],
                    "material_family": item["material_family"],
                    "texture_family": item["texture_family"],
                    "physical_geometry_values_m": list(values),
                    "geometry_kind": kind, "split_eligibility": [SEALED_SPLIT],
                    "license": item["license"], "source": item["source"],
                    "registry_sha256": item["sha256"],
                }
            )
    negatives = []
    for item in registry["hard_negatives"]:
        kind, values = GEOMETRY_PARAMS[item["geometry_family"]]
        negatives.append(
            {
                "model_name": item["id"], "negative_id": item["id"],
                "taxonomy": item["taxonomy"], "semantic_label": 0,
                "geometry_family": item["geometry_family"],
                "material_family": item["material_family"],
                "texture_family": item["texture_family"],
                "physical_geometry_values_m": list(values),
                "geometry_kind": kind, "split_eligibility": [SEALED_SPLIT],
                "license": item["license"], "source": item["source"],
                "registry_sha256": item["sha256"],
            }
        )
    included = assets + negatives
    worlds = []
    for profile in G5_WORLD_PROFILES:
        (
            world_id, material_id, layout_family, geometry_family,
            lighting_family, texture_id, light, direction, roughness,
        ) = profile
        texture_family, palette = GROUND_TEXTURE_SPECS[texture_id]
        ground = _ground_texture_png(
            texture_family, palette, registry["palette_rgb"],
            int(hashlib.sha256(world_id.encode()).hexdigest()[:12], 16),
        )
        ground_path = output_dir / "ground_textures" / f"{world_id}.png"
        cv2.imwrite(str(ground_path), cv2.cvtColor(ground, cv2.COLOR_RGB2BGR))
        includes = "".join(
            f'<include><uri>model://{item["model_name"]}</uri>'
            f'<pose>{-200 - index * 0.3:.3f} 200 -5 0 0 0</pose></include>'
            for index, item in enumerate(included)
        )
        rgba = GROUND_COLORS[material_id]
        text = f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="{world_id}">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows><diffuse>{light}</diffuse><direction>{direction}</direction></light>
  <model name="ground"><static>true</static><link name="ground">
    <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry></collision>
    <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
      <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><pbr><metal><albedo_map>ground_textures/{world_id}.png</albedo_map><roughness>{roughness}</roughness><metalness>0</metalness></metal></pbr></material></visual>
  </link></model>
  {_layout_models(layout_family)}
  {includes}
</world></sdf>
'''
        path = output_dir / f"{world_id}.sdf"
        path.write_text(text, encoding="utf-8")
        worlds.append(
            {
                "world_id": world_id, "material_id": material_id,
                "layout_family": layout_family, "geometry_family": geometry_family,
                "lighting_family": lighting_family,
                "ground_texture_family": texture_id,
                "path": path.name, "sha256": _sha256(path),
                "split_eligibility": [SEALED_SPLIT],
            }
        )
    camera_contract = read_production_camera_contract(
        xacro_path,
        xacro_overrides=camera_overrides,
        profile_id=camera_profile_id,
    )
    manifest = {
        "schema_version": 1, "dataset_domain": SEALED_SPLIT,
        "camera_contract": camera_contract,
        "native_capture_resolution": camera_contract["native_resolution"],
        "training_only_ground_truth": True, "development_access_forbidden": True,
        "scenes_per_world": SCENES_PER_WORLD,
        "frames_per_scene": FRAMES_PER_SCENE,
        "worlds": worlds, "assets": assets, "negative_assets": negatives,
        "sensor_topics": list(PRODUCTION_SENSOR_TOPICS),
        "gazebo_topics": list(GAZEBO_SENSOR_TOPICS),
        "registry_sha256": _sha256(registry_path),
    }
    path = output_dir / "g5_world_manifest.json"
    path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def finalize_g5_dataset(
    data_root: str | Path,
    output_dir: str | Path,
    development_world_manifest: str | Path,
    *,
    strict: bool = False,
) -> dict:
    root, output = Path(data_root), Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = root / "worlds" / "g5_world_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    development = json.loads(Path(development_world_manifest).read_text(encoding="utf-8"))
    errors = []
    worlds = {item["world_id"]: item for item in manifest["worlds"]}
    development_world_ids = {item["world_id"] for item in development["worlds"]}
    development_targets = {item["model_name"] for item in development["assets"]}
    development_negatives = {
        item["model_name"] for item in development["negative_assets"]
    }
    target_ids = {item["model_name"] for item in manifest["assets"]}
    negative_ids = {item["model_name"] for item in manifest["negative_assets"]}
    overlaps = {
        "worlds": sorted(set(worlds) & development_world_ids),
        "target_assets": sorted(target_ids & development_targets),
        "hard_negative_assets": sorted(negative_ids & development_negatives),
    }
    world_file_mismatches = []
    for world_id, world in worlds.items():
        path = root / "worlds" / world["path"]
        actual = _sha256(path) if path.is_file() else None
        if actual != world["sha256"]:
            world_file_mismatches.append(
                {"world_id": world_id, "declared": world["sha256"], "actual": actual}
            )
    scenes = sorted((root / "scenes").glob("scene_*"))
    frame_count = 0
    consistent = 0
    declared_scene_class_total = 0
    declared_scene_class_visible = 0
    pose_valid = 0
    sync_valid = 0
    camera_valid = 0
    tf_valid = 0
    exact_seen, phash_seen = {}, {}
    cross_world_exact, cross_world_phash = [], []
    for scene_dir in scenes:
        try:
            scene = json.loads((scene_dir / "scene_manifest.json").read_text(encoding="utf-8"))
            capture = json.loads((scene_dir / "capture_report.json").read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            errors.append({"scene": scene_dir.name, "reason": "manifest_unreadable", "error": str(exc)})
            continue
        if scene.get("split") != SEALED_SPLIT or scene.get("world_id") not in worlds:
            errors.append({"scene": scene_dir.name, "reason": "sealed_world_contract_invalid"})
        contract = scene.get("pose_reset_contract", {})
        pose_ok = (
            contract.get("all_world_assets_accounted_for") is True
            and int(contract.get("duplicate_asset_pose_names", -1)) == 0
        )
        pose_valid += int(pose_ok)
        if not pose_ok:
            errors.append({"scene": scene_dir.name, "reason": "pose_reset_invalid"})
        records = capture.get("records", [])
        if capture.get("capture_pass") is not True or len(records) != FRAMES_PER_SCENE:
            errors.append({"scene": scene_dir.name, "reason": "capture_gate_failed"})
        declared = Counter(
            int(item["semantic_label"])
            for item in scene.get("objects", [])
            if int(item.get("semantic_label") or 0) in range(1, 6)
        )
        sequence_max_observed: Counter = Counter()
        sequence_full_visibility_frames: Counter = Counter()
        for record in records:
            frame_count += 1
            paths = {name: scene_dir / relative for name, relative in record["paths"].items()}
            required = ("rgb", "semantic", "instance", "camera", "tf")
            if not all(paths[name].is_file() for name in required):
                errors.append({"scene": scene_dir.name, "frame": record.get("frame_index"), "reason": "frame_file_missing"})
                continue
            semantic = np.load(paths["semantic"], allow_pickle=False)
            instance = np.load(paths["instance"], allow_pickle=False)
            observed = Counter()
            for instance_id in (int(value) for value in np.unique(instance) if int(value)):
                values = semantic[instance == instance_id].astype(np.int64)
                majority = int(np.bincount(values, minlength=6).argmax())
                if majority in range(1, 6):
                    observed[majority] += 1
            for label in range(1, 6):
                sequence_max_observed[label] = max(
                    sequence_max_observed[label], observed[label]
                )
                if declared[label] > 0 and observed[label] >= declared[label]:
                    sequence_full_visibility_frames[label] += 1
            mismatch = any(observed[label] > declared[label] for label in range(1, 6))
            consistent += int(not mismatch)
            if mismatch:
                errors.append({"scene": scene_dir.name, "frame": record.get("frame_index"), "reason": "undeclared_pixel_target_count_exceeded"})
            sync_valid += int(record.get("exact_four_sensor_timestamp") is True)
            tf_valid += int(finite_pose(paths["tf"]))
            try:
                camera = json.loads(paths["camera"].read_text(encoding="utf-8"))
                camera_ok = (
                    len(camera.get("k", [])) == 9
                    and all(math.isfinite(float(value)) for value in camera["k"])
                )
            except (OSError, ValueError, json.JSONDecodeError):
                camera_ok = False
            camera_valid += int(camera_ok)
            identity = {"world_id": scene["world_id"], "scene": scene_dir.name, "frame": record.get("frame_index")}
            rgb_hash = _sha256(paths["rgb"])
            previous = exact_seen.get(rgb_hash)
            if previous and previous["world_id"] != identity["world_id"]:
                cross_world_exact.append([previous, identity])
            else:
                exact_seen[rgb_hash] = identity
            perceptual_hash = phash(paths["rgb"])
            candidates = phash_seen.setdefault(perceptual_hash, [])
            for candidate in candidates:
                if (
                    candidate["world_id"] != identity["world_id"]
                    and perceptual_near_duplicate(
                        Path(candidate["path"]), paths["rgb"]
                    )
                ):
                    cross_world_phash.append(
                        [
                            {key: candidate[key] for key in identity},
                            identity,
                        ]
                    )
            candidates.append({**identity, "path": str(paths["rgb"])})
        for label, declared_count in sorted(declared.items()):
            if declared_count <= 0:
                continue
            declared_scene_class_total += 1
            visible_frames = sequence_full_visibility_frames[label]
            visible = (
                sequence_max_observed[label] >= declared_count
                and visible_frames >= MIN_DECLARED_TARGET_VISIBLE_FRAMES
            )
            declared_scene_class_visible += int(visible)
            if not visible:
                errors.append(
                    {
                        "scene": scene_dir.name,
                        "reason": "declared_target_sequence_visibility_failed",
                        "semantic_label": label,
                        "declared": int(declared_count),
                        "maximum_observed_in_one_frame": int(
                            sequence_max_observed[label]
                        ),
                        "full_visibility_frames": int(visible_frames),
                        "minimum_required_frames": (
                            MIN_DECLARED_TARGET_VISIBLE_FRAMES
                        ),
                    }
                )
    gates = {
        "worlds_at_least_4": len(worlds) >= 4,
        "scenes_at_least_100": len(scenes) >= 100,
        "frames_at_least_1000": frame_count >= 1000,
        "unseen_world_ids": not overlaps["worlds"],
        "unseen_target_asset_ids": not overlaps["target_assets"],
        "unseen_hard_negative_asset_ids": not overlaps["hard_negative_assets"],
        "world_file_hashes_match_manifest": not world_file_mismatches,
        "pose_reset_100_percent": pose_valid == len(scenes),
        "manifest_pixel_consistency_100_percent": consistent == frame_count,
        "declared_target_sequence_visibility_100_percent": (
            declared_scene_class_visible == declared_scene_class_total
        ),
        "four_sensor_sync_100_percent": sync_valid == frame_count,
        "camera_info_100_percent": camera_valid == frame_count,
        "tf_100_percent": tf_valid == frame_count,
        "cross_world_exact_duplicate_zero": not cross_world_exact,
        "cross_world_phash_duplicate_zero": not cross_world_phash,
        "errors_zero": not errors,
    }
    passed = all(gates.values())
    dataset_content = _dataset_tree_digest(root)
    sealed = {
        "schema_version": 1, "dataset_id": SEALED_SPLIT,
        "worlds": sorted(worlds), "scenes": len(scenes), "frames": frame_count,
        "target_assets": sorted(target_ids),
        "hard_negative_assets": sorted(negative_ids),
        "lighting_material_layout_combinations": [
            [item["lighting_family"], item["material_id"], item["layout_family"]]
            for item in manifest["worlds"]
        ],
        "world_sha256": {key: worlds[key]["sha256"] for key in sorted(worlds)},
        "dataset_content": dataset_content,
        "sealed_by": "tzcup_g5_finalize_v1",
        "dataset_gate_pass": passed,
    }
    sealed["manifest_sha256"] = config_hash(sealed)
    qa = {
        "schema_version": 1, "dataset_id": SEALED_SPLIT,
        "world_count": len(worlds), "scene_count": len(scenes),
        "frame_count": frame_count, "overlaps": overlaps,
        "world_file_mismatches": world_file_mismatches,
        "scene_pose_reset_valid_rate": pose_valid / max(len(scenes), 1),
        "manifest_pixel_target_consistency_rate": consistent / max(frame_count, 1),
        "declared_target_sequence_visibility_rate": (
            declared_scene_class_visible / max(declared_scene_class_total, 1)
        ),
        "declared_target_scene_class_count": declared_scene_class_total,
        "four_sensor_sync_rate": sync_valid / max(frame_count, 1),
        "camera_info_valid_rate": camera_valid / max(frame_count, 1),
        "tf_valid_rate": tf_valid / max(frame_count, 1),
        "cross_world_exact_duplicate_count": len(cross_world_exact),
        "cross_world_phash_duplicate_count": len(cross_world_phash),
        "gates": gates, "errors": errors[:100], "G5_dataset_gate_pass": passed,
    }
    qa_path = output / "g5_dataset_qa.json"
    manifest_output = output / "g5_sealed_manifest.json"
    qa_path.write_text(json.dumps(qa, indent=2) + "\n", encoding="utf-8")
    manifest_output.write_text(json.dumps(sealed, indent=2) + "\n", encoding="utf-8")
    (output / "g5_sealed_manifest.sha256").write_text(
        f"{_sha256(manifest_output)}  g5_sealed_manifest.json\n", encoding="utf-8"
    )
    if strict and not passed:
        raise RuntimeError("G5 sealed dataset QA failed")
    return qa


def main_generate() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--development-registry", required=True)
    parser.add_argument("--assets-dir", required=True)
    parser.add_argument("--xacro", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--camera-x", type=float)
    parser.add_argument("--camera-y", type=float)
    parser.add_argument("--camera-z", type=float)
    parser.add_argument("--camera-pitch-rad", type=float)
    parser.add_argument("--camera-profile-id")
    args = parser.parse_args()
    values = (args.camera_x, args.camera_y, args.camera_z, args.camera_pitch_rad)
    if any(value is not None for value in values) and not all(
        value is not None for value in values
    ):
        parser.error("all four production camera overrides must be provided together")
    camera_overrides = None
    if all(value is not None for value in values):
        camera_overrides = dict(
            camera_x=args.camera_x,
            camera_y=args.camera_y,
            camera_z=args.camera_z,
            camera_pitch_rad=args.camera_pitch_rad,
        )
    print(json.dumps(generate_g5_worlds(
        args.development_registry,
        args.assets_dir,
        args.xacro,
        args.output_dir,
        camera_overrides=camera_overrides,
        camera_profile_id=args.camera_profile_id,
    ), indent=2))


def main_finalize() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--development-world-manifest", required=True)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    print(json.dumps(finalize_g5_dataset(
        args.data_root, args.output_dir, args.development_world_manifest,
        strict=args.strict,
    ), indent=2))
