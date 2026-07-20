from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
LEARNING = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(LEARNING))
from sanitation_learning.camera_mechanics import evaluate_all  # noqa: E402
from sanitation_learning.observability import load_policy, recognition_ready  # noqa: E402


CLASS_INDEX = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter", 4: "leaf_pile", 5: "puddle"}
CONFIGS = ("V1", "V2", "V4")
WORLDS = (
    "world_a_asphalt_campus", "world_b_concrete_sidewalk", "world_c_wet_dark_ground",
    "world_d_mixed_curb_vegetation", "world_e_tiled_plaza", "world_f_service_road",
)


def percentile(values, value):
    return float(np.percentile(values, value)) if values else None


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def distance_bucket(distance: float | None) -> str:
    if distance is None:
        return "invalid"
    if distance < 1.0:
        return "0.5-1m"
    if distance < 2.0:
        return "1-2m"
    if distance < 3.0:
        return "2-3m"
    return "3m+"


def collect_role(role_root: Path, config: str, world: str, role: str, policy_v1: dict, policy_v2: dict):
    capture = json.loads((role_root / "capture_report.json").read_text(encoding="utf-8"))
    rows = []
    self_fractions = []
    max_area = defaultdict(int)
    frame_cache = []
    for record in capture["records"]:
        semantic = np.load(role_root / record["paths"]["semantic"], allow_pickle=False)
        instances = np.load(role_root / record["paths"]["instance"], allow_pickle=False)
        depth = np.load(role_root / record["paths"]["depth"], allow_pickle=False)
        self_mask = semantic == 250
        self_fractions.append(float(self_mask.mean()))
        frame_rows = []
        for instance_id in (int(value) for value in np.unique(instances) if int(value) != 0):
            mask = instances == instance_id
            labels = semantic[mask].astype(np.int64)
            semantic_id = int(np.bincount(labels).argmax()) if labels.size else 0
            if semantic_id not in CLASS_INDEX:
                continue
            ys, xs = np.nonzero(mask)
            values = depth[mask]
            valid = np.isfinite(values) & (values > 0)
            x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
            bbox_self = self_mask[y0:y1 + 1, x0:x1 + 1]
            frame_edges_touched = sum((x0 == 0, y0 == 0, x1 == semantic.shape[1] - 1, y1 == semantic.shape[0] - 1))
            row = {
                "config": config, "world_id": world, "role": role,
                "frame_index": int(record["frame_index"]), "instance_id": instance_id,
                "class_id": CLASS_INDEX[semantic_id], "visibility": "visible",
                "mask_area_px": int(mask.sum()),
                "bbox_xyxy": [x0, y0, x1, y1],
                "bbox_shortest_side_px": int(min(x1 - x0 + 1, y1 - y0 + 1)),
                "distance_m": float(np.median(values[valid])) if valid.any() else None,
                "depth_valid_ratio": float(valid.mean()) if values.size else 0.0,
                "target_self_overlap": float(bbox_self.mean()) if bbox_self.size else 1.0,
                "boundary_completeness": 1.0 - 0.25 * frame_edges_touched,
                "rgb_path": str((role_root / record["paths"]["rgb"]).resolve()),
                "rgb_sha256": record["rgb_sha256"],
            }
            max_area[instance_id] = max(max_area[instance_id], row["mask_area_px"])
            frame_rows.append(row)
            rows.append(row)
        frame_cache.append(frame_rows)
    for row in rows:
        row["visible_fraction"] = row["mask_area_px"] / max_area[row["instance_id"]]
        row["occlusion"] = 1.0 - row["visible_fraction"]
        row["distance_bucket"] = distance_bucket(row["distance_m"])
        row["ready_v1"] = recognition_ready(row, policy_v1)
        row["ready_v2"] = recognition_ready(row, policy_v2)
    return rows, {
        "capture_pass": bool(capture["capture_pass"]),
        "captured_frames": int(capture["captured_frames"]),
        "exact_timestamp_frames": sum(bool(item["exact_four_sensor_timestamp"]) for item in capture["records"]),
        "self_pixel_fraction_p50": percentile(self_fractions, 50),
        "self_pixel_fraction_p95": percentile(self_fractions, 95),
    }


def select_blind_samples(rows: list[dict], output: Path, per_class: int = 40) -> dict:
    if output.exists():
        raise RuntimeError(f"blind audit directory already exists: {output}")
    crops = output / "crops"
    crops.mkdir(parents=True)
    truth = []
    blind_index = []
    counts = Counter()
    worlds = defaultdict(set)
    for class_id in CLASS_INDEX.values():
        candidates = [row for row in rows if row["class_id"] == class_id]
        candidates.sort(key=lambda row: (row["world_id"], row["distance_bucket"], round(row["occlusion"], 1), row["frame_index"], row["instance_id"]))
        # Round-robin worlds before filling remaining slots.
        selected = []
        for world in WORLDS:
            subset = [row for row in candidates if row["world_id"] == world]
            selected.extend(subset[: min(7, len(subset))])
        seen = {(row["world_id"], row["frame_index"], row["instance_id"]) for row in selected}
        selected.extend(row for row in candidates if (row["world_id"], row["frame_index"], row["instance_id"]) not in seen)
        selected = selected[:per_class]
        for row in selected:
            sample_id = f"sample_{len(truth):04d}"
            image = cv2.imread(row["rgb_path"], cv2.IMREAD_COLOR)
            x0, y0, x1, y1 = row["bbox_xyxy"]
            margin_x = max(4, int((x1 - x0 + 1) * 0.25))
            margin_y = max(4, int((y1 - y0 + 1) * 0.25))
            crop = image[max(0, y0 - margin_y):min(image.shape[0], y1 + margin_y + 1), max(0, x0 - margin_x):min(image.shape[1], x1 + margin_x + 1)]
            crop_path = crops / f"{sample_id}.png"
            if crop.size == 0 or not cv2.imwrite(str(crop_path), crop):
                raise RuntimeError(f"failed to write crop {sample_id}")
            blind_index.append({
                "sample_id": sample_id,
                "crop": f"crops/{crop_path.name}",
                "review_fields": {"class": None, "target_present": None, "suitable_for_recognition": None, "self_occluded": None, "confidence": None},
            })
            truth.append({
                "sample_id": sample_id, "class_id": row["class_id"], "world_id": row["world_id"],
                "distance_bucket": row["distance_bucket"], "occlusion": row["occlusion"],
                "ready_v2": row["ready_v2"], "source_rgb_sha256": row["rgb_sha256"],
                "crop_sha256": sha256(crop_path),
            })
            counts[row["class_id"]] += 1
            worlds[row["class_id"]].add(row["world_id"])
    (output / "blind_index.json").write_text(json.dumps(blind_index, indent=2) + "\n", encoding="utf-8")
    (output / "truth_mapping_not_for_reviewers.json").write_text(json.dumps(truth, indent=2) + "\n", encoding="utf-8")
    for reviewer in ("reviewer_a", "reviewer_b"):
        shutil.copy2(output / "blind_index.json", output / f"{reviewer}_responses.json")
    balanced = all(counts[class_id] >= 40 for class_id in CLASS_INDEX.values())
    six_worlds = len({item["world_id"] for item in truth}) >= 6
    blocker = "two_independent_human_reviewer_responses_not_available" if balanced and six_worlds else "balanced_blind_audit_dataset_gate_failed"
    return {
        "sample_count": len(truth),
        "samples_by_class": dict(counts),
        "worlds_by_class": {key: sorted(value) for key, value in worlds.items()},
        "balanced_40_per_class_pass": balanced,
        "six_worlds_represented": six_worlds,
        "reviewer_response_count": 0,
        "independent_reviewers_required": 2,
        "manual_gate_pass": False,
        "manual_gate_blocker": blocker,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--rows-output", required=True)
    parser.add_argument("--blind-audit-output", required=True)
    parser.add_argument("--audit-extra-root", action="append", default=[])
    args = parser.parse_args()
    root = Path(args.root)
    policy_v1_path = LEARNING / "config" / "perception_evaluability_policy.yaml"
    policy_v2_path = LEARNING / "config" / "perception_evaluability_policy_v2.yaml"
    config_path = LEARNING / "config" / "stage5br5_active_observation.yaml"
    policy_v1, policy_v2 = load_policy(policy_v1_path), load_policy(policy_v2_path)
    config_doc = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    mechanics = evaluate_all(config_doc)
    all_rows = []
    roles = {}
    for config in CONFIGS:
        roles[config] = {}
        for world in WORLDS:
            roles[config][world] = {}
            for role in ("discovery", "verification"):
                rows, runtime = collect_role(root / config / world / role, config, world, role, policy_v1, policy_v2)
                all_rows.extend(rows)
                roles[config][world][role] = runtime

    config_summary = {}
    for config in CONFIGS:
        rows = [row for row in all_rows if row["config"] == config]
        verification = [row for row in rows if row["role"] == "verification"]
        discovery = [row for row in rows if row["role"] == "discovery"]
        self_p95 = max(roles[config][world]["verification"]["self_pixel_fraction_p95"] for world in WORLDS)
        overlap_max = max((row["target_self_overlap"] for row in verification), default=1.0)
        runtime_pass = (
            all(roles[config][world][role]["capture_pass"] and roles[config][world][role]["captured_frames"] >= 10 and roles[config][world][role]["exact_timestamp_frames"] >= 10 for world in WORLDS for role in ("discovery", "verification"))
            and self_p95 <= 0.05 and overlap_max <= 0.05
            and mechanics["camera_results"][config]["mechanical_gate_pass"]
        )
        conversions = []
        class_conversions = defaultdict(lambda: [0, 0])
        for world in WORLDS:
            before = [row for row in discovery if row["world_id"] == world and not row["ready_v2"]]
            after_ready = {row["instance_id"] for row in verification if row["world_id"] == world and row["ready_v2"]}
            candidate_classes = {}
            for row in before:
                candidate_classes.setdefault(row["instance_id"], row["class_id"])
            conversions.extend((world, instance_id, class_id, instance_id in after_ready) for instance_id, class_id in candidate_classes.items())
            for instance_id, class_id in candidate_classes.items():
                class_conversions[class_id][1] += 1
                class_conversions[class_id][0] += int(instance_id in after_ready)
        config_summary[config] = {
            "world_count": len(WORLDS), "capture_role_count": len(WORLDS) * 2,
            "captured_frames": sum(roles[config][world][role]["captured_frames"] for world in WORLDS for role in ("discovery", "verification")),
            "all_exact_timestamp_gate_pass": all(roles[config][world][role]["exact_timestamp_frames"] >= 10 for world in WORLDS for role in ("discovery", "verification")),
            "verification_self_pixels_p95_worst_world": self_p95,
            "target_self_overlap_max": overlap_max,
            "mechanical_gate_pass": mechanics["camera_results"][config]["mechanical_gate_pass"],
            "runtime_camera_gate_pass": runtime_pass,
            "ready_v1_fraction": sum(row["ready_v1"] for row in verification) / max(len(verification), 1),
            "ready_v2_fraction": sum(row["ready_v2"] for row in verification) / max(len(verification), 1),
            "view_replay_conversion_not_active_observation": sum(item[3] for item in conversions) / max(len(conversions), 1),
            "view_replay_conversion_candidate_count": len(conversions),
            "view_replay_conversion_by_class": {key: {"converted": value[0], "matched_candidates": value[1], "fraction": value[0] / max(value[1], 1)} for key, value in class_conversions.items()},
        }

    runtime_eligible = [config for config in CONFIGS if config_summary[config]["runtime_camera_gate_pass"]]
    audit_camera = max(runtime_eligible, key=lambda item: (config_summary[item]["ready_v2_fraction"], -config_summary[item]["verification_self_pixels_p95_worst_world"])) if runtime_eligible else None
    audit_rows = [row for row in all_rows if row["config"] == audit_camera and row["role"] == "verification"] if audit_camera else []
    extra_audit_captures = 0
    if audit_camera:
        for extra_root_raw in args.audit_extra_root:
            extra_root = Path(extra_root_raw)
            for world in WORLDS:
                role_root = extra_root / audit_camera / world / "verification"
                if not (role_root / "capture_report.json").exists():
                    continue
                extra_rows, runtime = collect_role(role_root, audit_camera, world, "verification", policy_v1, policy_v2)
                if not runtime["capture_pass"] or runtime["captured_frames"] < 10:
                    raise RuntimeError(f"incomplete audit capture: {role_root}")
                audit_rows.extend(extra_rows)
                extra_audit_captures += 1
    manual = select_blind_samples(audit_rows, Path(args.blind_audit_output)) if audit_camera else {
        "sample_count": 0, "manual_gate_pass": False, "manual_gate_blocker": "no_runtime_camera_candidate_passed"
    }
    manual["audit_camera_candidate"] = audit_camera
    manual["extra_verification_capture_count"] = extra_audit_captures

    rows_output = Path(args.rows_output)
    rows_output.parent.mkdir(parents=True, exist_ok=True)
    with rows_output.open("w", encoding="utf-8", newline="\n") as stream:
        for row in all_rows:
            stream.write(json.dumps(row, sort_keys=True) + "\n")

    report = {
        "schema_version": 1, "stage": "Stage5BR5",
        "same_asset_seed_pose_and_trajectory_within_world": True,
        "worlds": list(WORLDS), "camera_configs_executed": list(CONFIGS),
        "camera_config_mechanically_pruned": mechanics["mechanically_pruned_candidates"],
        "runtime_capture_count": len(CONFIGS) * len(WORLDS) * 2,
        "runtime_frame_count": len(CONFIGS) * len(WORLDS) * 2 * 10,
        "policy_v1_sha256": sha256(policy_v1_path),
        "policy_v2_sha256": sha256(policy_v2_path),
        "policy_v2_frozen_before_model_training": bool(policy_v2["frozen_before_model_training"]),
        "policy_v2_training_permitted": bool(policy_v2["training_permitted"]),
        "mechanics": mechanics,
        "camera_results": config_summary,
        "runtime_eligible_camera_candidates": runtime_eligible,
        "manual_audit": manual,
        "camera_selected": None,
        "camera_selection_gate_pass": False,
        "camera_selection_blocker": manual["manual_gate_blocker"] if runtime_eligible else "no_runtime_camera_candidate_passed",
        "active_observation_runtime_executed": False,
        "active_observation_runtime_reason": "camera cannot be frozen before required two-reviewer manual gate",
        "oracle_candidate_only": True,
        "competition_evidence": False,
        "model_micro_overfit_executed": False,
        "dataset_120_1200_executed": False,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({
        "runtime_frame_count": report["runtime_frame_count"],
        "runtime_eligible": runtime_eligible,
        "manual_audit": manual,
        "camera_results": config_summary,
        "camera_selected": None,
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
