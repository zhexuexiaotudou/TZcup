from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(PACKAGE))
from sanitation_learning.observability import build_report, load_policy, recognition_ready, summarize_rows  # noqa: E402


CLASS_INDEX = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter", 4: "leaf_pile", 5: "puddle"}
DISTANCE_BUCKETS = ((0.5, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0))


def collect(role_root: Path, policy: dict) -> tuple[list[dict], dict]:
    capture = json.loads((role_root / "capture_report.json").read_text(encoding="utf-8"))
    raw = []
    maximum_area = {}
    self_fractions = []
    for record in capture["records"]:
        semantic = np.load(role_root / record["paths"]["semantic"], allow_pickle=False)
        instances = np.load(role_root / record["paths"]["instance"], allow_pickle=False)
        depth = np.load(role_root / record["paths"]["depth"], allow_pickle=False)
        self_fractions.append(float(np.mean(semantic == 250)))
        for instance_id in (int(v) for v in np.unique(instances) if int(v) != 0):
            mask = instances == instance_id
            labels = semantic[mask].astype(np.int64)
            semantic_id = int(np.bincount(labels).argmax()) if labels.size else 0
            if semantic_id not in CLASS_INDEX:
                continue
            ys, xs = np.nonzero(mask)
            values = depth[mask]
            valid = np.isfinite(values) & (values > 0)
            row = {
                "frame_index": int(record["frame_index"]), "instance_id": instance_id,
                "class_id": CLASS_INDEX[semantic_id], "visibility": "visible",
                "mask_area_px": int(mask.sum()),
                "bbox_shortest_side_px": int(min(xs.max() - xs.min() + 1, ys.max() - ys.min() + 1)),
                "distance_m": float(np.median(values[valid])) if valid.any() else None,
                "depth_valid_ratio": float(valid.mean()) if values.size else 0.0,
            }
            raw.append(row)
            key = instance_id
            maximum_area[key] = max(maximum_area.get(key, 0), row["mask_area_px"])
    for row in raw:
        reference = maximum_area[row["instance_id"]]
        row["visible_fraction"] = row["mask_area_px"] / reference if reference else None
        row["occlusion"] = 1.0 - row["visible_fraction"] if row["visible_fraction"] is not None else None
        row["recognition_ready"] = recognition_ready(row, policy)
    runtime = {
        "capture_pass": bool(capture["capture_pass"]),
        "captured_frames": int(capture["captured_frames"]),
        "exact_timestamp_frames": sum(bool(row["exact_four_sensor_timestamp"]) for row in capture["records"]),
        "camera_xyz_m": capture["camera_xyz_m"],
        "optical_frame": capture["optical_frame"],
        "self_pixel_fraction": {
            "p10": float(np.percentile(self_fractions, 10)),
            "p50": float(np.percentile(self_fractions, 50)),
            "p90": float(np.percentile(self_fractions, 90)),
        },
    }
    return raw, runtime


def distance_buckets(rows: list[dict]) -> dict:
    report = {}
    for low, high in DISTANCE_BUCKETS:
        subset = [row for row in rows if row.get("distance_m") is not None and low <= row["distance_m"] < high]
        report[f"{low:g}-{high:g}m"] = {
            "all_visible": len(subset),
            "recognition_ready": sum(bool(row["recognition_ready"]) for row in subset),
            "non_ready": sum(not bool(row["recognition_ready"]) for row in subset),
        }
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.root)
    policy_path = PACKAGE / "config" / "perception_evaluability_policy.yaml"
    config_path = PACKAGE / "config" / "stage5br4_active_perception.yaml"
    policy = load_policy(policy_path)
    config_doc = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    configs = {}
    captured_rows = {}
    for camera_id in ("C0", "C1", "C2", "C3"):
        roles = ("discovery", "verification") if camera_id == "C3" else ("discovery",)
        role_reports = {}
        for role in roles:
            rows, runtime = collect(root / camera_id / role, policy)
            captured_rows[(camera_id, role)] = rows
            role_reports[role] = {
                "runtime": runtime,
                "partitions": summarize_rows(rows, policy),
                "distance_buckets": distance_buckets(rows),
            }
        geometry = build_report([], policy_path, config_path, camera_id)
        configs[camera_id] = {
            "camera_spec": config_doc["camera_configs"][camera_id],
            "roles": role_reports,
            "ground_coverage": geometry["ground_coverage"],
            "estimated_uncompressed_rgbd_bandwidth_mbps": geometry["estimated_uncompressed_rgbd_bandwidth_mbps"],
            "mounting_and_collision_risk": geometry["mounting_and_collision_risk"],
        }

    discovery = captured_rows[("C3", "discovery")]
    verification = captured_rows[("C3", "verification")]
    discovery_non_ready = {(row["instance_id"], row["frame_index"]) for row in discovery if not row["recognition_ready"]}
    verification_ready_ids = {row["instance_id"] for row in verification if row["recognition_ready"]}
    candidate_ids = {instance_id for instance_id, _ in discovery_non_ready}
    converted = candidate_ids & verification_ready_ids
    conversion = len(converted) / len(candidate_ids) if candidate_ids else None
    report = {
        "schema_version": 1,
        "stage": "Stage5BR4",
        "world_id": "world_a_asphalt_campus",
        "scene_seed": 11,
        "same_world_asset_pose_seed_and_commanded_trajectory": True,
        "trajectory_replay_semantics": "C3 verification resets all object and vehicle poses, then replays the same 0.35 m/s command and frame-motion gate",
        "evaluability_policy_sha256": build_report([], policy_path, config_path, "C0")["evaluability_policy_sha256"],
        "camera_configs": configs,
        "active_observation": {
            "candidate_instances": len(candidate_ids),
            "converted_to_recognition_ready": len(converted),
            "ready_conversion": conversion,
            "gate_min": 0.90,
            "gate_pass": conversion is not None and conversion >= 0.90,
            "small_scale_single_scene_audit_only": True,
        },
        "camera_ablation_runtime_pass": all(
            role["runtime"]["capture_pass"] and role["runtime"]["captured_frames"] == 10
            for config in configs.values() for role in config["roles"].values()
        ),
        "camera_selected_for_model_training": None,
        "selection_blocker": "single_scene_camera_audit_does_not_satisfy_multi_world_manual_recognizability_and_collision_envelope_gate",
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({
        "camera_ablation_runtime_pass": report["camera_ablation_runtime_pass"],
        "active_observation": report["active_observation"],
        "ready_fractions": {cid: {role: data["partitions"]["recognition_ready_fraction"] for role, data in cfg["roles"].items()} for cid, cfg in configs.items()},
        "self_pixel_p50": {cid: {role: data["runtime"]["self_pixel_fraction"]["p50"] for role, data in cfg["roles"].items()} for cid, cfg in configs.items()},
    }, indent=2))


if __name__ == "__main__":
    main()
