from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_learning"
import sys
sys.path.insert(0, str(PACKAGE))

from sanitation_learning.observability import build_report, write_report  # noqa: E402


def load_rows(records_path: Path, data_root: Path) -> list[dict]:
    rows = [json.loads(line) for line in records_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    maximum_area = {}
    for row in rows:
        key = (int(row["scene_seed"]), int(row["instance_id"]))
        maximum_area[key] = max(maximum_area.get(key, 0), int(row.get("mask_area_px", 0)))
    enriched = []
    for source in rows:
        row = dict(source)
        row["class_id"] = row.get("semantic_class")
        if row.get("visibility") != "visible":
            enriched.append(row)
            continue
        scene = data_root / "scenes" / f"scene_{int(row['scene_seed']):04d}"
        frame = f"frame_{int(row['frame_index']):02d}.npy"
        instance_path = scene / "instance" / frame
        depth_path = scene / "depth" / frame
        if not instance_path.is_file() or not depth_path.is_file():
            row.update({"distance_m": None, "depth_valid_ratio": None, "visible_fraction": None, "occlusion": None, "raw_measurement_available": False})
            enriched.append(row)
            continue
        instances = np.load(instance_path, allow_pickle=False)
        depth = np.load(depth_path, allow_pickle=False)
        mask = instances == int(row["instance_id"])
        values = depth[mask]
        valid = np.isfinite(values) & (values > 0)
        reference = maximum_area[(int(row["scene_seed"]), int(row["instance_id"]))]
        visible_fraction = float(mask.sum() / reference) if reference else None
        row.update({
            "distance_m": float(np.median(values[valid])) if valid.any() else None,
            "depth_valid_ratio": float(valid.mean()) if values.size else 0.0,
            "visible_fraction": visible_fraction,
            "occlusion": max(0.0, 1.0 - visible_fraction) if visible_fraction is not None else None,
            "raw_measurement_available": True,
            "visible_fraction_semantics": "instance area divided by same-scene maximum observed area; occlusion proxy, not unoccluded CAD truth",
        })
        enriched.append(row)
    return enriched


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--records", required=True)
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--policy", default=str(PACKAGE / "config" / "perception_evaluability_policy.yaml"))
    parser.add_argument("--config", default=str(PACKAGE / "config" / "stage5br4_active_perception.yaml"))
    parser.add_argument("--camera-id", default="C0")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rows = load_rows(Path(args.records), Path(args.data_root))
    report = build_report(rows, args.policy, args.config, args.camera_id)
    report["source"] = {
        "records": str(Path(args.records).resolve()),
        "data_root": str(Path(args.data_root).resolve()),
        "row_count": len(rows),
        "runtime_camera_config_measured": args.camera_id == "C0",
    }
    write_report(report, args.output)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
