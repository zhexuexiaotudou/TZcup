#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import time

import cv2
import numpy as np


CANDIDATES = ((256, 192), (384, 288), (512, 384), (640, 384))
CLASS_NAMES = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter", 4: "leaf_pile", 5: "puddle"}


def stats(values):
    return {"count": len(values), "p10": float(np.percentile(values, 10)) if values else None, "p50": float(np.percentile(values, 50)) if values else None, "p90": float(np.percentile(values, 90)) if values else None}


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--data-root", required=True); parser.add_argument("--output", required=True); args = parser.parse_args()
    root = Path(args.data_root); frames = []
    for scene in sorted((root/"scenes").glob("scene_*")):
        report = json.loads((scene/"capture_report.json").read_text())
        for record in report["records"]: frames.append((scene/record["paths"]["rgb"], scene/record["paths"]["instance"], scene/record["paths"]["semantic"]))
    reports = []
    for width, height in CANDIDATES:
        shortest = defaultdict(list); areas = defaultdict(list); visible = defaultdict(int); io_bytes = 0; started = time.perf_counter()
        for rgb_path, instance_path, semantic_path in frames:
            rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR); instance = np.load(instance_path, allow_pickle=False); semantic = np.load(semantic_path, allow_pickle=False)
            io_bytes += rgb_path.stat().st_size + instance_path.stat().st_size + semantic_path.stat().st_size
            resized_rgb = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_AREA)
            resized_instance = cv2.resize(instance.astype(np.int32), (width, height), interpolation=cv2.INTER_NEAREST)
            resized_semantic = cv2.resize(semantic, (width, height), interpolation=cv2.INTER_NEAREST)
            _ = resized_rgb.astype(np.float32) / 255.0
            present_classes = set()
            for instance_id in (int(v) for v in np.unique(resized_instance) if int(v) != 0):
                mask = resized_instance == instance_id; labels = resized_semantic[mask].astype(np.int64); label = int(np.bincount(labels, minlength=6).argmax())
                if label not in CLASS_NAMES: continue
                ys, xs = np.nonzero(mask); shortest[CLASS_NAMES[label]].append(int(min(xs.max()-xs.min()+1, ys.max()-ys.min()+1))); areas[CLASS_NAMES[label]].append(int(mask.sum())); present_classes.add(CLASS_NAMES[label])
            for name in present_classes: visible[name] += 1
        elapsed = time.perf_counter() - started
        reports.append({"resolution": [width, height], "processed_frames": len(frames), "preprocess_wall_sec": elapsed, "preprocess_fps": len(frames)/elapsed, "source_io_bytes": io_bytes, "source_io_mib_per_sec": io_bytes/elapsed/(1024**2), "bbox_shortest_side_px": {name: stats(shortest[name]) for name in CLASS_NAMES.values()}, "mask_area_px": {name: stats(areas[name]) for name in CLASS_NAMES.values()}, "visible_frame_fraction": {name: visible[name]/len(frames) for name in CLASS_NAMES.values()}, "training_peak_vram_mib": None, "onnx_latency_ms": None, "onnx_model_bytes": None, "pending_fields_reason": "measured only after each split-model screening attempt"})
    discrete = ("plastic_bottle", "metal_can", "paper_litter")
    ranked = sorted(reports, key=lambda item: (sum(item["bbox_shortest_side_px"][name]["p10"] or 0 for name in discrete), sum(item["mask_area_px"][name]["p10"] or 0 for name in discrete), item["preprocess_fps"]), reverse=True)
    selected = [item["resolution"] for item in ranked[:2]]
    report = {"schema_version": 1, "stage": "Stage5BR3 native-once offline resolution scan", "native_capture_resolution": [640, 480], "frame_count": len(frames), "candidates": reports, "selected_for_model_screening": selected, "selection_limit": 2, "selection_basis": "maximize summed discrete-class bbox-short-side p10, then mask-area p10, then preprocessing throughput", "resolution_scan_pass": len(frames)==800 and len(selected)<=2}
    output=Path(args.output); output.parent.mkdir(parents=True, exist_ok=True); output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({"frame_count":len(frames),"selected_for_model_screening":selected,"resolution_scan_pass":report["resolution_scan_pass"]},indent=2)); return 0 if report["resolution_scan_pass"] else 2


if __name__ == "__main__": raise SystemExit(main())
