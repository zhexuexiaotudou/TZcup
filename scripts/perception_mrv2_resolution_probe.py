#!/usr/bin/env python3
"""Bounded R640/R960/R1280 pre-training comparison for MRV2-A."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import sys
import time

import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g4_data import index_instance_records  # noqa: E402
from sanitation_learning.g4_direct_fcos import build_direct_fcos, direct_predictions  # noqa: E402
from sanitation_learning.g4_evaluation import discrete_metrics, match_discrete_predictions  # noqa: E402
from perception_prod_x1_full_pipeline import candidate_size_metrics, load_partition, sha256  # noqa: E402
from perception_mrv2_a_train import RESOLUTIONS  # noqa: E402


def percentile(values, q):
    return float(np.percentile(np.asarray(values), q))


def profile(model, rows, instances, device, input_size, threshold):
    sample = rows[:1]
    for _ in range(3):
        direct_predictions(model, sample, instances, device=device, score_threshold=threshold, batch_size=1, input_size=input_size, top_k=16)
    torch.cuda.synchronize(); torch.cuda.reset_peak_memory_stats(); samples = []
    for _ in range(20):
        started = time.perf_counter()
        direct_predictions(model, sample, instances, device=device, score_threshold=threshold, batch_size=1, input_size=input_size, top_k=16)
        torch.cuda.synchronize(); samples.append((time.perf_counter() - started) * 1000)
    return {
        "samples": len(samples), "p50_ms": statistics.median(samples),
        "p95_ms": percentile(samples, 95), "max_ms": max(samples),
        "peak_allocated_vram_bytes": int(torch.cuda.max_memory_allocated()),
        "scope": "single full frame with repository preprocessing and file IO",
    }


def filtered_metrics(raw, threshold):
    frames = []
    for frame in raw:
        items = [item for item in frame["predictions"] if float(item["score"]) >= threshold][:16]
        frames.append({**frame, "predictions": items, "detections": items})
    metrics = discrete_metrics(match_discrete_predictions(frames))
    size = candidate_size_metrics(frames)
    return {"discrete": metrics, "candidate_size": size}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("MRV2 resolution probe requires CUDA")
    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=False)
    threshold = float(checkpoint["frozen_threshold_from_train_world_holdout"])
    rows, records = load_partition(args.data_root, args.evidence_dir, allowed_splits={"val"})
    instances = index_instance_records(records)
    results = {}
    for resolution, input_size in RESOLUTIONS.items():
        print(f"[MRV2 resolution] R{resolution} {input_size}", flush=True)
        model = build_direct_fcos(input_size=input_size).to(device)
        model.load_state_dict(checkpoint["state_dict"], strict=True); model.eval()
        raw = direct_predictions(model, rows, instances, device=device, score_threshold=0.01, batch_size=2 if resolution == 640 else 1, input_size=input_size, top_k=100)
        results[f"R{resolution}"] = {
            "input_size": input_size,
            "raw_score_0_01_top16": filtered_metrics(raw, 0.01),
            "x3_frozen_threshold_top16": filtered_metrics(raw, threshold),
            "performance": profile(model, rows, instances, device, input_size, threshold),
        }
        del model; torch.cuda.empty_cache()
    report = {
        "schema_version": 1, "stage": "MRV2-A-RESOLUTION-PROBE",
        "model": {"path": args.checkpoint.as_posix(), "sha256": sha256(args.checkpoint)},
        "split": "VAL_development_only", "frozen_X3_threshold": threshold,
        "results": results,
        "selection_policy": "prefer the smallest resolution that improves small recall without violating FP/min and the 200ms detector budget",
        "G5_SEALED_FINAL_read": False, "legacy_G4_D6_read": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
