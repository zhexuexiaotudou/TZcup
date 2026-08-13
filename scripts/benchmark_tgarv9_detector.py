#!/usr/bin/env python3
"""Conditional 300-500 frame CUDA deployability pre-screen for T2/T3."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402


CLASS_PROMPT = ("plastic_bottle", "metal_can", "paper_litter")


def prepare_inference_model(model):
    if model.cfg.get("test_dataloader") is None:
        model.cfg.test_dataloader = {"dataset": {"pipeline": model.cfg.test_pipeline}}
    if model.__class__.__name__ == "GroundingDINO":
        return {"text_prompt": CLASS_PROMPT, "custom_entities": True}
    return {}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--selection-report", type=Path, required=True)
    parser.add_argument("--coco", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--windows-root", required=True)
    parser.add_argument("--container-root", required=True)
    parser.add_argument("--frame-count", type=int, default=400)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    selection = json.loads(args.selection_report.read_text())
    pass_keys = [key for key in selection if key.endswith("_HOLDOUT_PASS")]
    if len(pass_keys) != 1 or not selection[pass_keys[0]]:
        raise RuntimeError("deployability pre-screen is forbidden before HOLDOUT pass")
    if sha256(args.checkpoint) != args.expected_sha256.lower():
        raise RuntimeError("selected checkpoint SHA-256 mismatch")
    payload = json.loads(args.coco.read_text())
    if not 300 <= args.frame_count <= 500:
        raise ValueError("pre-screen requires 300-500 frames")
    images = [payload["images"][index % len(payload["images"])] for index in range(args.frame_count)]
    paths = [row["file_name"].replace(args.windows_root, args.container_root).replace("\\", "/") for row in images]
    patch_mmdet_cuda_nms()
    try:
        from transformers import BertConfig
    except ImportError:
        BertConfig = None
    if BertConfig is not None:
        original_bert_config_from_pretrained = BertConfig.from_pretrained

        def eager_bert_config_from_pretrained(*config_args, **config_kwargs):
            config = original_bert_config_from_pretrained(*config_args, **config_kwargs)
            config._attn_implementation = "eager"
            return config

        BertConfig.from_pretrained = eager_bert_config_from_pretrained
    import mmcv.ops.multi_scale_deform_attn as deform_attn
    deform_attn.IS_CUDA_AVAILABLE = False
    import torch
    from mmdet.apis import inference_detector, init_detector
    model = init_detector(str(args.config), str(args.checkpoint), palette="random", device="cuda:0")
    inference_kwargs = prepare_inference_model(model)
    for path in paths[:20]:
        with torch.inference_mode(), torch.autocast(device_type="cuda", dtype=torch.float16):
            inference_detector(model, path, **inference_kwargs)
    torch.cuda.synchronize()
    durations = []
    with torch.inference_mode():
        for path in paths:
            started = time.perf_counter()
            with torch.autocast(device_type="cuda", dtype=torch.float16):
                inference_detector(model, path, **inference_kwargs)
            torch.cuda.synchronize()
            durations.append(time.perf_counter() - started)
    total = sum(durations)
    effective_hz = len(durations) / total
    report = {
        "schema_version": 1,
        "protocol": "TGARV9",
        "stage": "DETECTOR_DEPLOYABILITY_PRE_SCREEN",
        "mode": "CUDA batch=1 inference_mode AMP",
        "warmup_frames": 20,
        "measured_frames": len(durations),
        "effective_hz": effective_hz,
        "median_latency_ms": statistics.median(durations) * 1000.0,
        "p95_latency_ms": percentile(durations, 0.95) * 1000.0,
        "minimum_effective_hz": 5.0,
        "DEPLOYABILITY_PRE_SCREEN_PASS": effective_hz >= 5.0,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    return 0 if report["DEPLOYABILITY_PRE_SCREEN_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
