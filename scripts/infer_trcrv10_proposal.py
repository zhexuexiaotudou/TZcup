#!/usr/bin/env python3
"""Run one registered historical detector as a TRCRV10 proposal generator."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402


CLASS_PROMPT = ("plastic_bottle", "metal_can", "paper_litter")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def prepare_model(model) -> dict:
    if model.cfg.get("test_dataloader") is None:
        model.cfg.test_dataloader = {"dataset": {"pipeline": model.cfg.test_pipeline}}
    if model.__class__.__name__ == "GroundingDINO":
        return {"text_prompt": CLASS_PROMPT, "custom_entities": True}
    return {}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--coco", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--candidate-id", required=True)
    parser.add_argument("--expected-checkpoint-sha256", required=True)
    parser.add_argument("--expected-config-sha256")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--windows-root")
    parser.add_argument("--container-root")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    checkpoint_sha = sha256(args.checkpoint)
    config_sha = sha256(args.config)
    if checkpoint_sha != args.expected_checkpoint_sha256:
        raise RuntimeError("checkpoint SHA-256 mismatch")
    if args.expected_config_sha256 and config_sha != args.expected_config_sha256:
        raise RuntimeError("config SHA-256 mismatch")

    patch_mmdet_cuda_nms()
    try:
        from transformers import BertConfig
    except ImportError:
        BertConfig = None
    if BertConfig is not None:
        original = BertConfig.from_pretrained

        def eager(*config_args, **config_kwargs):
            config = original(*config_args, **config_kwargs)
            config._attn_implementation = "eager"
            return config

        BertConfig.from_pretrained = eager
    try:
        import mmcv.ops.multi_scale_deform_attn as deform_attn
        deform_attn.IS_CUDA_AVAILABLE = False
    except ImportError:
        pass
    from mmdet.apis import inference_detector, init_detector

    payload = json.loads(args.coco.read_text(encoding="utf-8"))
    model = init_detector(str(args.config), str(args.checkpoint), palette="random", device="cuda:0")
    inference_kwargs = prepare_model(model)
    frames = []
    for offset in range(0, len(payload["images"]), args.batch_size):
        batch = payload["images"][offset:offset + args.batch_size]
        paths = [row["file_name"] for row in batch]
        if args.windows_root and args.container_root:
            paths = [path.replace(args.windows_root, args.container_root).replace("\\", "/") for path in paths]
        outputs = inference_detector(model, paths, **inference_kwargs)
        outputs = outputs if isinstance(outputs, list) else [outputs]
        for image, output in zip(batch, outputs):
            pred = output.pred_instances.to("cpu")
            frames.append({
                "image_id": int(image["id"]),
                "scene": image["scene"],
                "mission_id": image["mission_id"],
                "frame_index": int(image["frame_index"]),
                "negative_only": bool(image["negative_only"]),
                "detections": [
                    {
                        "bbox_xyxy": [float(value) for value in box],
                        "score": float(score),
                        "source_class_label": int(label),
                    }
                    for box, score, label in zip(
                        pred.bboxes.tolist(), pred.scores.tolist(), pred.labels.tolist()
                    )
                ],
            })
    report = {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "stage": "TRCRV10-03-PROPOSAL-RAW-INFERENCE",
        "candidate_id": args.candidate_id,
        "score_semantics": "max_class_score_as_litter_objectness",
        "checkpoint_sha256": checkpoint_sha,
        "config_sha256": config_sha,
        "input_index_sha256": sha256(args.coco),
        "frame_count": len(frames),
        "threshold_tuning_performed": False,
        "frames": frames,
        "G10_DEV_VAL_SEALED_read": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
