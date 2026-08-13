#!/usr/bin/env python3
"""Run a frozen MMDetection checkpoint over a G9 COCO frame stream."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--coco", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--windows-root")
    parser.add_argument("--container-root")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if sha256(args.checkpoint) != args.expected_sha256:
        raise RuntimeError("checkpoint SHA-256 mismatch")
    patch_mmdet_cuda_nms()
    from mmdet.apis import inference_detector, init_detector

    payload = json.loads(args.coco.read_text(encoding="utf-8"))
    model = init_detector(str(args.config), str(args.checkpoint), device="cuda:0")
    frames = []
    for offset in range(0, len(payload["images"]), args.batch_size):
        batch = payload["images"][offset : offset + args.batch_size]
        paths = [row["file_name"] for row in batch]
        if args.windows_root and args.container_root:
            paths = [path.replace(args.windows_root, args.container_root).replace("\\", "/") for path in paths]
        outputs = inference_detector(model, paths)
        outputs = outputs if isinstance(outputs, list) else [outputs]
        for image, output in zip(batch, outputs):
            pred = output.pred_instances.to("cpu")
            frames.append({
                "image_id": int(image["id"]),
                "mission_id": image["mission_id"],
                "frame_index": int(image["frame_index"]),
                "negative_only": bool(image["negative_only"]),
                "detections": [
                    {"bbox_xyxy": [float(value) for value in box], "score": float(score), "label": int(label)}
                    for box, score, label in zip(pred.bboxes.tolist(), pred.scores.tolist(), pred.labels.tolist())
                ],
            })
    report = {"schema_version": 1, "protocol": "TGARV9", "stage": "G9_ROUTE_A_RAW_INFERENCE", "checkpoint_sha256": args.expected_sha256, "config_sha256": sha256(args.config), "frame_count": len(frames), "frames": frames, "VAL_NEW_read": False, "G5_V2_read": False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
