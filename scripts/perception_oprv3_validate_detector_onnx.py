#!/usr/bin/env python3
"""Validate MRV2-A ONNX outputs against the exact PyTorch checkpoint."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from perception_oprv3_moving_benchmark import load_detector, sha256  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--onnx", type=Path, required=True)
    parser.add_argument("--sample-rgb", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("detector parity validation requires CUDA PyTorch")
    model, metadata = load_detector(args.checkpoint, device)
    width, height = metadata["input_size"]
    bgr = cv2.imread(args.sample_rgb.as_posix(), cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError(f"failed to read sample RGB: {args.sample_rgb}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_CUBIC)
    images = np.ascontiguousarray(
        resized.transpose(2, 0, 1)[None], dtype=np.float32
    ) / 255.0
    with torch.no_grad():
        expected = model([torch.from_numpy(images[0]).to(device)])[0]
    expected_arrays = [
        expected["boxes"].detach().cpu().numpy(),
        expected["scores"].detach().cpu().numpy(),
        expected["labels"].detach().cpu().numpy(),
    ]
    import onnxruntime as ort

    session = ort.InferenceSession(
        args.onnx.as_posix(), providers=["CPUExecutionProvider"]
    )
    actual = session.run(None, {session.get_inputs()[0].name: images})
    same_shapes = all(a.shape == b.shape for a, b in zip(expected_arrays, actual))
    box_error = (
        float(np.max(np.abs(expected_arrays[0] - actual[0])))
        if same_shapes and actual[0].size
        else None
    )
    score_error = (
        float(np.max(np.abs(expected_arrays[1] - actual[1])))
        if same_shapes and actual[1].size
        else None
    )
    labels_equal = same_shapes and np.array_equal(expected_arrays[2], actual[2])
    thresholds = {
        "maximum_box_absolute_error": 0.01,
        "maximum_score_absolute_error": 1e-4,
    }
    parity_pass = bool(
        same_shapes
        and labels_equal
        and box_error is not None
        and box_error <= thresholds["maximum_box_absolute_error"]
        and score_error is not None
        and score_error <= thresholds["maximum_score_absolute_error"]
    )
    report = {
        "schema_version": 1,
        "protocol": "OPRV3-MRV2-A-ONNX-PARITY",
        "checkpoint_sha256": sha256(args.checkpoint),
        "onnx_sha256": sha256(args.onnx),
        "sample_rgb_sha256": sha256(args.sample_rgb),
        "detection_count": int(actual[0].shape[0]),
        "same_shapes": same_shapes,
        "labels_equal": bool(labels_equal),
        "maximum_box_absolute_error": box_error,
        "maximum_score_absolute_error": score_error,
        "thresholds": thresholds,
        "pass": parity_pass,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if parity_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
