#!/usr/bin/env python3
"""Replay one frozen score-reviewed frame set through an ONNX model."""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from competition_perception_score import score


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
from sanitation_perception.preprocessing import (  # noqa: E402
    CLASS_ORDER,
    preprocess_rgb,
    resize_labels,
)


def predict_raw(session, image_rgb, minimum_pixels=24):
    logits = session.run(["logits"], {"images": preprocess_rgb(image_rgb)})[0][0]
    shifted = logits - logits.max(axis=0, keepdims=True)
    probability = np.exp(shifted) / np.exp(shifted).sum(axis=0, keepdims=True)
    labels = np.argmax(logits, axis=0).astype(np.uint8)
    full_size = resize_labels(labels, image_rgb.shape[1], image_rgb.shape[0])
    predictions = []
    for class_index, class_id in enumerate(CLASS_ORDER[1:], 1):
        count, components, stats, _ = cv2.connectedComponentsWithStats(
            (full_size == class_index).astype(np.uint8), 8
        )
        for component in range(1, count):
            if int(stats[component, cv2.CC_STAT_AREA]) < minimum_pixels:
                continue
            x = int(stats[component, cv2.CC_STAT_LEFT])
            y = int(stats[component, cv2.CC_STAT_TOP])
            width = int(stats[component, cv2.CC_STAT_WIDTH])
            height = int(stats[component, cv2.CC_STAT_HEIGHT])
            mask = cv2.resize(
                (components == component).astype(np.uint8),
                (128, 96),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
            confidence = float(probability[class_index][mask].mean()) if mask.any() else 0.0
            predictions.append({
                "class_id": class_id,
                "confidence": confidence,
                "xyxy": [x, y, x + width, y + height],
            })
    return predictions


def load_raw_output_presence(summary_path):
    summary = json.loads(Path(summary_path).read_text(encoding="utf-8"))
    frames = summary.get("selected_frames")
    if not isinstance(frames, list) or not frames:
        raise ValueError("summary selected_frames must be a nonempty list")
    presence = {}
    for frame in frames:
        frame_id = str(frame.get("frame_id", ""))
        if not frame_id or frame_id in presence:
            raise ValueError("summary contains missing or duplicate frame ids")
        if type(frame.get("raw_output_present")) is not bool:
            raise ValueError("summary raw_output_present must be boolean")
        presence[frame_id] = frame["raw_output_present"]
    return presence


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--expected-sha256", default="")
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit("fresh output required")
    model_sha256 = hashlib.sha256(args.model.read_bytes()).hexdigest()
    if args.expected_sha256 and model_sha256 != args.expected_sha256:
        raise SystemExit("model hash mismatch")
    import onnxruntime as ort

    session = ort.InferenceSession(str(args.model), providers=["CPUExecutionProvider"])
    source = json.loads(args.frames.read_text(encoding="utf-8"))
    presence = load_raw_output_presence(args.summary) if args.summary else None
    replay = []
    for frame in source:
        output_present = True if presence is None else presence.get(frame["frame_id"])
        if output_present is None:
            raise SystemExit(f"summary has no raw_output_present for {frame['frame_id']}")
        image_path = Path(frame["image_path"])
        if not image_path.is_absolute():
            image_path = args.frames.parent / image_path
        image = cv2.cvtColor(cv2.imread(str(image_path)), cv2.COLOR_BGR2RGB)
        replay.append({
            "frame_id": frame["frame_id"],
            "image_path": frame["image_path"],
            "predictions": predict_raw(session, image) if output_present else [],
            "truth": frame["truth"],
        })
    result = score(replay)
    result.update({
        "model_sha256": model_sha256,
        "replay_status": "OFFLINE_FROZEN_FRAME_REPLAY",
        "policy_replay_status": "NOT_RUN",
        "scored_raw_output_frame_count": sum(bool(frame["predictions"]) for frame in replay),
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "status": result["status"],
        "model_sha256": model_sha256,
        "class_metrics": result["class_metrics"],
    }, indent=2))


if __name__ == "__main__":
    main()
