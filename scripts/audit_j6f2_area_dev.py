#!/usr/bin/env python3
"""Replay the recovered development-only AUTO-05 area ONNX on TRAIN frames."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re

import cv2
import numpy as np


FORBIDDEN_TOKENS = ("G5_V2", "SEALED_FINAL", "DEV_VAL")
EXPECTED_MODEL_SHA256 = "82a408f17c81f0aebe68debcb5385eccde859308f59fe8ab2e8bcff72414b3eb"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _normalized_path_components(path: Path) -> tuple[str, ...]:
    return tuple(
        re.sub(r"[^A-Z0-9]+", "_", component.upper()).strip("_")
        for component in path.resolve().parts
    )


def validate_nonsealed_path(path: Path) -> None:
    components = _normalized_path_components(path)
    matched = [
        token
        for token in FORBIDDEN_TOKENS
        if any(token in component for component in components)
    ]
    if matched:
        raise ValueError(f"forbidden calibration/evaluation source: {matched}")


def validate_development_path(path: Path) -> None:
    validate_nonsealed_path(path)
    component_words = {
        word
        for component in _normalized_path_components(path)
        for word in component.split("_")
    }
    if not component_words.intersection({"TRAIN", "DEVELOPMENT"}):
        raise ValueError("area replay requires an explicitly named TRAIN/development root")


def frame_records(root: Path, maximum_frames: int | None) -> list[dict]:
    if maximum_frames is not None and maximum_frames <= 0:
        raise ValueError("maximum_frames must be positive when provided")
    validate_development_path(root)
    root = root.resolve(strict=True)
    if not root.is_dir():
        raise ValueError("development root must be a directory")
    records = []
    for rgb in sorted(root.rglob("rgb/*.png")):
        scene = rgb.parent.parent
        depth = scene / "depth" / f"{rgb.stem}.npy"
        semantic = scene / "semantic" / f"{rgb.stem}.npy"
        if depth.is_file() and semantic.is_file():
            resolved = {}
            for name, candidate in (("rgb", rgb), ("depth", depth), ("semantic", semantic)):
                if candidate.is_symlink():
                    raise ValueError(f"symlinked development frame is not allowed: {candidate}")
                resolved_candidate = candidate.resolve(strict=True)
                try:
                    resolved_candidate.relative_to(root)
                except ValueError as exc:
                    raise ValueError(
                        f"development frame escapes the declared root: {candidate}"
                    ) from exc
                validate_nonsealed_path(resolved_candidate)
                resolved[name] = resolved_candidate
            records.append(resolved)
        if maximum_frames is not None and len(records) >= maximum_frames:
            break
    if not records:
        raise ValueError("no paired RGB/depth/semantic development frames found")
    return records


def evaluate(model: Path, root: Path, maximum_frames: int | None = None) -> dict:
    import onnxruntime as ort
    from sanitation_perception.legacy_area_development import (
        decode_legacy_area,
        preprocess_legacy_area,
    )

    validate_nonsealed_path(model)
    model = model.resolve(strict=True)
    if not model.is_file():
        raise ValueError("area model must be a regular file")
    model_sha = sha256(model)
    if model_sha != EXPECTED_MODEL_SHA256:
        raise ValueError("recovered area model SHA-256 mismatch")
    records = frame_records(root, maximum_frames)
    session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
    if session.get_providers()[0] != "CPUExecutionProvider":
        raise RuntimeError("development replay provider mismatch")
    input_name = session.get_inputs()[0].name
    intersection = {"leaf_pile": 0, "puddle": 0}
    union = {"leaf_pile": 0, "puddle": 0}
    positive_frames = {"leaf_pile": 0, "puddle": 0}
    negative_false_positive_frames = {"leaf_pile": 0, "puddle": 0}
    frame_hashes = []
    for record in records:
        bgr = cv2.imread(str(record["rgb"]), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError(f"failed to decode RGB frame: {record['rgb']}")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        depth = np.load(record["depth"], allow_pickle=False).astype(np.float32)
        semantic = np.load(record["semantic"], allow_pickle=False)
        if depth.ndim != 2 or depth.shape != rgb.shape[:2]:
            raise ValueError(f"RGB/depth dimensions differ: {record['depth']}")
        if semantic.ndim != 2 or semantic.shape != rgb.shape[:2]:
            raise ValueError(f"RGB/semantic dimensions differ: {record['semantic']}")
        if not np.issubdtype(semantic.dtype, np.integer):
            raise ValueError(f"semantic labels must use an integer dtype: {record['semantic']}")
        logits = session.run(None, {input_name: preprocess_legacy_area(rgb, depth)})[0]
        decoded = decode_legacy_area(logits)
        truth = cv2.resize(
            semantic.astype(np.uint8),
            (512, 384),
            interpolation=cv2.INTER_NEAREST,
        )
        for class_name, label in (("leaf_pile", 4), ("puddle", 5)):
            predicted = decoded[class_name]["mask"]
            expected = truth == label
            intersection[class_name] += int(np.logical_and(predicted, expected).sum())
            union[class_name] += int(np.logical_or(predicted, expected).sum())
            positive_frames[class_name] += int(expected.any())
            negative_false_positive_frames[class_name] += int(
                not expected.any() and predicted.sum() >= 20
            )
        frame_hashes.append(
            {
                "rgb": str(record["rgb"]),
                "rgb_sha256": sha256(record["rgb"]),
                "depth_sha256": sha256(record["depth"]),
                "semantic_sha256": sha256(record["semantic"]),
            }
        )
    iou = {
        name: intersection[name] / union[name] if union[name] else None
        for name in intersection
    }
    finite_iou = [value for value in iou.values() if value is not None]
    return {
        "schema_version": 1,
        "stage": "J6F2_AREA_DEVELOPMENT_TRAIN_REPLAY",
        "development_only": True,
        "competition_claim_allowed": False,
        "release_allowed": False,
        "not_journey6_runtime": True,
        "runtime_backend": "PC_ONNX_CPU",
        "model_sha256": model_sha,
        "preprocess_contract": "auto05_attempt3_rgb_depth_edge_contrast_saturation_7ch",
        "class_order": ["leaf_pile", "puddle"],
        "thresholds": {"leaf_pile": 0.9, "puddle": 0.3},
        "frame_count": len(records),
        "positive_frames": positive_frames,
        "iou_by_class": iou,
        "macro_iou": sum(finite_iou) / len(finite_iou) if finite_iou else None,
        "negative_false_positive_frames": negative_false_positive_frames,
        "J6_PC_AREA_FUNCTIONAL_PASS": False,
        "truth_boundary": (
            "TRAIN replay proves only current executable compatibility. It is not live "
            "Gazebo, formal Area Gate, Journey 6, release, or product evidence."
        ),
        "frames": frame_hashes,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--development-root", type=Path, required=True)
    parser.add_argument("--maximum-frames", type=int)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args.model, args.development_root, args.maximum_frames)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key != "frames"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
