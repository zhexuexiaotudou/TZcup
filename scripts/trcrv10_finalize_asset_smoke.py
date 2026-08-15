#!/usr/bin/env python3
"""Bind a cold-start positive Gazebo render smoke to the G10 asset audit."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


TARGET_LABELS = {"plastic_bottle": 1, "metal_can": 2, "paper_litter": 3}


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ref(path: Path) -> dict:
    return {"path": str(path.resolve()), "size_bytes": path.stat().st_size, "sha256": sha256(path)}


def write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def largest_boxes(scene: Path) -> dict[str, dict]:
    best: dict[str, dict] = {}
    for mask_path in sorted((scene / "semantic").glob("*.npy")):
        mask = np.load(mask_path)
        for class_id, label in TARGET_LABELS.items():
            ys, xs = np.where(mask == label)
            if not len(xs):
                continue
            bbox = [int(xs.min()), int(ys.min()), int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1)]
            short = min(bbox[2:])
            if class_id not in best or short > best[class_id]["short_side_px"]:
                best[class_id] = {"short_side_px": short, "frame": mask_path.stem, "bbox_xywh": bbox}
    return best


def write_montage(scene: Path, boxes: dict[str, dict], output: Path) -> None:
    crops = []
    for class_id in TARGET_LABELS:
        row = boxes[class_id]
        image = cv2.imread(str(scene / "rgb" / f"{row['frame']}.png"))
        x, y, width, height = row["bbox_xywh"]
        pad = 10
        crop = image[max(0, y - pad):min(image.shape[0], y + height + pad), max(0, x - pad):min(image.shape[1], x + width + pad)]
        crop = cv2.resize(crop, (240, 240), interpolation=cv2.INTER_NEAREST)
        cv2.putText(crop, f"{class_id} {row['short_side_px']}px", (5, 22), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1, cv2.LINE_AA)
        crops.append(crop)
    if not cv2.imwrite(str(output), np.concatenate(crops, axis=1)):
        raise RuntimeError("failed to write asset smoke montage")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--gazebo-log", type=Path, required=True)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = read(args.scene / "capture_report.json")
    manifest = read(args.scene / "scene_manifest.json")
    if report.get("capture_pass") is not True:
        raise RuntimeError("cold-start Gazebo capture did not pass")
    if report.get("captured_frames") != report.get("requested_frames") or report.get("captured_frames", 0) < 20:
        raise RuntimeError("cold-start Gazebo capture is incomplete")
    if report.get("sensor_odom_sync", {}).get("pass") is not True:
        raise RuntimeError("sensor/odom synchronization gate failed")
    present = {row.get("class_id") for row in manifest.get("objects", [])}
    if not set(TARGET_LABELS) <= present:
        raise RuntimeError("positive smoke does not contain all three target classes")
    boxes = largest_boxes(args.scene)
    if set(boxes) != set(TARGET_LABELS):
        raise RuntimeError("all three target classes were not visible in semantic GT")
    gazebo_text = args.gazebo_log.read_text(encoding="utf-8", errors="replace")
    forbidden = ("Unable to find file", "Error parsing XML", "Unable to load", "SDF error")
    found = [token for token in forbidden if token in gazebo_text]
    if found:
        raise RuntimeError(f"Gazebo resource/render errors remain: {found}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_montage(args.scene, boxes, args.output)
    audit = read(args.audit)
    if audit.get("post_fix_pass") is not True:
        raise RuntimeError("structural asset audit has not passed")
    audit["runtime_render_validation"] = {
        "status": "PASS_COLD_START_POSITIVE_SMOKE",
        "scope": "renderability_and_sensor_chain_only_not_identifiability_gate",
        "capture_pass": True,
        "requested_frames": report["requested_frames"],
        "captured_frames": report["captured_frames"],
        "sensor_odom_maximum_skew_ns": report["sensor_odom_sync"]["maximum_skew_ns"],
        "largest_target_boxes": boxes,
        "evidence": {
            "capture_report": ref(args.scene / "capture_report.json"),
            "scene_manifest": ref(args.scene / "scene_manifest.json"),
            "gazebo_log": ref(args.gazebo_log),
            "montage": ref(args.output),
        },
        "known_non_error_warning": "gz_frame_id is copied as an extension element for semantic/instance cameras",
    }
    write(args.audit, audit)
    print(json.dumps(audit["runtime_render_validation"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
