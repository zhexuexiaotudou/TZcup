#!/usr/bin/env python3
"""Select a hash-bound ODCV5 golden-frame parity manifest."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


DISCRETE_LABELS = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _domain_tags(root: Path, manifest: dict, record: dict, semantic: np.ndarray) -> list[str]:
    text = root.as_posix().lower()
    tags = {str(manifest.get("world_id", "unknown_world"))}
    phase = str(record.get("motion_phase", "straight_approach"))
    tags.add(phase)
    if "turn" in text or "turn" in phase:
        tags.add("turn")
        tags.add("behind_vehicle_fov_entry")
    if "occlusion" in text or any(
        item.get("occlusion_bucket") not in (None, "none")
        for item in manifest.get("objects", [])
    ):
        tags.add("occlusion")
    if "reflection" in text:
        tags.add("reflection")
    ground = str(manifest.get("ground_material_executed_by_world", ""))
    if "wet" in ground or "wet" in str(manifest.get("world_id", "")):
        tags.add("wet_road")
    present = {DISCRETE_LABELS[label] for label in DISCRETE_LABELS if np.any(semantic == label)}
    tags.update(present)
    for item in manifest.get("objects", []):
        if item.get("class_id") in present and item.get("size_bucket") == "small":
            tags.add("small")
    if not present:
        tags.add("negative_only")
    taxonomies = {
        str(item.get("taxonomy"))
        for item in manifest.get("objects", []) if item.get("taxonomy")
    }
    if "shadow_edge" in taxonomies:
        tags.add("shadow")
    if "road_marking_fragment" in taxonomies or "paver_joint" in taxonomies:
        tags.add("road_paint_or_marking")
    if taxonomies:
        tags.add("clutter")
    lighting = str(manifest.get("lighting_executed_by_world", ""))
    if "evening" in lighting or "overcast" in lighting:
        tags.add("dark_background")
    if "high_noon" in lighting or "concrete_light" in ground:
        tags.add("bright_pavement")
    return sorted(tags)


def collect_candidates(roots: list[Path]) -> list[dict]:
    candidates = []
    seen_roots = set()
    for root in roots:
        root = root.resolve()
        if root in seen_roots:
            raise ValueError(f"duplicate source root: {root}")
        seen_roots.add(root)
        for scene in sorted((root / "scenes").glob("scene_*")):
            manifest_path = scene / "scene_manifest.json"
            report_path = scene / "capture_report.json"
            if not manifest_path.is_file() or not report_path.is_file():
                continue
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            report = json.loads(report_path.read_text(encoding="utf-8"))
            if report.get("capture_pass") is not True:
                continue
            if report.get("captured_frames") != report.get("requested_frames"):
                raise ValueError(f"partial capture is forbidden: {scene}")
            for record in report.get("records", []):
                paths = {
                    name: scene / record["paths"][name]
                    for name in ("rgb", "depth", "semantic", "instance", "camera", "tf")
                }
                if not all(path.is_file() for path in paths.values()):
                    raise ValueError(f"missing persisted sensor file in {scene}")
                rgb_hash = sha256(paths["rgb"])
                if record.get("rgb_sha256") and record["rgb_sha256"] != rgb_hash:
                    raise ValueError(f"RGB hash drift: {paths['rgb']}")
                semantic = np.load(paths["semantic"], allow_pickle=False)
                present = sorted(
                    DISCRETE_LABELS[label]
                    for label in DISCRETE_LABELS if np.any(semantic == label)
                )
                frame_id = hashlib.sha256(
                    f"{root.as_posix()}|{scene.name}|{record['frame_index']}|{rgb_hash}".encode()
                ).hexdigest()[:24]
                candidates.append({
                    "frame_id": frame_id,
                    "source_root": root.as_posix(),
                    "scene_seed": int(manifest["scene_seed"]),
                    "frame_index": int(record["frame_index"]),
                    "world_id": manifest["world_id"],
                    "positive": bool(present),
                    "discrete_classes": present,
                    "domains": _domain_tags(root, manifest, record, semantic),
                    "paths": {name: path.as_posix() for name, path in paths.items()},
                    "sha256": {name: sha256(path) for name, path in paths.items()},
                })
    return candidates


def select_manifest(candidates: list[dict], *, positive_count: int, negative_count: int) -> dict:
    positives = [item for item in candidates if item["positive"]]
    negatives = [item for item in candidates if not item["positive"]]
    priority_domains = (
        "turn", "behind_vehicle_fov_entry", "occlusion", "reflection",
        "wet_road", "small", "shadow", "road_paint_or_marking",
        "clutter", "dark_background", "bright_pavement",
    )
    selected: list[dict] = []
    selected_ids = set()
    for domain in priority_domains:
        for item in positives:
            if domain in item["domains"] and item["frame_id"] not in selected_ids:
                selected.append(item)
                selected_ids.add(item["frame_id"])
                if sum(domain in row["domains"] for row in selected) >= 5:
                    break
    for class_name in DISCRETE_LABELS.values():
        for item in positives:
            if class_name in item["discrete_classes"] and item["frame_id"] not in selected_ids:
                selected.append(item)
                selected_ids.add(item["frame_id"])
                if sum(class_name in row["discrete_classes"] for row in selected) >= 10:
                    break
    for item in positives:
        if len(selected) >= positive_count:
            break
        if item["frame_id"] not in selected_ids:
            selected.append(item)
            selected_ids.add(item["frame_id"])
    selected = selected[:positive_count]
    selected.extend(negatives[:negative_count])
    if sum(item["positive"] for item in selected) < positive_count:
        raise ValueError("insufficient positive golden frames")
    if sum(not item["positive"] for item in selected) < negative_count:
        raise ValueError("insufficient negative golden frames")
    rgb_hashes = [item["sha256"]["rgb"] for item in selected]
    if len(rgb_hashes) != len(set(rgb_hashes)):
        raise ValueError("exact duplicate RGB frame selected")
    coverage = {
        tag: sum(tag in item["domains"] for item in selected)
        for tag in (*DISCRETE_LABELS.values(), *priority_domains, "negative_only")
    }
    required_coverage_complete = all(coverage.get(tag, 0) > 0 for tag in (
        *DISCRETE_LABELS.values(), *priority_domains, "negative_only"
    ))
    return {
        "schema_version": 1,
        "protocol": "ONLINE-DOMAIN-CLOSURE-V5",
        "stage": "ODCV5-01-GOLDEN-SELECTION",
        "selection_independent_of_model_output": True,
        "positive_frames": sum(item["positive"] for item in selected),
        "negative_frames": sum(not item["positive"] for item in selected),
        "exact_rgb_duplicates": 0,
        "coverage": coverage,
        "required_coverage_complete": required_coverage_complete,
        "frames": selected,
        "G5_SEALED_FINAL_read": False,
        "G5_V2_read": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, action="append", required=True)
    parser.add_argument("--positive-count", type=int, default=100)
    parser.add_argument("--negative-count", type=int, default=50)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = select_manifest(
        collect_candidates(args.data_root),
        positive_count=args.positive_count,
        negative_count=args.negative_count,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "path": args.output.as_posix(),
        "sha256": sha256(args.output),
        "positive_frames": report["positive_frames"],
        "negative_frames": report["negative_frames"],
        "coverage": report["coverage"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
