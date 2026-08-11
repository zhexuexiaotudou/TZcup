#!/usr/bin/env python3
"""Contract tests for the development-only G6 corpus builder."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import cv2


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g6_dataset import (  # noqa: E402
    G6Plan,
    METAL_DOMAINS,
    NEGATIVE_AREA_TAXONOMIES,
    build_g6_dataset,
    load_jsonl,
)
from audit_oprv3_g6 import audit  # noqa: E402


def smoke_plan() -> G6Plan:
    return G6Plan(
        scenes_by_split={
            "train": 4,
            "val": 2,
            **{f"development_d{index}": 1 for index in range(1, 6)},
        },
        frames_per_scene=10,
        small_bucket_minimums={"lt8": 2, "8_12": 2, "12_18": 2, "18_32": 2},
        small_class_minimums={"plastic_bottle": 2, "metal_can": 2, "paper_litter": 2},
        metal_domain_minimum=1,
        train_negative_area_frames_minimum=20,
        formal=False,
    )


def test_g6_builds_all_required_reports_and_never_reads_sealed(tmp_path: Path) -> None:
    qa = build_g6_dataset(tmp_path / "g6", smoke_plan())
    assert qa["G6_DATASET_PASS"] is True
    assert qa["sealed_final_read"] is False
    assert all(qa["gates"].values())
    reports = tmp_path / "g6/reports"
    assert {path.name for path in reports.iterdir()} == {
        "G6_DATASET_QA.json",
        "G6_SPLIT_MANIFEST.json",
        "G6_SMALL_OBJECT_DISTRIBUTION.json",
        "G6_METAL_CAN_DOMAIN_MATRIX.json",
        "G6_NEGATIVE_AREA_TAXONOMY.json",
    }


def test_g6_split_assets_worlds_and_phashes_are_isolated(tmp_path: Path) -> None:
    root = tmp_path / "g6"
    build_g6_dataset(root, smoke_plan())
    split = json.loads((root / "reports/G6_SPLIT_MANIFEST.json").read_text())
    assert split["cross_world_overlap"] == 0
    assert split["cross_asset_overlap"] == 0
    rows = load_jsonl(root / "G6_FRAME_MANIFEST.jsonl")
    by_split = {}
    for row in rows:
        by_split.setdefault(row["split"], set()).add(row["perceptual_hash"])
    names = list(by_split)
    assert all(
        not by_split[left] & by_split[right]
        for index, left in enumerate(names)
        for right in names[index + 1 :]
    )


def test_g6_meets_small_metal_and_negative_taxonomy_quotas(tmp_path: Path) -> None:
    root = tmp_path / "g6"
    plan = smoke_plan()
    build_g6_dataset(root, plan)
    small = json.loads((root / "reports/G6_SMALL_OBJECT_DISTRIBUTION.json").read_text())
    assert all(
        small["train_bucket_counts"][name] >= minimum
        for name, minimum in plan.small_bucket_minimums.items()
    )
    assert all(
        small["train_small_class_counts"][name] >= minimum
        for name, minimum in plan.small_class_minimums.items()
    )
    metal = json.loads((root / "reports/G6_METAL_CAN_DOMAIN_MATRIX.json").read_text())
    assert set(metal["train_domain_counts"]) == set(METAL_DOMAINS)
    assert min(metal["train_domain_counts"].values()) >= 1
    negative = json.loads((root / "reports/G6_NEGATIVE_AREA_TAXONOMY.json").read_text())
    assert negative["train_hard_frame_count"] >= 20
    assert all(
        negative["taxonomy_counts_by_split"][name]["val"] > 0
        for name in NEGATIVE_AREA_TAXONOMIES
    )


def test_g6_frame_labels_match_instance_records(tmp_path: Path) -> None:
    root = tmp_path / "g6"
    build_g6_dataset(root, smoke_plan())
    frames = load_jsonl(root / "G6_FRAME_MANIFEST.jsonl")
    instances = load_jsonl(root / "G6_INSTANCE_RECORDS.jsonl")
    assert len(frames) == sum(smoke_plan().scenes_by_split.values()) * 10
    assert all(item["mask_area_px"] > 0 for item in instances)
    assert all(item["bbox_short_side_px"] >= 1 for item in instances)
    assert all(item["visible"] and not item["truncated"] for item in instances)
    assert all(item["split"] != "sealed_final" for item in instances)


def test_independent_g6_audit_detects_pixel_tampering(tmp_path: Path) -> None:
    root = tmp_path / "g6"
    build_g6_dataset(root, smoke_plan())
    # The formal count gate intentionally remains false for a smoke corpus,
    # while every actual file/pixel contract passes before tampering.
    clean = audit(root)
    assert clean["mismatch_count"] == 0
    row = load_jsonl(root / "G6_FRAME_MANIFEST.jsonl")[0]
    semantic_path = root / row["semantic_path"]
    semantic = cv2.imread(str(semantic_path), cv2.IMREAD_UNCHANGED)
    semantic[200, 200] = 5
    assert cv2.imwrite(str(semantic_path), semantic)
    tampered = audit(root)
    assert tampered["G6_INDEPENDENT_AUDIT_PASS"] is False
    assert tampered["mismatch_count"] > 0
