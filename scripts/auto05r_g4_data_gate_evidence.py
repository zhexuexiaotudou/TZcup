#!/usr/bin/env python3
"""Generate deterministic compact G4 data-gate evidence for Git.

Only schemas, hashes, counts and registries are written; raw simulator
frames, bags and model binaries are never committed.  The generated evidence
records the existing ``G4_dataset_gate_pass`` decision and points at the
external raw evidence by file name + SHA-256 only.

Run:
    py -3 scripts/auto05r_g4_data_gate_evidence.py \
        --qa-dir <g4 qa_formal dir> \
        --world-manifest <g4_world_manifest.json> \
        --output-dir artifacts/auto05r_g4_data_gate
"""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_json(value, indent: int = 2) -> str:
    return json.dumps(value, indent=indent, sort_keys=True, ensure_ascii=False) + "\n"


def _write(output_dir: Path, name: str, payload) -> Path:
    path = output_dir / name
    path.write_text(_canonical_json(payload), encoding="utf-8")
    return path


def _load_jsonl(path: Path) -> list[dict]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _role_name(split: str) -> str:
    return "legacy_G4_D6_diagnostic" if split == "test" else split


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--qa-dir", required=True, type=Path)
    parser.add_argument("--world-manifest", required=True, type=Path)
    parser.add_argument(
        "--asset-registry",
        default=str(
            ROOT
            / "starter_ws"
            / "src"
            / "sanitation_learning"
            / "config"
            / "g4_asset_registry.yaml"
        ),
        type=Path,
    )
    parser.add_argument(
        "--output-dir",
        default=str(ROOT / "artifacts" / "auto05r_g4_data_gate"),
        type=Path,
    )
    args = parser.parse_args()

    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    qa = json.loads((args.qa_dir / "g4_dataset_qa.json").read_text(encoding="utf-8"))
    split_manifest = json.loads(
        (args.qa_dir / "split_manifest.json").read_text(encoding="utf-8")
    )
    leakage = json.loads(
        (args.qa_dir / "leakage_report.json").read_text(encoding="utf-8")
    )
    frames = _load_jsonl(args.qa_dir / "g4_frame_manifest.jsonl")
    instances = _load_jsonl(args.qa_dir / "g4_instance_records.jsonl")
    world_manifest = json.loads(args.world_manifest.read_text(encoding="utf-8"))
    asset_registry = yaml.safe_load(args.asset_registry.read_text(encoding="utf-8"))

    frame_counts = Counter()
    world_frame_counts = Counter()
    negative_frames = Counter()
    tf_valid_frames = Counter()
    for row in frames:
        role = _role_name(row["split"])
        frame_counts[role] += 1
        world_frame_counts[row["world_id"]] += 1
        if row.get("negative_only"):
            negative_frames[role] += 1
        if row.get("tf_valid"):
            tf_valid_frames[role] += 1

    instance_counts = Counter()
    class_counts_by_role: dict[str, Counter] = Counter()
    for record in instances:
        role = _role_name(record["split"])
        instance_counts[role] += 1
        class_counts_by_role[(role, record.get("semantic_class", "background"))] += 1
    per_role_class_counts = {
        role: {
            class_name: int(
                class_counts_by_role[(role, class_name)]
            )
            for class_name in sorted(
                {
                    name
                    for (role_key, name) in class_counts_by_role
                    if role_key == role
                }
            )
        }
        for role in sorted(set(instance_counts))
    }

    registry_summary = {
        "target_variants_total": asset_registry.get("target_variants_total"),
        "per_class_variant_counts": asset_registry.get(
            "per_class_variant_counts"
        ),
        "per_class_split_counts": asset_registry.get(
            "per_class_split_counts"
        ),
        "hard_negative_families_total": asset_registry.get(
            "hard_negative_families_total"
        ),
        "hard_negative_family_split_counts": asset_registry.get(
            "hard_negative_family_split_counts"
        ),
        "required_paper_taxonomies": sorted(
            asset_registry.get("required_paper_taxonomies", [])
        ),
        "instance_record_counts_by_role": dict(instance_counts),
        "per_role_class_counts": per_role_class_counts,
    }

    worlds = world_manifest.get("worlds", [])
    world_summary = {
        "world_count": len(worlds),
        "native_capture_resolution": world_manifest.get(
            "native_capture_resolution"
        ),
        "worlds": [
            {
                "world_id": item["world_id"],
                "split_eligibility": item.get("split_eligibility"),
            }
            for item in sorted(worlds, key=lambda item: item["world_id"])
        ],
        "frame_counts_by_world": {
            world_id: int(world_frame_counts[world_id])
            for world_id in sorted(world_frame_counts)
        },
        "source_sha256": _sha256(args.world_manifest),
    }

    external_index = {
        "location_contract": (
            "raw G4 frames, bags and checkpoints stay outside Git; only "
            "compact summaries and manifest hashes are committed"
        ),
        "files": [
            {
                "relative_name": path.name,
                "sha256": _sha256(path),
                "bytes": path.stat().st_size,
            }
            for path in sorted(
                (
                    args.qa_dir / "g4_dataset_qa.json",
                    args.qa_dir / "g4_frame_manifest.jsonl",
                    args.qa_dir / "g4_instance_records.jsonl",
                    args.qa_dir / "leakage_report.json",
                    args.qa_dir / "split_manifest.json",
                    args.world_manifest,
                )
            )
        ],
    }

    split_summary = {
        "schema_version": 1,
        "roles": {
            role: {
                "frames": int(frame_counts[role]),
                "negative_only_frames": int(negative_frames[role]),
                "tf_valid_frames": int(tf_valid_frames[role]),
            }
            for role in sorted(set(frame_counts))
        },
        "role_rename": {"test": "legacy_G4_D6_diagnostic"},
        "historical_data_gate_test_used_for_model_selection": (
            split_manifest.get("test_used_for_model_selection", False)
        ),
        "post_capture_contamination": {
            "legacy_G4_D6_contaminated": True,
            "historical_screening_attempts_that_read_legacy": 2,
            "used_in_historical_screening_pass_fail": True,
            "allowed_in_current_development_screening": False,
        },
        "historical_split_manifest": split_manifest,
    }

    qa_summary = {
        "schema_version": 1,
        "stage": qa.get("stage"),
        "task": qa.get("task"),
        "G4_dataset_gate_pass": qa.get("G4_dataset_gate_pass"),
        "quality_gates_pass": qa.get("quality_gates_pass"),
        "formal_scale": qa.get("formal_scale"),
        "scene_count": qa.get("scene_count"),
        "frame_count": qa.get("frame_count"),
        "world_count": qa.get("world_count"),
        "gates": qa.get("gates"),
        "errors": qa.get("errors", []),
        "historical_data_gate_test_used_for_model_selection": qa.get(
            "test_used_for_model_selection"
        ),
        "post_capture_contamination": {
            "legacy_G4_D6_contaminated": True,
            "historical_screening_attempts_that_read_legacy": 2,
            "used_in_historical_screening_pass_fail": True,
            "allowed_in_current_development_screening": False,
        },
        "source_sha256": _sha256(args.qa_dir / "g4_dataset_qa.json"),
    }

    written = {
        "g4_dataset_qa.json": _write(output, "g4_dataset_qa.json", qa_summary),
        "split_manifest.json": _write(
            output, "split_manifest.json", split_summary
        ),
        "leakage_report.json": _write(output, "leakage_report.json", leakage),
        "asset_registry_summary.json": _write(
            output, "asset_registry_summary.json", registry_summary
        ),
        "world_manifest_summary.json": _write(
            output, "world_manifest_summary.json", world_summary
        ),
        "raw_external_evidence_index.json": _write(
            output, "raw_external_evidence_index.json", external_index
        ),
    }
    artifact_manifest = {
        "schema_version": 1,
        "evidence_id": "auto05r_g4_data_gate",
        "G4_dataset_gate_pass": bool(qa.get("G4_dataset_gate_pass")),
        "scope": (
            "capture_and_annotation_quality_only; does not restore the "
            "post-capture contaminated legacy test as a final set"
        ),
        "generation": {
            "deterministic": True,
            "generator": "scripts/auto05r_g4_data_gate_evidence.py",
            "source_sha256_index": external_index,
        },
        "files": {
            name: {
                "sha256": _sha256(path),
                "bytes": path.stat().st_size,
            }
            for name, path in sorted(written.items())
        },
    }
    _write(output, "artifact_manifest.json", artifact_manifest)
    print(
        json.dumps(
            {
                "G4_dataset_gate_pass": artifact_manifest[
                    "G4_dataset_gate_pass"
                ],
                "output_dir": str(output),
                "files": sorted(artifact_manifest["files"]),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
