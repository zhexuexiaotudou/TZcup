#!/usr/bin/env python3
"""Build a hard-linked formal G4 + D1-D5 development screening view."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil


ROLES = ("D1", "D2", "D3", "D4", "D5")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _hardlink_or_copy(source: str, target: str) -> str:
    try:
        os.link(source, target)
    except OSError:
        shutil.copy2(source, target)
    return target


def _read_jsonl(path: Path) -> list[dict]:
    return [
        json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def build_screening_view(
    base_data: Path,
    base_evidence: Path,
    diagnostics: dict[str, tuple[Path, Path]],
    output_data: Path,
    output_evidence: Path,
) -> dict:
    if set(diagnostics) != set(ROLES):
        raise ValueError("screening view requires exactly D1-D5 diagnostics")
    if output_data.exists() or output_evidence.exists():
        raise FileExistsError("screening output paths must not already exist")
    shutil.copytree(base_data, output_data, copy_function=_hardlink_or_copy)
    output_evidence.mkdir(parents=True)
    for item in base_evidence.iterdir():
        if item.is_file():
            _hardlink_or_copy(str(item), str(output_evidence / item.name))

    frame_rows = _read_jsonl(base_evidence / "g4_frame_manifest.jsonl")
    instance_rows = _read_jsonl(base_evidence / "g4_instance_records.jsonl")
    factor_reports = {}
    for role in ROLES:
        data_root, evidence_root = diagnostics[role]
        qa = json.loads(
            (evidence_root / "factorized_diagnostic_qa.json").read_text(
                encoding="utf-8"
            )
        )
        if qa.get("role") != role or qa.get("factorized_diagnostic_pass") is not True:
            raise RuntimeError(f"{role} diagnostic evidence is not passing")
        role_frames = _read_jsonl(
            evidence_root / "raw_g4_qa" / "g4_frame_manifest.jsonl"
        )
        role_instances = _read_jsonl(
            evidence_root / "raw_g4_qa" / "g4_instance_records.jsonl"
        )
        if len(role_frames) != 100 or any(row.get("split") != role for row in role_frames):
            raise RuntimeError(f"{role} frame evidence is incomplete")
        for scene_dir in sorted((data_root / "scenes").glob("scene_*")):
            target = output_data / "scenes" / scene_dir.name
            if target.exists():
                raise RuntimeError(f"diagnostic scene collides with formal data: {scene_dir.name}")
            shutil.copytree(scene_dir, target, copy_function=_hardlink_or_copy)
        frame_rows.extend(role_frames)
        instance_rows.extend(role_instances)
        factor_reports[role] = {
            "scene_count": qa["scene_count"],
            "frame_count": qa["frame_count"],
            "qa_sha256": _sha256(evidence_root / "factorized_diagnostic_qa.json"),
        }

    frame_keys = [
        (int(row["scene_seed"]), int(row["frame_index"])) for row in frame_rows
    ]
    if len(frame_keys) != len(set(frame_keys)):
        raise RuntimeError("formal and diagnostic frame keys overlap")
    frames_path = output_evidence / "g4_frame_manifest.jsonl"
    instances_path = output_evidence / "g4_instance_records.jsonl"
    _write_jsonl(frames_path, frame_rows)
    _write_jsonl(instances_path, instance_rows)
    counts = {
        split: sum(row.get("split") == split for row in frame_rows)
        for split in ("train", "val", "test", *ROLES)
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-05R",
        "formal_data_root": str(base_data),
        "formal_dataset_qa_sha256": _sha256(base_evidence / "g4_dataset_qa.json"),
        "output_data_root": str(output_data),
        "output_evidence_root": str(output_evidence),
        "frame_counts_by_role": counts,
        "total_frames": len(frame_rows),
        "total_instance_records": len(instance_rows),
        "factorized_diagnostics": factor_reports,
        "legacy_test_preserved_as_non_gating": True,
        "G5_SEALED_FINAL_included": False,
        "frame_manifest_sha256": _sha256(frames_path),
        "instance_records_sha256": _sha256(instances_path),
    }
    (output_evidence / "screening_dataset_build_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-data", required=True, type=Path)
    parser.add_argument("--base-evidence", required=True, type=Path)
    parser.add_argument("--output-data", required=True, type=Path)
    parser.add_argument("--output-evidence", required=True, type=Path)
    for role in ROLES:
        parser.add_argument(f"--{role.lower()}-data", required=True, type=Path)
        parser.add_argument(f"--{role.lower()}-evidence", required=True, type=Path)
    args = parser.parse_args()
    diagnostics = {
        role: (
            getattr(args, f"{role.lower()}_data"),
            getattr(args, f"{role.lower()}_evidence"),
        )
        for role in ROLES
    }
    report = build_screening_view(
        args.base_data,
        args.base_evidence,
        diagnostics,
        args.output_data,
        args.output_evidence,
    )
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
