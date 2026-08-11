#!/usr/bin/env python3
"""Prepare deterministic full-frame COCO splits for bounded OPR-C."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.opr_c_rtmdet import (  # noqa: E402
    bounded_frames,
    index_instances,
    load_jsonl,
    to_coco,
)


SEED = 20260813


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g6-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-fit-frames", type=int, default=2400)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    audit = json.loads((args.g6_root / "G6_INDEPENDENT_AUDIT.json").read_text())
    if not audit["G6_INDEPENDENT_AUDIT_PASS"]:
        raise RuntimeError("OPR-C requires passed G6 independent audit")
    frames = load_jsonl(args.g6_root / "G6_FRAME_MANIFEST.jsonl")
    instances = load_jsonl(args.g6_root / "G6_INSTANCE_RECORDS.jsonl")
    instances_by_frame = index_instances(instances)
    train = [row for row in frames if row["split"] == "train"]
    holdout_world = sorted({row["world_id"] for row in train})[-1]
    fit_rows = bounded_frames(
        [row for row in train if row["world_id"] != holdout_world],
        instances_by_frame,
        args.max_fit_frames,
        SEED,
    )
    splits = {
        "fit": fit_rows,
        "holdout": [row for row in train if row["world_id"] == holdout_world],
        "val": [row for row in frames if row["split"] == "val"],
    }
    report = {
        "schema_version": 1,
        "stage": "OPRV3-05-OPR-C-COCO-PREP",
        "source_dataset": "G6_DEVELOPMENT_OPRV3_V1",
        "holdout_world": holdout_world,
        "G5_SEALED_FINAL_read": False,
        "splits": {},
    }
    for name, rows in splits.items():
        payload = to_coco(rows, instances_by_frame)
        path = args.output / f"{name}.json"
        path.write_text(json.dumps(payload, separators=(",", ":")) + "\n")
        report["splits"][name] = {
            "frames": len(payload["images"]),
            "instances": len(payload["annotations"]),
            "negative_frames": sum(not any(a["image_id"] == image["id"] for a in payload["annotations"]) for image in payload["images"]),
            "worlds": sorted({image["world_id"] for image in payload["images"]}),
            "sha256": sha256(path),
        }
    (args.output / "OPR_C_COCO_PREP_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
