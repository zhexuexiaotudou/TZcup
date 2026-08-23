#!/usr/bin/env python3
"""Select 100 TRAIN images and run pinned D1 PT/ONNX parity in Docker."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
BLOCKED_PATH_TOKENS = ("DEV_VAL", "G5_V2", "SEALED_FINAL")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def remap_path(value: str, old_prefix: Path, new_prefix: Path) -> Path:
    normalized = value.replace("/", "\\")
    old = str(old_prefix).replace("/", "\\").rstrip("\\")
    if not normalized.lower().startswith((old + "\\").lower()):
        raise RuntimeError(f"dataset path is outside declared stale prefix: {value}")
    suffix = normalized[len(old) + 1 :]
    return new_prefix.joinpath(*suffix.split("\\"))


def select_train_images(
    coco_path: Path,
    old_prefix: Path,
    new_prefix: Path,
    *,
    limit: int = 100,
) -> dict[str, Any]:
    payload = json.loads(coco_path.read_text(encoding="utf-8"))
    selected: list[dict[str, Any]] = []
    for image in sorted(payload.get("images", []), key=lambda item: int(item["id"])):
        if image.get("source_split") != "train":
            continue
        source_value = str(image.get("file_name", ""))
        if any(token.lower() in source_value.lower() for token in BLOCKED_PATH_TOKENS):
            raise RuntimeError(f"blocked split token in TRAIN image path: {source_value}")
        resolved = remap_path(source_value, old_prefix, new_prefix).resolve()
        root = new_prefix.resolve()
        if not resolved.is_relative_to(root) or not resolved.is_file():
            raise RuntimeError(f"mapped TRAIN image is missing or escapes root: {resolved}")
        selected.append(
            {
                "image_id": int(image["id"]),
                "relative_path": resolved.relative_to(root).as_posix(),
                "sha256": sha256(resolved),
                "source_split": "train",
                "negative_only": bool(image.get("negative_only", False)),
            }
        )
        if len(selected) == limit:
            break
    if len(selected) != limit:
        raise RuntimeError(f"required {limit} TRAIN images, found {len(selected)}")
    return {
        "schema_version": 1,
        "selection_rule": (
            "COCO images with source_split=train, sorted by integer image id; "
            "reject paths containing DEV_VAL, G5_V2, or SEALED_FINAL; take first 100"
        ),
        "source_coco": str(coco_path.resolve()),
        "source_coco_sha256": sha256(coco_path),
        "old_prefix": str(old_prefix),
        "mapped_root": str(new_prefix.resolve()),
        "image_count": len(selected),
        "images": selected,
    }


def mount(path: Path, target: str, mode: str = "rw") -> str:
    return f"{path.resolve()}:{target}:{mode}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--coco", type=Path, required=True)
    parser.add_argument("--old-prefix", type=Path, required=True)
    parser.add_argument("--development-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--yolov9-source", type=Path, required=True)
    parser.add_argument("--site-packages", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--image", default="tzcup/perception-product:v12-functional")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    evidence = args.evidence_root.resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    selection = select_train_images(
        args.coco.resolve(),
        args.old_prefix,
        args.development_root.resolve(),
    )
    selection_path = evidence / "D1_PARITY_IMAGE_SELECTION.json"
    selection_path.write_text(
        json.dumps(selection, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    command = [
        "docker",
        "run",
        "--rm",
        "--gpus",
        "all",
        "--entrypoint",
        "/bin/bash",
        "-e",
        "PYTHONPATH=/opt/d1site",
        "-v",
        mount(args.checkpoint.resolve().parent, "/models", "ro"),
        "-v",
        mount(args.model.resolve().parent, "/onnx", "ro"),
        "-v",
        mount(args.yolov9_source, "/source", "ro"),
        "-v",
        mount(args.site_packages, "/opt/d1site", "ro"),
        "-v",
        mount(args.development_root, "/devroot", "ro"),
        "-v",
        mount(evidence, "/evidence"),
        "-v",
        mount(ROOT / "scripts", "/tools", "ro"),
        args.image,
        "-lc",
        "python3 /tools/d1_export_worker.py parity "
        "--checkpoint /models/best.pt --source /source "
        f"--model /onnx/{args.model.name} "
        "--selection /evidence/D1_PARITY_IMAGE_SELECTION.json "
        "--development-root /devroot "
        "--output /evidence/D1_PT_ONNX_PARITY.json",
    ]
    result = subprocess.run(
        command,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )
    (evidence / "D1_PT_ONNX_PARITY.stdout.log").write_text(
        result.stdout, encoding="utf-8"
    )
    (evidence / "D1_PT_ONNX_PARITY.stderr.log").write_text(
        result.stderr, encoding="utf-8"
    )
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
    else:
        print(result.stdout)
    parity_path = evidence / "D1_PT_ONNX_PARITY.json"
    export_path = evidence / "D1_EXPORT_REPORT.json"
    if parity_path.is_file() and export_path.is_file():
        parity = json.loads(parity_path.read_text(encoding="utf-8"))
        export_report = json.loads(export_path.read_text(encoding="utf-8"))
        export_report["parity_status"] = (
            "passed" if parity.get("parity_pass") else "failed"
        )
        export_report["parity_report"] = {
            "path": str(parity_path),
            "sha256": sha256(parity_path),
            "image_count": parity.get("image_count"),
        }
        export_path.write_text(
            json.dumps(export_report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
