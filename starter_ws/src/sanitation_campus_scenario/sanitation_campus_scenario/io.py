"""Atomic output helpers for generated scenarios."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Mapping
import xml.etree.ElementTree as ET

from .generator import GenerationError


def write_episode(output_dir: str | Path, files: Mapping[str, str]) -> Path:
    """Write a complete episode atomically, refusing to overwrite any path."""
    target = Path(output_dir).resolve()
    if target.exists():
        raise GenerationError(f"output already exists: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    temp = Path(tempfile.mkdtemp(prefix=f".{target.name}.tmp-", dir=target.parent))
    try:
        for relative, content in sorted(files.items()):
            relative_path = Path(relative)
            if relative_path.is_absolute() or ".." in relative_path.parts:
                raise GenerationError(f"unsafe output path: {relative}")
            destination = temp / relative_path
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(content, encoding="utf-8", newline="\n")
        ET.parse(temp / "public/world.sdf")
        json.loads((temp / "public/episode_manifest.json").read_text(encoding="utf-8"))
        json.loads((temp / "evaluator/episode_manifest.json").read_text(encoding="utf-8"))
        json.loads((temp / "environment/pedestrian_schedule.json").read_text(encoding="utf-8"))
        json.loads((temp / "evaluator/ground_truth.json").read_text(encoding="utf-8"))
        os.replace(temp, target)
    except Exception:
        shutil.rmtree(temp, ignore_errors=True)
        raise
    return target


def write_json_file(output_path: str | Path, value: object) -> Path:
    target = Path(output_path).resolve()
    if target.exists():
        raise GenerationError(f"output already exists: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{target.name}.tmp-", dir=target.parent)
    temp = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(value, stream, ensure_ascii=False, sort_keys=True, indent=2)
            stream.write("\n")
        json.loads(temp.read_text(encoding="utf-8"))
        os.replace(temp, target)
    except Exception:
        temp.unlink(missing_ok=True)
        raise
    return target
