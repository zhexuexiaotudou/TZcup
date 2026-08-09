#!/usr/bin/env python3
"""Create a new AUTO-05R dataset by overlaying validated scene recaptures."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def overlay_scenes(base: Path, overlays: list[Path], output: Path) -> dict:
    if not base.is_dir():
        raise FileNotFoundError(f"base dataset is missing: {base}")
    if not overlays:
        raise ValueError("at least one scene overlay is required")
    if output.exists():
        raise FileExistsError(f"overlay output must not already exist: {output}")
    manifest_relative = Path("worlds/g4_world_manifest.json")
    base_manifest_sha = _sha256(base / manifest_relative)
    base_scenes = {item.name: item for item in (base / "scenes").glob("scene_*")}
    replacements: dict[str, dict] = {}
    for overlay in overlays:
        if _sha256(overlay / manifest_relative) != base_manifest_sha:
            raise RuntimeError(f"overlay world manifest differs from base: {overlay}")
        for scene_dir in sorted((overlay / "scenes").glob("scene_*")):
            if scene_dir.name in replacements:
                raise RuntimeError(f"duplicate overlay scene: {scene_dir.name}")
            if scene_dir.name not in base_scenes:
                raise RuntimeError(f"overlay scene is absent from base: {scene_dir.name}")
            old_manifest = json.loads(
                (base_scenes[scene_dir.name] / "scene_manifest.json").read_text(
                    encoding="utf-8"
                )
            )
            new_manifest = json.loads(
                (scene_dir / "scene_manifest.json").read_text(encoding="utf-8")
            )
            for key in ("scene_seed", "world_id", "split"):
                if new_manifest.get(key) != old_manifest.get(key):
                    raise RuntimeError(
                        f"overlay identity changed for {scene_dir.name}: {key}"
                    )
            capture = json.loads(
                (scene_dir / "capture_report.json").read_text(encoding="utf-8")
            )
            if capture.get("capture_pass") is not True:
                raise RuntimeError(f"overlay capture failed: {scene_dir}")
            if len(capture.get("records", [])) != 10:
                raise RuntimeError(f"overlay capture is not 10 frames: {scene_dir}")
            replacements[scene_dir.name] = {
                "source": scene_dir,
                "world_id": new_manifest["world_id"],
                "scene_seed": new_manifest["scene_seed"],
            }
    shutil.copytree(base, output)
    for scene_name, replacement in sorted(replacements.items()):
        shutil.copytree(
            replacement["source"],
            output / "scenes" / scene_name,
            dirs_exist_ok=True,
        )
    report = {
        "schema_version": 1,
        "base": str(base),
        "base_world_manifest_sha256": base_manifest_sha,
        "output": str(output),
        "replacement_count": len(replacements),
        "replacements": [
            {
                "scene": name,
                "world_id": item["world_id"],
                "scene_seed": item["scene_seed"],
                "source": str(item["source"]),
                "capture_report_sha256": _sha256(
                    item["source"] / "capture_report.json"
                ),
            }
            for name, item in sorted(replacements.items())
        ],
    }
    report_path = output.parent / f"{output.name}_overlay_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument("--overlay", required=True, action="append", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(overlay_scenes(args.base, args.overlay, args.output), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
