#!/usr/bin/env python3
"""Fail-closed merge of isolated AUTO-05R Gazebo capture shards."""

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


def _tree_digest(root: Path) -> dict:
    digest = hashlib.sha256()
    count = 0
    size = 0
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        relative = path.relative_to(root).as_posix()
        file_size = path.stat().st_size
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(_sha256(path).encode("ascii"))
        digest.update(b"\0")
        digest.update(str(file_size).encode("ascii"))
        digest.update(b"\n")
        count += 1
        size += file_size
    return {"sha256": digest.hexdigest(), "file_count": count, "bytes": size}


def merge_shards(
    sources: list[Path], output: Path, manifest_relative: Path
) -> dict:
    if len(sources) < 2:
        raise ValueError("at least two isolated capture shards are required")
    if output.exists():
        raise FileExistsError(f"merge output must not already exist: {output}")
    source_records = []
    expected_static_digest = None
    expected_manifest_sha = None
    scene_names: set[str] = set()
    world_ids: set[str] = set()
    for source in sources:
        if not source.is_dir():
            raise FileNotFoundError(f"capture shard missing: {source}")
        manifest_path = source / manifest_relative
        manifest_sha = _sha256(manifest_path)
        static_digest = {
            name: _tree_digest(source / name) for name in ("models", "worlds")
        }
        if expected_manifest_sha is None:
            expected_manifest_sha = manifest_sha
            expected_static_digest = static_digest
        elif manifest_sha != expected_manifest_sha or static_digest != expected_static_digest:
            raise RuntimeError("shard model/world payloads are not byte-identical")
        local_scenes = sorted((source / "scenes").glob("scene_*"))
        local_worlds = set()
        for scene in local_scenes:
            if scene.name in scene_names:
                raise RuntimeError(f"duplicate scene across shards: {scene.name}")
            scene_manifest = json.loads(
                (scene / "scene_manifest.json").read_text(encoding="utf-8")
            )
            capture = json.loads(
                (scene / "capture_report.json").read_text(encoding="utf-8")
            )
            if capture.get("capture_pass") is not True:
                raise RuntimeError(f"capture gate failed in {scene}")
            if len(capture.get("records", [])) != 10:
                raise RuntimeError(f"capture frame count is not 10 in {scene}")
            scene_names.add(scene.name)
            local_worlds.add(scene_manifest["world_id"])
        if world_ids & local_worlds:
            raise RuntimeError(
                "worlds must be isolated by shard; overlap="
                + ",".join(sorted(world_ids & local_worlds))
            )
        world_ids.update(local_worlds)
        source_records.append(
            {
                "path": str(source),
                "scene_count": len(local_scenes),
                "world_ids": sorted(local_worlds),
                "manifest_sha256": manifest_sha,
                "models_worlds_digest": static_digest,
            }
        )

    output.mkdir(parents=True)
    shutil.copytree(sources[0] / "models", output / "models")
    shutil.copytree(sources[0] / "worlds", output / "worlds")
    (output / "scenes").mkdir()
    for source in sources:
        for scene in sorted((source / "scenes").glob("scene_*")):
            shutil.copytree(scene, output / "scenes" / scene.name)
    report = {
        "schema_version": 1,
        "source_count": len(sources),
        "sources": source_records,
        "world_count": len(world_ids),
        "world_ids": sorted(world_ids),
        "scene_count": len(scene_names),
        "output_dataset_digest": _tree_digest(output),
    }
    (output.parent / f"{output.name}_merge_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", action="append", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--manifest-relative", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(merge_shards(args.source, args.output, args.manifest_relative), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
