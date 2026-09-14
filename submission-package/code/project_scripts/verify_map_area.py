#!/usr/bin/env python3
"""Deterministically measure known occupancy area from PGM/YAML map pairs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import yaml


sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[1]
        / "starter_ws"
        / "src"
        / "sanitation_tasks"
    ),
)
from sanitation_tasks.map_quality import inspect_map  # noqa: E402


DEFAULT_MINIMUM_AREA_M2 = 20_000.0


class MapAreaError(ValueError):
    """Raised when map evidence cannot be assessed without guessing."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _path_label(path: Path, root: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(root.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def _map_metadata(yaml_path: Path) -> dict[str, Any]:
    try:
        metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except Exception as error:
        raise MapAreaError(f"cannot parse map YAML: {error}") from error
    if not isinstance(metadata, dict):
        raise MapAreaError("map YAML must contain a mapping")
    image = metadata.get("image")
    if not isinstance(image, str) or not image.strip():
        raise MapAreaError("map YAML must contain a non-empty image path")
    mode = str(metadata.get("mode", "trinary")).strip().lower()
    if mode != "trinary":
        raise MapAreaError(f"unsupported map mode: {mode!r}; expected 'trinary'")
    free_thresh = metadata.get("free_thresh", 0.25)
    occupied_thresh = metadata.get("occupied_thresh", 0.65)
    if (
        isinstance(free_thresh, bool)
        or isinstance(occupied_thresh, bool)
        or not isinstance(free_thresh, (int, float))
        or not isinstance(occupied_thresh, (int, float))
        or float(free_thresh) >= float(occupied_thresh)
    ):
        raise MapAreaError("free_thresh must be less than occupied_thresh")
    return metadata


def assess_map(
    yaml_path: Path,
    *,
    root: Path | None = None,
    minimum_area_m2: float = DEFAULT_MINIMUM_AREA_M2,
) -> dict[str, Any]:
    """Return an exact area report for one map YAML."""

    if not math.isfinite(minimum_area_m2) or minimum_area_m2 <= 0.0:
        raise MapAreaError("minimum area must be finite and positive")
    yaml_path = Path(yaml_path)
    root = Path.cwd() if root is None else Path(root)
    if not yaml_path.is_file():
        raise MapAreaError(f"map YAML does not exist: {yaml_path}")
    metadata = _map_metadata(yaml_path)
    image_path = (yaml_path.parent / metadata["image"]).resolve()
    if not image_path.is_file():
        raise MapAreaError(f"map image does not exist: {image_path}")
    try:
        measured = inspect_map(yaml_path)
    except Exception as error:
        raise MapAreaError(f"cannot inspect map: {error}") from error

    cell_area = measured["resolution_m"] ** 2
    known = measured["known_cells"]
    known_area = known * cell_area
    deficit = max(0.0, minimum_area_m2 - known_area)
    scale = math.sqrt(minimum_area_m2 / known_area) if known_area > 0.0 else None
    return {
        "map_yaml": _path_label(yaml_path, root),
        "map_image": _path_label(image_path, root),
        "map_yaml_sha256": _sha256(yaml_path),
        "map_image_sha256": _sha256(image_path),
        "mode": str(metadata.get("mode", "trinary")),
        "negate": int(metadata.get("negate", 0)),
        "resolution_m": measured["resolution_m"],
        "occupied_thresh": float(metadata.get("occupied_thresh", 0.65)),
        "free_thresh": float(metadata.get("free_thresh", 0.25)),
        "width_cells": measured["width_cells"],
        "height_cells": measured["height_cells"],
        "span_x_m": measured["span_x_m"],
        "span_y_m": measured["span_y_m"],
        "occupied_cells": measured["occupied_cells"],
        "free_cells": measured["free_cells"],
        "unknown_cells": measured["unknown_cells"],
        "known_cells": known,
        "occupied_area_m2": measured["occupied_cells"] * cell_area,
        "free_area_m2": measured["free_cells"] * cell_area,
        "known_area_m2": known_area,
        "unknown_area_m2": measured["unknown_cells"] * cell_area,
        "known_fraction": known / (measured["width_cells"] * measured["height_cells"]),
        "minimum_known_area_m2": minimum_area_m2,
        "area_gate": known_area >= minimum_area_m2,
        "deficit_m2": deficit,
        "required_isotropic_linear_scale": scale,
    }


def _candidate_yaml_paths(root: Path) -> Iterable[Path]:
    suffixes = {".yaml", ".yml"}
    return sorted(
        (
            path
            for path in root.rglob("*")
            if path.is_file() and path.suffix.lower() in suffixes
        ),
        key=lambda path: path.relative_to(root).as_posix(),
    )


def scan_maps(
    root: Path,
    *,
    minimum_area_m2: float = DEFAULT_MINIMUM_AREA_M2,
) -> dict[str, Any]:
    """Scan a tree for PGM-backed map YAMLs and return deterministic results."""

    root = Path(root).resolve()
    if not root.is_dir():
        raise MapAreaError(f"scan root is not a directory: {root}")
    maps: list[dict[str, Any]] = []
    excluded: list[dict[str, str]] = []
    errors: list[dict[str, str]] = []
    for yaml_path in _candidate_yaml_paths(root):
        try:
            raw = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        except Exception as error:
            errors.append(
                {
                    "map_yaml": _path_label(yaml_path, root),
                    "error": f"cannot parse YAML: {error}",
                }
            )
            continue
        if not isinstance(raw, dict) or not isinstance(raw.get("image"), str):
            continue
        if Path(raw["image"]).suffix.lower() != ".pgm":
            continue
        image_path = (yaml_path.parent / raw["image"]).resolve()
        if not image_path.is_file():
            errors.append(
                {
                    "map_yaml": _path_label(yaml_path, root),
                    "error": f"map image does not exist: {_path_label(image_path, root)}",
                }
            )
            continue
        mode = str(raw.get("mode", "trinary")).strip().lower()
        if mode != "trinary":
            excluded.append(
                {
                    "map_yaml": _path_label(yaml_path, root),
                    "mode": mode,
                    "reason": "not a trinary occupancy map",
                }
            )
            continue
        try:
            maps.append(
                assess_map(
                    yaml_path,
                    root=root,
                    minimum_area_m2=minimum_area_m2,
                )
            )
        except MapAreaError as error:
            errors.append(
                {
                    "map_yaml": _path_label(yaml_path, root),
                    "error": str(error),
                }
            )
    maps.sort(key=lambda row: row["map_yaml"])
    excluded.sort(key=lambda row: row["map_yaml"])
    errors.sort(key=lambda row: row["map_yaml"])
    passing = [row for row in maps if row["area_gate"]]
    largest = max(maps, key=lambda row: row["known_area_m2"]) if maps else None
    return {
        "schema_version": 1,
        "scan_root": ".",
        "minimum_known_area_m2": minimum_area_m2,
        "map_count": len(maps),
        "pass_count": len(passing),
        "pass": bool(passing),
        "largest_known_area_map_yaml": largest["map_yaml"] if largest else None,
        "largest_known_area_m2": largest["known_area_m2"] if largest else None,
        "maps": maps,
        "excluded_maps": excluded,
        "errors": errors,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Measure free+occupied occupancy-grid area from PGM/YAML pairs. "
            "Unknown cells are excluded. This command never starts ROS."
        )
    )
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--map-yaml", action="append", default=[], type=Path)
    parser.add_argument(
        "--minimum-area-m2",
        type=float,
        default=DEFAULT_MINIMUM_AREA_M2,
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args(argv)
    root = args.root.resolve()

    if args.map_yaml:
        maps = [
            assess_map(
                path,
                root=root,
                minimum_area_m2=args.minimum_area_m2,
            )
            for path in args.map_yaml
        ]
        maps.sort(key=lambda row: row["map_yaml"])
        passing = [row for row in maps if row["area_gate"]]
        largest = max(maps, key=lambda row: row["known_area_m2"])
        report = {
            "schema_version": 1,
            "scan_root": ".",
            "minimum_known_area_m2": args.minimum_area_m2,
            "map_count": len(maps),
            "pass_count": len(passing),
            "pass": bool(passing),
            "largest_known_area_map_yaml": largest["map_yaml"],
            "largest_known_area_m2": largest["known_area_m2"],
            "maps": maps,
            "excluded_maps": [],
            "errors": [],
        }
    else:
        report = scan_maps(root, minimum_area_m2=args.minimum_area_m2)

    serialized = json.dumps(report, ensure_ascii=True, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    else:
        print(serialized, end="")
    if report["errors"]:
        return 1
    if args.require_pass and not report["pass"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
