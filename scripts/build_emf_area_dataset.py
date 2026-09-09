#!/usr/bin/env python3
"""Build the non-sealed EMFJ6V3 area-model screening manifest.

The builder consumes only explicitly labelled TRAIN/HOLDOUT capture roots.  It
does not infer a split from a directory name and it never opens a sealed data
source.  Every RGB/depth/semantic triplet and the capture metadata that binds
it are SHA-256 locked in the output manifest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from collections import Counter
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any

import cv2
import numpy as np

SCHEMA_VERSION = "emfj6v3.area_dataset_manifest.v1"
ALLOWED_SPLITS = ("TRAIN", "HOLDOUT")
ALLOWED_SEMANTIC_IDS = tuple(range(6))
TARGET_SEMANTIC_IDS = (4, 5)
SEMANTIC_CLASSES = {
    0: "background",
    1: "plastic_bottle",
    2: "metal_can",
    3: "paper_litter",
    4: "leaf_pile",
    5: "puddle",
}
FORBIDDEN_MARKERS = ("G5", "G5_V2", "VAL_NEW", "DEV_VAL", "SEALED")


class DatasetContractError(ValueError):
    """Raised when an input violates the fail-closed dataset contract."""


def _marker_pattern(marker: str) -> re.Pattern[str]:
    return re.compile(
        rf"(?:^|[^A-Z0-9]){re.escape(marker.upper())}(?:$|[^A-Z0-9])",
        re.IGNORECASE,
    )


FORBIDDEN_PATTERNS = tuple(
    (marker, _marker_pattern(marker)) for marker in FORBIDDEN_MARKERS
)


def _reject_forbidden(value: object, *, field: str) -> None:
    rendered = str(value)
    normalized_words = re.sub(r"[^A-Z0-9]+", "_", rendered.upper()).split("_")
    if any(word.startswith("G5V2") or word == "G5" for word in normalized_words):
        raise DatasetContractError(f"forbidden marker 'G5' in {field}: {rendered}")
    for marker, pattern in FORBIDDEN_PATTERNS:
        if pattern.search(rendered):
            raise DatasetContractError(
                f"forbidden marker {marker!r} in {field}: {rendered}"
            )


def _canonical_split(value: object, *, field: str) -> str:
    _reject_forbidden(value, field=field)
    canonical = str(value).strip().upper()
    if canonical not in ALLOWED_SPLITS:
        raise DatasetContractError(
            f"{field} must be one of {ALLOWED_SPLITS}, got {value!r}"
        )
    return canonical


def _source_split(value: object, *, explicit_split: str, field: str) -> str:
    """Validate legacy capture split metadata against an explicit root role."""

    _reject_forbidden(value, field=field)
    source = str(value).strip().lower()
    allowed = {"TRAIN": {"train"}, "HOLDOUT": {"holdout", "val"}}[explicit_split]
    if source not in allowed:
        raise DatasetContractError(
            f"{field}={value!r} is incompatible with explicit root split {explicit_split}"
        )
    return source


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_sha256(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise DatasetContractError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise DatasetContractError(f"JSON root must be an object: {path}")
    return payload


def _require_child(
    path_value: object, *, scene_dir: Path, root: Path, field: str
) -> Path:
    if not isinstance(path_value, str) or not path_value.strip():
        raise DatasetContractError(f"{field} must be a non-empty path string")
    _reject_forbidden(path_value, field=field)
    candidate = Path(path_value)
    if not candidate.is_absolute():
        candidate = scene_dir / candidate
    candidate = candidate.resolve()
    _reject_forbidden(candidate, field=field)
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise DatasetContractError(
            f"{field} escapes explicit root {root}: {candidate}"
        ) from exc
    if not candidate.is_file():
        raise DatasetContractError(f"missing paired file for {field}: {candidate}")
    return candidate


def _semantic_counts(path: Path) -> tuple[Counter[int], tuple[int, int], str]:
    if path.suffix.lower() != ".npy":
        raise DatasetContractError(
            f"semantic tensor must be a non-pickled .npy file: {path}"
        )
    try:
        semantic = np.load(path, allow_pickle=False)
    except (OSError, ValueError) as exc:
        raise DatasetContractError(
            f"cannot safely load semantic tensor {path}: {exc}"
        ) from exc
    if semantic.ndim != 2:
        raise DatasetContractError(
            f"semantic tensor must be 2-D, got shape {semantic.shape} at {path}"
        )
    if not np.issubdtype(semantic.dtype, np.integer):
        raise DatasetContractError(
            f"semantic tensor must have integer dtype, got {semantic.dtype} at {path}"
        )
    ids, counts = np.unique(semantic, return_counts=True)
    observed = {int(value) for value in ids.tolist()}
    unknown = sorted(observed.difference(ALLOWED_SEMANTIC_IDS))
    if unknown:
        raise DatasetContractError(f"unknown semantic IDs {unknown} at {path}")
    return (
        Counter(
            {
                int(value): int(count)
                for value, count in zip(ids.tolist(), counts.tolist())
            }
        ),
        (int(semantic.shape[0]), int(semantic.shape[1])),
        str(semantic.dtype),
    )


def _validate_modalities(
    resolved: dict[str, Path],
) -> tuple[Counter[int], dict[str, Any]]:
    rgb = cv2.imread(str(resolved["rgb"]), cv2.IMREAD_COLOR)
    if rgb is None or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise DatasetContractError(f"cannot decode RGB image: {resolved['rgb']}")
    try:
        depth = np.load(resolved["depth"], allow_pickle=False)
    except (OSError, ValueError) as exc:
        raise DatasetContractError(
            f"cannot safely load depth tensor {resolved['depth']}: {exc}"
        ) from exc
    if depth.ndim != 2:
        raise DatasetContractError(
            f"depth tensor must be 2-D, got shape {depth.shape} at {resolved['depth']}"
        )
    if not (
        np.issubdtype(depth.dtype, np.integer)
        or np.issubdtype(depth.dtype, np.floating)
    ):
        raise DatasetContractError(
            f"depth tensor must have real numeric dtype, got {depth.dtype}"
        )
    nan_count = int(np.isnan(depth).sum())
    negative_inf_count = int(np.isneginf(depth).sum())
    finite_negative_count = int(np.count_nonzero(np.isfinite(depth) & (depth < 0)))
    valid_positive_finite_count = int(
        np.count_nonzero(np.isfinite(depth) & (depth > 0))
    )
    if finite_negative_count or valid_positive_finite_count <= 0:
        raise DatasetContractError(
            "depth tensor contains invalid values: "
            f"nan={nan_count}, negative_inf={negative_inf_count}, "
            f"finite_negative={finite_negative_count}, "
            f"valid_positive_finite={valid_positive_finite_count} at {resolved['depth']}"
        )
    positive_inf_count = int(np.isposinf(depth).sum())
    finite_count = int(np.isfinite(depth).sum())
    counts, semantic_hw, semantic_dtype = _semantic_counts(resolved["semantic"])
    rgb_hw = (int(rgb.shape[0]), int(rgb.shape[1]))
    depth_hw = (int(depth.shape[0]), int(depth.shape[1]))
    if rgb_hw != depth_hw or rgb_hw != semantic_hw:
        raise DatasetContractError(
            "RGB/depth/semantic dimensions differ: "
            f"rgb={rgb_hw}, depth={depth_hw}, semantic={semantic_hw}"
        )
    return counts, {
        "shape_hw": list(rgb_hw),
        "rgb_dtype": str(rgb.dtype),
        "depth_dtype": str(depth.dtype),
        "semantic_dtype": semantic_dtype,
        "depth_finite_pixel_fraction": finite_count / int(depth.size),
        "depth_valid_positive_finite_pixel_fraction": (
            valid_positive_finite_count / int(depth.size)
        ),
        "depth_positive_inf_pixel_count": positive_inf_count,
        "depth_nan_pixel_count": nan_count,
        "depth_negative_inf_pixel_count": negative_inf_count,
        "depth_finite_negative_pixel_count": 0,
    }


def _scene_manifests(root: Path) -> list[Path]:
    direct = root / "scene_manifest.json"
    if direct.is_file():
        return [direct]
    paths = sorted(
        (path.resolve() for path in root.rglob("scene_manifest.json")),
        key=lambda path: path.as_posix().casefold(),
    )
    if not paths:
        raise DatasetContractError(
            f"no scene_manifest.json found under explicit root: {root}"
        )
    return paths


def _parse_root_specs(root_specs: Iterable[tuple[str, Path]]) -> list[tuple[str, Path]]:
    normalized: list[tuple[str, Path]] = []
    seen: set[tuple[str, str]] = set()
    for split_value, path_value in root_specs:
        split = _canonical_split(split_value, field="explicit root split")
        root = Path(path_value).expanduser().resolve()
        _reject_forbidden(root, field=f"{split} root")
        if not root.is_dir():
            raise DatasetContractError(
                f"explicit {split} root is not a directory: {root}"
            )
        key = (split, str(root).casefold())
        if key in seen:
            raise DatasetContractError(f"duplicate explicit root: {split}={root}")
        seen.add(key)
        normalized.append((split, root))
    normalized.sort(key=lambda item: (item[0], item[1].as_posix().casefold()))
    present = {split for split, _ in normalized}
    missing = sorted(set(ALLOWED_SPLITS).difference(present))
    if missing:
        raise DatasetContractError(
            f"explicit roots must include both TRAIN and HOLDOUT; missing {missing}"
        )
    return normalized


def build_manifest(root_specs: Iterable[tuple[str, Path]]) -> dict[str, Any]:
    """Return a deterministic, SHA-locked area dataset manifest."""

    roots = _parse_root_specs(root_specs)
    source_roots: list[dict[str, Any]] = []
    scenes: list[dict[str, Any]] = []
    excluded_scenes: list[dict[str, Any]] = []
    frames: list[dict[str, Any]] = []
    pixel_counts: Counter[int] = Counter()
    positive_frame_counts: Counter[int] = Counter()
    source_object_counts: Counter[int] = Counter()
    small_object_counts: Counter[int] = Counter()
    negative_domain_counts: Counter[str] = Counter()
    negative_only_scene_counts: Counter[str] = Counter()
    negative_only_frame_counts: Counter[str] = Counter()
    negative_only_world_ids: dict[str, set[str]] = {
        split: set() for split in ALLOWED_SPLITS
    }
    seen_frame_keys: set[tuple[str, str, str, str, int]] = set()
    seen_triplets: set[tuple[str, str, str]] = set()

    for root_index, (split, root) in enumerate(roots):
        root_id = f"{split}_{root_index:03d}"
        source_roots.append({"root_id": root_id, "split": split, "path": str(root)})
        for scene_manifest_path in _scene_manifests(root):
            _reject_forbidden(scene_manifest_path, field="scene manifest path")
            scene_dir = scene_manifest_path.parent.resolve()
            try:
                scene_dir.relative_to(root)
            except ValueError as exc:
                raise DatasetContractError(
                    f"scene directory escapes explicit root {root}: {scene_dir}"
                ) from exc
            capture_report_path = scene_dir / "capture_report.json"
            if not capture_report_path.is_file():
                excluded_scenes.append(
                    {
                        "root_id": root_id,
                        "split": split,
                        "scene_manifest_path": scene_manifest_path.relative_to(
                            root
                        ).as_posix(),
                        "scene_manifest_sha256": _sha256_file(scene_manifest_path),
                        "reason": "missing_capture_report",
                    }
                )
                continue
            scene_manifest = _load_json(scene_manifest_path)
            capture_report = _load_json(capture_report_path)
            source_split = _source_split(
                scene_manifest.get("split"),
                explicit_split=split,
                field=f"{scene_manifest_path} split",
            )
            world_id = scene_manifest.get("world_id")
            if not isinstance(world_id, str) or not world_id.strip():
                raise DatasetContractError(f"missing world_id in {scene_manifest_path}")
            _reject_forbidden(world_id, field=f"{scene_manifest_path} world_id")
            scene_id = scene_manifest.get("scene_id", scene_dir.name)
            if not isinstance(scene_id, str) or not scene_id.strip():
                raise DatasetContractError(f"invalid scene_id in {scene_manifest_path}")
            _reject_forbidden(scene_id, field=f"{scene_manifest_path} scene_id")
            objects = scene_manifest.get("objects", [])
            if not isinstance(objects, list):
                raise DatasetContractError(
                    f"objects must be a list in {scene_manifest_path}"
                )
            negative_only = scene_manifest.get("negative_only")
            if negative_only is not None and type(negative_only) is not bool:
                raise DatasetContractError(
                    f"negative_only must be boolean when declared in {scene_manifest_path}"
                )
            scene_negative_domains: set[str] = set()
            for object_index, item in enumerate(objects):
                if not isinstance(item, dict):
                    raise DatasetContractError(
                        f"object {object_index} is not an object in {scene_manifest_path}"
                    )
                semantic_label = item.get("semantic_label")
                if (
                    isinstance(semantic_label, bool)
                    or not isinstance(semantic_label, int)
                    or semantic_label not in ALLOWED_SEMANTIC_IDS
                ):
                    raise DatasetContractError(
                        f"object {object_index} has invalid semantic_label in {scene_manifest_path}"
                    )
                source_object_counts[semantic_label] += 1
                if negative_only is True and semantic_label != 0:
                    raise DatasetContractError(
                        f"negative_only scene declares target objects in {scene_manifest_path}"
                    )
                if str(item.get("size_bucket", "")).lower() in {"small", "small_area"}:
                    small_object_counts[semantic_label] += 1
                if semantic_label == 0:
                    taxonomy = item.get("taxonomy")
                    if isinstance(taxonomy, str) and taxonomy.strip():
                        _reject_forbidden(
                            taxonomy,
                            field=f"{scene_manifest_path} object {object_index} taxonomy",
                        )
                        scene_negative_domains.add(taxonomy.strip())
                        negative_domain_counts[taxonomy.strip()] += 1
            records = capture_report.get("records")
            if not isinstance(records, list) or not records:
                raise DatasetContractError(
                    f"capture report has no records: {capture_report_path}"
                )
            if negative_only is True:
                negative_only_scene_counts[split] += 1
                negative_only_world_ids[split].add(world_id)

            scene_record = {
                "root_id": root_id,
                "split": split,
                "source_split": source_split,
                "world_id": world_id,
                "scene_id": scene_id,
                "mission_id": scene_id,
                "scene_seed": scene_manifest.get("scene_seed"),
                "scene_manifest_path": scene_manifest_path.relative_to(root).as_posix(),
                "scene_manifest_sha256": _sha256_file(scene_manifest_path),
                "capture_report_path": capture_report_path.relative_to(root).as_posix(),
                "capture_report_sha256": _sha256_file(capture_report_path),
                "negative_only": negative_only,
                "negative_only_declared": negative_only is not None,
                "negative_domains": sorted(scene_negative_domains),
            }
            scenes.append(scene_record)

            ordered_records: list[tuple[int, int, dict[str, Any]]] = []
            for ordinal, record in enumerate(records):
                if not isinstance(record, dict):
                    raise DatasetContractError(
                        f"record {ordinal} is not an object in {capture_report_path}"
                    )
                frame_index = record.get("frame_index")
                if isinstance(frame_index, bool) or not isinstance(frame_index, int):
                    raise DatasetContractError(
                        f"record {ordinal} has non-integer frame_index in {capture_report_path}"
                    )
                ordered_records.append((frame_index, ordinal, record))

            for frame_index, ordinal, record in sorted(ordered_records):
                frame_key = (root_id, split, world_id, scene_id, frame_index)
                if frame_key in seen_frame_keys:
                    raise DatasetContractError(f"duplicate frame identity: {frame_key}")
                seen_frame_keys.add(frame_key)
                paths = record.get("paths")
                if not isinstance(paths, dict):
                    raise DatasetContractError(
                        f"record {ordinal} has no paths object in {capture_report_path}"
                    )
                resolved = {
                    modality: _require_child(
                        paths.get(modality),
                        scene_dir=scene_dir,
                        root=root,
                        field=f"record {ordinal} {modality}",
                    )
                    for modality in ("rgb", "depth", "semantic")
                }
                triplet = tuple(
                    str(resolved[name]).casefold()
                    for name in ("rgb", "depth", "semantic")
                )
                if triplet in seen_triplets:
                    raise DatasetContractError(
                        f"paired triplet reused by multiple frames: {triplet}"
                    )
                seen_triplets.add(triplet)
                counts, modality_contract = _validate_modalities(resolved)
                if negative_only is True and any(
                    count > 0 for semantic_id, count in counts.items() if semantic_id != 0
                ):
                    raise DatasetContractError(
                        f"negative_only scene contains positive semantic GT: {scene_manifest_path}"
                    )
                pixel_counts.update(counts)
                for semantic_id, count in counts.items():
                    if semantic_id != 0 and count > 0:
                        positive_frame_counts[semantic_id] += 1

                frames.append(
                    {
                        "root_id": root_id,
                        "split": split,
                        "world_id": world_id,
                        "scene_id": scene_id,
                        "mission_id": scene_id,
                        "negative_only": negative_only,
                        "frame_index": frame_index,
                        "timestamp_ns": record.get("timestamp_ns"),
                        "paths": {
                            name: resolved[name].relative_to(root).as_posix()
                            for name in ("rgb", "depth", "semantic")
                        },
                        "sha256": {
                            name: _sha256_file(resolved[name])
                            for name in ("rgb", "depth", "semantic")
                        },
                        "gt_source": {
                            "type": "gazebo_ground_truth_semantic_image",
                            "path": resolved["semantic"].relative_to(root).as_posix(),
                            "sha256": _sha256_file(resolved["semantic"]),
                        },
                        "semantic_pixel_counts": {
                            str(semantic_id): counts.get(semantic_id, 0)
                            for semantic_id in ALLOWED_SEMANTIC_IDS
                        },
                        "modality_contract": modality_contract,
                    }
                )
                if negative_only is True:
                    negative_only_frame_counts[split] += 1

    scenes.sort(
        key=lambda row: (row["split"], row["world_id"], row["scene_id"], row["root_id"])
    )
    excluded_scenes.sort(
        key=lambda row: (row["split"], row["root_id"], row["scene_manifest_path"])
    )
    frames.sort(
        key=lambda row: (
            row["split"],
            row["world_id"],
            row["scene_id"],
            row["frame_index"],
            row["root_id"],
        )
    )
    missing_targets = [
        semantic_id
        for semantic_id in TARGET_SEMANTIC_IDS
        if positive_frame_counts[semantic_id] <= 0
    ]
    failure_reasons = [
        f"missing_positive_semantic_id:{semantic_id}" for semantic_id in missing_targets
    ]
    if excluded_scenes:
        failure_reasons.append(
            f"excluded_incomplete_scene_count:{len(excluded_scenes)}"
        )
    missing_negative_only_splits = [
        split for split in ALLOWED_SPLITS if negative_only_scene_counts[split] <= 0
    ]
    failure_reasons.extend(
        f"missing_negative_only_scene:{split}"
        for split in missing_negative_only_splits
    )
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": "EMFJ6V3",
        "dataset_role": "area_model_screening_development_only",
        "development_only": True,
        "sealed_access_allowed": False,
        "allowed_splits": list(ALLOWED_SPLITS),
        "forbidden_markers": list(FORBIDDEN_MARKERS),
        "source_roots": source_roots,
        "scene_count": len(scenes),
        "excluded_scene_count": len(excluded_scenes),
        "excluded_scenes": excluded_scenes,
        "frame_count": len(frames),
        "scenes": scenes,
        "frames": frames,
        "semantic_audit": {
            "allowed_ids": list(ALLOWED_SEMANTIC_IDS),
            "class_by_id": {
                str(semantic_id): SEMANTIC_CLASSES[semantic_id]
                for semantic_id in ALLOWED_SEMANTIC_IDS
            },
            "observed_ids": [
                semantic_id
                for semantic_id in ALLOWED_SEMANTIC_IDS
                if pixel_counts[semantic_id] > 0
            ],
            "pixel_counts_by_id": {
                str(semantic_id): pixel_counts[semantic_id]
                for semantic_id in ALLOWED_SEMANTIC_IDS
            },
            "positive_frame_counts_by_id": {
                str(semantic_id): positive_frame_counts[semantic_id]
                for semantic_id in ALLOWED_SEMANTIC_IDS
            },
            "leaf_pile_positive_frame_count": positive_frame_counts[4],
            "puddle_positive_frame_count": positive_frame_counts[5],
        },
        "screening_dataset_contract": {
            "world_field": "scenes[].world_id",
            "seed_field": "scenes[].scene_seed",
            "mission_field": "frames[].mission_id",
            "frame_sha_field": "frames[].sha256.rgb",
            "gt_source_field": "frames[].gt_source",
            "source_object_count_by_semantic_id": {
                str(semantic_id): source_object_counts[semantic_id]
                for semantic_id in ALLOWED_SEMANTIC_IDS
            },
            "small_object_count_by_semantic_id": {
                str(semantic_id): small_object_counts[semantic_id]
                for semantic_id in ALLOWED_SEMANTIC_IDS
            },
            "negative_domain_counts": dict(sorted(negative_domain_counts.items())),
            "negative_only_scene_field": "scenes[].negative_only",
            "negative_only_frame_field": "frames[].negative_only",
            "negative_only_scene_counts_by_split": {
                split: negative_only_scene_counts[split] for split in ALLOWED_SPLITS
            },
            "negative_only_frame_counts_by_split": {
                split: negative_only_frame_counts[split] for split in ALLOWED_SPLITS
            },
            "negative_only_world_ids_by_split": {
                split: sorted(negative_only_world_ids[split])
                for split in ALLOWED_SPLITS
            },
        },
        "a4_area_dataset_ready": (
            not missing_targets
            and not excluded_scenes
            and not missing_negative_only_splits
        ),
        "failure_reasons": failure_reasons,
        "truth_boundary": (
            "a4_area_dataset_ready proves only this non-sealed manifest's pairing, "
            "hash lock, semantic-ID audit, positive presence of IDs 4 and 5, "
            "and declared negative-only scenes in each split; "
            "it is not an overall A4 gate result"
        ),
    }
    manifest["manifest_sha256"] = _canonical_sha256(manifest)
    return manifest


def write_manifest(manifest: dict[str, Any], output: Path) -> None:
    output = output.expanduser().resolve()
    _reject_forbidden(output, field="output path")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _split_root(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("expected SPLIT=PATH")
    split, path = value.split("=", 1)
    if not split.strip() or not path.strip():
        raise argparse.ArgumentTypeError("expected non-empty SPLIT=PATH")
    return split, Path(path)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--split-root",
        action="append",
        required=True,
        type=_split_root,
        metavar="SPLIT=PATH",
        help="explicit non-sealed TRAIN or HOLDOUT capture root; repeat as needed",
    )
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        manifest = build_manifest(args.split_root)
        write_manifest(manifest, args.output)
    except DatasetContractError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "output": str(args.output.resolve()),
                "manifest_sha256": manifest["manifest_sha256"],
                "frame_count": manifest["frame_count"],
                "leaf_pile_positive_frame_count": manifest["semantic_audit"][
                    "leaf_pile_positive_frame_count"
                ],
                "puddle_positive_frame_count": manifest["semantic_audit"][
                    "puddle_positive_frame_count"
                ],
                "a4_area_dataset_ready": manifest["a4_area_dataset_ready"],
                "failure_reasons": manifest["failure_reasons"],
            },
            sort_keys=True,
        )
    )
    return 0 if manifest["a4_area_dataset_ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
