"""Pure-Python validation core for the S100P DOSOD-to-EdgeSAM adapter.

This module deliberately knows nothing about ROS messages.  The board-facing
node supplies small ``dict`` objects extracted from ``ai_msgs`` and uses these
helpers before it publishes any formal product observation.  Invalid upstream
data raises :class:`S100PProductAdapterError`; callers must treat that as a
fail-closed event and withhold planning outputs.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from math import isfinite
from pathlib import Path
from types import MappingProxyType
from typing import Any, Iterable, Mapping, Sequence


FROZEN_CLASS_IDS = frozenset(
    ("litter_cube", "fallen_leaves", "dust_or_soil", "puddle")
)
GROUND_DIRT_CLASS_IDS = frozenset(("fallen_leaves", "dust_or_soil", "puddle"))
FROZEN_CLASS_ORDER = (
    "litter_cube",
    "fallen_leaves",
    "dust_or_soil",
    "puddle",
)
FORMAL_S100P_MARCH = "nash-m"
FORMAL_S100P_BOARD = "RDK S100P"
FORMAL_S100P_SOC = "Journey 6P"
FORMAL_S100P_PLATFORM = "rdk_s100"
DOSOD_VOCABULARY_RELATIVE_PATH = "dosod/tzcup_offline_vocabulary.json"
BOARD_ARTIFACT_SPECS = {
    "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm": (
        "project_four_class_dosod_s100p_detector",
        "c50129b5badf6ed7bb85e692ab493d8bdb58da6a",
    ),
    DOSOD_VOCABULARY_RELATIVE_PATH: (
        "frozen_project_prompt_vocabulary",
        "c50129b5badf6ed7bb85e692ab493d8bdb58da6a",
    ),
    "edgesam/edgesam_encoder_512.hbm": (
        "edgesam_512_s100p_image_encoder",
        "d24d99671f41a9c0003061248bded64a481e9059",
    ),
    "edgesam/edgesam_decoder_512.hbm": (
        "edgesam_512_s100p_box_prompt_decoder",
        "d24d99671f41a9c0003061248bded64a481e9059",
    ),
}


class S100PProductAdapterError(ValueError):
    """An upstream board message cannot safely become a product observation."""


@dataclass(frozen=True)
class BoardArtifactContract:
    """Verified board artifacts and the exact labels DOSOD is allowed to emit."""

    model_hashes: Mapping[str, str]
    emitted_label_to_class_id: Mapping[str, str]


@dataclass(frozen=True)
class Roi:
    """A source-image ROI in x/y/width/height form."""

    x_offset: float
    y_offset: float
    width: float
    height: float

    @property
    def xyxy(self) -> tuple[float, float, float, float]:
        return (
            self.x_offset,
            self.y_offset,
            self.x_offset + self.width,
            self.y_offset + self.height,
        )


@dataclass(frozen=True)
class Detection:
    """One validated DOSOD ROI with its frozen project class."""

    class_id: str
    confidence: float
    roi: Roi
    source_index: int


@dataclass(frozen=True)
class EdgeSamPromptBatch:
    """The exact prompt batch whose later EdgeSAM labels may be decoded."""

    stamp_ns: int
    image_width: int
    image_height: int
    prompts: tuple[Detection, ...]


@dataclass(frozen=True)
class DecodedEdgeSamLabels:
    """Binary masks in prompt order, retained as plain immutable tuples."""

    stamp_ns: int
    image_width: int
    image_height: int
    prompts: tuple[Detection, ...]
    masks: tuple[tuple[bool, ...], ...]


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise S100PProductAdapterError(f"{label} must be a mapping")
    return value


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise S100PProductAdapterError(f"{label} must be a non-empty string")
    return value.strip()


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise S100PProductAdapterError(f"{label} must be a finite number")
    result = float(value)
    if not isfinite(result):
        raise S100PProductAdapterError(f"{label} must be a finite number")
    return result


def _positive_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise S100PProductAdapterError(f"{label} must be a positive integer")
    return value


def _stamp_ns(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise S100PProductAdapterError(f"{label} must be a non-negative integer nanosecond stamp")
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _string_sequence(value: Any, label: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise S100PProductAdapterError(f"{label} must be a JSON array")
    return tuple(_nonempty_string(item, f"{label}[{index}]") for index, item in enumerate(value))


def load_verified_board_artifact_contract(
    *,
    artifact_manifest_path: str | Path,
    artifact_paths: Mapping[str, str | Path],
) -> BoardArtifactContract:
    """Bind the board's model files and emitted DOSOD labels to frozen assets.

    ``hobot_dosod`` publishes the first string from every offline-vocabulary
    group.  The mapping therefore accepts only those first labels, rather than
    every embedding synonym.  Any mismatch in the manifest, model files,
    vocabulary structure or declared S100P target rejects adapter start-up.
    """

    if not isinstance(artifact_paths, Mapping):
        raise S100PProductAdapterError("artifact_paths must be a mapping")
    expected_paths = frozenset(BOARD_ARTIFACT_SPECS)
    if frozenset(artifact_paths) != expected_paths:
        raise S100PProductAdapterError("board artifact path set does not match frozen contract")
    manifest_path = Path(artifact_manifest_path)
    if not manifest_path.is_file():
        raise S100PProductAdapterError(f"board artifact manifest is missing: {manifest_path}")
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise S100PProductAdapterError(
            f"board artifact manifest is unreadable: {type(exc).__name__}"
        ) from exc
    if not isinstance(payload, Mapping) or payload.get("schema_version") != 1:
        raise S100PProductAdapterError("board artifact manifest schema is unsupported")
    runtime = payload.get("board_runtime_contract")
    if not isinstance(runtime, Mapping) or runtime != {
        "platform": FORMAL_S100P_PLATFORM,
        "board": FORMAL_S100P_BOARD,
        "soc": FORMAL_S100P_SOC,
        "march": FORMAL_S100P_MARCH,
    }:
        raise S100PProductAdapterError("board artifact manifest target is not the formal S100P nash-m contract")
    rows = payload.get("artifacts")
    if not isinstance(rows, Mapping):
        raise S100PProductAdapterError("board artifact manifest has no artifact mapping")

    hashes: dict[str, str] = {}
    vocabulary_row: Mapping[str, Any] | None = None
    vocabulary_path: Path | None = None
    for relative, (expected_role, expected_revision) in BOARD_ARTIFACT_SPECS.items():
        row = rows.get(relative)
        if not isinstance(row, Mapping):
            raise S100PProductAdapterError(f"board artifact manifest row is missing: {relative}")
        if row.get("model_role") != expected_role or row.get("source_revision") != expected_revision:
            raise S100PProductAdapterError(f"board artifact provenance mismatch: {relative}")
        expected_sha = row.get("sha256")
        expected_size = row.get("byte_size")
        if not isinstance(expected_sha, str) or len(expected_sha) != 64:
            raise S100PProductAdapterError(f"board artifact SHA is invalid: {relative}")
        if isinstance(expected_size, bool) or not isinstance(expected_size, int) or expected_size <= 0:
            raise S100PProductAdapterError(f"board artifact byte size is invalid: {relative}")
        path = Path(artifact_paths[relative])
        if not path.is_file():
            raise S100PProductAdapterError(f"required board artifact is missing: {path}")
        actual_sha = _sha256_file(path)
        if actual_sha != expected_sha.lower() or path.stat().st_size != expected_size:
            raise S100PProductAdapterError(f"board artifact hash or byte size mismatch: {relative}")
        hashes[relative] = actual_sha
        if relative == DOSOD_VOCABULARY_RELATIVE_PATH:
            vocabulary_row = row
            vocabulary_path = path

    assert vocabulary_row is not None and vocabulary_path is not None
    semantic_ids = _string_sequence(
        vocabulary_row.get("semantic_class_ids"),
        "vocabulary.semantic_class_ids",
    )
    declared_labels = _string_sequence(
        vocabulary_row.get("emitted_labels"),
        "vocabulary.emitted_labels",
    )
    if semantic_ids != FROZEN_CLASS_ORDER or len(set(declared_labels)) != len(declared_labels):
        raise S100PProductAdapterError("vocabulary semantic class order is not frozen")
    try:
        vocabulary = json.loads(vocabulary_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise S100PProductAdapterError(
            f"frozen DOSOD vocabulary is unreadable: {type(exc).__name__}"
        ) from exc
    if not isinstance(vocabulary, list) or len(vocabulary) != len(FROZEN_CLASS_ORDER):
        raise S100PProductAdapterError("frozen DOSOD vocabulary must contain exactly four groups")
    emitted_labels: list[str] = []
    for index, group in enumerate(vocabulary):
        if not isinstance(group, list) or not group:
            raise S100PProductAdapterError(f"frozen DOSOD vocabulary group {index} is empty")
        emitted_labels.append(_nonempty_string(group[0], f"frozen DOSOD vocabulary group {index}[0]"))
    if tuple(emitted_labels) != declared_labels:
        raise S100PProductAdapterError("DOSOD emitted labels do not match frozen manifest")
    return BoardArtifactContract(
        model_hashes=MappingProxyType(hashes),
        emitted_label_to_class_id=MappingProxyType(
            dict(zip(emitted_labels, FROZEN_CLASS_ORDER, strict=True))
        ),
    )


def roi_from_ai_like(value: Mapping[str, Any]) -> Roi:
    """Parse one ``ai_msgs/Roi.rect``-like mapping without ROS imports."""

    row = _mapping(value, "ROI")
    rect = _mapping(row.get("rect"), "ROI.rect")
    roi = Roi(
        x_offset=_finite_number(rect.get("x_offset"), "ROI.rect.x_offset"),
        y_offset=_finite_number(rect.get("y_offset"), "ROI.rect.y_offset"),
        width=_finite_number(rect.get("width"), "ROI.rect.width"),
        height=_finite_number(rect.get("height"), "ROI.rect.height"),
    )
    if roi.width <= 0.0 or roi.height <= 0.0:
        raise S100PProductAdapterError("ROI width and height must be positive")
    if roi.x_offset < 0.0 or roi.y_offset < 0.0:
        raise S100PProductAdapterError("ROI offsets must be non-negative")
    return roi


def detections_from_ai_like(
    targets: Iterable[Mapping[str, Any]],
    *,
    allowed_class_ids: frozenset[str] = FROZEN_CLASS_IDS,
    emitted_label_to_class_id: Mapping[str, str] | None = None,
) -> tuple[Detection, ...]:
    """Flatten DOSOD ``targets`` into frozen-class detections.

    A target can carry its class on ``target.type`` or on each ``roi.type``.
    When an emitted-label mapping is supplied, both source labels are
    canonicalized before comparison.  A malformed, unknown or inconsistent
    class rejects the entire message instead of silently dropping a potential
    obstacle/dirt.
    """

    if not isinstance(allowed_class_ids, frozenset) or not allowed_class_ids:
        raise S100PProductAdapterError("allowed_class_ids must be a non-empty frozenset")
    if emitted_label_to_class_id is not None:
        if not isinstance(emitted_label_to_class_id, Mapping) or not emitted_label_to_class_id:
            raise S100PProductAdapterError("emitted label mapping must be a non-empty mapping")
        if any(
            not isinstance(label, str)
            or not label.strip()
            or canonical not in allowed_class_ids
            for label, canonical in emitted_label_to_class_id.items()
        ):
            raise S100PProductAdapterError("emitted label mapping is invalid")

    def canonicalize(label: str | None) -> str | None:
        if label is None:
            return None
        if emitted_label_to_class_id is None:
            return label
        canonical = emitted_label_to_class_id.get(label)
        if canonical is None:
            raise S100PProductAdapterError(f"unknown frozen project class: {label}")
        return canonical

    rows = tuple(targets)
    detections: list[Detection] = []
    source_index = 0
    for target_index, raw_target in enumerate(rows):
        target = _mapping(raw_target, f"target[{target_index}]")
        target_type_raw = target.get("type")
        target_type = canonicalize(
            _nonempty_string(target_type_raw, f"target[{target_index}].type")
            if target_type_raw not in (None, "")
            else None
        )
        rois_raw = target.get("rois")
        if not isinstance(rois_raw, Sequence) or isinstance(rois_raw, (str, bytes)):
            raise S100PProductAdapterError(f"target[{target_index}].rois must be a sequence")
        for roi_index, raw_roi in enumerate(rois_raw):
            roi_row = _mapping(raw_roi, f"target[{target_index}].rois[{roi_index}]")
            roi_type_raw = roi_row.get("type")
            roi_type = canonicalize(
                _nonempty_string(roi_type_raw, f"target[{target_index}].rois[{roi_index}].type")
                if roi_type_raw not in (None, "")
                else None
            )
            if target_type and roi_type and target_type != roi_type:
                raise S100PProductAdapterError("target and ROI class ids disagree")
            class_id = roi_type or target_type
            if class_id is None:
                raise S100PProductAdapterError("DOSOD ROI has no class id")
            if class_id not in allowed_class_ids:
                raise S100PProductAdapterError(f"unknown frozen project class: {class_id}")
            confidence = _finite_number(
                roi_row.get("confidence", target.get("confidence")),
                f"target[{target_index}].rois[{roi_index}].confidence",
            )
            if not 0.0 <= confidence <= 1.0:
                raise S100PProductAdapterError("DOSOD confidence must be in [0, 1]")
            detections.append(
                Detection(
                    class_id=class_id,
                    confidence=confidence,
                    roi=roi_from_ai_like(roi_row),
                    source_index=source_index,
                )
            )
            source_index += 1
    return tuple(detections)


def ground_dirt_prompt_batch(
    detections: Iterable[Detection],
    *,
    stamp_ns: int,
    image_width: int,
    image_height: int,
    max_per_class: int = 3,
    max_area_fraction: float = 0.45,
) -> EdgeSamPromptBatch:
    """Create deterministic bounded EdgeSAM prompts from valid DOSOD output."""

    stamp = _stamp_ns(stamp_ns, "stamp_ns")
    width = _positive_int(image_width, "image_width")
    height = _positive_int(image_height, "image_height")
    if isinstance(max_per_class, bool) or not isinstance(max_per_class, int) or max_per_class < 1:
        raise S100PProductAdapterError("max_per_class must be a positive integer")
    fraction = _finite_number(max_area_fraction, "max_area_fraction")
    if not 0.0 < fraction <= 1.0:
        raise S100PProductAdapterError("max_area_fraction must be in (0, 1]")

    rows = tuple(detections)
    if any(not isinstance(row, Detection) for row in rows):
        raise S100PProductAdapterError("detections must contain Detection values only")
    selected: list[Detection] = []
    for class_id in sorted(GROUND_DIRT_CLASS_IDS):
        candidates = [row for row in rows if row.class_id == class_id]
        candidates.sort(key=lambda row: (-row.confidence, row.source_index))
        for row in candidates:
            area_fraction = (row.roi.width * row.roi.height) / float(width * height)
            if area_fraction <= 0.0 or area_fraction > fraction:
                continue
            selected.append(row)
            if sum(item.class_id == class_id for item in selected) >= max_per_class:
                break
    return EdgeSamPromptBatch(stamp, width, height, tuple(selected))


def _same_roi(left: Roi, right: Roi) -> bool:
    left_x1, left_y1, left_x2, left_y2 = left.xyxy
    right_x1, right_y1, right_x2, right_y2 = right.xyxy
    intersection = max(0.0, min(left_x2, right_x2) - max(left_x1, right_x1)) * max(
        0.0, min(left_y2, right_y2) - max(left_y1, right_y1)
    )
    union = left.width * left.height + right.width * right.height - intersection
    if union <= 0.0 or intersection / union < 0.90:
        return False
    left_cx = left.x_offset + left.width / 2.0
    left_cy = left.y_offset + left.height / 2.0
    right_cx = right.x_offset + right.width / 2.0
    right_cy = right.y_offset + right.height / 2.0
    return (
        abs(left_cx - right_cx) <= max(2.0, right.width * 0.02)
        and abs(left_cy - right_cy) <= max(2.0, right.height * 0.02)
    )


def decode_edgesam_label_features(
    batch: EdgeSamPromptBatch,
    *,
    output_stamp_ns: int,
    feature_values: Iterable[Any],
    capture_width: int,
    capture_height: int,
    expected_capture_width: int,
    expected_capture_height: int,
    output_prompt_rois: Iterable[Mapping[str, Any]],
    output_prompt_class_ids: Iterable[str],
) -> DecodedEdgeSamLabels:
    """Decode an S100 EdgeSAM capture only when its origin is unambiguous.

    The official S100 parser emits label 0 for background and one-based labels
    in the same order as its ROI prompt list.  Exact stamp, image dimensions,
    ROI count and ROI order checks prevent an asynchronously delayed output
    from being projected using another camera frame's geometry.
    """

    if not isinstance(batch, EdgeSamPromptBatch):
        raise S100PProductAdapterError("batch must be an EdgeSamPromptBatch")
    if not batch.prompts:
        raise S100PProductAdapterError("EdgeSAM capture has no expected prompts")
    if _stamp_ns(output_stamp_ns, "output_stamp_ns") != batch.stamp_ns:
        raise S100PProductAdapterError("EdgeSAM output stamp does not match prompt batch")
    width = _positive_int(capture_width, "capture_width")
    height = _positive_int(capture_height, "capture_height")
    expected_width = _positive_int(expected_capture_width, "expected_capture_width")
    expected_height = _positive_int(expected_capture_height, "expected_capture_height")
    # The official 512 S100P decoder publishes a 512x288 network-space label
    # map for the selected HBM.  Its exact shape is part of the frozen model
    # contract; accepting an arbitrary same-aspect-ratio capture could bind a
    # delayed or differently configured model output to the wrong frame.
    if (width, height) != (expected_width, expected_height):
        raise S100PProductAdapterError(
            "EdgeSAM capture dimensions do not match the frozen model contract"
        )

    output_rois = tuple(roi_from_ai_like(row) for row in output_prompt_rois)
    expected_rois = tuple(row.roi for row in batch.prompts)
    output_classes = tuple(
        _nonempty_string(value, "EdgeSAM output prompt class")
        for value in output_prompt_class_ids
    )
    expected_classes = tuple(row.class_id for row in batch.prompts)
    if len(output_rois) != len(expected_rois):
        raise S100PProductAdapterError("EdgeSAM output ROI count does not match prompt batch")
    if output_classes != expected_classes:
        raise S100PProductAdapterError("EdgeSAM output class order does not match prompt batch")
    if any(not _same_roi(observed, expected) for observed, expected in zip(output_rois, expected_rois)):
        raise S100PProductAdapterError("EdgeSAM output ROI geometry does not overlap its prompt")

    values = tuple(feature_values)
    if not values:
        raise S100PProductAdapterError("EdgeSAM capture features are empty")
    if len(values) != width * height:
        raise S100PProductAdapterError("EdgeSAM capture feature count does not match dimensions")
    labels: list[int] = []
    for index, raw_value in enumerate(values):
        value = _finite_number(raw_value, f"EdgeSAM feature[{index}]")
        rounded = int(value)
        if value != float(rounded):
            raise S100PProductAdapterError("EdgeSAM capture labels must be integral")
        if rounded < 0 or rounded > len(batch.prompts):
            raise S100PProductAdapterError("EdgeSAM capture contains a label outside the prompt batch")
        labels.append(rounded)
    masks = tuple(
        tuple(label == prompt_index for label in labels)
        for prompt_index in range(1, len(batch.prompts) + 1)
    )
    return DecodedEdgeSamLabels(
        stamp_ns=batch.stamp_ns,
        image_width=width,
        image_height=height,
        prompts=batch.prompts,
        masks=masks,
    )
