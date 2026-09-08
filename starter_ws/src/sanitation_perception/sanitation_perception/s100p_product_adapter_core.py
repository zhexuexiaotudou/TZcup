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
from math import floor, isfinite
from pathlib import Path
import struct
from types import MappingProxyType
from typing import Any, Iterable, Mapping, Sequence
from collections import OrderedDict
import time


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
S100P_POSTPROCESS_THRESHOLDS = MappingProxyType(
    {
        "litter_cube": 0.005,
        "fallen_leaves": 0.0025,
        "dust_or_soil": 0.002,
        "puddle": 0.003,
    }
)
FORMAL_S100P_MARCH = "nash-m"
FORMAL_S100P_BOARD = "RDK S100P"
FORMAL_S100P_SOC = "Journey 6P"
FORMAL_S100P_PLATFORM = "rdk_s100"
S100P_EDGESAM_MODEL_INPUT_WIDTH = 512
S100P_EDGESAM_MODEL_INPUT_HEIGHT = 512
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


class ExactStampRgbdCache:
    """Bounded source-frame cache consumed exactly once by a DOSOD stamp."""

    def __init__(self, limit: int) -> None:
        if isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
            raise S100PProductAdapterError("RGB-D cache limit must be positive")
        self._limit = limit
        self._rgb: OrderedDict[int, Any] = OrderedDict()
        self._depth: OrderedDict[int, Any] = OrderedDict()
        self._camera_info: OrderedDict[int, Any] = OrderedDict()
        self._consumed: OrderedDict[int, None] = OrderedDict()
        self._consumed_frontier: int | None = None

    def _put(self, table: OrderedDict[int, Any], stamp_ns: int, value: Any) -> None:
        stamp = _stamp_ns(stamp_ns, "source stamp")
        if stamp in self._consumed or (
            self._consumed_frontier is not None
            and stamp <= self._consumed_frontier
            and not any(stamp in cached for cached in (self._rgb, self._depth, self._camera_info))
        ):
            raise S100PProductAdapterError("source stamp is at or before the consumed frontier")
        table[stamp] = value
        table.move_to_end(stamp)
        while len(table) > self._limit:
            table.popitem(last=False)

    def put_rgb(self, stamp_ns: int, value: Any) -> None:
        self._put(self._rgb, stamp_ns, value)

    def put_depth(self, stamp_ns: int, value: Any) -> None:
        self._put(self._depth, stamp_ns, value)

    def put_camera_info(self, stamp_ns: int, value: Any) -> None:
        self._put(self._camera_info, stamp_ns, value)

    def consume(self, stamp_ns: int) -> tuple[Any, Any, Any]:
        stamp = _stamp_ns(stamp_ns, "DOSOD stamp")
        values = (self._rgb.get(stamp), self._depth.get(stamp), self._camera_info.get(stamp))
        if any(value is None for value in values):
            raise S100PProductAdapterError("DOSOD stamp has no exact RGB-D/CameraInfo source tuple")
        self._rgb.pop(stamp)
        self._depth.pop(stamp)
        self._camera_info.pop(stamp)
        self._consumed[stamp] = None
        self._consumed_frontier = max(stamp, self._consumed_frontier or stamp)
        self._consumed.move_to_end(stamp)
        while len(self._consumed) > self._limit:
            self._consumed.popitem(last=False)
        return values

    def has_exact(self, stamp_ns: int) -> bool:
        stamp = _stamp_ns(stamp_ns, "DOSOD stamp")
        return all(
            table.get(stamp) is not None
            for table in (self._rgb, self._depth, self._camera_info)
        )


class PendingDosodCache:
    """Bounded raw-DOSOD waitlist for exact RGB-D/CameraInfo arrival races."""

    def __init__(self, limit: int, max_age_ns: int) -> None:
        if isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
            raise S100PProductAdapterError("pending DOSOD limit must be positive")
        if isinstance(max_age_ns, bool) or not isinstance(max_age_ns, int) or max_age_ns < 1:
            raise S100PProductAdapterError("pending DOSOD age must be positive")
        self._limit = limit
        self._max_age_ns = max_age_ns
        self._rows: OrderedDict[int, tuple[Any, int]] = OrderedDict()
        self._terminal: OrderedDict[int, None] = OrderedDict()
        self._terminal_frontier: int | None = None

    def put(self, stamp_ns: int, value: Any, *, now_ns: int | None = None) -> tuple[int, ...]:
        stamp = _stamp_ns(stamp_ns, "DOSOD stamp")
        if stamp <= 0:
            raise S100PProductAdapterError("DOSOD stamp must be positive")
        if stamp in self._rows or stamp in self._terminal or (
            self._terminal_frontier is not None and stamp <= self._terminal_frontier
        ):
            raise S100PProductAdapterError("duplicate DOSOD stamp")
        now = time.monotonic_ns() if now_ns is None else _stamp_ns(now_ns, "monotonic time")
        self._rows[stamp] = (value, now)
        evicted: list[int] = []
        while len(self._rows) > self._limit:
            evicted.append(self._rows.popitem(last=False)[0])
        return tuple(evicted)

    def expire(self, now_ns: int | None = None) -> tuple[int, ...]:
        reference = time.monotonic_ns() if now_ns is None else _stamp_ns(now_ns, "monotonic time")
        expired = tuple(
            stamp for stamp, (_, inserted_ns) in self._rows.items()
            if reference > inserted_ns and reference - inserted_ns > self._max_age_ns
        )
        for stamp in expired:
            self._rows.pop(stamp)
            self._remember_terminal(stamp)
        return expired

    def _remember_terminal(self, stamp: int) -> None:
        self._terminal[stamp] = None
        self._terminal_frontier = max(stamp, self._terminal_frontier or stamp)
        self._terminal.move_to_end(stamp)
        while len(self._terminal) > self._limit:
            self._terminal.popitem(last=False)

    def take_if_ready(
        self, stamp_ns: int, source_cache: ExactStampRgbdCache
    ) -> tuple[Any, tuple[Any, Any, Any]] | None:
        stamp = _stamp_ns(stamp_ns, "DOSOD stamp")
        row = self._rows.get(stamp)
        if row is None or not source_cache.has_exact(stamp):
            return None
        self._rows.pop(stamp)
        self._remember_terminal(stamp)
        return row[0], source_cache.consume(stamp)


class PendingEdgeSamCache:
    """Bounded DOSOD-to-EdgeSAM handoff consumed on either terminal outcome."""

    def __init__(self, limit: int) -> None:
        if isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
            raise S100PProductAdapterError("pending frame limit must be positive")
        self._limit = limit
        self._rows: OrderedDict[int, Any] = OrderedDict()

    def put(self, stamp_ns: int, value: Any) -> None:
        stamp = _stamp_ns(stamp_ns, "DOSOD stamp")
        if stamp in self._rows:
            raise S100PProductAdapterError("DOSOD stamp is already pending")
        self._rows[stamp] = value
        while len(self._rows) > self._limit:
            self._rows.popitem(last=False)

    def consume(self, stamp_ns: int) -> Any:
        stamp = _stamp_ns(stamp_ns, "EdgeSAM stamp")
        try:
            return self._rows.pop(stamp)
        except KeyError as exc:
            raise S100PProductAdapterError("EdgeSAM stamp has no pending DOSOD frame") from exc


def requires_edgesam_handoff(batch: "EdgeSamPromptBatch") -> bool:
    """Only a nonempty validated prompt batch may remain pending for EdgeSAM."""
    if not isinstance(batch, EdgeSamPromptBatch):
        raise S100PProductAdapterError("EdgeSAM handoff requires an EdgeSamPromptBatch")
    return bool(batch.prompts)


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


def filter_s100p_product_detections(
    detections: Iterable[Detection],
) -> tuple[Detection, ...]:
    """Apply the frozen project class gates after official DOSOD postprocess."""
    rows = tuple(detections)
    if any(not isinstance(row, Detection) for row in rows):
        raise S100PProductAdapterError("detections must contain Detection values only")
    if any(row.class_id not in S100P_POSTPROCESS_THRESHOLDS for row in rows):
        raise S100PProductAdapterError("detection class is outside the frozen S100P domain")
    return tuple(
        row for row in rows
        if row.confidence >= S100P_POSTPROCESS_THRESHOLDS[row.class_id]
    )


def validate_dosod_source_rois(
    detections: Iterable[Detection], *, source_width: int = 848, source_height: int = 480
) -> tuple[Detection, ...]:
    """Accept only official DOSOD ROIs already expressed in the source image."""
    width = _positive_int(source_width, "source width")
    height = _positive_int(source_height, "source height")
    rows = tuple(detections)
    for row in rows:
        if not isinstance(row, Detection):
            raise S100PProductAdapterError("detections must contain Detection values only")
        x1, y1, x2, y2 = row.roi.xyxy
        if (
            not all(isfinite(value) for value in (x1, y1, x2, y2))
            or x1 < 0.0
            or y1 < 0.0
            or x2 > width
            or y2 > height
            or x2 <= x1
            or y2 <= y1
        ):
            raise S100PProductAdapterError("DOSOD ROI is outside the 848x480 source frame")
    return rows


def validate_exact_rgbd_projection_binding(
    *,
    rgb_stamp_ns: int,
    rgb_frame_id: str,
    rgb_width: int,
    rgb_height: int,
    depth_stamp_ns: int,
    depth_frame_id: str,
    depth_width: int,
    depth_height: int,
    depth_encoding: str,
    camera_info_stamp_ns: int,
    camera_info_frame_id: str,
    camera_info_width: int,
    camera_info_height: int,
    camera_k: Iterable[Any],
) -> None:
    """Reject any RGB-D/CameraInfo tuple not bound to one source frame."""
    stamp = _stamp_ns(rgb_stamp_ns, "RGB stamp")
    frame_id = _nonempty_string(rgb_frame_id, "RGB frame id")
    width = _positive_int(rgb_width, "RGB width")
    height = _positive_int(rgb_height, "RGB height")
    if (
        _stamp_ns(depth_stamp_ns, "depth stamp") != stamp
        or _nonempty_string(depth_frame_id, "depth frame id") != frame_id
        or _positive_int(depth_width, "depth width") != width
        or _positive_int(depth_height, "depth height") != height
        or depth_encoding not in {"16UC1", "32FC1"}
    ):
        raise S100PProductAdapterError("depth is not exactly bound to the RGB frame")
    try:
        # RCLPY exposes CameraInfo.k as a numpy.ndarray on the S100P.  Freeze
        # any iterable before validation so its contents are checked exactly
        # once; this accepts that runtime representation without weakening any
        # of the projection-binding invariants below.
        camera_values = tuple(camera_k)
    except TypeError as exc:
        raise S100PProductAdapterError("CameraInfo.K must be an iterable of nine values") from exc
    if (
        _stamp_ns(camera_info_stamp_ns, "CameraInfo stamp") != stamp
        or _nonempty_string(camera_info_frame_id, "CameraInfo frame id") != frame_id
        or _positive_int(camera_info_width, "CameraInfo width") != width
        or _positive_int(camera_info_height, "CameraInfo height") != height
        or isinstance(camera_k, (str, bytes))
        or len(camera_values) != 9
    ):
        raise S100PProductAdapterError("CameraInfo is not exactly bound to the RGB frame")
    values = tuple(_finite_number(value, "CameraInfo.K") for value in camera_values)
    if values[0] <= 0.0 or values[4] <= 0.0:
        raise S100PProductAdapterError("CameraInfo focal lengths must be positive")


def validate_public_map_binding(
    *,
    frame_id: str,
    expected_frame_id: str,
    width: int,
    height: int,
    resolution: float,
    origin_values: Sequence[Any],
    data_length: int,
) -> None:
    """Validate the complete public OccupancyGrid geometry before projection."""
    if _nonempty_string(frame_id, "map frame") != _nonempty_string(expected_frame_id, "expected map frame"):
        raise S100PProductAdapterError("public occupancy grid frame is not the configured map frame")
    cells = _positive_int(width, "map width") * _positive_int(height, "map height")
    if _finite_number(resolution, "map resolution") <= 0.0:
        raise S100PProductAdapterError("map resolution must be positive and finite")
    if not isinstance(origin_values, Sequence) or isinstance(origin_values, (str, bytes)) or len(origin_values) != 7:
        raise S100PProductAdapterError("map origin must contain seven finite pose values")
    tuple(_finite_number(value, "map origin") for value in origin_values)
    if isinstance(data_length, bool) or not isinstance(data_length, int) or data_length != cells:
        raise S100PProductAdapterError("map data length does not match width times height")


def validate_exact_tf_binding(
    *, image_stamp_ns: int, transform_stamp_ns: int, max_age_s: float
) -> str:
    """Validate exact-time dynamic TF or explicitly stamped-zero static TF."""
    image_stamp = _stamp_ns(image_stamp_ns, "RGB stamp")
    if image_stamp <= 0:
        raise S100PProductAdapterError("RGB stamp must be positive")
    transform_stamp = _stamp_ns(transform_stamp_ns, "TF stamp")
    maximum = _finite_number(max_age_s, "TF max age")
    if maximum < 0.0 or maximum > 0.75:
        raise S100PProductAdapterError("TF max age must be within the frozen [0, 0.75] seconds")
    if transform_stamp == 0:
        return "static"
    if transform_stamp > image_stamp:
        raise S100PProductAdapterError("map TF is from the future of the RGB frame")
    if (image_stamp - transform_stamp) * 1.0e-9 > maximum:
        raise S100PProductAdapterError("map TF is stale for the RGB frame")
    return "dynamic"


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


def _float32(value: float | int, label: str) -> float:
    """Replay the official S100 C++ ``float`` rounding without NumPy."""

    try:
        result = struct.unpack("<f", struct.pack("<f", float(value)))[0]
    except (OverflowError, struct.error) as exc:
        raise S100PProductAdapterError(
            f"{label} is not representable as binary32"
        ) from exc
    if not isfinite(result):
        raise S100PProductAdapterError(f"{label} must be finite")
    return result


def _roi_integer(value: float, label: str) -> int:
    if not isfinite(value) or value != floor(value):
        raise S100PProductAdapterError(
            f"{label} must be an integral pixel coordinate"
        )
    return int(value)


def canonicalize_official_s100p_edgesam_roi(
    roi: Roi,
    *,
    source_image_width: int,
    source_image_height: int,
    model_input_width: int = S100P_EDGESAM_MODEL_INPUT_WIDTH,
    model_input_height: int = S100P_EDGESAM_MODEL_INPUT_HEIGHT,
) -> Roi:
    """Replay the official S100 ``mono_edgesam`` ROI publication exactly.

    This is the upstream chain from ``AiMsgManage::GetTargetRois`` through
    ``ResizeNV12Img``, ``GenScaleBox`` and the integer ROS ROI fields.  It is
    deliberately not an overlap heuristic: a non-canonical observed ROI is a
    different prompt and must be rejected.
    """

    if not isinstance(roi, Roi):
        raise S100PProductAdapterError("EdgeSAM prompt ROI must be a Roi")
    source_width = _positive_int(source_image_width, "EdgeSAM source image width")
    source_height = _positive_int(source_image_height, "EdgeSAM source image height")
    model_width = _positive_int(model_input_width, "EdgeSAM model input width")
    model_height = _positive_int(model_input_height, "EdgeSAM model input height")
    if (model_width, model_height) != (
        S100P_EDGESAM_MODEL_INPUT_WIDTH,
        S100P_EDGESAM_MODEL_INPUT_HEIGHT,
    ):
        raise S100PProductAdapterError(
            "EdgeSAM model input shape is not the frozen 512x512 S100 contract"
        )

    x = _roi_integer(roi.x_offset, "EdgeSAM prompt ROI x_offset")
    y = _roi_integer(roi.y_offset, "EdgeSAM prompt ROI y_offset")
    width = _roi_integer(roi.width, "EdgeSAM prompt ROI width")
    height = _roi_integer(roi.height, "EdgeSAM prompt ROI height")
    right = x + width
    bottom = y + height
    if (
        x < 0
        or y < 0
        or width <= 0
        or height <= 0
        or right > source_width
        or bottom > source_height
    ):
        raise S100PProductAdapterError("EdgeSAM prompt ROI is outside the source image")

    ratio_width = _float32(
        _float32(source_width, "EdgeSAM source image width")
        / _float32(model_width, "EdgeSAM model input width"),
        "EdgeSAM width resize ratio",
    )
    ratio_height = _float32(
        _float32(source_height, "EdgeSAM source image height")
        / _float32(model_height, "EdgeSAM model input height"),
        "EdgeSAM height resize ratio",
    )
    ratio = max(ratio_width, ratio_height)
    if ratio <= 0.0:
        raise S100PProductAdapterError("EdgeSAM resize ratio must be positive")
    if ratio == ratio_width:
        resized_width = model_width
        resized_height = int(
            _float32(
                _float32(source_height, "EdgeSAM source image height") / ratio,
                "EdgeSAM resized height",
            )
        )
    else:
        resized_width = int(
            _float32(
                _float32(source_width, "EdgeSAM source image width") / ratio,
                "EdgeSAM resized width",
            )
        )
        resized_height = model_height
    remainder = resized_width % 16
    if remainder:
        resized_width -= remainder
        if resized_width <= 0:
            raise S100PProductAdapterError("EdgeSAM aligned resize width is invalid")
        ratio = _float32(
            _float32(source_width, "EdgeSAM source image width")
            / _float32(resized_width, "EdgeSAM aligned resize width"),
            "EdgeSAM aligned resize ratio",
        )
        resized_height = int(
            _float32(
                _float32(source_height, "EdgeSAM source image height") / ratio,
                "EdgeSAM aligned resized height",
            )
        )
    if resized_height % 2:
        resized_height -= 1
    if (
        resized_width <= 0
        or resized_height <= 0
        or resized_width > model_width
        or resized_height > model_height
    ):
        raise S100PProductAdapterError("EdgeSAM official resize geometry is invalid")

    # Official AiMsgManage makes left/top even and right/bottom odd before the
    # prompt reaches GenScaleBox.  Keep the product boundary fail-closed when
    # that produces a degenerate source ROI.
    left = x + (x % 2)
    top = y + (y % 2)
    right -= 0 if right % 2 else 1
    bottom -= 0 if bottom % 2 else 1
    if (
        left < 0
        or top < 0
        or right > source_width
        or bottom > source_height
        or right <= left
        or bottom <= top
    ):
        raise S100PProductAdapterError(
            "EdgeSAM parity canonicalization makes the ROI invalid"
        )

    q_left, q_top, q_right, q_bottom = (
        _float32(
            _float32(endpoint, "EdgeSAM canonical endpoint") / ratio,
            "EdgeSAM model-space endpoint",
        )
        for endpoint in (left, top, right, bottom)
    )
    # ``src/s100/edgesam_node.cpp`` clamps the floating model-space Bbox
    # before it assigns the four integer ROI fields.  The assignment then
    # truncates the offset and the (right - left)/(bottom - top) extents
    # independently; only after that does it multiply those integer fields
    # back by the float ratio.  Do not reject a valid source ROI merely
    # because the official node takes this boundary-clamp path.
    q_left = max(q_left, 0.0)
    q_top = max(q_top, 0.0)
    q_right = min(q_right, float(model_width - 1))
    q_bottom = min(q_bottom, float(model_height - 1))
    pre_x = int(floor(q_left))
    pre_y = int(floor(q_top))
    pre_width = int(
        floor(_float32(q_right - q_left, "EdgeSAM model-space width"))
    )
    pre_height = int(
        floor(_float32(q_bottom - q_top, "EdgeSAM model-space height"))
    )
    if pre_x < 0 or pre_y < 0 or pre_width <= 0 or pre_height <= 0:
        raise S100PProductAdapterError("EdgeSAM model-space ROI is invalid")

    output = Roi(
        float(
            int(
                _float32(
                    _float32(pre_x, "EdgeSAM pre-ROI x") * ratio,
                    "EdgeSAM output ROI x",
                )
            )
        ),
        float(
            int(
                _float32(
                    _float32(pre_y, "EdgeSAM pre-ROI y") * ratio,
                    "EdgeSAM output ROI y",
                )
            )
        ),
        float(
            int(
                _float32(
                    _float32(pre_width, "EdgeSAM pre-ROI width") * ratio,
                    "EdgeSAM output ROI width",
                )
            )
        ),
        float(
            int(
                _float32(
                    _float32(pre_height, "EdgeSAM pre-ROI height") * ratio,
                    "EdgeSAM output ROI height",
                )
            )
        ),
    )
    if (
        output.width <= 0.0
        or output.height <= 0.0
        or output.x_offset < 0.0
        or output.y_offset < 0.0
        or output.x_offset + output.width > source_width
        or output.y_offset + output.height > source_height
    ):
        raise S100PProductAdapterError(
            "EdgeSAM canonical output ROI is outside the source image"
        )
    return output


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
    expected_rois = tuple(
        canonicalize_official_s100p_edgesam_roi(
            row.roi,
            source_image_width=batch.image_width,
            source_image_height=batch.image_height,
        )
        for row in batch.prompts
    )
    output_classes = tuple(
        _nonempty_string(value, "EdgeSAM output prompt class")
        for value in output_prompt_class_ids
    )
    expected_classes = tuple(row.class_id for row in batch.prompts)
    if len(output_rois) != len(expected_rois):
        raise S100PProductAdapterError("EdgeSAM output ROI count does not match prompt batch")
    if output_classes != expected_classes:
        raise S100PProductAdapterError("EdgeSAM output class order does not match prompt batch")
    if output_rois != expected_rois:
        raise S100PProductAdapterError("EdgeSAM output ROIs do not exactly match official S100 canonical prompts")

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
