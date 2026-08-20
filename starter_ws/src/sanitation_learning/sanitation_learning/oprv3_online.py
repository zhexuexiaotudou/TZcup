"""OPRV3 evaluator-side online encounter and actionable-window contracts.

Ground truth is accepted only by :func:`evaluate_eventual_metrics`.  Production
observations deliberately contain neither target identity nor target pose; the
evaluator supplies an independent association table after inference.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import math
from typing import Iterable, Mapping


class GateKind(str, Enum):
    OFFICIAL = "OFFICIAL_GATE"
    INTERNAL_DIAGNOSTIC = "INTERNAL_DIAGNOSTIC_GATE"
    ONLINE_PRODUCT = "ONLINE_PRODUCT_GATE"


@dataclass(frozen=True)
class ClassWindow:
    class_id: str
    minimum_actionable_range_m: float
    maximum_actionable_range_m: float
    minimum_visible_frames: int
    minimum_visibility_ratio: float
    minimum_depth_valid_ratio: float = 0.8

    def __post_init__(self) -> None:
        if not 0.0 <= self.minimum_visibility_ratio <= 1.0:
            raise ValueError("minimum_visibility_ratio must be within [0, 1]")
        if not 0.0 <= self.minimum_depth_valid_ratio <= 1.0:
            raise ValueError("minimum_depth_valid_ratio must be within [0, 1]")
        if self.minimum_visible_frames < 1:
            raise ValueError("minimum_visible_frames must be positive")
        if self.maximum_actionable_range_m <= self.minimum_actionable_range_m:
            raise ValueError(f"empty actionable window for {self.class_id}")


@dataclass(frozen=True)
class ThresholdPolicy:
    observation_threshold: float
    track_confirmation_threshold: float
    clean_action_threshold: float
    confirmation_observations: int

    def __post_init__(self) -> None:
        if not (
            0.0 <= self.observation_threshold
            < self.track_confirmation_threshold
            < self.clean_action_threshold
            <= 1.0
        ):
            raise ValueError("observation, confirmation and action thresholds must be strictly ordered")
        if self.confirmation_observations < 2:
            raise ValueError("confirmation requires multiple observations")

    def disposition(self, score: float, observation_count: int) -> str:
        if score < self.observation_threshold:
            return "discard"
        if score < self.track_confirmation_threshold or observation_count < self.confirmation_observations:
            return "non_actionable_observation"
        if score < self.clean_action_threshold:
            return "confirmed_track_not_cleanable"
        return "clean_action_eligible"


@dataclass(frozen=True)
class ObservableTargetEncounter:
    """One evaluator-only GT target state at one frame timestamp."""

    target_id: str
    class_id: str
    stamp_s: float
    in_camera_frustum: bool
    visibility_ratio: float
    depth_valid_ratio: float
    distance_m: float | None
    bbox_short_side_px: float | None = None
    mask_area_px: float | None = None
    occluded: bool = False


@dataclass(frozen=True)
class ProductionObservation:
    """Output visible to production; it has no GT identity or GT coordinates."""

    observation_id: str
    stamp_s: float
    predicted_class_id: str
    score: float
    track_id: str | None
    track_confirmed: bool
    predicted_map_xy_m: tuple[float, float] | None

    def __post_init__(self) -> None:
        if not 0.0 <= self.score <= 1.0:
            raise ValueError("score must be within [0, 1]")


@dataclass(frozen=True)
class EvaluatorMatch:
    """Independent evaluator association; never passed back to production."""

    observation_id: str
    target_id: str | None
    localization_error_m: float | None


def validate_gate_provenance(payload: Mapping) -> None:
    if payload.get("schema_version") != 1:
        raise ValueError("gate provenance schema_version must be 1")
    gates = payload.get("gates")
    if not isinstance(gates, list) or not gates:
        raise ValueError("gate provenance must contain gates")
    identifiers: set[str] = set()
    for gate in gates:
        gate_id = gate.get("id")
        if not gate_id or gate_id in identifiers:
            raise ValueError(f"missing or duplicate gate id: {gate_id!r}")
        identifiers.add(gate_id)
        GateKind(gate.get("kind"))
        if gate.get("source_type") not in {
            "competition",
            "existing_project_internal",
            "new_online_product_protocol",
        }:
            raise ValueError(f"invalid source_type for {gate_id}")
        if not gate.get("source_paths"):
            raise ValueError(f"source_paths required for {gate_id}")
        if gate["kind"] == GateKind.OFFICIAL.value and gate.get("verified") is not True:
            raise ValueError(f"unverified material cannot be an OFFICIAL_GATE: {gate_id}")


def production_schema_fields() -> set[str]:
    return set(ProductionObservation.__dataclass_fields__)


def _longest_consecutive_stamps(rows: list[ObservableTargetEncounter], frame_period_s: float) -> int:
    if not rows:
        return 0
    maximum = current = 1
    tolerance = frame_period_s * 1.5
    for previous, current_row in zip(rows, rows[1:]):
        if 0.0 < current_row.stamp_s - previous.stamp_s <= tolerance:
            current += 1
        else:
            current = 1
        maximum = max(maximum, current)
    return maximum


def _safe_ratio(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def evaluate_eventual_metrics(
    encounters: Iterable[ObservableTargetEncounter],
    observations: Iterable[ProductionObservation],
    matches: Iterable[EvaluatorMatch],
    class_windows: Mapping[str, ClassWindow],
    *,
    frame_rate_hz: float,
    map_localization_tolerance_m: float = 0.10,
) -> dict:
    """Evaluate every GT target without post-hoc target-set filtering."""

    if frame_rate_hz <= 0.0:
        raise ValueError("frame_rate_hz must be positive")
    encounter_rows = list(encounters)
    if not encounter_rows:
        raise ValueError("at least one evaluator GT encounter row is required")
    grouped: dict[str, list[ObservableTargetEncounter]] = {}
    for row in encounter_rows:
        if row.class_id not in class_windows:
            raise ValueError(f"missing frozen class window for {row.class_id}")
        grouped.setdefault(row.target_id, []).append(row)

    observation_rows = list(observations)
    observation_by_id = {row.observation_id: row for row in observation_rows}
    if len(observation_by_id) != len(observation_rows):
        raise ValueError("production observation_id values must be unique")
    match_by_target: dict[str, list[tuple[ProductionObservation, EvaluatorMatch]]] = {}
    seen_match_ids: set[str] = set()
    for match in matches:
        if match.observation_id in seen_match_ids:
            raise ValueError(f"duplicate evaluator match for {match.observation_id}")
        seen_match_ids.add(match.observation_id)
        observation = observation_by_id.get(match.observation_id)
        if observation is None:
            raise ValueError(f"match references unknown observation {match.observation_id}")
        if match.target_id is not None:
            if match.target_id not in grouped:
                raise ValueError(f"match references unknown GT target {match.target_id}")
            match_by_target.setdefault(match.target_id, []).append((observation, match))

    target_records: list[dict] = []
    frame_period_s = 1.0 / frame_rate_hz
    for target_id, rows in sorted(grouped.items()):
        rows.sort(key=lambda row: row.stamp_s)
        class_ids = {row.class_id for row in rows}
        if len(class_ids) != 1:
            raise ValueError(f"GT target class changed for {target_id}")
        class_id = next(iter(class_ids))
        window = class_windows[class_id]
        frustum_rows = [row for row in rows if row.in_camera_frustum]
        visible_rows = [
            row for row in frustum_rows
            if not row.occluded
            and row.visibility_ratio >= window.minimum_visibility_ratio
            and row.depth_valid_ratio >= window.minimum_depth_valid_ratio
            and row.distance_m is not None
            and math.isfinite(row.distance_m)
        ]
        actionable_rows = [
            row for row in visible_rows
            if window.minimum_actionable_range_m <= float(row.distance_m) <= window.maximum_actionable_range_m
        ]
        consecutive = _longest_consecutive_stamps(actionable_rows, frame_period_s)
        entered = consecutive >= window.minimum_visible_frames
        actionable_stamps = {row.stamp_s for row in actionable_rows} if entered else set()
        target_matches = [
            pair for pair in match_by_target.get(target_id, [])
            if any(abs(pair[0].stamp_s - stamp) <= frame_period_s * 0.51 for stamp in actionable_stamps)
        ]
        detected = bool(target_matches)
        correct = any(observation.predicted_class_id == class_id for observation, _ in target_matches)
        confirmed = any(observation.track_confirmed for observation, _ in target_matches)
        localized = any(
            match.localization_error_m is not None
            and math.isfinite(match.localization_error_m)
            and match.localization_error_m <= map_localization_tolerance_m
            for _, match in target_matches
        )
        clean_opportunity_missed = entered and not (detected and correct and confirmed and localized)
        if not frustum_rows:
            partition = "never_in_camera_frustum"
        elif not visible_rows:
            partition = "occluded_entirely"
        elif entered:
            partition = "entered_actionable_window"
        else:
            partition = "visible_but_never_actionable"
        target_records.append({
            "target_id": target_id,
            "class_id": class_id,
            "partition": partition,
            "entered_actionable_window": entered,
            "maximum_consecutive_actionable_frames": consecutive,
            "eventual_detection": detected,
            "eventual_classification": correct,
            "eventual_track_confirmation": confirmed,
            "eventual_map_localization": localized,
            "clean_opportunity_missed": clean_opportunity_missed,
        })

    all_ids = {record["target_id"] for record in target_records}
    partition_ids = {
        record["target_id"]
        for record in target_records
        if record["partition"] in {
            "never_in_camera_frustum", "occluded_entirely",
            "entered_actionable_window", "visible_but_never_actionable",
        }
    }
    if partition_ids != all_ids:
        raise AssertionError("GT partition is not exhaustive")
    entered_records = [row for row in target_records if row["entered_actionable_window"]]
    detected_records = [row for row in entered_records if row["eventual_detection"]]
    classified_records = [row for row in entered_records if row["eventual_classification"]]
    confirmed_records = [row for row in entered_records if row["eventual_track_confirmation"]]
    localized_records = [row for row in entered_records if row["eventual_map_localization"]]
    missed_records = [row for row in entered_records if row["clean_opportunity_missed"]]
    counts = {
        "all_gt_targets": len(target_records),
        "never_in_camera_frustum": sum(row["partition"] == "never_in_camera_frustum" for row in target_records),
        "occluded_entirely": sum(row["partition"] == "occluded_entirely" for row in target_records),
        "visible_but_never_actionable": sum(row["partition"] == "visible_but_never_actionable" for row in target_records),
        "entered_actionable_window": len(entered_records),
        "detected_in_window": len(detected_records),
        "missed_in_window": len(entered_records) - len(detected_records),
        "clean_opportunity_missed": len(missed_records),
    }
    return {
        "schema_version": 1,
        "protocol": "OPRV3",
        "gt_boundary": {
            "production_observation_has_gt_identity": False,
            "production_observation_has_gt_coordinates": False,
            "association_owner": "independent_evaluator",
        },
        "class_windows": {name: asdict(window) for name, window in sorted(class_windows.items())},
        "counts": counts,
        "metrics": {
            "eventual_detection_recall": _safe_ratio(len(detected_records), len(entered_records)),
            "eventual_correct_class_recall": _safe_ratio(len(classified_records), len(entered_records)),
            "eventual_track_confirmation_recall": _safe_ratio(len(confirmed_records), len(entered_records)),
            "eventual_map_localization_recall": _safe_ratio(len(localized_records), len(entered_records)),
            "actionable_window_miss_rate": _safe_ratio(len(entered_records) - len(detected_records), len(entered_records)),
            "clean_opportunity_miss_rate": _safe_ratio(len(missed_records), len(entered_records)),
        },
        "targets": target_records,
        "subset_audit": {
            "all_gt_target_ids": sorted(all_ids),
            "partition_is_exhaustive": partition_ids == all_ids,
            "model_result_used_to_define_eligibility": False,
        },
    }
