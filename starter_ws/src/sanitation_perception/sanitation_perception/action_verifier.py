"""Independent, fail-closed authorization between classification and scheduling."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import math


class ActionVerdict(str, Enum):
    ACCEPT = "ACCEPT"
    OBSERVE_AGAIN = "OBSERVE_AGAIN"
    DEFER = "DEFER"
    REJECT = "REJECT"


@dataclass(frozen=True)
class ActionVerifierConfig:
    actionable_classes: tuple[str, ...]
    minimum_class_confidence: float
    maximum_background_probability: float
    reject_background_probability: float
    minimum_observations: int
    defer_after_observations: int
    maximum_covariance_trace: float
    maximum_map_disagreement_m: float
    minimum_view_separation_rad: float
    maximum_reobserve_count: int

    @classmethod
    def from_pipeline_manifest(cls, manifest: dict) -> "ActionVerifierConfig":
        try:
            values = dict(manifest["runtime"]["action_verifier"])
            values["actionable_classes"] = tuple(values["actionable_classes"])
            config = cls(**values)
        except (KeyError, TypeError) as exc:
            raise ValueError("pipeline action_verifier contract is incomplete") from exc
        config.validate()
        return config

    def validate(self) -> None:
        if not self.actionable_classes or len(set(self.actionable_classes)) != len(self.actionable_classes):
            raise ValueError("actionable classes must be a non-empty unique list")
        probabilities = (
            self.minimum_class_confidence,
            self.maximum_background_probability,
            self.reject_background_probability,
        )
        if any(not 0.0 <= float(value) <= 1.0 for value in probabilities):
            raise ValueError("action-verifier probability thresholds must be in [0, 1]")
        if self.maximum_background_probability >= self.reject_background_probability:
            raise ValueError("background accept threshold must be below reject threshold")
        if self.minimum_observations < 2:
            raise ValueError("action verifier requires multi-frame evidence")
        if self.defer_after_observations < self.minimum_observations:
            raise ValueError("defer threshold cannot precede minimum observations")
        if self.maximum_covariance_trace <= 0.0 or self.maximum_map_disagreement_m <= 0.0:
            raise ValueError("map safety thresholds must be positive")
        if not 0.0 <= self.minimum_view_separation_rad <= math.pi:
            raise ValueError("minimum view separation must be in [0, pi]")
        if self.maximum_reobserve_count < 0 or self.maximum_reobserve_count > 2:
            raise ValueError("product contract permits at most two re-observations")


@dataclass(frozen=True)
class ActionVerification:
    track_uuid: str
    verdict: ActionVerdict
    reasons: tuple[str, ...]
    reobserve_count: int
    checks: dict[str, bool]

    def to_record(self) -> dict:
        payload = asdict(self)
        payload["verdict"] = self.verdict.value
        payload["reasons"] = list(self.reasons)
        return payload


def _angular_separation(values) -> float:
    normalized = [float(value) % math.tau for value in values]
    return max(
        (
            abs((first - second + math.pi) % math.tau - math.pi)
            for index, first in enumerate(normalized)
            for second in normalized[index + 1 :]
        ),
        default=0.0,
    )


class ProductActionVerifier:
    """Authorize only accumulated online evidence; never infer actuator commands."""

    FORBIDDEN_SOURCE_TOKENS = (
        "ground_truth",
        "gazebo_registry",
        "evaluation_registry",
    )

    def __init__(self, config: ActionVerifierConfig) -> None:
        config.validate()
        self.config = config
        self.reobserve_counts: dict[str, int] = {}
        self.last_verdicts: dict[str, ActionVerdict] = {}

    def evaluate(self, track, mapped_target, *, depth_valid: bool) -> ActionVerification:
        source = str(track.source_backend).lower()
        if any(token in source for token in self.FORBIDDEN_SOURCE_TOKENS):
            raise ValueError("GT control violation: ActionVerifier source rejected")
        posterior = track.class_posterior
        background_probability = float(posterior.get("background", 0.0))
        map_disagreement_m = math.hypot(
            float(track.x_m) - float(mapped_target.map_x_m),
            float(track.y_m) - float(mapped_target.map_y_m),
        )
        checks = {
            "class_confidence": (
                str(track.class_id) != "background"
                and float(track.class_confidence)
                >= self.config.minimum_class_confidence
            ),
            "background_or_unknown": (
                background_probability
                <= self.config.maximum_background_probability
            ),
            "multi_frame_consistency": (
                int(track.observation_count) >= self.config.minimum_observations
            ),
            "depth_valid": bool(depth_valid),
            "projection_covariance": (
                float(track.covariance_trace)
                <= self.config.maximum_covariance_trace
            ),
            "track_persistence": (
                int(mapped_target.observation_count)
                >= self.config.minimum_observations
            ),
            "map_consistency": (
                map_disagreement_m <= self.config.maximum_map_disagreement_m
            ),
            "multi_view_agreement": (
                self.config.minimum_view_separation_rad == 0.0
                or _angular_separation(mapped_target.view_directions_rad)
                >= self.config.minimum_view_separation_rad
            ),
        }
        if str(track.class_id) not in self.config.actionable_classes:
            verdict = ActionVerdict.REJECT
            reasons = ("unsupported_or_unknown_class",)
        elif background_probability >= self.config.reject_background_probability:
            verdict = ActionVerdict.REJECT
            reasons = ("background_or_unknown",)
        elif all(checks.values()):
            verdict = ActionVerdict.ACCEPT
            reasons = ()
        else:
            reasons = tuple(name for name, passed in checks.items() if not passed)
            current = max(
                self.reobserve_counts.get(str(track.uuid), 0),
                int(getattr(mapped_target, "reobserve_count", 0)),
            )
            self.reobserve_counts[str(track.uuid)] = current
            can_reobserve = (
                current < self.config.maximum_reobserve_count
                and int(track.observation_count)
                < self.config.defer_after_observations
            )
            verdict = ActionVerdict.OBSERVE_AGAIN if can_reobserve else ActionVerdict.DEFER
            if can_reobserve:
                self.reobserve_counts[str(track.uuid)] = current + 1
        self.last_verdicts[str(track.uuid)] = verdict
        return ActionVerification(
            track_uuid=str(track.uuid),
            verdict=verdict,
            reasons=reasons,
            reobserve_count=self.reobserve_counts.get(str(track.uuid), 0),
            checks=checks,
        )
