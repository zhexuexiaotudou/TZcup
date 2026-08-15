"""Fail-closed lifecycle and watchdog state for product perception."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field


LIFECYCLE_STATES = {"UNCONFIGURED", "INACTIVE", "ACTIVE", "DEGRADED", "ERROR"}


@dataclass(frozen=True)
class WatchdogConfig:
    camera_stale_ms: float
    maximum_latency_ms: float
    sustained_latency_samples: int
    maximum_consecutive_tf_errors: int
    maximum_consecutive_session_errors: int

    @classmethod
    def from_pipeline_manifest(cls, manifest: dict) -> "WatchdogConfig":
        try:
            config = cls(**manifest["runtime"]["watchdog"])
        except (KeyError, TypeError) as exc:
            raise ValueError("pipeline watchdog configuration is incomplete") from exc
        if min(
            config.camera_stale_ms,
            config.maximum_latency_ms,
            config.sustained_latency_samples,
            config.maximum_consecutive_tf_errors,
            config.maximum_consecutive_session_errors,
        ) <= 0:
            raise ValueError("watchdog thresholds must be positive")
        return config


@dataclass
class ProductHealth:
    config: WatchdogConfig
    state: str = "UNCONFIGURED"
    reason: str = "not_configured"
    last_frame_monotonic_s: float | None = None
    last_inference_monotonic_s: float | None = None
    consecutive_tf_errors: int = 0
    consecutive_session_errors: int = 0
    oom_count: int = 0
    latency_ms: deque = field(default_factory=deque)

    def transition(self, state: str, reason: str) -> None:
        if state not in LIFECYCLE_STATES:
            raise ValueError(f"unknown product perception state: {state}")
        allowed = {
            "UNCONFIGURED": {"INACTIVE", "ERROR"},
            "INACTIVE": {"ACTIVE", "UNCONFIGURED", "ERROR"},
            "ACTIVE": {"INACTIVE", "DEGRADED", "ERROR"},
            "DEGRADED": {"INACTIVE", "ACTIVE", "ERROR"},
            "ERROR": {"UNCONFIGURED"},
        }
        if state not in allowed[self.state]:
            raise ValueError(f"invalid lifecycle transition {self.state}->{state}")
        self.state = state
        self.reason = reason

    def record_frame(self, now_s: float) -> None:
        self.last_frame_monotonic_s = float(now_s)

    def record_inference(self, now_s: float, latency_ms: float) -> None:
        self.last_inference_monotonic_s = float(now_s)
        self.consecutive_session_errors = 0
        self.latency_ms.append(float(latency_ms))
        while len(self.latency_ms) > self.config.sustained_latency_samples:
            self.latency_ms.popleft()

    def record_tf_success(self) -> None:
        self.consecutive_tf_errors = 0

    def record_tf_error(self) -> None:
        self.consecutive_tf_errors += 1

    def record_session_error(self, *, oom: bool = False) -> None:
        self.consecutive_session_errors += 1
        self.oom_count += int(oom)
        if oom:
            self.state = "ERROR"
            self.reason = "inference_oom"

    def evaluate(self, now_s: float) -> str:
        if self.state not in {"ACTIVE", "DEGRADED"}:
            return self.state
        if self.last_frame_monotonic_s is None or (
            float(now_s) - self.last_frame_monotonic_s
        ) * 1000.0 > self.config.camera_stale_ms:
            self.state, self.reason = "DEGRADED", "camera_stale"
        elif self.consecutive_tf_errors >= self.config.maximum_consecutive_tf_errors:
            self.state, self.reason = "DEGRADED", "tf_unavailable"
        elif self.consecutive_session_errors >= self.config.maximum_consecutive_session_errors:
            self.state, self.reason = "ERROR", "inference_session_failure"
        elif len(self.latency_ms) == self.config.sustained_latency_samples and all(
            value > self.config.maximum_latency_ms for value in self.latency_ms
        ):
            self.state, self.reason = "DEGRADED", "latency_sustained_over_limit"
        elif self.state == "DEGRADED":
            self.state, self.reason = "ACTIVE", "watchdog_recovered"
        return self.state

    @property
    def perception_spot_clean_allowed(self) -> bool:
        return self.state == "ACTIVE"

    def snapshot(self, now_s: float) -> dict:
        self.evaluate(now_s)
        last_age_ms = None
        if self.last_inference_monotonic_s is not None:
            last_age_ms = (float(now_s) - self.last_inference_monotonic_s) * 1000.0
        return {
            "state": self.state,
            "reason": self.reason,
            "perception_spot_clean_allowed": self.perception_spot_clean_allowed,
            "consecutive_tf_errors": self.consecutive_tf_errors,
            "consecutive_session_errors": self.consecutive_session_errors,
            "oom_count": self.oom_count,
            "last_inference_age_ms": last_age_ms,
        }
