#!/usr/bin/env python3
"""Deterministic incremental-encoder quantization used by the formal vehicle."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence


TAU = 2.0 * math.pi


def encoder_count(position_rad: float, counts_per_revolution: int) -> int:
    """Return the nearest incremental count, with ties away from zero."""

    if counts_per_revolution <= 0:
        raise ValueError("counts_per_revolution must be positive")
    if not math.isfinite(position_rad):
        raise ValueError("position_rad must be finite")
    scaled = position_rad * counts_per_revolution / TAU
    return math.floor(scaled + 0.5) if scaled >= 0.0 else math.ceil(scaled - 0.5)


@dataclass(frozen=True)
class QuantizedEncoderSample:
    counts: tuple[int, ...]
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]


class EncoderGroupQuantizer:
    """Quantize an ordered group and derive velocity from count increments."""

    def __init__(self, joint_names: Sequence[str], counts_per_revolution: int) -> None:
        if not joint_names or len(set(joint_names)) != len(joint_names):
            raise ValueError("joint_names must be non-empty and unique")
        if counts_per_revolution <= 0:
            raise ValueError("counts_per_revolution must be positive")
        self.joint_names = tuple(joint_names)
        self.counts_per_revolution = int(counts_per_revolution)
        self._previous_counts: tuple[int, ...] | None = None
        self._previous_stamp_ns: int | None = None

    def sample(
        self,
        stamp_ns: int,
        positions_rad: Mapping[str, float],
    ) -> QuantizedEncoderSample:
        missing = [name for name in self.joint_names if name not in positions_rad]
        if missing:
            raise KeyError(f"missing encoder joints: {missing}")
        counts = tuple(
            encoder_count(float(positions_rad[name]), self.counts_per_revolution)
            for name in self.joint_names
        )
        step_rad = TAU / self.counts_per_revolution
        quantized_positions = tuple(count * step_rad for count in counts)

        velocities = tuple(0.0 for _ in counts)
        if (
            self._previous_counts is not None
            and self._previous_stamp_ns is not None
            and stamp_ns > self._previous_stamp_ns
        ):
            elapsed_s = (stamp_ns - self._previous_stamp_ns) * 1.0e-9
            velocities = tuple(
                (count - previous) * step_rad / elapsed_s
                for count, previous in zip(counts, self._previous_counts, strict=True)
            )

        # A non-monotonic clock is a new baseline.  This keeps reset/replay
        # deterministic and prevents a negative-time velocity impulse.
        self._previous_counts = counts
        self._previous_stamp_ns = int(stamp_ns)
        return QuantizedEncoderSample(counts, quantized_positions, velocities)

