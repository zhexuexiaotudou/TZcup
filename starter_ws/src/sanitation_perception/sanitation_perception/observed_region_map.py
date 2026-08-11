"""History of regions actually observed by the moving onboard camera."""

from __future__ import annotations

from sanitation_perception.camera_frustum_model import FrustumSweep


class ObservedRegionMap:
    def __init__(self, mission_id: str, maximum_history: int = 4096):
        if not mission_id:
            raise ValueError("mission_id is required")
        if maximum_history < 1:
            raise ValueError("maximum_history must be positive")
        self.mission_id = mission_id
        self.maximum_history = maximum_history
        self._sweeps: list[FrustumSweep] = []

    @property
    def sweeps(self) -> tuple[FrustumSweep, ...]:
        return tuple(self._sweeps)

    def record(self, sweep: FrustumSweep) -> None:
        sweep.validate()
        if sweep.mission_id != self.mission_id:
            raise ValueError("frustum sweep belongs to another mission")
        if self._sweeps and sweep.stamp_ns < self._sweeps[-1].stamp_ns:
            raise ValueError("frustum sweeps must be chronological")
        self._sweeps.append(sweep)
        if len(self._sweeps) > self.maximum_history:
            del self._sweeps[: len(self._sweeps) - self.maximum_history]

    def visible_at(
        self,
        *,
        x_m: float,
        y_m: float,
        stamp_ns: int,
        camera_frame_id: str,
        image_frame_id: str,
    ) -> bool:
        return any(
            sweep.stamp_ns == stamp_ns
            and sweep.camera_frame_id == camera_frame_id
            and sweep.image_frame_id == image_frame_id
            and sweep.contains(x_m, y_m)
            for sweep in reversed(self._sweeps)
        )

    def reobserved_after(
        self,
        *,
        x_m: float,
        y_m: float,
        after_stamp_ns: int,
        up_to_stamp_ns: int,
    ) -> bool:
        """Return whether a later camera sweep covered a mapped location."""
        return any(
            after_stamp_ns < sweep.stamp_ns <= up_to_stamp_ns
            and sweep.contains(x_m, y_m)
            for sweep in reversed(self._sweeps)
        )

    def to_records(self) -> list[dict]:
        return [sweep.to_record() for sweep in self._sweeps]

    @classmethod
    def from_records(cls, mission_id: str, records: list[dict]) -> "ObservedRegionMap":
        observed = cls(mission_id)
        for record in records:
            observed.record(FrustumSweep(**record))
        return observed
