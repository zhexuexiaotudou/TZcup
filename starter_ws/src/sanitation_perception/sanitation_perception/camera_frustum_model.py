"""Auditable planar camera-FOV model used to gate online target creation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class FrustumSweep:
    sweep_id: str
    mission_id: str
    stamp_ns: int
    camera_frame_id: str
    image_frame_id: str
    camera_x_m: float
    camera_y_m: float
    camera_yaw_rad: float
    horizontal_fov_rad: float
    minimum_range_m: float
    maximum_range_m: float

    def validate(self) -> None:
        if not self.sweep_id or not self.mission_id:
            raise ValueError("sweep_id and mission_id are required")
        if not self.camera_frame_id or not self.image_frame_id:
            raise ValueError("camera and image frame identifiers are required")
        if self.stamp_ns < 0:
            raise ValueError("stamp_ns must be non-negative")
        if not 0.0 < self.horizontal_fov_rad < math.tau:
            raise ValueError("horizontal FOV must be in (0, 2*pi)")
        if self.minimum_range_m < 0.0 or self.maximum_range_m <= self.minimum_range_m:
            raise ValueError("camera range is invalid")

    def contains(self, x_m: float, y_m: float) -> bool:
        distance = math.hypot(x_m - self.camera_x_m, y_m - self.camera_y_m)
        if distance < self.minimum_range_m or distance > self.maximum_range_m:
            return False
        bearing = math.atan2(y_m - self.camera_y_m, x_m - self.camera_x_m)
        delta = (bearing - self.camera_yaw_rad + math.pi) % math.tau - math.pi
        return abs(delta) <= self.horizontal_fov_rad * 0.5

    def to_record(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class CameraFrustumModel:
    horizontal_fov_rad: float
    minimum_range_m: float
    maximum_range_m: float

    def make_sweep(
        self,
        *,
        sweep_id: str,
        mission_id: str,
        stamp_ns: int,
        camera_frame_id: str,
        image_frame_id: str,
        camera_x_m: float,
        camera_y_m: float,
        camera_yaw_rad: float,
    ) -> FrustumSweep:
        sweep = FrustumSweep(
            sweep_id=sweep_id,
            mission_id=mission_id,
            stamp_ns=stamp_ns,
            camera_frame_id=camera_frame_id,
            image_frame_id=image_frame_id,
            camera_x_m=camera_x_m,
            camera_y_m=camera_y_m,
            camera_yaw_rad=camera_yaw_rad,
            horizontal_fov_rad=self.horizontal_fov_rad,
            minimum_range_m=self.minimum_range_m,
            maximum_range_m=self.maximum_range_m,
        )
        sweep.validate()
        return sweep
