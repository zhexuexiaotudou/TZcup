from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    bbox_xyxy: tuple[float, float, float, float]
    mask: object | None
    source_model: str

    def validate(self) -> None:
        if not self.label or not self.source_model:
            raise ValueError("detection label and source model are required")
        if not 0.0 <= self.score <= 1.0:
            raise ValueError("detection score must be in [0, 1]")
        x1, y1, x2, y2 = self.bbox_xyxy
        if not all(math.isfinite(value) for value in self.bbox_xyxy) or x2 <= x1 or y2 <= y1:
            raise ValueError("bbox_xyxy must be a finite positive rectangle")

    def to_record(self) -> dict:
        self.validate()
        payload = asdict(self)
        payload["bbox_xyxy"] = list(self.bbox_xyxy)
        return payload


@dataclass(frozen=True)
class DetectorFrame:
    frame_id: str
    detections: tuple[Detection, ...]

    def validate(self) -> None:
        if not self.frame_id:
            raise ValueError("frame_id is required")
        for detection in self.detections:
            detection.validate()

    def to_record(self) -> dict:
        self.validate()
        return {"frame_id": self.frame_id, "detections": [item.to_record() for item in self.detections]}


@dataclass(frozen=True)
class Track:
    track_id: str
    score: float
    age: int
    lost_count: int
    bbox_xyxy: tuple[float, float, float, float] | None = None
    mask: object | None = None

    def validate(self) -> None:
        if not self.track_id or self.age < 1 or self.lost_count < 0:
            raise ValueError("track identity, age, and lost count are invalid")
        if not 0.0 <= self.score <= 1.0:
            raise ValueError("track score must be in [0, 1]")
        if self.bbox_xyxy is None and self.mask is None:
            raise ValueError("track requires a bbox or mask")


@dataclass(frozen=True)
class TrackerFrame:
    frame_id: str
    tracks: tuple[Track, ...]

    def validate(self) -> None:
        if not self.frame_id:
            raise ValueError("frame_id is required")
        for track in self.tracks:
            track.validate()
