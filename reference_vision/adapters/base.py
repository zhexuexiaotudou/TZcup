from __future__ import annotations

from collections.abc import Callable, Iterable

from reference_vision.contracts import Detection, DetectorFrame, Track, TrackerFrame


class CallableDetectorAdapter:
    def __init__(
        self,
        *,
        model_id: str,
        predictor: Callable[[object, tuple[str, ...]], Iterable[dict]],
        label_map: dict[str, str] | None = None,
    ):
        if predictor is None:
            raise RuntimeError("official reference dependency is unavailable")
        self.model_id = model_id
        self.predictor = predictor
        self.label_map = label_map or {}

    def detect(self, frame_id: str, image: object, prompts: tuple[str, ...] = ()) -> DetectorFrame:
        detections = []
        for record in self.predictor(image, prompts):
            source_label = str(record["label"])
            detections.append(Detection(
                label=self.label_map.get(source_label, source_label),
                score=float(record["score"]),
                bbox_xyxy=tuple(float(value) for value in record["bbox_xyxy"]),
                mask=record.get("mask"),
                source_model=self.model_id,
            ))
        result = DetectorFrame(frame_id, tuple(detections))
        result.validate()
        return result


class CallableTrackerAdapter:
    def __init__(self, *, model_id: str, predictor: Callable[[object], Iterable[dict]]):
        if predictor is None:
            raise RuntimeError("official reference dependency is unavailable")
        self.model_id = model_id
        self.predictor = predictor

    def track(self, frame_id: str, seeded_frame: object) -> TrackerFrame:
        tracks = tuple(Track(
            track_id=str(record["track_id"]),
            score=float(record["score"]),
            age=int(record["age"]),
            lost_count=int(record.get("lost_count", 0)),
            bbox_xyxy=(tuple(float(value) for value in record["bbox_xyxy"]) if record.get("bbox_xyxy") else None),
            mask=record.get("mask"),
        ) for record in self.predictor(seeded_frame))
        result = TrackerFrame(frame_id, tracks)
        result.validate()
        return result
