import pytest

from sanitation_perception.tracking import TargetTracker
from sanitation_spot_cleaning.coordinator import Preflight, SpotCleaningCoordinator


def confirmed_tracker(source="onnxruntime"):
    tracker = TargetTracker(confirmation_observations=1)
    tracker.update([{"class_id": "leaf_pile", "target_type": "area", "cleaning_policy": "local_coverage", "x_m": 1.0, "y_m": 0.0, "confidence": 0.99, "covariance_trace": 0.002, "source_backend": source}], now=0.0)
    return tracker


def test_deferred_cleaning_requires_preflight_and_resumes_coverage():
    coordinator = SpotCleaningCoordinator(confirmed_tracker())
    track = coordinator.queue_confirmed()[0]
    rejected = coordinator.clean(track.uuid, Preflight(False, True, True, 0.3, 0.002, 0.1))
    assert rejected["result"] == "deferred"
    event = coordinator.clean(track.uuid, Preflight(True, True, True, 0.3, 0.002, 0.1), 0.95)
    assert event["result"] == "cleaned"
    assert event["brush_final"] is False
    assert coordinator.coverage_resumed


def test_gt_track_is_rejected_before_cleaning():
    with pytest.raises(ValueError, match="ground-truth"):
        confirmed_tracker("ground_truth")
