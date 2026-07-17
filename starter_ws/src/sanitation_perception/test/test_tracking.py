import pytest

from sanitation_perception.tracking import TargetTracker


def detection(source="onnxruntime"):
    return {"class_id": "plastic_bottle", "target_type": "discrete", "cleaning_policy": "spot_clean", "x_m": 1.0, "y_m": 2.0, "confidence": 0.99, "covariance_trace": 0.002, "source_backend": source}


def test_multiframe_confirmation_and_no_duplicate_task():
    tracker = TargetTracker(confirmation_observations=3)
    for index in range(3):
        tracks = tracker.update([detection()], now=index * 0.1)
    assert len(tracks) == 1
    assert tracks[0].observation_count == 3
    assert tracks[0].state == "CONFIRMED"
    tracker.transition(tracks[0].uuid, "QUEUED")
    assert tracker.update([detection()], now=0.4)[0].state == "QUEUED"


def test_ground_truth_cannot_enter_decision_tracker():
    with pytest.raises(ValueError, match="ground-truth"):
        TargetTracker().update([detection("ground_truth")])
