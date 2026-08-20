import pytest

from sanitation_perception.tracker_v2 import ProductTrackerV2, TrackerV2Config


def config(**overrides):
    values = {
        "association_distance_m": 0.30,
        "close_recovery_distance_m": 0.10,
        "minimum_image_iou": 0.05,
        "maximum_observation_gap_s": 0.5,
        "occlusion_recovery_s": 2.0,
        "duplicate_distance_m": 0.08,
        "confirmation_observations": 3,
        "confirmation_class_posterior": 0.70,
        "confirmation_score_ema": 0.60,
        "score_ema_alpha": 0.5,
        "defer_after_observations": 4,
    }
    values.update(overrides)
    return TrackerV2Config(**values)


def detection(x=1.0, cls="plastic_bottle", confidence=0.9, source="onnxruntime"):
    return {
        "x_m": x,
        "y_m": 2.0,
        "covariance_trace": 0.002,
        "class_id": cls,
        "confidence": confidence,
        "class_probabilities": {
            "plastic_bottle": 0.8 if cls == "plastic_bottle" else 0.1,
            "metal_can": 0.8 if cls == "metal_can" else 0.1,
            "background": 0.1,
        },
        "bbox_xyxy": [10.0, 10.0, 30.0, 30.0],
        "source_backend": source,
    }


def test_class_agnostic_association_accumulates_posterior_and_ema():
    tracker = ProductTrackerV2(config())
    first = tracker.update([detection(cls="metal_can", confidence=0.4)], 0.0)[0]
    stable_uuid = first.uuid
    tracker.update([detection(x=1.02, confidence=0.9)], 0.1)
    track = tracker.update([detection(x=1.01, confidence=0.9)], 0.2)[0]
    assert track.uuid == stable_uuid
    assert track.class_id == "plastic_bottle"
    assert track.score_ema > 0.4
    assert track.state == "READY_FOR_VERIFICATION"


def test_time_distance_and_iou_gate_then_occlusion_recovery():
    tracker = ProductTrackerV2(config())
    track = tracker.update([detection()], 0.0)[0]
    tracker.update([], 0.6)
    assert track.state == "LOST"
    recovered = tracker.update([detection(x=1.02)], 1.0)[0]
    assert recovered.uuid == track.uuid
    assert recovered.state == "TENTATIVE"
    tracks = tracker.update([detection(x=2.0)], 1.1)
    assert len(tracks) == 2


def test_low_confidence_defers_and_never_directly_confirms():
    tracker = ProductTrackerV2(config(defer_after_observations=3))
    low = detection(confidence=0.2)
    low["class_probabilities"] = {"plastic_bottle": 0.4, "background": 0.6}
    for index in range(3):
        tracks = tracker.update([low], index * 0.1)
    assert tracks[0].state == "DEFERRED"


def test_duplicate_suppression_and_gt_rejection():
    tracker = ProductTrackerV2(
        config(association_distance_m=0.01, duplicate_distance_m=0.20)
    )
    tracks = tracker.update([detection(x=1.0), detection(x=1.1)], 0.0)
    assert len(tracks) == 1
    with pytest.raises(ValueError, match="ground-truth"):
        tracker.update([detection(source="ground_truth")], 0.1)


def test_config_is_manifest_driven():
    manifest = {"runtime": {"tracker_v2": config().__dict__}}
    assert TrackerV2Config.from_pipeline_manifest(manifest) == config()
    del manifest["runtime"]["tracker_v2"]["score_ema_alpha"]
    with pytest.raises(ValueError, match="incomplete"):
        TrackerV2Config.from_pipeline_manifest(manifest)


def test_area_polygon_and_physical_area_follow_the_latest_observation():
    tracker = ProductTrackerV2(config())
    area = detection(cls="leaf_pile")
    area["class_probabilities"] = {"leaf_pile": 0.9, "background": 0.1}
    area["target_type"] = "AREA"
    area["polygon_xy_m"] = ((0.0, 0.0), (1.0, 0.0), (1.0, 0.5))
    area["physical_area_m2"] = 0.25
    first = tracker.update([area], 0.0)[0]
    assert first.target_type == "AREA"
    assert first.physical_area_m2 == pytest.approx(0.25)
    area["polygon_xy_m"] = ((0.0, 0.0), (1.2, 0.0), (1.2, 0.5))
    area["physical_area_m2"] = 0.30
    updated = tracker.update([area], 0.1)[0]
    assert updated.polygon_xy_m[1] == (1.2, 0.0)
    assert updated.physical_area_m2 == pytest.approx(0.30)


def test_area_and_discrete_observations_never_share_a_track():
    tracker = ProductTrackerV2(config())
    discrete = detection(cls="paper_litter")
    discrete["target_type"] = "DISCRETE"
    area = detection(x=1.01, cls="leaf_pile")
    area["class_probabilities"] = {"leaf_pile": 0.9, "background": 0.1}
    area["target_type"] = "AREA"

    tracks = tracker.update([discrete, area], 0.0)

    assert len(tracks) == 2
    assert {track.target_type for track in tracks} == {"DISCRETE", "AREA"}


def test_area_polygon_suppresses_contained_discrete_duplicate_only():
    tracker = ProductTrackerV2(config())
    discrete = detection(x=1.0, cls="paper_litter")
    discrete["target_type"] = "DISCRETE"
    area = detection(x=1.01, cls="leaf_pile")
    area["class_probabilities"] = {"leaf_pile": 0.9, "background": 0.1}
    area["target_type"] = "AREA"
    area["polygon_xy_m"] = (
        (0.8, 1.8),
        (1.2, 1.8),
        (1.2, 2.2),
        (0.8, 2.2),
    )

    tracks = tracker.update([discrete, area], 0.0)

    assert len(tracks) == 1
    assert tracks[0].target_type == "AREA"

    adjacent = detection(x=1.5, cls="metal_can")
    adjacent["target_type"] = "DISCRETE"
    tracks = tracker.update([area, adjacent], 0.1)
    assert {track.target_type for track in tracks} == {"AREA", "DISCRETE"}
