from reference_vision.adapters.grounding_dino import GroundingDinoAdapter, PROMPT_SETS
from reference_vision.adapters.sam2_tracker import Sam2TrackerAdapter
from reference_vision.benchmark import ranking_key, validate_moving_camera_manifest
from reference_vision.cadence import AdaptiveDiscoveryCadence


def test_detector_adapter_emits_unified_contract():
    adapter = GroundingDinoAdapter(lambda _image, prompts: [{
        "label": prompts[0],
        "score": 0.91,
        "bbox_xyxy": [1, 2, 30, 40],
        "mask": None,
    }])
    record = adapter.detect("frame-1", object(), PROMPT_SETS["closed"]).to_record()
    assert record == {
        "frame_id": "frame-1",
        "detections": [{
            "label": "plastic_bottle",
            "score": 0.91,
            "bbox_xyxy": [1.0, 2.0, 30.0, 40.0],
            "mask": None,
            "source_model": "grounding-dino-online-x2",
        }],
    }


def test_tracker_adapter_emits_age_and_lost_count():
    adapter = Sam2TrackerAdapter(lambda _frame: [{
        "track_id": "track-1",
        "score": 0.8,
        "age": 5,
        "lost_count": 1,
        "bbox_xyxy": [1, 2, 3, 4],
    }])
    frame = adapter.track("frame-2", object())
    assert frame.tracks[0].age == 5
    assert frame.tracks[0].lost_count == 1


def test_moving_camera_manifest_and_adaptive_cadence():
    tags = ["near_target", "far_target", "turning", "occlusion", "light_shadow", "negative_only", "late_fov_entry"]
    manifest = {"sequences": [
        {"source": "gazebo_onboard_rgb", "duration_s": 60, "tags": tags if index == 0 else []}
        for index in range(10)
    ]}
    validate_moving_camera_manifest(manifest)
    cadence = AdaptiveDiscoveryCadence()
    assert cadence.should_discover(frame_index=0, cumulative_distance_m=0.0, scene_change=0.0, track_confidence_drop=0.0, new_region=False) == (True, "initial")
    assert cadence.should_discover(frame_index=1, cumulative_distance_m=0.1, scene_change=0.0, track_confidence_drop=0.0, new_region=False) == (False, "tracking_only")
    assert cadence.should_discover(frame_index=2, cumulative_distance_m=0.1, scene_change=0.3, track_confidence_drop=0.0, new_region=False) == (True, "scene_change")


def test_pre_fov_false_discovery_is_ranked_last():
    base = {"recall": 0.9, "small_object_recall": 0.8, "false_candidates_per_min": 1.0, "temporal_stability": 0.9, "latency_ms": 100, "memory_mb": 1000, "license_deployment_suitable": True}
    clean = dict(base, pre_fov_false_discovery=0)
    invalid = dict(base, pre_fov_false_discovery=1, recall=1.0)
    assert ranking_key(clean) < ranking_key(invalid)
