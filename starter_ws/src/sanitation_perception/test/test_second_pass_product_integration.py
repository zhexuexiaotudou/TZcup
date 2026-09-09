import numpy as np

from sanitation_perception.action_verifier import (
    ActionVerifierConfig,
    ActionVerdict,
    ProductActionVerifier,
)
from sanitation_perception.close_range_evidence import (
    CameraInfoContract,
    D1SecondPassEvidenceProvider,
    Observation,
)
from sanitation_perception.dynamic_trash_map import DynamicTrashMap
from sanitation_perception.observation_model import MapPoseMeasurement, TargetObservation
from sanitation_perception.pretrained_contracts import Detection
from sanitation_perception.tracker_v2 import ProductTrackerV2, TrackerV2Config
from sanitation_perception.trash_map_messages import TargetState

from online_map_test_support import record_sweep


def test_second_pass_requires_tracker_map_and_independent_verifier_to_confirm():
    camera = CameraInfoContract(160, 120, 100.0, 100.0, 80.0, 60.0)
    rgb = np.zeros((120, 160, 3), dtype=np.uint8)
    depth = np.full((120, 160), 1.5, dtype=np.float32)
    match = Detection(
        (15.0, 15.0, 75.0, 75.0),
        0.95,
        "plastic_bottle",
        "plastic_bottle",
    )
    evidence = D1SecondPassEvidenceProvider(lambda _crop: [match]).evaluate(
        rgb,
        (40.0, 30.0, 100.0, 90.0),
        [Observation("plastic_bottle", 0.90, 1, 0.0)],
        depth,
        camera,
        stamp_ns=2,
        view_direction_rad=0.2,
    )
    assert evidence.status == "READY_FOR_ACTION_VERIFIER"
    assert not evidence.confirmed

    tracker = ProductTrackerV2(
        TrackerV2Config(
            association_distance_m=0.3,
            close_recovery_distance_m=0.1,
            minimum_image_iou=0.05,
            maximum_observation_gap_s=0.5,
            occlusion_recovery_s=2.0,
            duplicate_distance_m=0.08,
            confirmation_observations=3,
            confirmation_class_posterior=0.7,
            confirmation_score_ema=0.6,
            score_ema_alpha=0.35,
            defer_after_observations=5,
        )
    )
    detection = evidence.as_tracker_detection(
        x_m=2.0,
        y_m=0.0,
        covariance_trace=0.01,
        bbox_xyxy=(40.0, 30.0, 100.0, 90.0),
    )
    dynamic_map = DynamicTrashMap.start_new("j6f2-integration")
    target = None
    track = None
    for index in range(3):
        stamp_ns = 1_000_000_000 + index * 100_000_000
        stamp_s = stamp_ns / 1_000_000_000.0
        track = tracker.update([detection], stamp_s)[0]
        image_frame_id = record_sweep(dynamic_map, stamp_ns)
        target = dynamic_map.ingest(
            TargetObservation(
                observation_id=f"d1-second-pass-{index}",
                mission_id=dynamic_map.mission_id,
                stamp_ns=stamp_ns,
                camera_frame_id="camera_color_optical_frame",
                image_frame_id=image_frame_id,
                source_model="d1_littercam_development_only",
                source_backend=track.source_backend,
                target_type="DISCRETE",
                class_probabilities=track.class_posterior,
                confidence=track.class_confidence,
                map_pose=MapPoseMeasurement(
                    x_m=track.x_m,
                    y_m=track.y_m,
                    covariance_xx=0.005,
                    covariance_yy=0.005,
                ),
                bbox_xyxy=track.bbox_xyxy,
                estimated_size_m=(0.07, 0.07, 0.22),
                view_direction_rad=0.1 * index,
                in_current_fov=True,
            )
        )
    assert track is not None and target is not None
    assert track.state == "READY_FOR_VERIFICATION"
    assert target.track_state == TargetState.TRACKED

    verifier = ProductActionVerifier(
        ActionVerifierConfig(
            actionable_classes=("plastic_bottle", "metal_can", "paper_litter"),
            minimum_class_confidence=0.8,
            maximum_background_probability=0.1,
            reject_background_probability=0.9,
            minimum_observations=3,
            defer_after_observations=6,
            maximum_covariance_trace=0.03,
            maximum_map_disagreement_m=0.15,
            minimum_view_separation_rad=0.0,
            maximum_reobserve_count=2,
        )
    )
    verdict = verifier.evaluate(track, target, depth_valid=True)
    assert verdict.verdict == ActionVerdict.ACCEPT
    assert target.track_state == TargetState.TRACKED
    dynamic_map.apply_action_verdict(
        target.uuid,
        verdict.verdict.value,
        1_300_000_000,
        "d1_second_pass_all_checks_passed",
    )
    assert target.track_state == TargetState.CONFIRMED
