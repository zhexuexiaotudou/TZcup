import numpy as np
import pytest

from sanitation_perception.close_range_evidence import (
    CameraInfoContract,
    D1_CLASS_MAPPING,
    D1SecondPassConfig,
    D1SecondPassEvidenceProvider,
    Observation,
)
from sanitation_perception.pretrained_contracts import Detection


CAMERA = CameraInfoContract(160, 120, 100.0, 100.0, 80.0, 60.0)
BOX = (40.0, 30.0, 100.0, 90.0)


def test_d1_class_mapping_is_exact_and_fail_closed():
    assert D1_CLASS_MAPPING == {
        "plastic_bottle": "plastic_bottle",
        "drinks_can": "metal_can",
        "paper_waste": "paper_litter",
        "cigarette_butt": "background_or_unknown",
        "fast_food_packaging": "background_or_unknown",
        "plastic_bag": "background_or_unknown",
        "coffee_cup": "background_or_unknown",
        "glass_bottle": "background_or_unknown",
        "food_wrapper": "background_or_unknown",
        "general_litter": "background_or_unknown",
    }


def inputs(distance=1.5):
    rgb = np.zeros((120, 160, 3), dtype=np.uint8)
    depth = np.full((120, 160), distance, dtype=np.float32)
    return rgb, depth


def detection(source_class="plastic_bottle", score=0.9, bbox=(15.0, 15.0, 75.0, 75.0)):
    mapping = {
        "plastic_bottle": "plastic_bottle",
        "drinks_can": "metal_can",
        "paper_waste": "paper_litter",
    }
    return Detection(bbox, score, source_class, mapping.get(source_class, "background_or_unknown"))


def provider(results, **overrides):
    config = D1SecondPassConfig(**overrides)
    return D1SecondPassEvidenceProvider(lambda _crop: list(results), config)


def test_far_small_candidate_waits_without_running_model():
    calls = []
    instance = D1SecondPassEvidenceProvider(lambda crop: calls.append(crop) or [])
    rgb, depth = inputs(distance=5.0)
    result = instance.evaluate(
        rgb, (40.0, 30.0, 60.0, 50.0), [], depth, CAMERA,
        stamp_ns=1, view_direction_rad=0.0,
    )
    assert result.status == "WAIT_CLOSE_RANGE"
    assert calls == []


def test_near_two_view_match_is_only_ready_for_action_verifier():
    rgb, depth = inputs()
    history = [Observation("plastic_bottle", 0.8, 1, 0.0)]
    result = provider([detection()]).evaluate(
        rgb, BOX, history, depth, CAMERA,
        stamp_ns=2, view_direction_rad=0.2,
    )
    assert result.status == "READY_FOR_ACTION_VERIFIER"
    assert result.product_class == "plastic_bottle"
    assert result.agreeing_views == 2
    assert result.confirmed is False and result.clean_now is False
    tracker_input = result.as_tracker_detection(
        x_m=1.0, y_m=2.0, covariance_trace=0.02, bbox_xyxy=BOX
    )
    assert tracker_input["source_backend"] == "development_d1_second_pass_onnx"
    assert "confirmed" not in tracker_input and "clean_now" not in tracker_input


@pytest.mark.parametrize(
    ("results", "reason"),
    [
        ([], "no_overlapping_second_pass_match"),
        ([detection("coffee_cup")], "non_target_second_pass_class"),
    ],
)
def test_no_match_and_non_target_fail_closed(results, reason):
    rgb, depth = inputs()
    result = provider(results).evaluate(
        rgb, BOX, [], depth, CAMERA,
        stamp_ns=1, view_direction_rad=0.0,
    )
    assert reason in result.reasons
    assert result.product_class == "background_or_unknown"
    assert result.unknown_probability == 1.0


@pytest.mark.parametrize("score", [float("nan"), float("inf"), -0.1, 1.1])
def test_nonfinite_or_out_of_range_detection_score_is_no_match(score):
    rgb, depth = inputs()
    result = provider([detection(score=score)]).evaluate(
        rgb, BOX, [], depth, CAMERA, stamp_ns=1, view_direction_rad=0.0
    )
    assert result.status == "OBSERVE_AGAIN"
    assert result.reasons == ("no_overlapping_second_pass_match",)


def test_conflicting_matches_require_reobserve_then_defer():
    rgb, depth = inputs()
    results = [detection(), detection("drinks_can", bbox=(15.0, 15.0, 75.0, 75.0))]
    instance = provider(results)
    first = instance.evaluate(
        rgb, BOX, [], depth, CAMERA,
        stamp_ns=1, view_direction_rad=0.0, reobserve_count=1,
    )
    last = instance.evaluate(
        rgb, BOX, [], depth, CAMERA,
        stamp_ns=2, view_direction_rad=0.1, reobserve_count=2,
    )
    assert first.status == "OBSERVE_AGAIN"
    assert last.status == "DEFER"


def test_invalid_depth_defers_before_inference():
    calls = []
    instance = D1SecondPassEvidenceProvider(lambda crop: calls.append(crop) or [detection()])
    rgb, depth = inputs()
    depth[:] = np.nan
    result = instance.evaluate(
        rgb, BOX, [], depth, CAMERA,
        stamp_ns=1, view_direction_rad=0.0,
    )
    assert result.status == "DEFER"
    assert result.reasons == ("insufficient_valid_depth_coverage",)
    assert calls == []


def test_sparse_depth_and_far_large_bbox_fail_closed_before_inference():
    calls = []
    instance = D1SecondPassEvidenceProvider(lambda crop: calls.append(crop) or [detection()])
    rgb, depth = inputs(distance=np.nan)
    depth[31, 41] = 1.0
    sparse = instance.evaluate(
        rgb, BOX, [], depth, CAMERA, stamp_ns=1, view_direction_rad=0.0
    )
    assert sparse.status == "DEFER"
    assert sparse.reasons == ("insufficient_valid_depth_coverage",)
    _, far_depth = inputs(distance=10.0)
    far = instance.evaluate(
        rgb, BOX, [], far_depth, CAMERA, stamp_ns=2, view_direction_rad=0.0
    )
    assert far.status == "DEFER"
    assert far.reasons == ("bbox_depth_geometry_conflict",)
    assert calls == []


def test_class_flip_does_not_count_as_agreeing_view():
    rgb, depth = inputs()
    history = [Observation("metal_can", 0.95, 1, 0.0)]
    result = provider([detection()]).evaluate(
        rgb, BOX, history, depth, CAMERA,
        stamp_ns=2, view_direction_rad=0.2,
    )
    assert result.status == "OBSERVE_AGAIN"
    assert result.agreeing_views == 1


def test_duplicate_stamp_does_not_satisfy_multiframe_gate():
    rgb, depth = inputs()
    history = [Observation("plastic_bottle", 0.95, 2, 0.0)]
    result = provider([detection()]).evaluate(
        rgb, BOX, history, depth, CAMERA,
        stamp_ns=2, view_direction_rad=0.2,
    )
    assert result.status == "OBSERVE_AGAIN"
    assert result.agreeing_views == 1


def test_same_view_and_future_history_do_not_satisfy_multiframe_gate():
    rgb, depth = inputs()
    same_view = provider([detection()]).evaluate(
        rgb, BOX, [Observation("plastic_bottle", 0.95, 1, 0.2)], depth, CAMERA,
        stamp_ns=2, view_direction_rad=0.2,
    )
    future = provider([detection()]).evaluate(
        rgb, BOX, [Observation("plastic_bottle", 0.95, 3, 0.0)], depth, CAMERA,
        stamp_ns=2, view_direction_rad=0.2,
    )
    assert same_view.status == "OBSERVE_AGAIN"
    assert "insufficient_view_separation" in same_view.reasons
    assert future.status == "OBSERVE_AGAIN"
    assert future.agreeing_views == 1


def test_ready_evidence_cannot_bypass_tracker_and_verifier():
    rgb, depth = inputs()
    result = provider([detection()]).evaluate(
        rgb,
        BOX,
        [Observation("plastic_bottle", 0.9, 1, 0.0)],
        depth,
        CAMERA,
        stamp_ns=2,
        view_direction_rad=0.1,
    )
    record = result.__dict__
    assert record["action_verifier_required"] is True
    assert record["confirmed"] is False
    assert record["clean_now"] is False
