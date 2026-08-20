from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from sanitation_perception.product_pipeline_node import (
    SUPPORTED_RUNTIME_CONTRACT,
    cleaning_event_target_state,
    load_onnxruntime,
    product_target_size,
    optical_forward_yaw,
    stamp_nanoseconds,
    track_to_online_observation,
    validate_product_runtime_contract,
)
from sanitation_perception.trash_map_messages import TargetState


class Stamp:
    sec = 12
    nanosec = 345


class Header:
    stamp = Stamp()


class Message:
    header = Header()


def test_rgb_stamp_is_converted_exactly_for_timestamped_tf():
    assert stamp_nanoseconds(Message()) == 12_000_000_345


def test_frustum_yaw_uses_optical_z_axis():
    matrix = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 1.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
    assert optical_forward_yaw(matrix) == pytest.approx(1.5707963267948966)


def test_cleaning_events_map_only_to_explicit_product_states():
    assert cleaning_event_target_state("scheduled") == TargetState.SCHEDULED
    assert cleaning_event_target_state("post_verify_pending") == TargetState.POST_VERIFY
    assert cleaning_event_target_state("cleaned") == TargetState.CLEANED
    with pytest.raises(ValueError, match="unsupported product cleaning event"):
        cleaning_event_target_state("actuator_success_means_cleaned")


def test_product_contract_rejects_cpu_or_missing_iobinding():
    runtime = {
        "postprocess_contract": SUPPORTED_RUNTIME_CONTRACT,
        "required_provider": "CUDAExecutionProvider",
        "io_binding_required": True,
        "cpu_fallback_forbidden": True,
        "maximum_candidates": 16,
        "minimum_valid_depth_ratio": 0.05,
        "minimum_area_region_pixels": 20,
        "minimum_area_region_m2": 0.05,
        "minimum_area_region_m2_by_class": {
            "leaf_pile": 0.02,
            "puddle": 0.05,
        },
        "minimum_rgb_stddev": 2.0,
        "maximum_dark_or_saturated_fraction": 0.98,
        "action_verifier": {
            "actionable_classes": [
                "plastic_bottle", "metal_can", "paper_litter", "leaf_pile", "puddle"
            ],
            "minimum_class_confidence": 0.80,
            "maximum_background_probability": 0.10,
            "reject_background_probability": 0.90,
            "minimum_observations": 3,
            "defer_after_observations": 6,
            "maximum_covariance_trace": 0.03,
            "maximum_map_disagreement_m": 0.15,
            "minimum_view_separation_rad": 0.0,
            "maximum_reobserve_count": 2,
        },
        "dynamic_trash_map": {
            "association_distance_m": 0.30,
            "confirmation_observations": 3,
            "confirmation_class_posterior": 0.70,
            "confirmation_confidence": 0.60,
            "maximum_covariance_trace": 0.03,
            "lost_after_s": 1.0,
            "reject_after_s": 5.0,
            "maximum_observation_history": 64,
        },
        "camera_frustum": {
            "horizontal_fov_rad": 1.4,
            "minimum_range_m": 0.1,
            "maximum_range_m": 8.0,
        },
    }
    validate_product_runtime_contract({"runtime": runtime})
    for key, value in (
        ("required_provider", "CPUExecutionProvider"),
        ("io_binding_required", False),
        ("cpu_fallback_forbidden", False),
        ("maximum_candidates", 0),
        ("minimum_valid_depth_ratio", 0.0),
        ("minimum_area_region_pixels", 2),
        ("minimum_area_region_m2", 0.0),
        ("minimum_rgb_stddev", 0.0),
        ("maximum_dark_or_saturated_fraction", 1.0),
    ):
        invalid = {"runtime": {**runtime, key: value}}
        with pytest.raises(RuntimeError):
            validate_product_runtime_contract(invalid)

    invalid = {
        "runtime": {
            **runtime,
            "minimum_area_region_m2_by_class": {"leaf_pile": 0.02},
        }
    }
    with pytest.raises(RuntimeError, match="leaf_pile and puddle"):
        validate_product_runtime_contract(invalid)


def test_area_track_preserves_physical_area_in_online_observation():
    track = SimpleNamespace(
        uuid="area-track",
        source_backend="onnxruntime",
        target_type="AREA",
        class_posterior={"leaf_pile": 0.99, "background": 0.01},
        score_ema=0.99,
        x_m=1.0,
        y_m=2.0,
        z_m=0.0,
        covariance_trace=0.01,
        bbox_xyxy=None,
        polygon_xy_m=((0.9, 1.9), (1.1, 1.9), (1.0, 2.1)),
        physical_area_m2=0.038,
        estimated_size_m=(0.038, 0.0, 0.0),
    )
    observation = track_to_online_observation(
        track,
        mission_id="mission",
        stamp_ns=1,
        camera_frame_id="camera",
        image_frame_id="image",
        source_model="leaf-onnx",
    )
    assert observation.estimated_size_m == (0.038, 0.0, 0.0)
    assert product_target_size(track, (1.0, 2.0, 3.0)) == (0.038, 0.0, 0.0)


def test_repository_placeholder_cannot_activate_product_runtime():
    path = Path(__file__).parents[1] / "config" / "perception_pipeline_manifest.yaml"
    pipeline = yaml.safe_load(path.read_text(encoding="utf-8"))
    with pytest.raises(RuntimeError, match="postprocess contract"):
        validate_product_runtime_contract(pipeline)


def test_missing_onnxruntime_is_a_lifecycle_error_not_an_import_crash():
    def missing(_name):
        raise ModuleNotFoundError("missing")

    with pytest.raises(RuntimeError, match="remains inactive"):
        load_onnxruntime(missing)
