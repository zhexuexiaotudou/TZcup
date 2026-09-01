import json
import math

import pytest

from sanitation_manipulation.formal_grasp_core import (
    DryBinSample,
    GraspRequest,
    ParkingObservation,
    build_target_conditioned_waypoints,
    material_for_measured_mass,
    validate_wrist_recheck,
    verify_bin_increment,
)


def _request(**updates):
    raw = {
        "schema_version": 2,
        "target_id": "perception-track-17",
        "frame_id": "map",
        "pose": {
            "x_m": 12.0, "y_m": -4.0, "z_m": 0.015,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
        },
        "size_m": [0.030, 0.030, 0.030],
        "material": "unknown",
        "confidence": 0.93,
        "truth_used": False,
    }
    raw.update(updates)
    return json.dumps(raw)


def test_request_requires_target_3d_pose_size_material_and_truth_isolation():
    request = GraspRequest.from_json(_request())
    assert request.geometry.z_m == pytest.approx(0.015)
    assert request.geometry.size_m == pytest.approx((0.03, 0.03, 0.03))
    assert request.geometry.material == "unknown"
    with pytest.raises(ValueError, match="truth-backed"):
        GraspRequest.from_json(_request(truth_used=True))
    with pytest.raises(ValueError, match="keys must equal"):
        GraspRequest.from_json(_request(model_name="object_17"))
    with pytest.raises(ValueError, match="schema_version"):
        GraspRequest.from_json(_request(schema_version=1))
    without_material = json.loads(_request())
    del without_material["material"]
    assert GraspRequest.from_json(json.dumps(without_material)).geometry.material == "unknown"
    with pytest.raises(ValueError, match="must be unknown"):
        GraspRequest.from_json(_request(material="PET"))


def test_target_conditioned_waypoints_move_with_pose_and_yaw():
    first = GraspRequest.from_json(_request()).geometry
    second_raw = json.loads(_request())
    second_raw["pose"].update(
        {"x_m": 12.04, "y_m": -4.03, "qz": math.sin(0.3), "qw": math.cos(0.3)}
    )
    second = GraspRequest.from_json(json.dumps(second_raw)).geometry
    a = build_target_conditioned_waypoints(first)
    b = build_target_conditioned_waypoints(second)
    assert a.pick.x_m != b.pick.x_m and a.pick.y_m != b.pick.y_m
    assert a.pick.qx != b.pick.qx and a.pick.qy != b.pick.qy
    assert a.pregrasp.z_m > a.pick.z_m
    assert a.lift.z_m > a.pick.z_m


def test_wrist_recheck_must_match_track_material_pose_and_size():
    original = GraspRequest.from_json(_request())
    refined_raw = json.loads(_request())
    refined_raw["pose"]["x_m"] += 0.02
    refined = GraspRequest.from_json(json.dumps(refined_raw))
    assert validate_wrist_recheck(original, refined)[0] is True
    refined_raw["target_id"] = "other-track"
    mismatch = GraspRequest.from_json(json.dumps(refined_raw))
    assert validate_wrist_recheck(original, mismatch)[0] is False


def test_parking_requires_stationary_base_and_exact_side_pick_window():
    good = ParkingObservation(0.300, -0.950, 0.0, 0.0, 0.1, 0.1)
    assert good.validate()[0] is True
    assert ParkingObservation(0.0, 0.0, 0.0, 0.0, 0.1, 0.1).validate() == (
        False,
        "target_outside_physical_pick_window",
    )
    assert ParkingObservation(0.300, -0.950, 0.02, 0.0, 0.1, 0.1).validate()[0] is False


def test_bin_verification_classifies_material_only_after_physical_increment():
    baseline = DryBinSample(True, 2, 0.10, False)
    stable_pet = tuple(DryBinSample(True, 3, 0.13726, False) for _ in range(8))
    assert verify_bin_increment(baseline, stable_pet) == (
        True,
        "physical_cube_stably_verified_in_dry_bin",
        pytest.approx(0.03726),
    )
    assert material_for_measured_mass(0.03726) == "PET"
    assert material_for_measured_mass(0.050) is None
