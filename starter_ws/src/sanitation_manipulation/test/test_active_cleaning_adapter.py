import json

import pytest

from sanitation_manipulation.active_cleaning_adapter import (
    ActiveCleaningManipulationAdapter,
    MockExecutionProfile,
    SingleTargetGraspRequest,
    main,
)
from sanitation_manipulation.cube_geometry import CubeCandidate
from sanitation_manipulation.cube_task import VerificationEvidence


def _request(target_id="cube-1"):
    return SingleTargetGraspRequest(
        target_id,
        CubeCandidate((0.35, 0.08, 0.015), (0.03, 0.03, 0.03), 0.2, 49, 0.0),
    )


def test_clearance_requires_verified_bin_registration_and_is_idempotent():
    adapter = ActiveCleaningManipulationAdapter()
    first = adapter.execute(_request())
    second = adapter.execute(_request())
    assert first.cleared and first.verified_in_bin
    assert second.cleared and second.verified_in_bin
    assert first.evidence["verification"]["target_registered_in_bin"] is True
    assert first.evidence["bin_contract"] == {
        "internal_size_m": [0.2, 0.2, 0.1],
        "single_layer": True,
        "stacked": False,
        "maximum_targets": 20,
        "current_count": 1,
        "packing_proof": False,
    }
    assert adapter.controller.collection_bin.count == 1


def test_adapter_preserves_20_target_bin_contract_across_requests():
    adapter = ActiveCleaningManipulationAdapter()
    for index in range(20):
        assert adapter.execute(_request(f"cube-{index}")).cleared
    overflow = adapter.execute(_request("cube-overflow"))
    assert not overflow.cleared
    assert overflow.reason == "bin_target_limit_reached"
    assert overflow.evidence["bin_contract"]["current_count"] == 20


def test_two_failed_grasps_never_return_cleared():
    rejected = VerificationEvidence(gripper_width_ok=True)
    adapter = ActiveCleaningManipulationAdapter(
        MockExecutionProfile(grasp_evidence=(rejected, rejected))
    )
    decision = adapter.execute(_request())
    assert not decision.cleared
    assert not decision.verified_in_bin
    assert decision.attempts == 2
    assert decision.state == "DEFERRED"
    assert decision.reason == "grasp_attempt_limit_reached"
    assert len(decision.evidence["verification"]["attempt_evidence"]) == 2


def test_unverified_placement_never_returns_cleared():
    accepted = VerificationEvidence(gripper_effort_ok=True, source_location_absent=True)
    adapter = ActiveCleaningManipulationAdapter(
        MockExecutionProfile(grasp_evidence=(accepted,), place_outcomes=(False,))
    )
    decision = adapter.execute(_request())
    assert not decision.cleared
    assert decision.evidence["verification"]["controller_placed_in_bin"] is False
    assert decision.reason == "bin_placement_not_verified"


def test_request_is_strictly_single_target_and_30_mm():
    raw = _request().to_mapping()
    raw["target_ids"] = ["cube-1", "cube-2"]
    with pytest.raises(ValueError, match="request keys"):
        SingleTargetGraspRequest.from_mapping(raw)
    raw = _request().to_mapping()
    raw["target_id"] = ["cube-1", "cube-2"]
    with pytest.raises(ValueError, match="target_id must be a string"):
        SingleTargetGraspRequest.from_mapping(raw)
    with pytest.raises(ValueError, match="30 mm"):
        SingleTargetGraspRequest(
            "cube-1",
            CubeCandidate((0.0, 0.0, 0.025), (0.05, 0.03, 0.03), 0.0, 10, 0.0),
        )


def test_structured_evidence_cannot_claim_real_robot_authority():
    evidence = ActiveCleaningManipulationAdapter().execute(_request()).evidence
    assert evidence["authority"] == {
        "evidence_level": "MOCK_TASK_SEMANTICS_ONLY",
        "evidence_authority": False,
        "placeholder_evidence_only": True,
        "real_robot_evidence": False,
        "gazebo_runtime_evidence": False,
        "measured_urdf_used": False,
        "moveit_or_hardware_execution_used": False,
        "truth_used_for_control": False,
    }
    assert len(evidence["request_sha256"]) == 64
    assert len(evidence["evidence_sha256"]) == 64


def test_cli_writes_success_and_fail_closed_reports(tmp_path):
    request_path = tmp_path / "request.json"
    request_path.write_text(json.dumps(_request().to_mapping()), encoding="utf-8")
    success_path = tmp_path / "success.json"
    assert main(["--request", str(request_path), "--output", str(success_path)]) == 0
    assert json.loads(success_path.read_text(encoding="utf-8"))["decision"]["cleared"] is True

    failure_path = tmp_path / "failure.json"
    assert main([
        "--request", str(request_path),
        "--output", str(failure_path),
        "--mock-profile", "grasp_fail",
    ]) == 2
    failure = json.loads(failure_path.read_text(encoding="utf-8"))
    assert failure["decision"]["cleared"] is False
    assert failure["decision"]["attempts"] == 2
