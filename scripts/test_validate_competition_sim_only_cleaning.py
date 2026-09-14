import importlib.util


SPEC = importlib.util.spec_from_file_location(
    "competition_sim_only_cleaning",
    __file__.replace("test_validate_", "validate_"),
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _binding():
    return {
        "session_manifest_sha256": "a" * 64,
        "snapshot": {"source_inventory_sha256": "b" * 64},
    }


def _reports():
    binding = _binding()
    return (
        {"status": "FORMAL_FIRST_MAP_THEN_SAVED_MAP_CLEANING_PASSED", "passed": True, "acceptance_session_binding": binding},
        {"status": "FORMAL_GROUND_DIRT_PHYSICAL_CLEANING_PASSED", "passed": True, "acceptance_session_binding": binding},
        {"status": "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_PASSED", "passed": True, "runtime_gate_binding": {"acceptance_session_binding": binding}},
        {"report_id": MODULE.PERCEPTION_REPORT_ID, "status": MODULE.PERCEPTION_STATUS, "passed": True, "acceptance_session_binding": binding, "online_inference": True, "garbage_detection_and_localization": True, "truth_used_for_control": False},
        {"report_id": MODULE.EMERGENCY_REPORT_ID, "status": MODULE.EMERGENCY_STATUS, "passed": True, "acceptance_session_binding": binding, "emergency_braking_s": 0.8, "emergency_stop_command_observed": True, "safe_recovery_verified": True},
        {"report_id": MODULE.POST_CLEAN_REPORT_ID, "status": MODULE.POST_CLEAN_STATUS, "passed": True, "acceptance_session_binding": binding, "post_clean_verification": True, "truth_used_for_control": False},
    )


def test_cleaning_route_passes_without_a12_or_grasp():
    report = MODULE.evaluate(*_reports())
    assert report["component_evidence_complete"] is True
    assert report["status"] == MODULE.COMPLETE_STATUS
    assert report["evidence_kind"] == "COMPOSITE_COMPONENT_EVIDENCE_ONLY"
    assert report["grasp_status"] == "NOT_EXECUTED"
    assert report["cube_target_count"] == 0
    assert report["a12_contract_used"] is False


def test_missing_perception_or_emergency_report_blocks_without_claiming_grasp():
    map_lifecycle, ground_dirt, dynamic, _, _, _ = _reports()
    report = MODULE.evaluate(map_lifecycle, ground_dirt, dynamic, None, None, None)
    assert report["component_evidence_complete"] is False
    assert "online_garbage_detection_and_localization_passed" in report["blockers"]
    assert "emergency_braking_and_recovery_passed" in report["blockers"]
    assert "post_clean_verification_passed" in report["blockers"]
    assert report["grasp_status"] == "NOT_EXECUTED"


def test_mixed_frozen_sessions_fail_closed():
    map_lifecycle, ground_dirt, dynamic, perception, emergency, post_clean = _reports()
    dynamic["runtime_gate_binding"]["acceptance_session_binding"] = {
        "session_manifest_sha256": "c" * 64,
        "snapshot": {"source_inventory_sha256": "b" * 64},
    }
    report = MODULE.evaluate(map_lifecycle, ground_dirt, dynamic, perception, emergency, post_clean)
    assert report["component_evidence_complete"] is False
    assert "all_simulation_reports_share_frozen_session_and_source" in report["blockers"]


def test_perception_estop_and_post_clean_require_recognized_identity_bound_reports():
    map_lifecycle, ground_dirt, dynamic, perception, emergency, post_clean = _reports()
    perception["report_id"] = "unrecognized"
    emergency.pop("acceptance_session_binding")
    post_clean["status"] = "STALE"
    report = MODULE.evaluate(map_lifecycle, ground_dirt, dynamic, perception, emergency, post_clean)
    assert report["component_evidence_complete"] is False
    assert "online_garbage_detection_and_localization_passed" in report["blockers"]
    assert "emergency_braking_and_recovery_passed" in report["blockers"]
    assert "post_clean_verification_passed" in report["blockers"]
    assert "all_simulation_reports_share_frozen_session_and_source" in report["blockers"]
