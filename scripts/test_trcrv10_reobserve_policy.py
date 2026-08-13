from trcrv10_reobserve_policy import MAX_REOBSERVE, ReobserveConfig, decide


def base(**updates):
    row = {"action_verifier_decision": "OBSERVE_AGAIN", "reobserve_count": 0,
           "reachable_for_visual_confirmation": True, "coverage_path_improves_view": True,
           "bbox_short_side_px": 20, "depth_valid_fraction": .9, "occlusion_ratio": .1,
           "map_covariance_m2": .01}
    row.update(updates)
    return row


def test_reobserve_is_bounded_and_prefers_coverage() -> None:
    result = decide(base(), ReobserveConfig(64))
    assert result["decision"] == "OBSERVE_AGAIN"
    assert result["mode"] == "CONTINUE_COVERAGE_APPROACH"
    assert MAX_REOBSERVE == 2
    assert decide(base(reobserve_count=2), ReobserveConfig(64))["decision"] == "DEFER"


def test_safety_and_unreachable_never_force_navigation_or_cleaning() -> None:
    assert decide(base(dynamic_obstacle=True), ReobserveConfig(64))["decision"] == "DEFER"
    assert decide(base(reachable_for_visual_confirmation=False), ReobserveConfig(64))["decision"] == "UNREACHABLE_FOR_VISUAL_CONFIRMATION"


def test_only_action_verified_can_confirm() -> None:
    assert decide(base(action_verifier_decision="ACCEPT"), ReobserveConfig(64))["decision"] == "CONFIRMED"
