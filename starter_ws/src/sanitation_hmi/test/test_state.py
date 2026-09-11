from sanitation_hmi.state import VisualizationState


def test_state_reports_source_freshness_and_truth_boundaries():
    state = VisualizationState(
        reference={
            "scene": {"name": "test"},
            "mission": {"id": "mission"},
            "truth_targets": [],
        }
    )
    state.update_vehicle(1.0, 2.0, 0.5, 0.2)
    state.update_map(
        width=2,
        height=2,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        data=[0, 100, -1, 0],
    )
    state.emergency_stop = False
    state.touch("safety")
    snapshot = state.snapshot()
    assert snapshot["system_status"] == "ready"
    assert snapshot["sources"]["slam_map"]["status"] == "live"
    assert snapshot["capabilities"]["task_dispatch"] is False
    assert snapshot["capabilities"]["emergency_stop"] is True
    assert snapshot["trajectory"][0][:3] == [1.0, 2.0, 0.5]


def test_invalid_map_is_rejected_without_fake_replacement():
    state = VisualizationState()
    state.update_map(
        width=2,
        height=2,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        data=[0],
    )
    snapshot = state.snapshot()
    assert snapshot["slam_map"] is None
    assert snapshot["sources"]["slam_map"]["status"] == "error"


def test_state_summary_omits_replay_samples_until_requested():
    state = VisualizationState()
    state.update_vehicle(0.0, 0.0, 0.0, 0.0)
    state.update_vehicle(1.0, 0.0, 0.0, 0.1)
    summary = state.snapshot()
    full = state.snapshot(include_replay_samples=True)
    assert "samples" not in summary["replay"]
    assert summary["replay"]["sample_count"] == 2
    assert len(full["replay"]["samples"]) == 2


def test_session_replay_does_not_extend_stale_brush_observation(monkeypatch):
    monkeypatch.setattr("sanitation_hmi.state._now", lambda: 100.0)
    state = VisualizationState()
    state.brush_enabled = True
    state.touch("brush", at=100.0)
    state.update_vehicle(0.0, 0.0, 0.0, 0.1)
    monkeypatch.setattr("sanitation_hmi.state._now", lambda: 104.0)
    state.update_vehicle(1.0, 0.0, 0.0, 0.1)
    state.brush_enabled = False
    state.touch("brush", at=104.0)
    state.update_vehicle(2.0, 0.0, 0.0, 0.1)
    samples = state.snapshot(include_replay_samples=True)["replay"]["samples"]
    assert [row["brush"] for row in samples] == [True, None, False]


def test_stale_mission_is_not_current_success_and_observation_is_preserved():
    state = VisualizationState()
    state.coverage_state = "COMPLETED"
    state.spot_state = "COMPLETED"
    state.brush_enabled = True
    state.coverage_metrics["actual_ratio"] = 1.0
    for source in ("coverage_state", "spot_state", "brush", "coverage_metrics"):
        state.touch(source, at=100.0)
    fresh = state.snapshot(now=101.0)["mission"]
    assert fresh["coverage_state"] == "COMPLETED"
    assert fresh["coverage_metrics"]["actual_ratio"] == 1.0
    stale = state.snapshot(now=111.0)["mission"]
    assert stale["coverage_state"] == "数据不可用"
    assert stale["spot_state"] == "数据不可用"
    assert stale["brush_enabled"] is None
    assert stale["coverage_metrics"]["actual_ratio"] is None
    assert stale["last_observed"]["coverage_state"] == "COMPLETED"
    assert stale["last_observed"]["coverage_metrics"]["actual_ratio"] == 1.0
    assert state.coverage_state == "COMPLETED"


def test_loaded_successful_replay_never_enables_live_or_safety_capabilities():
    state = VisualizationState()
    state.set_replay({"success": True, "samples": [{"t": 1, "x": 0, "y": 0}]})
    snapshot = state.snapshot()
    assert snapshot["replay"]["success"] is True
    assert snapshot["system_status"] == "offline"
    assert snapshot["safety"]["status"] == "unknown"
    assert snapshot["capabilities"]["emergency_stop"] is False
    assert snapshot["capabilities"]["task_dispatch"] is False
    assert snapshot["mission"]["coverage_state"] == "数据不可用"
