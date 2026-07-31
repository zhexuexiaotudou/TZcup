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
