from sanitation_hmi.live_state import LiveMissionState, _bounded_points


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def test_live_state_tracks_components_paths_and_truth_boundary():
    clock = FakeClock()
    state = LiveMissionState(
        expected_components=3,
        geometry={"outer_polygon": [[0, 0], [2, 0], [2, 2], [0, 2]]},
        clock=clock,
    )
    clock.now = 1.0
    state.update_state("PLANNING")
    state.update_component({"state": "EXECUTING_SWATH", "kind": "swath", "index": 0})
    state.update_evaluation_sample(
        0.0, 0.0, 0.0, brush_enabled=True, coverage_state="EXECUTING_SWATH"
    )
    state.update_estimated_pose(0.03, -0.01, 0.02)
    state.update_velocity(0.4, 0.1)
    state.update_planned_path([[0, 0], [1, 0], [2, 0]])

    clock.now = 2.0
    state.update_component({"state": "EXECUTING_TURN", "kind": "turn", "index": 0})
    state.update_evaluation_sample(
        1.0, 0.0, 0.0, brush_enabled=False, coverage_state="EXECUTING_TURN"
    )
    snapshot = state.snapshot()

    assert snapshot["status"] == "EXECUTING_TURN"
    assert snapshot["progress"]["completed_components"] == 1
    assert snapshot["progress"]["active_component_number"] == 2
    assert snapshot["vehicle"]["estimated_pose_map"] == [0.03, -0.01, 0.02]
    assert snapshot["cleaning"]["brush_enabled"] is False
    assert len(snapshot["visualization"]["evaluation_only_trajectory"]) == 2
    assert len(snapshot["visualization"]["evaluation_only_cleaned_trajectory"]) == 1
    assert snapshot["claim_boundary"]["ground_truth_usage"] == (
        "evaluation_and_visualization_only"
    )
    assert snapshot["claim_boundary"]["learned_perception_pass"] is False


def test_terminal_state_finishes_current_component_without_overcounting():
    state = LiveMissionState(expected_components=1)
    state.update_component({"state": "EXECUTING_SWATH", "kind": "swath", "index": 0})
    state.update_state("COMPLETED")
    snapshot = state.snapshot()
    assert snapshot["terminal"] is True
    assert snapshot["progress"]["completed_components"] == 1
    assert snapshot["progress"]["ratio"] == 1.0


def test_late_evaluation_sample_does_not_reopen_terminal_state():
    state = LiveMissionState(expected_components=1)
    state.update_component({"state": "EXECUTING_SWATH", "kind": "swath", "index": 0})
    state.update_state("COMPLETED")
    state.update_evaluation_sample(
        1.0,
        2.0,
        0.3,
        brush_enabled=False,
        coverage_state="EXECUTING_SWATH",
    )

    snapshot = state.snapshot()

    assert snapshot["status"] == "COMPLETED"
    assert snapshot["terminal"] is True


def test_path_decimation_preserves_last_point():
    points = [[index, -index] for index in range(1000)]
    sampled = _bounded_points(points, maximum=100)
    assert len(sampled) <= 101
    assert sampled[0] == [0.0, 0.0]
    assert sampled[-1] == [999.0, -999.0]
