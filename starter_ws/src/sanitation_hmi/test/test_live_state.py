from sanitation_hmi.live_state import (
    OCCUPANCY_GRID_MAX_AXIS,
    LiveMissionState,
    _bounded_points,
)


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
    assert snapshot["cleaning"]["brush_enabled"] is None
    assert snapshot["cleaning"]["evaluation_sample_brush_enabled"] is False
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


def test_semantic_component_updates_dynamic_plan_size():
    state = LiveMissionState(expected_components=17)
    state.update_component({
        "state": "EXECUTING_SHIFT",
        "kind": "SHIFT",
        "index": 2,
        "component_id": "connector-00-translate",
        "expected_components": 25,
    })
    snapshot = state.snapshot()
    assert snapshot["progress"]["expected_components"] == 25
    assert snapshot["progress"]["current_component"] == "connector-00-translate"


def test_final_product_state_is_only_populated_by_valid_live_topic_payload():
    clock = FakeClock()
    state = LiveMissionState(clock=clock)
    unavailable = state.snapshot()["final_demo"]
    assert unavailable["status"] == "unavailable"
    assert unavailable["field_dimensions_m"] is None

    state.update_final_demo_state(
        '{"field_dimensions_m":[200,100],"vehicle":"A300",'
        '"stage":"MAP_SAVED","map_sha256":"' + "a" * 64 + '",'
        '"perception_provider":"pc","formal_product_acceptance":false}'
    )
    snapshot = state.snapshot()["final_demo"]
    assert snapshot == {
        "status": "live",
        "reason": None,
        "field_dimensions_m": [200.0, 100.0],
        "vehicle": "A300",
        "stage": "MAP_SAVED",
        "map_sha256": "a" * 64,
        "perception_provider": "pc",
        "formal_product_acceptance": False,
        "source_topic": "/final_demo/state",
        "age_sec": 0.0,
    }


def test_final_product_state_rejects_wrong_profile_and_becomes_stale():
    clock = FakeClock()
    state = LiveMissionState(clock=clock)
    state.update_final_demo_state('{"field_dimensions_m":[16,12]}')
    malformed = state.snapshot()["final_demo"]
    assert malformed["status"] == "error"
    assert malformed["field_dimensions_m"] is None

    state.update_final_demo_state(
        '{"field_dimensions_m":[200,100],"vehicle":"A300",'
        '"stage":"MAPPING","map_sha256":null,'
        '"perception_provider":"unavailable","formal_product_acceptance":false}'
    )
    clock.now = 5.1
    stale = state.snapshot()["final_demo"]
    assert stale["status"] == "stale"
    assert stale["stage"] == "MAPPING"


def test_formal_odom_preview_and_base_command_are_distinct_from_map_and_truth():
    state = LiveMissionState()
    state.update_velocity(0.2, 0.1)
    state.update_formal_odometry_preview(-97.5, 0.2, 0.05)
    state.update_formal_base_command_velocity(0.4, -0.1)
    # The generic command can still arrive, but cannot make the live formal
    # base-command display oscillate between two command sources.
    state.update_velocity(0.1, 0.2)

    snapshot = state.snapshot()

    assert snapshot["vehicle"]["estimated_pose_map"] is None
    assert snapshot["vehicle"]["odometry_preview_pose_odom"] == [-97.5, 0.2, 0.05]
    assert snapshot["vehicle"]["linear_speed_m_s"] == 0.4
    assert snapshot["vehicle"]["angular_speed_rad_s"] == -0.1
    assert snapshot["vehicle"]["speed_source"] == "/base_controller/cmd_vel"
    assert snapshot["visualization"]["odometry_preview_trajectory_odom"] == [[-97.5, 0.2]]
    assert snapshot["claim_boundary"]["odometry_preview_usage"] == (
        "live_odom_frame_preview_not_map_localization_or_truth"
    )


def test_live_map_is_compact_and_keeps_occupied_cells_conservatively():
    state = LiveMissionState()
    width, height = 500, 400
    data = [-1] * (width * height)
    # A grid-aligned occupied point must survive beside free and unknown cells.
    data[9 * width + 12] = 100
    data[50 * width + 50] = 0
    state.update_occupancy_grid(
        width=width,
        height=height,
        resolution=0.05,
        origin_x=-3.0,
        origin_y=2.0,
        data=data,
    )

    grid = state.snapshot()["visualization"]["occupancy_grid"]

    assert grid["width"] <= OCCUPANCY_GRID_MAX_AXIS
    assert grid["height"] <= OCCUPANCY_GRID_MAX_AXIS
    assert len(grid["data"]) == grid["width"] * grid["height"]
    assert grid["downsample_stride"] > 1
    assert 100 in grid["data"]
    assert grid["source_dimensions"] == [500, 400]


def test_live_brush_topic_is_not_overwritten_by_evaluation_sample_history():
    state = LiveMissionState()
    state.update_brush(True)
    state.update_evaluation_sample(
        1.0, 2.0, 0.0, brush_enabled=False, coverage_state="EXECUTING_SWATH"
    )

    cleaning = state.snapshot()["cleaning"]

    assert cleaning["brush_enabled"] is True
    assert cleaning["evaluation_sample_brush_enabled"] is False


def test_operator_values_are_unknown_until_a_fresh_source_message_arrives():
    clock = FakeClock()
    state = LiveMissionState(clock=clock)

    initial = state.snapshot()
    assert initial["vehicle"]["linear_speed_m_s"] is None
    assert initial["cleaning"]["brush_enabled"] is None
    assert initial["cleaning"]["emergency_stop"] is None
    assert initial["live_inputs"]["speed"]["status"] == "unavailable"
    assert initial["live_inputs"]["brush"]["status"] == "unavailable"
    assert initial["live_inputs"]["emergency_stop"]["status"] == "unavailable"

    state.update_velocity(0.4, -0.1)
    state.update_brush(False)
    state.update_emergency_stop(False)
    live = state.snapshot()
    assert live["vehicle"]["linear_speed_m_s"] == 0.4
    assert live["cleaning"]["brush_enabled"] is False
    assert live["cleaning"]["emergency_stop"] is False
    assert all(
        live["live_inputs"][name]["status"] == "live"
        for name in ("speed", "brush", "emergency_stop")
    )

    clock.now = 5.1
    stale = state.snapshot()
    assert all(
        stale["live_inputs"][name]["status"] == "stale"
        for name in ("speed", "brush", "emergency_stop")
    )


def test_mapping_and_saved_map_runtime_sources_are_independent_and_freshness_tracked():
    clock = FakeClock()
    state = LiveMissionState(clock=clock)

    state.update_mapping_lifecycle("mapping_running")
    state.update_mapping_map_ready(False)
    state.update_mapping_explorer("navigating_frontier")
    state.update_saved_map_coverage("TRANSIT")
    live = state.snapshot()["live_inputs"]

    assert live["mapping_lifecycle"]["value"] == "mapping_running"
    assert live["mapping_map_ready"]["value"] is False
    assert live["mapping_explorer"]["value"] == "navigating_frontier"
    assert live["saved_map_coverage"]["value"] == "TRANSIT"
    assert all(
        live[name]["status"] == "live"
        for name in (
            "mapping_lifecycle",
            "mapping_map_ready",
            "mapping_explorer",
            "saved_map_coverage",
        )
    )

    clock.now = 5.1
    stale = state.snapshot()["live_inputs"]
    assert stale["mapping_lifecycle"]["status"] == "stale"
    assert stale["saved_map_coverage"]["status"] == "stale"


def test_live_inputs_are_freshness_tracked_without_any_synthetic_fallback():
    clock = FakeClock()
    state = LiveMissionState(clock=clock)
    unavailable = state.snapshot()["live_inputs"]
    assert unavailable["front_camera"]["status"] == "unavailable"
    assert unavailable["perception_targets"]["count"] is None

    state.update_front_camera(b"real-png", width=848, height=480)
    state.update_perception_targets(3)
    state.update_perception_diagnostics([
        {"name": "dosod", "message": "inference_ok", "level": 0},
        {"name": "edgesam", "message": "inference_ok", "level": 0},
    ])
    live = state.snapshot()["live_inputs"]
    assert live["front_camera"] == {
        "topic": "/sensors/front_rgbd/depth/image_rect_raw/image",
        "error": None,
        "width": 848,
        "height": 480,
        "age_sec": 0.0,
        "status": "live",
    }
    assert live["perception_targets"]["count"] == 3
    assert live["perception_diagnostics"]["statuses"][1]["name"] == "edgesam"
    assert state.front_camera_png() == b"real-png"

    state.update_live_input_error("front_camera", "unsupported encoding")
    errored = state.snapshot()["live_inputs"]["front_camera"]
    assert errored["status"] == "error"
    assert state.front_camera_png() is None

    state.update_front_camera(b"new-real-png", width=848, height=480)

    clock.now = 5.1
    stale = state.snapshot()["live_inputs"]
    assert stale["front_camera"]["status"] == "stale"
    assert stale["perception_targets"]["status"] == "stale"
    assert stale["perception_diagnostics"]["status"] == "stale"
    assert state.front_camera_png() is None
