import pytest

from sanitation_manipulation.core import (
    BinState,
    BRUSH_ONLY_CLASSES,
    ManipulationController,
    Target,
    generate_grasps,
    simulate_trial,
    transform_point,
)


def test_perception_to_grasp_transform():
    matrix = [
        [1.0, 0.0, 0.0, 0.10],
        [0.0, 1.0, 0.0, -0.05],
        [0.0, 0.0, 1.0, 0.20],
        [0.0, 0.0, 0.0, 1.0],
    ]
    assert transform_point(matrix, (0.2, 0.1, 0.0)) == pytest.approx(
        (0.3, 0.05, 0.2)
    )


def test_leaf_and_puddle_never_enter_grasp_pipeline():
    for class_id in BRUSH_ONLY_CLASSES:
        target = Target("target", class_id, 0.4, 0.0, 0.05, 0.001)
        assert generate_grasps(target) == []


def test_reachable_trials_complete_without_collision_or_joint_violation():
    for class_id in ("plastic_bottle", "metal_can", "paper_litter"):
        results = [simulate_trial(class_id, seed, False) for seed in range(30)]
        assert sum(row["pick_success"] for row in results) >= 27
        assert sum(row["collision_count"] for row in results) == 0
        assert sum(row["joint_limit_violation_count"] for row in results) == 0
        assert all(not row["truth_used_for_control"] for row in results)


def test_unreachable_and_estop_fail_closed():
    target = Target("far", "plastic_bottle", 1.2, 0.0, 0.05, 0.001)
    controller = ManipulationController()
    result = controller.execute(target, BinState(), 0.5)
    assert not result["success"]
    assert result["reason"] == "unreachable_fail_closed"
    controller.emergency_stop()
    stopped = controller.execute(target, BinState(), 0.5)
    assert stopped["reason"] == "estop_active"
    assert stopped["terminal_state"] == "FAILED_SAFE"


def test_40_l_bin_rejects_overfill_and_routes_full():
    bin_state = BinState()
    assert bin_state.capacity_l >= 40.0
    assert bin_state.reserve(39.8)
    controller = ManipulationController()
    target = Target("bottle", "plastic_bottle", 0.4, 0.0, 0.05, 0.001)
    result = controller.execute(target, bin_state, 0.5)
    assert not result["success"]
    assert result["bin_full_route"]
    assert bin_state.observable["fill_l"] == pytest.approx(39.8)
