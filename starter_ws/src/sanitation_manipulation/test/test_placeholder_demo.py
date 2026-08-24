from sanitation_manipulation.placeholder_demo import run_placeholder_demo


def test_placeholder_demo_closes_without_claiming_real_robot_evidence():
    summary = run_placeholder_demo()
    assert summary["success"] is True
    assert summary["detected_cube_count"] == 1
    assert summary["grasp_candidate_count"] == 2
    assert summary["task_state"] == "CLEARED"
    assert summary["placed_in_bin"] is True
    assert summary["placeholder_evidence_only"] is True
    assert summary["real_robot_evidence"] is False
    assert summary["gazebo_truth_used_for_control"] is False
