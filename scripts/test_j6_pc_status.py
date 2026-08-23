from j6_pc_status import evaluate


def test_j6_status_is_fail_closed_when_evidence_is_missing():
    status, blockers = evaluate(
        model_inventory={},
        model_selection={},
        sdk_inventory={},
        loopback_report={},
        bundle_manifest={},
    )
    assert not any(status["statuses"].values())
    assert len(blockers["blockers"]) == 4
    assert status["board_metrics"]["FPS"] is None
    assert status["board_metrics"]["board_30_seed"] == "not_run"


def test_j6_status_requires_every_loopback_safety_fact():
    report = {
        "duration_s": 1800,
        "gt_control_violation_count": 0,
        "pc_duplicate_algorithm_node_count": 0,
        "j6_command_authority_pass": True,
        "command_timeout_safe_stop": True,
        "network_loss_safe_stop": True,
        "stale_command_replay_count": 0,
    }
    status, _ = evaluate(
        model_inventory={},
        model_selection={},
        sdk_inventory={},
        loopback_report=report,
        bundle_manifest={},
    )
    assert status["statuses"]["J6_LOOPBACK_HIL_READY"] is True
    report.pop("network_loss_safe_stop")
    status, _ = evaluate(
        model_inventory={},
        model_selection={},
        sdk_inventory={},
        loopback_report=report,
        bundle_manifest={},
    )
    assert status["statuses"]["J6_LOOPBACK_HIL_READY"] is False
