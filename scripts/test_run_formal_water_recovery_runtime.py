from __future__ import annotations

import json
from pathlib import Path

from finalize_formal_water_recovery_acceptance import combine


ROOT = Path(__file__).resolve().parents[1]


def test_runner_uses_fresh_isolated_launch_for_both_scenarios() -> None:
    source = (ROOT / "scripts/run_formal_water_recovery_runtime.sh").read_text(
        encoding="utf-8"
    )
    assert "--scenario" in source
    assert 'scenario}" != "normal"' in source
    assert 'scenario}" != "full"' in source
    assert "formal_vehicle_sim.launch.py" in source
    assert "water_evaluation_interfaces:=true" in source
    assert "setsid ros2 launch" in source
    assert "cleanup_launch" in source
    assert 'run_scenario normal' in source
    assert 'run_scenario full' in source
    assert "finalize_formal_water_recovery_acceptance.py" in source


def test_aggregate_fails_closed_unless_both_runtime_episodes_pass(tmp_path: Path) -> None:
    normal = tmp_path / "normal.json"
    full = tmp_path / "full.json"
    normal.write_text(
        json.dumps({
            "scenario": "normal_recovery",
            "passed": True,
            "metrics": {"recovery_rate": 0.96},
        }),
        encoding="utf-8",
    )
    full.write_text(
        json.dumps({
            "scenario": "full_tank_fail_closed",
            "passed": False,
        }),
        encoding="utf-8",
    )
    report = combine(normal, full)
    assert report["passed"] is False
    assert report["status"] == "FAILED"

    full.write_text(
        json.dumps({
            "scenario": "full_tank_fail_closed",
            "passed": True,
        }),
        encoding="utf-8",
    )
    report = combine(normal, full)
    assert report["passed"] is True
    assert report["status"] == "FORMAL_WATER_RECOVERY_ACCEPTANCE_PASSED"
