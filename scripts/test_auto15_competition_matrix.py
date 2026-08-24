from __future__ import annotations

import json
from pathlib import Path

from auto15_competition_matrix import build_matrix


ROOT = Path(__file__).resolve().parents[1]


def test_matrix_is_complete_and_fail_closed() -> None:
    state = json.loads(
        (ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json").read_text(
            encoding="utf-8"
        )
    )
    matrix = build_matrix(state)
    assert matrix["scenario_count"] == 18
    assert matrix["simulation_competition_matrix_pass"] is False
    assert matrix["executed_integrated_missions"] == 0
    assert "AUTO-08" in matrix["blocking_dependencies"]
    assert all(
        row["integrated_execution_status"] == "NOT_EXECUTED"
        for row in matrix["scenarios"]
    )


def test_passing_components_are_not_promoted_to_integrated_results() -> None:
    state = json.loads(
        (ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json").read_text(
            encoding="utf-8"
        )
    )
    matrix = build_matrix(state)
    app = next(row for row in matrix["scenarios"] if row["scenario_id"] == "app")
    assert app["component_evidence_status"] == "AVAILABLE"
    assert app["formal_mission_count"] == 0
    assert app["video_count"] == 0
    assert app["mcap_count"] == 0
