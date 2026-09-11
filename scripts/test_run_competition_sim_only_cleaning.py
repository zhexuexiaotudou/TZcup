from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_competition_sim_only_cleaning.sh"


def test_cleaning_runner_isolated_from_a12_and_cube_route():
    text = RUNNER.read_text(encoding="utf-8")
    assert "run_formal_saved_map_cleaning_lifecycle.sh" in text
    assert "run_formal_first_map_dynamic_prerequisite.sh" in text
    assert "run_formal_ground_dirt_cleaning_runtime.sh" in text
    assert "run_formal_dynamic_obstacle_avoidance.sh" in text
    assert "run_formal_single_episode_cleaning_mission.sh" not in text
    assert "FORMAL_A12_SCENARIO" in text
    assert "FORMAL_CUBE_TARGET_COUNT" in text
    assert "validate_competition_sim_only_cleaning.py" in text
    assert "FORMAL_DYNAMIC_SAVED_MAP_ROOT=\"${map_root}\"" in text
    assert "FORMAL_COMPETITION_SIM_ONLY_RUNTIME_WS" in text
    assert "FORMAL_COMPETITION_SIM_ONLY_CLOSURE_MANIFEST" in text
    assert "sanitation-campus-scenario generate" in text
    assert "FORMAL_DYNAMIC_EPISODE_ROOT=\"${episode}\"" in text
    for script in (
        "run_formal_first_map_dynamic_prerequisite.sh",
        "run_formal_saved_map_cleaning_lifecycle.sh",
        "run_formal_ground_dirt_cleaning_runtime.sh",
        "run_formal_dynamic_obstacle_avoidance.sh",
    ):
        assert f'bash "${{repo_root}}/scripts/{script}"' in text
    assert "SIM_DEMO_VERIFIED" not in text
    assert "Golden Mission" not in text
