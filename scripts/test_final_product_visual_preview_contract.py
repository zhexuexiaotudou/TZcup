from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = (ROOT / "scripts/run_final_product_visual_preview.sh").read_text(encoding="utf-8")
WINDOWS = (ROOT / "scripts/run_final_product_visual_preview.ps1").read_text(encoding="utf-8")


def test_preview_uses_final_a300_map_then_hard_restart_topology() -> None:
    assert "FINAL_VISUAL_PREVIEW_NOT_PRODUCT_PASS" in RUNNER
    assert '"product_pass": False' in RUNNER
    assert '"field_dimensions_m": [200, 100]' in RUNNER
    assert '"vehicle": "A300"' in RUNNER
    assert RUNNER.count("formal_campus_map_lifecycle.launch.py") == 2
    assert "mission_mode:=mapping gui:=true" in RUNNER
    assert "mission_mode:=cleaning cleaning_planner:=full_coverage gui:=true" in RUNNER
    assert RUNNER.index("mission_mode:=mapping") < RUNNER.index("mission_mode:=cleaning")
    assert "write_state HARD_RESTART" in RUNNER
    assert "stop_pid \"${mapping_launch_pid}\"" in RUNNER


def test_preview_reuses_live_hmi_and_refuses_formal_acceptance_claims() -> None:
    assert "python3 -m sanitation_hmi.live_server" in RUNNER
    assert "publish_final_product_visual_state.py" in RUNNER
    assert "perception_provider\": \"unavailable\"" in RUNNER
    assert "formal_product_acceptance\": False" in RUNNER
    assert "run_formal_first_map_dynamic_prerequisite.sh" not in RUNNER
    assert "run_formal_saved_map_cleaning_lifecycle.sh" not in RUNNER
    assert "AUTO-17" not in RUNNER and "Ackermann" not in RUNNER


def test_preview_is_fresh_scoped_and_has_a_windows_preflight_entry() -> None:
    assert "refusing stale preview run root" in RUNNER
    assert "preview run root must remain below TZcup project root" in RUNNER
    assert "preview runtime workspace must remain below TZcup project root" in RUNNER
    assert "preview episode root must remain below TZcup project root" in RUNNER
    assert "--preflight" in RUNNER
    assert "setsid ros2 launch" in RUNNER
    assert 'coverage_report="${cleaning_root}/coverage_execution.json"' in RUNNER
    assert 'report.get("terminal_state") != "COMPLETED"' in RUNNER
    assert 'kill -KILL -- "-${pid}"' in RUNNER
    assert "MappingRosDomain and CleaningRosDomain must differ" in WINDOWS
    assert "run_final_product_visual_preview.sh" in WINDOWS
    assert "Replace('\\', '/')" in WINDOWS
