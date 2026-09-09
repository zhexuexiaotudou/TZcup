from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAPPING = (ROOT / "scripts/run_formal_first_map_dynamic_prerequisite.sh").read_text(encoding="utf-8")
CLEANING = (ROOT / "scripts/run_formal_saved_map_cleaning_lifecycle.sh").read_text(encoding="utf-8")
RUNNER = (ROOT / "scripts/run_final_product_visual_demo.sh").read_text(encoding="utf-8")
WINDOWS = (ROOT / "scripts/run_final_product_visual_demo.ps1").read_text(encoding="utf-8")
PUBLISHER = (ROOT / "scripts/publish_final_product_visual_state.py").read_text(encoding="utf-8")


def test_formal_gui_override_is_strict_and_only_replaces_launch_value() -> None:
    for source in (MAPPING, CLEANING):
        assert 'FORMAL_VISUAL_GUI="${FORMAL_VISUAL_GUI:-false}"' in source
        assert '"${FORMAL_VISUAL_GUI}" != "false"' in source
        assert '"${FORMAL_VISUAL_GUI}" != "true"' in source
        assert 'gui:="${FORMAL_VISUAL_GUI}"' in source
    assert MAPPING.count('gui:="${FORMAL_VISUAL_GUI}"') == 1
    assert CLEANING.count('gui:="${FORMAL_VISUAL_GUI}"') == 2


def test_final_visual_runner_is_fresh_serial_and_never_redefines_contracts() -> None:
    assert 'refusing stale final-product visual run root' in RUNNER
    assert 'map root and terminal output must be below the fresh run root' in RUNNER
    assert 'bash "${repo_root}/scripts/run_formal_first_map_dynamic_prerequisite.sh"' in RUNNER
    assert 'bash "${repo_root}/scripts/run_formal_saved_map_cleaning_lifecycle.sh"' in RUNNER
    assert RUNNER.index('run_formal_first_map_dynamic_prerequisite.sh') < RUNNER.index('run_formal_saved_map_cleaning_lifecycle.sh')
    assert 'FORMAL_VISUAL_GUI="${formal_visual_gui}"' in RUNNER
    assert 'FORMAL_VEHICLE_SNAPSHOT_MANIFEST="${vehicle_snapshot}"' in RUNNER
    assert 'ROS_DOMAIN_ID="${mapping_ros_domain}"' in RUNNER
    assert '"ROS_DOMAIN_ID=${cleaning_ros_domain}"' in RUNNER
    assert 'mission_mode:=mapping' not in RUNNER
    assert 'ros2 launch' not in RUNNER
    assert 'python3 -m sanitation_hmi.live_server' in RUNNER
    assert '-p web_root:="${repo_root}/starter_ws/src/sanitation_hmi/web"' in RUNNER
    assert 'ros2 run sanitation_hmi sanitation_live_dashboard' not in RUNNER
    assert 'publish_final_product_visual_state.py' in RUNNER
    assert 'write_final_demo_state MAPPING ""' in RUNNER
    assert 'write_final_demo_state MAP_SAVED "${map_sha256}"' in RUNNER
    assert 'write_final_demo_state HARD_RESTART "${map_sha256}"' in RUNNER
    assert 'write_final_demo_state RELOAD_LOCALIZE "${map_sha256}"' in RUNNER
    assert 'write_final_demo_state COVERAGE "${map_sha256}"' in RUNNER
    assert 'write_final_demo_state PRODUCT_TERMINAL "${map_sha256}"' in RUNNER
    assert '"final_demo_stage": "PRODUCT_TERMINAL"' in RUNNER
    assert '"--dashboard-port", "$DashboardPort"' in WINDOWS
    assert '"--dashboard-output"' in WINDOWS


def test_perception_modes_are_fail_closed_and_unavailable_has_terminal_boundary() -> None:
    assert 'unavailable|pc|s100p' in RUNNER
    assert 'unavailable perception permits only full_coverage' in RUNNER
    assert 'pc perception requires rl_dirt_priority plus an artifact directory and policy checkpoint' in RUNNER
    assert 'PERCEPTION_BLOCKED_NOT_PRODUCT_PASS' in RUNNER
    assert 'product_pass": False' in RUNNER
    s100p_block = RUNNER[RUNNER.index('  s100p)'):RUNNER.index('esac\n\nif "${preflight_only}"')]
    assert 'formal board bridge and receipt are both required; no fallback is allowed' in s100p_block
    assert 'refusing PC or full-coverage fallback' in s100p_block
    assert 'run_formal_saved_map_cleaning_lifecycle.sh' not in s100p_block


def test_windows_entry_preserves_the_explicit_mode_and_wsl_forwarding_contract() -> None:
    assert '[ValidateSet("unavailable", "pc", "s100p")]' in WINDOWS
    assert '[ValidateSet("true", "false")]' in WINDOWS
    assert 'MappingRosDomain and CleaningRosDomain must differ' in WINDOWS
    assert '"--vehicle-snapshot"' in WINDOWS
    assert '"--formal-visual-gui", $FormalVisualGui' in WINDOWS
    assert '"--s100p-board-bridge"' in WINDOWS
    assert 'run_final_product_visual_demo.sh' in WINDOWS
    assert "Replace('\\', '/')" in WINDOWS


def test_live_state_publisher_is_one_hz_and_refuses_acceptance_claims() -> None:
    assert 'create_publisher(String, "/final_demo/state", qos)' in PUBLISHER
    assert 'self.create_timer(period_sec, self._publish)' in PUBLISHER
    assert 'args.period_sec != 1.0' in PUBLISHER
    assert 'payload.get("formal_product_acceptance") is not False' in PUBLISHER
    assert '"field_dimensions_m") != [200, 100]' in PUBLISHER
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in PUBLISHER


def test_demo_runner_fails_closed_on_visual_progress_and_hmi_receipts() -> None:
    assert "phase_progress_timeout_sec=600" in RUNNER
    assert 'canonical scan ready; starting standard autostart SLAM lifecycle' in RUNNER
    assert "dashboard_has_first_map" in RUNNER
    assert "mapping progress watchdog timed out" in RUNNER
    assert "cleaning progress watchdog timed out before hard-restart receipt" in RUNNER
    assert 'http.client.HTTPConnection("127.0.0.1", int(port), timeout=1.0)' in RUNNER
    assert "urllib.request" not in RUNNER
    assert 'wait_for_hmi_receipt MAPPING ""' in RUNNER
    assert 'wait_for_hmi_receipt HARD_RESTART "${map_sha256}"' in RUNNER
    assert 'wait_for_hmi_receipt COVERAGE "${map_sha256}"' in RUNNER
    assert 'wait_for_hmi_receipt PRODUCT_TERMINAL "${map_sha256}"' in RUNNER
    assert RUNNER.index('wait_for_hmi_receipt PRODUCT_TERMINAL "${map_sha256}"') < RUNNER.index(
        "FORMAL_FINAL_PRODUCT_VISUAL_TERMINAL"
    )
    assert '"hmi_terminal_telemetry_sha256"' in RUNNER
    assert '"artifact_sha256"' in RUNNER
    assert '"hard_restart_record"' in RUNNER
    assert '"mapping_handoff"' in RUNNER


def test_demo_runner_reaps_exact_child_runners_on_signal() -> None:
    assert 'mapping_runner_pid=""' in RUNNER
    assert 'cleaning_runner_pid=""' in RUNNER
    assert "stop_exact_child()" in RUNNER
    assert 'stop_exact_child "${cleaning_runner_pid}"' in RUNNER
    assert 'stop_exact_child "${mapping_runner_pid}"' in RUNNER
    assert "trap stop_visual_stack EXIT" in RUNNER
    assert "trap 'exit 130' INT" in RUNNER
    assert "trap 'exit 143' TERM" in RUNNER
    assert "coverage_stage_monitor_pid" not in RUNNER
