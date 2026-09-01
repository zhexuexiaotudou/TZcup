from __future__ import annotations

from pathlib import Path

from audit_formal_water_launch_log import RULES, STABLE_MARKER, audit


def test_stable_marker_matches_the_fail_closed_inactive_controller_loader() -> None:
    assert STABLE_MARKER == "[spawner_brush_controller]: Loaded recovery_controller"


def _allowed_lines() -> list[str]:
    return [
        '[gazebo-1] Warning [Utils.cc:132] [/sdf/model/link/sensor[@name="vn100"]/gz_frame_id:<urdf-string>:L0]: XML Element[gz_frame_id], child not defined in SDF',
        '[gazebo-1] Warning [Utils.cc:132] [/sdf/model/link/sensor[@name="zed_f9p"]/gz_frame_id:<urdf-string>:L0]: XML Element[gz_frame_id], child not defined in SDF',
        '[gazebo-1] Warning [Utils.cc:132] [/sdf/model/link/sensor[@name="utm30lx"]/gz_frame_id:<urdf-string>:L0]: XML Element[gz_frame_id], child not defined in SDF',
        "[gazebo-1] [WARN] [1.0] [gz_ros_control]: Waiting RM to load and initialize hardware...",
        "[gazebo-1] [WARN] [1.1] [gz_ros_control]: IMU sensor 'vn100' not found in hardware_info, skipping.",
        "[gazebo-1] [WARN] [1.2] [controller_manager.hardware_component.system.formal_vehicle_system]: Executor is not available during hardware component initialization for 'formal_vehicle_system'. Skipping node creation!",
        "[gazebo-1] [WARN] [1.3] [controller_manager]: Component 'formal_vehicle_system' does not have read or write statistics initialized, skipping registration.",
        "[gazebo-1] [WARN] [1.4] [gz_ros_control]:  Desired controller update period (0.004 s) is slower than the gazebo simulation period (0.001 s).",
    ]


def _write(tmp_path: Path, lines: list[str]) -> Path:
    path = tmp_path / "launch.log"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def test_exact_known_startup_warnings_pass(tmp_path: Path) -> None:
    report = audit(_write(tmp_path, _allowed_lines() + [STABLE_MARKER, "healthy"]))
    assert report["passed"] is True
    assert len(report["rules"]) == len(RULES)


def test_new_repeated_or_runtime_warning_fails(tmp_path: Path) -> None:
    allowed = _allowed_lines()
    reports = (
        audit(_write(tmp_path, allowed + ["[gazebo-1] [WARN] new warning", STABLE_MARKER])),
        audit(_write(tmp_path, allowed + [allowed[3], STABLE_MARKER])),
        audit(_write(tmp_path, allowed + [STABLE_MARKER, allowed[3]])),
    )
    assert all(report["passed"] is False for report in reports)
    assert reports[2]["checks"][
        "zero_warning_or_error_at_or_after_stable_window"
    ] is False


def test_error_and_missing_stable_marker_fail(tmp_path: Path) -> None:
    report = audit(_write(tmp_path, _allowed_lines() + ["[node-2] [ERROR] bad"]))
    assert report["passed"] is False
    assert report["checks"]["expected_stable_window_marker_count"] is False
    assert report["checks"]["zero_unexpected_warning_or_error_lines"] is False


def test_explicit_diagnostic_stable_marker_is_supported(tmp_path: Path) -> None:
    diagnostic_marker = "[DIAG_STABLE] typed cleaning motor telemetry ready"
    report = audit(
        _write(tmp_path, _allowed_lines() + [diagnostic_marker, "healthy"]),
        stable_marker=diagnostic_marker,
    )
    assert report["passed"] is True
    assert report["stable_window_marker"] == diagnostic_marker
    assert report["stable_window_marker_lines"] == [len(_allowed_lines()) + 1]


def test_two_expected_markers_bind_stable_window_to_final_activation(
    tmp_path: Path,
) -> None:
    first_marker_line = len(_allowed_lines()) + 1
    second_marker_line = first_marker_line + 2
    report = audit(
        _write(
            tmp_path,
            _allowed_lines()
            + [STABLE_MARKER, "preflight controller deactivation", STABLE_MARKER, "healthy"],
        ),
        expected_stable_marker_count=2,
    )
    assert report["passed"] is True
    assert report["expected_stable_window_marker_count"] == 2
    assert report["stable_window_marker_lines"] == [
        first_marker_line,
        second_marker_line,
    ]
    assert report["stable_window_start_line"] == second_marker_line
    assert report["runtime_diagnostics"] == []


def test_expected_marker_count_is_exact_and_unexpected_errors_fail_globally(
    tmp_path: Path,
) -> None:
    count_mismatch = audit(
        _write(tmp_path, _allowed_lines() + [STABLE_MARKER, "healthy"]),
        expected_stable_marker_count=2,
    )
    assert count_mismatch["passed"] is False
    assert count_mismatch["checks"]["expected_stable_window_marker_count"] is False

    unexpected_before_final_marker = audit(
        _write(
            tmp_path,
            _allowed_lines()
            + [
                STABLE_MARKER,
                "[gazebo-1] [ERROR] NodeShared::Publish() Error: Interrupted system call",
                STABLE_MARKER,
                "healthy",
            ],
        ),
        expected_stable_marker_count=2,
    )
    assert unexpected_before_final_marker["passed"] is False
    assert unexpected_before_final_marker["checks"][
        "zero_unexpected_warning_or_error_lines"
    ] is False
    assert unexpected_before_final_marker["runtime_diagnostics"] == []


def test_expected_marker_count_must_be_positive(tmp_path: Path) -> None:
    try:
        audit(_write(tmp_path, _allowed_lines() + [STABLE_MARKER]), expected_stable_marker_count=0)
    except ValueError as error:
        assert str(error) == "expected_stable_marker_count must be at least 1"
    else:
        raise AssertionError("expected a ValueError for a zero marker count")
