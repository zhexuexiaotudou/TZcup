"""Windows-only dry-run regression tests for the final four physical chains."""

from __future__ import annotations

from pathlib import Path

import run_formal_final_acceptance as orchestration


ROOT = Path(__file__).resolve().parents[1]


def _context(tmp_path: Path) -> orchestration.Context:
    runtime_ws = tmp_path / "frozen_runtime"
    return orchestration.Context(
        root=ROOT,
        runtime_ws=runtime_ws,
        integrated_build_manifest=runtime_ws / "integrated_build_manifest.json",
        perception_artifacts=tmp_path / "perception_assets",
        onnx_pythonpath=tmp_path / "onnx_overlay",
        run_root=tmp_path / "new_session",
        base_domain=90,
        episode_count=30,
    )


def test_windows_dry_run_lists_every_missing_frozen_runtime_handoff(
    tmp_path: Path, monkeypatch
) -> None:
    context = _context(tmp_path)

    def forbidden(*args, **kwargs):
        raise AssertionError(f"Windows dry-run must not start a subprocess: {args!r}")

    monkeypatch.setattr(orchestration.subprocess, "run", forbidden)
    result = orchestration.windows_dry_run(context)

    assert result["passed"] is False
    assert result["status"] == "FORMAL_FINAL_RUNTIME_WINDOWS_DRY_RUN_BLOCKED"
    assert result["execution_scope"] == {
        "windows_python_read_only": True,
        "started_wsl": False,
        "started_gazebo": False,
        "started_cadquery": False,
        "started_freecad": False,
        "system_bash_exe_invoked": False,
        "runtime_evidence_created": False,
    }
    assert {row["category"] for row in result["missing_requirements"]} >= {
        "closure_manifest",
        "build",
        "install",
        "build_log_directory",
        "integrated_build_manifest",
        "install_symlink_manifest",
        "windows_cold_start_manifest",
    }


def test_windows_dry_run_renders_the_final_four_chain_order_without_execution(
    tmp_path: Path,
) -> None:
    result = orchestration.windows_dry_run(_context(tmp_path))
    runners = result["four_chain_runner_order"]

    assert [row["step_id"] for row in runners] == [
        "chassis",
        "ground_dirt",
        "water_recovery",
        "physical_grasp",
    ]
    assert [row["runner"] for row in runners] == [
        "run_formal_vehicle_mobility_runtime.sh",
        "run_formal_ground_dirt_cleaning_runtime.sh",
        "run_formal_water_recovery_runtime.sh",
        "run_formal_grasp_executor_runtime.sh",
    ]
    assert runners[2]["requires_verified_typed_subclosure"] is True
    assert all(row["requires_fresh_resource_gate_before_launch"] for row in runners)
    assert len(runners[2]["commands"]) == 3
    assert all(row["windows_execution_permitted"] is False for row in runners)
    assert result["water_command_placeholder_only"] is True
    strict = result["strict_memory_recovery_contract"]
    assert strict["phase_order"] == [
        "windows_cold_memory_gate_and_single_worker_frozen_build",
        "record_and_verify_frozen_runtime_closure",
        "chassis",
        "ground_dirt",
        "water_recovery",
        "physical_grasp",
    ]
    assert strict["resource_gate"]["required_before_each_heavy_phase"] is True
    assert "rl_policy" in strict["resource_gate"]["heavy_step_ids"]
    assert strict["four_chain"] == {
        "strict_serial_execution": True,
        "stop_on_first_failure": True,
        "preserve_unfinalized_session_on_failure": True,
    }
    assert strict["parallelism"] == {
        "gazebo_max_parallel_processes": 1,
        "cad_execution_permitted": False,
        "board_execution_automatic": False,
    }


def test_windows_dry_run_is_exposed_as_an_explicit_cli_mode() -> None:
    args = orchestration.build_parser().parse_args(
        [
            "--windows-dry-run",
            "--runtime-ws",
            "C:/frozen-runtime",
            "--integrated-build-manifest",
            "C:/frozen-runtime/integrated_build_manifest.json",
        ]
    )
    assert args.windows_dry_run is True
