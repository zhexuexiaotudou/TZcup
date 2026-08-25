import argparse
import json
import os
import time
from pathlib import Path
from unittest.mock import patch

import pytest

import aggregate_integrated_functional_acceptance as acceptance


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def scenario_result(name: str) -> dict:
    common = {"passed": True}
    if name == "mobility":
        return {
            **common,
            "report_id": "tzcup_formal_vehicle_mobility_runtime_v1",
            "status": "FORMAL_VEHICLE_FORWARD_STOP_RUNTIME_PASSED",
        }
    if name in ("water_normal", "water_full"):
        return {
            **common,
            "schema_version": 1,
            "scenario": "normal_recovery" if name == "water_normal" else "full_tank_fail_closed",
            "status": "FORMAL_WATER_RECOVERY_SCENARIO_PASSED",
        }
    return {
        **common,
        "report_id": "tzcup_formal_physical_cube_pick_place_v1",
        "status": "PHYSICAL_CONTACT_GATED_PICK_LIFT_DEPOSIT_PASSED",
        "grasp_gate": {
            "attach_permitted": True,
            "left_cube_contact_count": 2,
            "right_cube_contact_count": 2,
            "state_ack_observed": False,
        },
        "attachment_constraint_proof": {
            "transport_state_ack_observed": False,
            "constraint_proven_by_rigid_motion_not_ack": True,
            "cube_lift_m": 0.24,
            "offset_change_m": 0.003,
        },
        "cube": {
            "present_after_deposit": True,
            "stable_inside_dry_bin": True,
            "settled_sim_duration_s": 4.0,
            "bin_floor_support_z_m": 0.469,
            "bin_floor_support_tolerance_m": 0.020,
            "settled_pose_m": {"x": -0.2, "y": 0.15, "z": 0.470},
        },
        "bin_load_bearing_contact": {
            "support_contact_count": 12,
            "vehicle_collision_names": ["dry_bin_floor_collision"],
            "support_contact_span_sim_s": 0.8,
            "persistent_support_observed": True,
        },
    }


def complete_context(tmp_path: Path) -> tuple[Path, dict, dict]:
    build_path = tmp_path / "build.json"
    write_json(build_path, {"snapshot": "immutable"})
    now = time.time_ns()
    rows = {}
    for index, name in enumerate(acceptance.SCENARIOS):
        result_path = tmp_path / f"{name}.json"
        write_json(result_path, scenario_result(name))
        launch_log = tmp_path / f"{name}.launch.log"
        runner_log = tmp_path / f"{name}.runner.log"
        launch_log.write_text("launch evidence\n", encoding="utf-8")
        runner_log.write_text("runner evidence\n", encoding="utf-8")
        artifact_ns = now + index * 20_000_000
        for artifact in (result_path, launch_log, runner_log):
            os.utime(artifact, ns=(artifact_ns, artifact_ns))
        rows[name] = {
            "started_epoch_ns": artifact_ns - 5_000_000,
            "finished_epoch_ns": artifact_ns + 5_000_000,
            "exit_code": 0,
            "ros_domain_id": 180 + index,
            "gz_partition": f"unique_{name}",
            "result": str(result_path),
            "result_sha256": acceptance.sha256_file(result_path),
            "launch_log": str(launch_log),
            "launch_log_sha256": acceptance.sha256_file(launch_log),
            "runner_log": str(runner_log),
            "runner_log_sha256": acceptance.sha256_file(runner_log),
            "cleanup_remaining_pids": 0,
        }
    context = {
        "schema_version": 1,
        "kind": "tzcup_integrated_acceptance_run_context",
        "run_id": "unit_test",
        "repo_root": str(tmp_path),
        "runtime_ws": str(tmp_path / "runtime"),
        "build_manifest": str(build_path),
        "build_manifest_sha256": acceptance.sha256_file(build_path),
        "started_epoch_ns": now - 1_000_000_000,
        "started_utc": acceptance.utc_iso(now - 1_000_000_000),
        "scenarios": rows,
    }
    build = {
        "build_started_epoch_ns": now - 3_000_000_000,
        "recorded_epoch_ns": now - 2_000_000_000,
        "git": {"commit": "a" * 40, "tree": "b" * 40, "dirty": False, "dirty_diff_sha256": "c" * 64},
        "source_inventory_sha256": "d" * 64,
        "installed_inventory_sha256": "e" * 64,
        "source_inventory": {
            "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc": {
                "sha256": "f" * 64
            },
            "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro": {
                "sha256": "1" * 64
            },
            "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/storage_system.xacro": {
                "sha256": "2" * 64
            },
        },
        "installed_inventory": {
            "install/sanitation_gazebo_control/lib/libDynamicPayloadSystem.so": {
                "sha256": "9" * 64
            },
            "install/sanitation_gazebo_control/lib/libWaterRecoverySystem.so": {
                "sha256": "0" * 64
            }
        },
        "runtime": {
            "ros_distro": "jazzy",
            "ros_base_package": "0.11.0-1noble.20260801.000000",
            "gazebo": "Gazebo Sim, version 8.11.0",
            "physics_engine": "gz-physics-bullet-featherstone-plugin",
        },
    }
    context_path = tmp_path / "context.json"
    write_json(context_path, context)
    return context_path, context, build


def aggregate_args(context: Path, output: Path) -> argparse.Namespace:
    return argparse.Namespace(
        context=context,
        output=output,
        finished_epoch_ns=time.time_ns() + 2_000_000_000,
    )


def test_complete_fresh_isolated_scenario_set_passes(tmp_path: Path) -> None:
    context_path, context, build = complete_context(tmp_path)
    output = tmp_path / "manifest.json"
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        assert acceptance.aggregate(aggregate_args(context_path, output)) == 0
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["passed"] is True
    assert report["source_bound"] is True
    assert set(report["scenario_results"]) == set(acceptance.SCENARIOS)
    assert any(name.endswith("WaterRecoverySystem.cc") for name in report["critical_file_sha256"])
    assert any(name.endswith("libWaterRecoverySystem.so") for name in report["critical_file_sha256"])
    assert any(name.endswith("urdf/high_fidelity/cleaning_mechanism.xacro") for name in report["critical_file_sha256"])
    assert any(name.endswith("urdf/high_fidelity/storage_system.xacro") for name in report["critical_file_sha256"])
    assert len({row["ros_domain_id"] for row in context["scenarios"].values()}) == 4
    assert len({row["gz_partition"] for row in context["scenarios"].values()}) == 4


def test_missing_scenario_fails_closed(tmp_path: Path) -> None:
    context_path, context, build = complete_context(tmp_path)
    del context["scenarios"]["water_full"]
    write_json(context_path, context)
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        with pytest.raises(acceptance.AcceptanceError, match="scenario set mismatch"):
            acceptance.aggregate(aggregate_args(context_path, tmp_path / "out.json"))


@pytest.mark.parametrize("mutation", ("passed", "exit_code", "status", "cleanup"))
def test_child_failure_propagates(tmp_path: Path, mutation: str) -> None:
    context_path, context, build = complete_context(tmp_path)
    row = context["scenarios"]["manipulation"]
    if mutation == "passed":
        result_path = Path(row["result"])
        result = json.loads(result_path.read_text())
        result["passed"] = False
        write_json(result_path, result)
        row["result_sha256"] = acceptance.sha256_file(result_path)
    elif mutation == "exit_code":
        row["exit_code"] = 7
    elif mutation == "status":
        result_path = Path(row["result"])
        result = json.loads(result_path.read_text())
        result["status"] = "FAILED"
        write_json(result_path, result)
        row["result_sha256"] = acceptance.sha256_file(result_path)
    else:
        row["cleanup_remaining_pids"] = 1
    write_json(context_path, context)
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        with pytest.raises(acceptance.AcceptanceError):
            acceptance.aggregate(aggregate_args(context_path, tmp_path / "out.json"))


def test_old_result_cannot_be_spliced_into_new_run(tmp_path: Path) -> None:
    context_path, context, build = complete_context(tmp_path)
    for row in context["scenarios"].values():
        row["started_epoch_ns"] += 10_000_000_000
        row["finished_epoch_ns"] += 10_000_000_000
    write_json(context_path, context)
    args = aggregate_args(context_path, tmp_path / "out.json")
    args.finished_epoch_ns = max(
        row["finished_epoch_ns"] for row in context["scenarios"].values()
    ) + 1
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        with pytest.raises(acceptance.AcceptanceError, match="stale"):
            acceptance.aggregate(args)


def test_manipulation_transport_ack_is_not_the_acceptance_truth(tmp_path: Path) -> None:
    context_path, _, build = complete_context(tmp_path)
    output = tmp_path / "manifest.json"
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        acceptance.aggregate(aggregate_args(context_path, output))
    result = json.loads(output.read_text(encoding="utf-8"))["scenario_results"]["manipulation"]
    assert result["grasp_gate"]["state_ack_observed"] is False
    assert result["attachment_constraint_proof"]["constraint_proven_by_rigid_motion_not_ack"] is True


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        ("not_present", "not physically present"),
        ("not_stable", "not stable"),
        ("no_support_contact", "load-bearing contact"),
        ("short_settle", "less than 3 simulated seconds"),
        ("unsupported_height", "outside floor support tolerance"),
    ),
)
def test_manipulation_deposit_physics_fail_closed(
    tmp_path: Path, mutation: str, message: str
) -> None:
    context_path, context, build = complete_context(tmp_path)
    row = context["scenarios"]["manipulation"]
    result_path = Path(row["result"])
    result = json.loads(result_path.read_text(encoding="utf-8"))
    if mutation == "not_present":
        result["cube"]["present_after_deposit"] = False
    elif mutation == "not_stable":
        result["cube"]["stable_inside_dry_bin"] = False
    elif mutation == "no_support_contact":
        result["bin_load_bearing_contact"]["support_contact_count"] = 0
    elif mutation == "short_settle":
        result["cube"]["settled_sim_duration_s"] = 2.99
    else:
        result["cube"]["settled_pose_m"]["z"] = 0.500
    write_json(result_path, result)
    row["result_sha256"] = acceptance.sha256_file(result_path)
    write_json(context_path, context)
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        with pytest.raises(acceptance.AcceptanceError, match=message):
            acceptance.aggregate(aggregate_args(context_path, tmp_path / "out.json"))


@pytest.mark.parametrize("field", ("ros_domain_id", "gz_partition"))
def test_scenarios_must_be_transport_isolated(tmp_path: Path, field: str) -> None:
    context_path, context, build = complete_context(tmp_path)
    context["scenarios"]["water_normal"][field] = context["scenarios"]["mobility"][field]
    write_json(context_path, context)
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        with pytest.raises(acceptance.AcceptanceError, match="unique"):
            acceptance.aggregate(aggregate_args(context_path, tmp_path / "out.json"))


@pytest.mark.parametrize("domain", (-1, 233))
def test_ros_domain_must_be_within_dds_range(tmp_path: Path, domain: int) -> None:
    context_path, context, build = complete_context(tmp_path)
    context["scenarios"]["water_normal"]["ros_domain_id"] = domain
    write_json(context_path, context)
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        with pytest.raises(acceptance.AcceptanceError, match="0..232"):
            acceptance.aggregate(aggregate_args(context_path, tmp_path / "out.json"))


@pytest.mark.parametrize("mutation", ("overlap", "after_run"))
def test_scenario_windows_are_ordered_and_bounded(
    tmp_path: Path, mutation: str
) -> None:
    context_path, context, build = complete_context(tmp_path)
    args = aggregate_args(context_path, tmp_path / "out.json")
    if mutation == "overlap":
        previous = context["scenarios"]["mobility"]
        context["scenarios"]["water_normal"]["started_epoch_ns"] = (
            previous["finished_epoch_ns"] - 1
        )
        message = "overlaps or is out of order"
    else:
        args.finished_epoch_ns = context["scenarios"]["water_full"]["finished_epoch_ns"]
        message = "finished after the integrated run"
    write_json(context_path, context)
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        with pytest.raises(acceptance.AcceptanceError, match=message):
            acceptance.aggregate(args)


def test_all_three_colcon_markers_must_postdate_build_start(tmp_path: Path) -> None:
    started_ns = time.time_ns() - 1_000_000_000
    for index, package in enumerate(acceptance.INSTALL_PACKAGES):
        marker = tmp_path / "build" / package / "colcon_build.rc"
        marker.parent.mkdir(parents=True)
        marker.write_text("0\n", encoding="utf-8")
        marker_ns = started_ns + (index + 1) * 10_000_000
        os.utime(marker, ns=(marker_ns, marker_ns))
    rows = acceptance.package_build_markers(tmp_path, started_ns)
    assert set(rows) == set(acceptance.INSTALL_PACKAGES)

    stale = tmp_path / "build" / "sanitation_manipulation" / "colcon_build.rc"
    os.utime(stale, ns=(started_ns, started_ns))
    with pytest.raises(acceptance.AcceptanceError, match="sanitation_manipulation"):
        acceptance.package_build_markers(tmp_path, started_ns)


def test_each_compiled_plugin_must_appear_exactly_once() -> None:
    started_ns = 10
    valid = {
        "install/sanitation_gazebo_control/lib/libDynamicPayloadSystem.so": {
            "sha256": "1" * 64,
            "size_bytes": 1,
            "mtime_epoch_ns": 11,
        },
        "install/sanitation_gazebo_control/lib/libWaterRecoverySystem.so": {
            "sha256": "2" * 64,
            "size_bytes": 1,
            "mtime_epoch_ns": 12,
        },
    }
    assert set(acceptance.plugin_rows(valid, started_ns)) == set(
        acceptance.PLUGIN_BASENAMES
    )
    duplicate = dict(valid)
    duplicate["install/duplicate/lib/libDynamicPayloadSystem.so"] = valid[
        "install/sanitation_gazebo_control/lib/libDynamicPayloadSystem.so"
    ]
    with pytest.raises(acceptance.AcceptanceError, match="exactly once"):
        acceptance.plugin_rows(duplicate, started_ns)


def test_source_install_contract_rejects_old_manipulation_install(
    tmp_path: Path,
) -> None:
    repo = tmp_path / "repo"
    runtime = tmp_path / "runtime"

    def materialize(path: Path, text: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    package_sources = {
        "sanitation_vehicle_description": {
            "launch/formal.launch.py": "value = 1\n",
            "urdf/formal.xacro": '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"/>\n',
            "worlds/formal.sdf": '<sdf version="1.10"><world name="formal"/></sdf>\n',
        },
        "sanitation_manipulation": {
            "launch/pick.launch.py": "value = 2\n",
            "urdf/pick.xacro": '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"/>\n',
            "worlds/pick.sdf": '<sdf version="1.10"><world name="pick"/></sdf>\n',
        },
    }
    for package, files in package_sources.items():
        source_root = repo / acceptance.PACKAGE_SOURCE_DIRS[package]
        install_root = runtime / "install" / package / "share" / package
        for relative, text in files.items():
            materialize(source_root / relative, text)
            materialize(install_root / relative, text)

    source_python = (
        repo
        / acceptance.PACKAGE_SOURCE_DIRS["sanitation_manipulation"]
        / "sanitation_manipulation"
    )
    install_python = (
        runtime
        / "install/sanitation_manipulation/lib/python3.12/site-packages/sanitation_manipulation"
    )
    materialize(source_python / "__init__.py", "VALUE = 1\n")
    materialize(source_python / "core.py", "def ready():\n    return True\n")
    materialize(install_python / "__init__.py", "VALUE = 1\n")
    materialize(install_python / "core.py", "def ready():\n    return True\n")

    rows = acceptance.source_install_contract(repo, runtime)
    assert {row["category"] for row in rows.values()} == {
        "launch",
        "urdf",
        "worlds",
        "python_module",
    }
    materialize(install_python / "core.py", "def ready():\n    return False\n")
    with pytest.raises(acceptance.AcceptanceError, match="Python module is stale"):
        acceptance.source_install_contract(repo, runtime)


@pytest.mark.parametrize("field", ("ros_distro", "ros_base_package", "gazebo"))
def test_missing_ros_or_gazebo_version_fails_closed(field: str) -> None:
    runtime = {
        "ros_distro": "jazzy",
        "ros_base_package": "0.11.0",
        "gazebo": "Gazebo Sim 8.11.0",
    }
    runtime[field] = None
    with pytest.raises(acceptance.AcceptanceError, match="versions are missing"):
        acceptance.require_runtime_versions(runtime)


def test_build_snapshot_rejects_changed_source_hash(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    runtime = tmp_path / "runtime"
    repo.mkdir()
    runtime.mkdir()
    source_v1 = {"source": {"sha256": "1" * 64, "size_bytes": 1, "mtime_epoch_ns": 1}}
    source_v2 = {"source": {"sha256": "2" * 64, "size_bytes": 1, "mtime_epoch_ns": 2}}
    installed = {"artifact": {"sha256": "3" * 64, "size_bytes": 1, "mtime_epoch_ns": 3}}
    git = {"commit": "a", "tree": "b", "dirty": False, "dirty_diff_sha256": "c"}
    manifest_path = tmp_path / "build.json"
    write_json(
        manifest_path,
        {
            "schema_version": 1,
            "kind": "tzcup_integrated_acceptance_build_snapshot",
            "build_started_epoch_ns": 1,
            "recorded_epoch_ns": 2,
            "repo_root": str(repo.resolve()),
            "runtime_ws": str(runtime.resolve()),
            "git": git,
            "source_inventory_sha256": acceptance.inventory_digest(source_v1),
            "installed_inventory_sha256": acceptance.inventory_digest(installed),
            "runtime": {
                "ros_distro": "jazzy",
                "ros_base_package": "0.11.0",
                "gazebo": "Gazebo Sim 8.11.0",
            },
        },
    )
    with patch.object(acceptance, "git_snapshot", return_value=git), patch.object(
        acceptance, "inventory", side_effect=[source_v2, installed]
    ):
        with pytest.raises(acceptance.AcceptanceError, match="critical source hash"):
            acceptance.validate_build_snapshot(manifest_path, repo, runtime)


def test_runner_has_exact_partition_cleanup_and_four_unique_offsets() -> None:
    runner = Path(__file__).with_name("run_integrated_functional_acceptance.sh").read_text(encoding="utf-8")
    assert runner.index("source /opt/ros/jazzy/setup.bash") < runner.index("set -u")
    assert 'needle = ("GZ_PARTITION=" + sys.argv[1]).encode()' in runner
    assert "signal.SIGINT, signal.SIGTERM, signal.SIGKILL" in runner
    assert 'run_wrapped_scenario "mobility" 0' in runner
    assert 'run_water_scenario "water_normal" 1' in runner
    assert 'run_water_scenario "water_full" 2' in runner
    assert 'run_wrapped_scenario "manipulation" 3' in runner
    assert "rm -f --" in runner


def test_runner_refuses_reused_run_directory_and_publishes_manifest_atomically() -> None:
    runner = Path(__file__).with_name("run_integrated_functional_acceptance.sh").read_text(encoding="utf-8")
    assert 'mkdir -p "${evidence_root}"' in runner
    assert 'if ! mkdir "${run_dir}" 2>/dev/null; then' in runner
    assert 'mkdir -p "${run_dir}"' not in runner
    assert "Refusing to reuse integrated acceptance run directory" in runner
    assert 'manifest_tmp="${manifest}.pending.$$"' in runner
    assert '--context "${context}" --output "${manifest_tmp}"' in runner
    assert 'mv -- "${manifest_tmp}" "${manifest}"' in runner
    assert runner.index('--output "${manifest_tmp}"') < runner.index(
        'mv -- "${manifest_tmp}" "${manifest}"'
    )


def test_runner_exit_trap_cleans_exact_active_partition() -> None:
    runner = Path(__file__).with_name("run_integrated_functional_acceptance.sh").read_text(encoding="utf-8")
    assert 'active_partition=""' in runner
    assert 'active_partition="${partition}"' in runner
    assert 'cleanup_partition "${active_partition}"' in runner
    assert "trap cleanup_active_scenario EXIT" in runner
    assert "trap 'handle_signal 130' INT" in runner
    assert "trap 'handle_signal 143' TERM" in runner
    assert runner.count('active_partition=""') >= 3
