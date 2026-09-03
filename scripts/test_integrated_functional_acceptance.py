import argparse
import json
import os
import shutil
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
            "report_id": "tzcup_formal_a300_drivetrain_runtime_v1",
            "status": "FORMAL_A300_DRIVETRAIN_FORWARD_STOP_RUNTIME_PASSED",
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
            "mass_kg": 0.03726,
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
        "dry_bin_monitor": {
            "sensor_ready": True,
            "contained_object_count": 1,
            "contained_mass_kg": 0.03726,
            "mass_tolerance_kg": 1e-5,
            "full": False,
            "post_release_sample_observed": True,
            "monitor_is_observation_only": True,
        },
        "inventory_mass": {
            "physical_cube_count": 1,
            "physical_material_mass_kg": 0.03726,
            "dynamic_dry_payload_command_count": 0,
            "dynamic_dry_payload_added_kg": 0.0,
            "double_count_prevented": True,
            "aggregation_or_reserve_payload_substitution": False,
        },
        "evaluator_interface_audit": {
            "delete_entity_calls": 0,
            "set_pose_or_remove_after_task_start": False,
            "physical_cube_deleted_after_deposit": False,
        },
        "material": "PET",
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
    installed_xacro = (
        tmp_path
        / "runtime/install/share/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
    )
    layout = tmp_path / "runtime/install/.colcon_install_layout"
    layout.parent.mkdir(parents=True, exist_ok=True)
    layout.write_text("merged\n", encoding="utf-8")
    surface_path = tmp_path / "side_brush_sdf_surface.json"
    write_json(surface_path, {
        "status": "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED",
        "expanded_sdf_sha256": "7" * 64,
        "source": {"mode": "xacro_to_gz_sdf", "path": str(installed_xacro)},
        "runtime_effectiveness": {
            "dart_effective_from_surface_friction_ode": ["mu", "mu2"],
            "serialized_but_not_consumed_by_gz_physics_7_dart": [
                "kp", "kd", "max_vel", "min_depth"
            ],
        },
    })
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
        "material": "PET",
        "side_brush_sdf_surface": {
            "path": str(surface_path),
            "sha256": acceptance.sha256_file(surface_path),
            "expanded_sdf_sha256": "7" * 64,
            "source_xacro": str(installed_xacro),
            "runtime_effectiveness": {
                "dart_effective_from_surface_friction_ode": ["mu", "mu2"],
                "serialized_but_not_consumed_by_gz_physics_7_dart": [
                    "kp", "kd", "max_vel", "min_depth"
                ],
            },
        },
        "scenarios": rows,
    }
    build = {
        "build_started_epoch_ns": now - 3_000_000_000,
        "recorded_epoch_ns": now - 2_000_000_000,
        "git": {"commit": "a" * 40, "tree": "b" * 40, "dirty": False, "dirty_diff_sha256": "c" * 64},
        "source_inventory_sha256": "d" * 64,
        "installed_inventory_sha256": "e" * 64,
        "source_inventory": {
            "starter_ws/src/sanitation_gazebo_control/src/DryBinMonitorSystem.cc": {
                "sha256": "3" * 64
            },
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
            },
            "install/sanitation_gazebo_control/lib/libDryBinMonitorSystem.so": {
                "sha256": "4" * 64
            }
        },
        "runtime": {
            "ros_distro": "jazzy",
            "ros_base_package": "0.11.0-1noble.20260801.000000",
            "gazebo": "Gazebo Sim, version 8.11.0",
            "physics_engine": "gz-physics-dartsim-plugin",
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
    assert any(name.endswith("DryBinMonitorSystem.cc") for name in report["critical_file_sha256"])
    assert any(name.endswith("libDryBinMonitorSystem.so") for name in report["critical_file_sha256"])
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


def test_failed_attempt_preserves_all_recorded_chain_evidence(tmp_path: Path) -> None:
    context_path, context, build = complete_context(tmp_path)
    context["scenarios"]["water_normal"]["exit_code"] = 9
    write_json(context_path, context)
    output = tmp_path / "failed-attempt.json"
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        assert acceptance.aggregate_attempt(aggregate_args(context_path, output)) == 3
    attempt = json.loads(output.read_text(encoding="utf-8"))
    assert attempt["passed"] is False
    assert attempt["status"] == "INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_FAILED"
    assert set(attempt["scenario_invocations"]) == set(acceptance.SCENARIOS)
    assert "water_normal exited nonzero" in attempt["failure"]


def test_formal_chain_cannot_pass_without_its_fresh_attempt_sidecar(tmp_path: Path) -> None:
    context_path, context, build = complete_context(tmp_path)
    context["formal_attempt_binding"] = {
        "session_path": "session.json",
        "session_sha256": "a" * 64,
        "session_started_epoch_ns": 1,
        "snapshot_path": "snapshot.json",
        "snapshot_sha256": "b" * 64,
        "source_build_manifest_sha256": "c" * 64,
    }
    write_json(context_path, context)
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        with pytest.raises(acceptance.AcceptanceError, match="sidecar is missing or changed"):
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


@pytest.mark.parametrize("material", sorted(acceptance.MATERIAL_MASS_KG))
def test_manipulation_material_contract_is_not_pet_hardcoded(
    tmp_path: Path, material: str
) -> None:
    context_path, context, build = complete_context(tmp_path)
    context["material"] = material
    result_path = Path(context["scenarios"]["manipulation"]["result"])
    result = json.loads(result_path.read_text(encoding="utf-8"))
    mass = acceptance.MATERIAL_MASS_KG[material]
    result["material"] = material
    result["cube"]["mass_kg"] = mass
    result["dry_bin_monitor"]["contained_mass_kg"] = mass
    result["inventory_mass"]["physical_material_mass_kg"] = mass
    write_json(result_path, result)
    context["scenarios"]["manipulation"]["result_sha256"] = acceptance.sha256_file(
        result_path
    )
    write_json(context_path, context)
    output = tmp_path / "manifest.json"
    with patch.object(acceptance, "validate_build_snapshot", return_value=build):
        acceptance.aggregate(aggregate_args(context_path, output))
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["material_contract"] == {
        "material": material,
        "cube_edge_m": 0.03,
        "expected_mass_kg": mass,
    }


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


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        ("sensor_not_ready", "dry-bin PET inventory telemetry"),
        ("wrong_count", "dry-bin PET inventory telemetry"),
        ("wrong_mass", "dry-bin PET inventory telemetry"),
        ("full", "dry-bin PET inventory telemetry"),
        ("not_pet", "required 0.03726 kg PET cube"),
        ("deleted", "deleted, aggregated, substituted"),
        ("aggregated", "deleted, aggregated, substituted"),
    ),
)
def test_manipulation_dry_bin_inventory_fails_closed(
    tmp_path: Path, mutation: str, message: str
) -> None:
    context_path, context, build = complete_context(tmp_path)
    row = context["scenarios"]["manipulation"]
    result_path = Path(row["result"])
    result = json.loads(result_path.read_text(encoding="utf-8"))
    if mutation == "sensor_not_ready":
        result["dry_bin_monitor"]["sensor_ready"] = False
    elif mutation == "wrong_count":
        result["dry_bin_monitor"]["contained_object_count"] = 2
    elif mutation == "wrong_mass":
        result["dry_bin_monitor"]["contained_mass_kg"] = 0.04
    elif mutation == "full":
        result["dry_bin_monitor"]["full"] = True
    elif mutation == "not_pet":
        result["material"] = "PP"
    elif mutation == "deleted":
        result["evaluator_interface_audit"]["physical_cube_deleted_after_deposit"] = True
    else:
        result["inventory_mass"]["aggregation_or_reserve_payload_substitution"] = True
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
        "install/sanitation_gazebo_control/lib/libDryBinMonitorSystem.so": {
            "sha256": "3" * 64,
            "size_bytes": 1,
            "mtime_epoch_ns": 13,
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


def test_source_install_contract_supports_one_merged_colcon_prefix(
    tmp_path: Path,
) -> None:
    repo = tmp_path / "repo"
    runtime = tmp_path / "runtime"

    def materialize(path: Path, text: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    materialize(runtime / "install/.colcon_install_layout", "merged\n")
    materialize(runtime / "install/setup.bash", "# merged\n")
    for package in acceptance.PACKAGE_SOURCE_DIRS:
        source_root = repo / acceptance.PACKAGE_SOURCE_DIRS[package]
        install_root = runtime / "install/share" / package
        for relative, text in {
            "launch/formal.launch.py": "value = 1\n",
            "urdf/formal.xacro": '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"/>\n',
            "worlds/formal.sdf": '<sdf version="1.10"><world name="formal"/></sdf>\n',
        }.items():
            materialize(source_root / relative, text)
            materialize(install_root / relative, text)
    source_python = (
        repo
        / acceptance.PACKAGE_SOURCE_DIRS["sanitation_manipulation"]
        / "sanitation_manipulation"
    )
    install_python = runtime / "install/lib/python3.12/site-packages/sanitation_manipulation"
    materialize(source_python / "__init__.py", "VALUE = 1\n")
    materialize(install_python / "__init__.py", "VALUE = 1\n")

    rows = acceptance.source_install_contract(repo, runtime)
    assert {row["category"] for row in rows.values()} == {
        "launch",
        "urdf",
        "worlds",
        "python_module",
    }
    assert acceptance.install_entries(runtime) == ["install"]


def test_record_and_validate_build_manifest_with_one_merged_prefix(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo = tmp_path / "repo"
    runtime = tmp_path / "runtime"

    def materialize(path: Path, text: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    materialize(repo / "source.txt", "source-bound\n")
    materialize(runtime / "install/.colcon_install_layout", "merged\n")
    materialize(runtime / "install/setup.bash", "# merged setup\n")
    for package in acceptance.PACKAGE_SOURCE_DIRS:
        source_root = repo / acceptance.PACKAGE_SOURCE_DIRS[package]
        install_root = runtime / "install/share" / package
        for relative, text in {
            "launch/formal.launch.py": "value = 1\n",
            "urdf/formal.xacro": '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"/>\n',
            "worlds/formal.sdf": '<sdf version="1.10"><world name="formal"/></sdf>\n',
        }.items():
            materialize(source_root / relative, text)
            materialize(install_root / relative, text)
    materialize(
        repo / "starter_ws/src/sanitation_gazebo_control/package.xml",
        "<package><name>sanitation_gazebo_control</name></package>\n",
    )
    source_python = (
        repo
        / acceptance.PACKAGE_SOURCE_DIRS["sanitation_manipulation"]
        / "sanitation_manipulation"
    )
    materialize(source_python / "__init__.py", "VALUE = 1\n")
    materialize(
        runtime / "install/lib/python3.12/site-packages/sanitation_manipulation/__init__.py",
        "VALUE = 1\n",
    )
    shutil.copytree(repo / "starter_ws/src", runtime / "src")
    materialize(runtime / acceptance.INSTALL_SYMLINK_REPORT, "")
    for basename in acceptance.PLUGIN_BASENAMES:
        materialize(runtime / "install/lib" / basename, f"binary:{basename}\n")

    started_ns = time.time_ns() - 5_000_000_000
    marker_ns = time.time_ns() - 1_000_000_000
    for package in acceptance.INSTALL_PACKAGES:
        marker = runtime / "build" / package / "colcon_build.rc"
        materialize(marker, "0\n")
        os.utime(marker, ns=(marker_ns, marker_ns))

    git = {
        "commit": "a" * 40,
        "tree": "b" * 40,
        "dirty": False,
        "dirty_diff_sha256": "c" * 64,
        "untracked": [],
    }
    versions = {
        "ros_distro": "jazzy",
        "ros_base_package": "0.11.0",
        "gazebo": "Gazebo Sim 8.11.0",
        "physics_engine": "gz-physics-dartsim-plugin",
        "python": "3.12.3",
        "platform": "linux",
    }
    monkeypatch.setattr(acceptance, "SOURCE_ROOTS", ("source.txt",))
    output = tmp_path / "integrated_build_manifest.json"
    args = argparse.Namespace(
        repo_root=repo,
        runtime_ws=runtime,
        build_started_epoch_ns=started_ns,
        output=output,
    )
    with patch.object(acceptance, "git_snapshot", return_value=git), patch.object(
        acceptance, "runtime_versions", return_value=versions
    ):
        assert acceptance.create_build_manifest(args) == 0
        manifest = acceptance.validate_build_snapshot(output, repo, runtime)

    assert manifest["installed_inventory"]
    assert all(name.startswith("install/") for name in manifest["installed_inventory"])
    assert not any(
        name.startswith("install/sanitation_vehicle_description/")
        for name in manifest["installed_inventory"]
    )
    assert set(manifest["compiled_plugins"]) == set(acceptance.PLUGIN_BASENAMES)
    assert manifest["source_install_contract"]
    assert manifest["frozen_source"]["matches_repository"] is True
    assert manifest["install_symlink_report"]["size_bytes"] == 0

    materialize(runtime / "install/setup.bash", "# changed after snapshot\n")
    with patch.object(acceptance, "git_snapshot", return_value=git):
        with pytest.raises(acceptance.AcceptanceError, match="installed runtime hash"):
            acceptance.validate_build_snapshot(output, repo, runtime)


def test_frozen_source_contract_rejects_runtime_source_drift(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    runtime = tmp_path / "runtime"
    for package in acceptance.INSTALL_PACKAGES:
        source = repo / "starter_ws/src" / package / "package.xml"
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(package, encoding="utf-8")
    shutil.copytree(repo / "starter_ws/src", runtime / "src")
    (runtime / "src/sanitation_manipulation/package.xml").write_text(
        "changed", encoding="utf-8"
    )
    with pytest.raises(acceptance.AcceptanceError, match="frozen runtime src differs"):
        acceptance.frozen_source_contract(repo, runtime)


def test_init_run_binds_side_brush_preflight_to_merged_installed_xacro(
    tmp_path: Path,
) -> None:
    runtime = tmp_path / "runtime"

    def materialize(path: Path, text: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    materialize(runtime / "install/.colcon_install_layout", "merged\n")
    installed_xacro = acceptance.installed_vehicle_xacro(runtime)
    materialize(installed_xacro, '<robot name="vehicle"/>\n')
    surface = tmp_path / "side_brush.json"
    write_json(
        surface,
        {
            "status": "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED",
            "expanded_sdf_sha256": "7" * 64,
            "source": {"mode": "xacro_to_gz_sdf", "path": str(installed_xacro)},
            "runtime_effectiveness": {
                "dart_effective_from_surface_friction_ode": ["mu", "mu2"]
            },
        },
    )
    build_manifest = tmp_path / "build.json"
    write_json(build_manifest, {"placeholder": True})
    context = tmp_path / "context.json"
    args = argparse.Namespace(
        repo_root=tmp_path,
        runtime_ws=runtime,
        build_manifest=build_manifest,
        context=context,
        run_id="merged-side-brush",
        started_epoch_ns=2,
        material="PET",
        side_brush_surface_audit=surface,
    )
    with patch.object(
        acceptance,
        "validate_build_snapshot",
        return_value={"recorded_epoch_ns": 1},
    ):
        assert acceptance.init_run(args) == 0
    saved = json.loads(context.read_text(encoding="utf-8"))
    assert saved["side_brush_sdf_surface"]["source_xacro"] == str(installed_xacro)

    wrong_surface = json.loads(surface.read_text(encoding="utf-8"))
    wrong_surface["source"]["path"] = str(tmp_path / "source-tree.xacro")
    write_json(surface, wrong_surface)
    context.unlink()
    with patch.object(
        acceptance,
        "validate_build_snapshot",
        return_value={"recorded_epoch_ns": 1},
    ):
        with pytest.raises(acceptance.AcceptanceError, match="frozen installed xacro"):
            acceptance.init_run(args)
    assert not context.exists()


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


def test_git_command_translates_windows_worktree_gitdir_for_wsl(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    (repo / ".git").write_text(
        "gitdir: F:/Project/TZcup/.git/worktrees/task\n", encoding="utf-8"
    )
    expected = "/mnt/f/Project/TZcup/.git/worktrees/task"
    with patch.object(acceptance.os.path, "isdir", return_value=True):
        monkeypatch.setattr(acceptance.os, "name", "posix")
        command = acceptance.git_command(repo, "rev-parse", "HEAD")
    assert f"--git-dir={expected}" in command
    assert f"--work-tree={repo.resolve()}" in command
    assert command[-2:] == ["rev-parse", "HEAD"]


def test_git_command_rejects_missing_declared_worktree_gitdir(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    (repo / ".git").write_text("gitdir: missing-admin-dir\n", encoding="utf-8")
    with pytest.raises(acceptance.AcceptanceError, match="declared Git worktree directory is missing"):
        acceptance.git_command(repo, "status")


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
    runtime_source = 'source "${runtime_ws}/install/setup.bash"'
    runtime_source_index = runner.index(runtime_source)
    assert runner.rfind("set +u", 0, runtime_source_index) >= 0
    assert runner.index("set -u", runtime_source_index) > runtime_source_index
    assert 'needle = ("GZ_PARTITION=" + sys.argv[1]).encode()' in runner
    assert "signal.SIGINT, signal.SIGTERM, signal.SIGKILL" in runner
    assert 'run_wrapped_scenario "mobility" 0' in runner
    assert 'run_water_scenario "water_normal" 1' in runner
    assert 'run_water_scenario "water_full" 2' in runner
    assert 'run_wrapped_scenario "manipulation" 3' in runner
    assert "validate_formal_side_brush_sdf_surface.py" in runner
    assert "--side-brush-surface-audit" in runner
    assert "formal_runtime_gate_binding.py" in runner
    assert 'runtime_binding="${INTEGRATED_ACCEPTANCE_RUNTIME_BINDING:-${contract_summary}.runtime_binding.json}"' in runner
    assert "start_simulation_safety_inputs:=true" in runner
    assert "start_power_system_simulators:=true" in runner
    assert "high_bandwidth_sensor_runtime:=false" in runner
    assert "rm -f --" in runner
    assert "import secrets; print(secrets.token_hex(32))" in runner
    assert 'FORMAL_ORCHESTRATED_STEP_SESSION_TOKEN="${session_token}"' in runner
    assert "missing outer formal orchestrated session token" in runner
    assert "formal integrated outer session requires a valid capability token" in runner

    build_preflight = runner.index('"${aggregator}" preflight')
    side_brush_preflight = runner.index("validate_formal_side_brush_sdf_surface.py")
    init_run = runner.index('"${aggregator}" init-run')
    first_scenario = runner.index('run_wrapped_scenario "mobility" 0')
    assert build_preflight < side_brush_preflight < init_run < first_scenario
    assert runner.index("formal_runtime_gate_binding.py") < first_scenario


def test_runner_refuses_reused_run_directory_and_publishes_manifest_atomically() -> None:
    runner = Path(__file__).with_name("run_integrated_functional_acceptance.sh").read_text(encoding="utf-8")
    assert 'mkdir -p "${evidence_root}"' in runner
    assert 'if ! mkdir "${run_dir}" 2>/dev/null; then' in runner
    assert 'mkdir -p "${run_dir}"' not in runner
    assert "Refusing to reuse integrated acceptance run directory" in runner
    assert "Refusing to overwrite integrated acceptance runtime binding" in runner
    assert "Refusing to overwrite integrated acceptance contract summary" in runner
    assert "must resolve to the canonical contract-summary sidecar" in runner
    assert 'expected_runtime_binding="$(python3 - "${contract_summary}"' in runner
    assert runner.index("must resolve to the canonical contract-summary sidecar") < runner.index(
        'mkdir -p "${evidence_root}"'
    )
    assert 'manifest_tmp="${manifest}.pending.$$"' in runner
    assert '--context "${context}" --output "${manifest_tmp}"' in runner
    assert 'mv -- "${manifest_tmp}" "${manifest}"' in runner
    assert runner.index('--output "${manifest_tmp}"') < runner.index(
        'mv -- "${manifest_tmp}" "${manifest}"'
    )
    assert '--material "${material}"' in runner
    assert '--runtime-binding "${runtime_binding}"' in runner
    assert "publish_integrated_basic_functional_acceptance.py" in runner
    assert '--manifest "${manifest}" --snapshot-manifest "${snapshot_manifest}"' in runner
    assert '--session-status "${session_status}" --runtime-closure "${runtime_closure}"' in runner
    assert '--output "${contract_summary}"' in runner
    assert '"${aggregator}" aggregate-attempt' in runner
    assert 'if (( aggregate_exit != 0 )); then' in runner
    assert 'Integrated acceptance failed attempt retained: ${manifest}' in runner
    assert runner.index('"${aggregator}" aggregate-attempt') < runner.index('publish_integrated_basic_functional_acceptance.py')


def test_runner_exit_trap_cleans_exact_active_partition() -> None:
    runner = Path(__file__).with_name("run_integrated_functional_acceptance.sh").read_text(encoding="utf-8")
    assert 'active_partition=""' in runner
    assert 'active_partition="${partition}"' in runner
    assert 'cleanup_partition "${active_partition}"' in runner
    assert "trap cleanup_active_scenario EXIT" in runner
    assert "trap 'handle_signal 130' INT" in runner
    assert "trap 'handle_signal 143' TERM" in runner
    assert runner.count('active_partition=""') >= 3
