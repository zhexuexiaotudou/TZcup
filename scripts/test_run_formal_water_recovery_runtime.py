from __future__ import annotations

import json
import hashlib
from pathlib import Path

from finalize_formal_water_recovery_acceptance import (
    FULL_REQUIRED_CHECKS,
    NORMAL_REQUIRED_CHECKS,
    TYPED_DIAG_REQUIRED_CHECKS,
    TYPED_CRITICAL_MANIFEST_REQUIRED_PATHS,
    TYPED_TRANSPORT_AUDIT_REQUIRED_CHECKS,
    _preembedded_world_binding_valid,
    combine,
)


ROOT = Path(__file__).resolve().parents[1]


def _surface_evidence(*, status: str = "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED") -> dict[str, object]:
    return {
        "schema_version": 2,
        "status": status,
        "expanded_sdf_sha256": "abc123",
        "central_roller": {
            "collision": "central_roller_link_collision",
            "radius_m": 0.100,
            "length_m": 0.620,
            "surface": {"mu": 0.08, "mu2": 0.08},
        },
    }


def _typed_evidence(tmp_path: Path) -> tuple[Path, Path, Path, Path, Path, str]:
    diag = tmp_path / "typed_diag.json"
    trace = tmp_path / "raw_frames.jsonl"
    runner = tmp_path / "typed_runner.sh"
    collector = tmp_path / "typed_collector.py"
    manifest = tmp_path / "critical_source_manifest.json"
    launch = tmp_path / "typed_launch.log"
    launch_audit = tmp_path / "typed_launch_audit.json"
    gz_info = tmp_path / "typed_gz_info.txt"
    ros_info = tmp_path / "typed_ros_info.txt"
    trace.write_text('{"frame":1}\n{"frame":2}\n', encoding="utf-8")
    launch.write_text("healthy\n", encoding="utf-8")
    launch_audit.write_text(json.dumps({"passed": True}), encoding="utf-8")
    gz_info.write_text("gz.msgs.Double_V\n", encoding="utf-8")
    ros_info.write_text("std_msgs/msg/Float64MultiArray\nPublisher count: 1\n", encoding="utf-8")
    diag.write_text(
        json.dumps(
            {
                "status": "FORMAL_TYPED_CLEANING_MOTOR_DIAG_PASSED",
                "passed": True,
                "checks": {name: True for name in TYPED_DIAG_REQUIRED_CHECKS},
                "metrics": {"raw_trace_frame_count": 2},
                "transport_audit": {
                    "passed": True,
                    "checks": {
                        name: True for name in TYPED_TRANSPORT_AUDIT_REQUIRED_CHECKS
                    },
                    "node_shared_publish_errors": [],
                    "topic_tagged_publish_failures": [],
                    "launch_log": str(launch),
                    "launch_log_sha256": __import__("hashlib").sha256(launch.read_bytes()).hexdigest(),
                    "launch_audit_json": str(launch_audit),
                    "launch_audit_sha256": __import__("hashlib").sha256(launch_audit.read_bytes()).hexdigest(),
                    "gazebo_topic_info": str(gz_info),
                    "gazebo_topic_info_sha256": __import__("hashlib").sha256(gz_info.read_bytes()).hexdigest(),
                    "ros_topic_info": str(ros_info),
                    "ros_topic_info_sha256": __import__("hashlib").sha256(ros_info.read_bytes()).hexdigest(),
                },
            }
        ),
        encoding="utf-8",
    )
    runner.write_text("#!/bin/sh\n", encoding="utf-8")
    collector.write_text("# collector\n", encoding="utf-8")
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "source_package_files_match_frozen_copy": True,
                "install_symlink_count": 0,
                "critical_files": [
                    {"path": path, "source_sha256": "b" * 64}
                    for path in sorted(TYPED_CRITICAL_MANIFEST_REQUIRED_PATHS)
                ],
            }
        ),
        encoding="utf-8",
    )
    return diag, trace, runner, collector, manifest, "a" * 64


def test_runner_uses_fresh_isolated_launch_for_both_scenarios() -> None:
    source = (ROOT / "scripts/run_formal_water_recovery_runtime.sh").read_text(
        encoding="utf-8"
    )
    assert "--scenario" in source
    assert 'scenario}" != "normal"' in source
    assert 'scenario}" != "full"' in source
    assert "formal_vehicle_sim.launch.py" in source
    assert "prepare_formal_preembedded_sensor_world.py" in source
    assert 'world:="${preembedded_world}" spawn_robot:=false' in source
    assert "water_${selected}_preembedded_sensor_world.sdf" in source
    assert "water_${selected}_preembedded_sensor_world.json" in source
    assert "water_evaluation_interfaces:=true" in source
    assert "squeegee_evaluation_interfaces:=true" in source
    assert '"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch' in source
    assert "cleanup_launch" in source
    assert "audit_formal_water_launch_log.py" in source
    assert '--expected-stable-marker-count "${expected_stable_marker_count}"' in source
    assert 'expected_stable_marker_count=1' in source
    assert 'expected_stable_marker_count=2' not in source
    assert "check_formal_water_preoperational_readiness.py" in source
    assert "preoperational_readiness.json" in source
    assert "validate_formal_side_brush_sdf_surface.py" in source
    assert "--normal-side-brush-surface" in source
    assert "--full-side-brush-surface" in source
    assert "--typed-diag" in source
    assert "--typed-raw-trace" in source
    assert "--critical-source-manifest" in source
    assert "--typed-cleaning-telemetry-source-sha256" in source
    assert 'run_scenario normal' in source
    assert 'run_scenario full' in source
    assert "finalize_formal_water_recovery_acceptance.py" in source
    assert "formal_runtime_gate_binding.py" in source
    assert "--runtime-binding" in source
    assert "FORMAL_ACCEPTANCE_SESSION" in source


def test_runner_stops_all_active_parameter_bridges_in_order_before_launch_cleanup() -> None:
    source = (ROOT / "scripts/run_formal_water_recovery_runtime.sh").read_text(
        encoding="utf-8"
    )

    function_start = source.index("formal_water_stop_active_parameter_bridges()")
    run_scenario_start = source.index("run_scenario()")
    shutdown = source[function_start:run_scenario_start]
    assert 'partition, timeline_arg = sys.argv[1:]' in shutdown
    assert 'Path("/proc").iterdir()' in shutdown
    assert 'f"GZ_PARTITION={partition}".encode() not in environment' in shutdown
    assert 'executable.name != "parameter_bridge"' in shutdown
    assert '"ros_gz_bridge" not in executable.parts' in shutdown
    assert 'argument.startswith("__node:=")' in shutdown
    assert 'if len(node_args) != 1' in shutdown
    assert 'missing = sorted(set(targets) - set(initial_nodes))' in shutdown
    assert 'unexpected = sorted(set(initial_nodes) - set(targets))' in shutdown
    assert 'if malformed or missing or unexpected or duplicates:' in shutdown
    assert 'if len(matches) != 1:' in shutdown
    assert 'os.kill(pid, signal.SIGINT)' in shutdown
    assert 'raise SystemExit(1)' in shutdown
    expected_order = (
        "water_evaluation_bridge",
        "formal_vehicle_product_bridge",
        "cleaning_actuator_scalar_bridge",
        "a300_drivetrain_bridge",
        "formal_squeegee_evaluation_bridge",
        "formal_brush_contact_evaluation_bridge",
        "charge_receptacle_contact_bridge",
        "wastewater_drain_contact_bridge",
        "front_bumper_contact_bridge",
        "rear_bumper_contact_bridge",
        "formal_auxiliary_bridge",
    )
    positions = [shutdown.index(f'"{node}"') for node in expected_order]
    assert positions == sorted(positions)
    assert shutdown.index('"pre_shutdown_census"') < shutdown.index(
        '"ordered_shutdown_started"'
    ) < shutdown.index("os.kill(pid, signal.SIGINT)")
    assert shutdown.index('"post_shutdown_census"') > shutdown.index(
        "os.kill(pid, signal.SIGINT)"
    ) and shutdown.index('"post_shutdown_census"') < shutdown.index(
        '"ordered_shutdown_completed"'
    )
    assert 'if remaining_nodes or malformed:' in shutdown
    assert shutdown.index('"formal_auxiliary_bridge"') > shutdown.index(
        '"formal_squeegee_evaluation_bridge"'
    )

    validator = source.index(
        'python3 "${repo_root}/scripts/validate_formal_water_recovery_runtime.py"'
    )
    ordered_stop = source.index(
        "formal_water_stop_active_parameter_bridges", run_scenario_start
    )
    cleanup = source.index("cleanup_launch", ordered_stop)
    audit = source.index(
        'python3 "${repo_root}/scripts/audit_formal_water_launch_log.py"'
    )
    assert validator < ordered_stop < cleanup < audit
    assert "FORMAL_WATER_REPOSITORY_ROOT" in source
    assert "FORMAL_TASK_ONLY_DIAGNOSTIC" in source
    assert "Repository-root override is forbidden for formal all-scenarios acceptance" in source


def test_all_scenarios_rotation_invalidates_old_canonical_before_preflight() -> None:
    source = (ROOT / "scripts/run_formal_water_recovery_runtime.sh").read_text(
        encoding="utf-8"
    )
    parse_end = source.index('if [[ "${scenario}" != "normal"')
    rotate = source.index('if [[ "${scenario}" == "all" ]]; then')
    ros_setup = source.index("source /opt/ros/jazzy/setup.bash")
    stale_rejection = source.index("Refusing stale water-recovery evidence")

    assert parse_end < rotate < ros_setup < stale_rejection
    assert '.superseded.$(date -u +%Y%m%dT%H%M%SZ).$$' in source
    assert 'for retained in "${formal_output}" "${runtime_binding}"; do' in source
    assert 'mv -- "${retained}" "${superseded}"' in source
    assert 'Refusing to overwrite retained superseded water-recovery evidence' in source
    assert '[[ -e "${retained}" || -L "${retained}" ]]' in source


def test_final_binding_is_verified_before_the_frozen_overlay_is_sourced() -> None:
    source = (ROOT / "scripts/run_formal_water_recovery_runtime.sh").read_text(
        encoding="utf-8"
    )
    snapshot_check = source.index("generate_formal_vehicle_snapshot.py")
    runtime_binding = source.index("formal_runtime_gate_binding.py")
    source_overlay = source.index('source "${runtime_setup}"')
    launch = source.index('"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch')

    assert snapshot_check < runtime_binding < source_overlay < launch


def test_cleanup_quarantines_runtime_binding_with_canonical_water_evidence() -> None:
    source = (ROOT / "scripts/run_formal_water_recovery_runtime.sh").read_text(
        encoding="utf-8"
    )
    assert 'formal_runtime_register_evidence_paths "${formal_output}" "${runtime_binding}"' in source
    assert '"${output_dir}/water_${preembedded_scenario}_preembedded_sensor_world.sdf"' in source
    assert '"${output_dir}/water_${preembedded_scenario}_preembedded_sensor_world.json"' in source


def test_runner_preembeds_the_frozen_vehicle_before_launching_contact_gates() -> None:
    source = (ROOT / "scripts/run_formal_water_recovery_runtime.sh").read_text(
        encoding="utf-8"
    )

    assert 'installed_package_share="$(ros2 pkg prefix --share sanitation_vehicle_description)"' in source
    assert 'expected_package_share="${runtime_install_root}/share/sanitation_vehicle_description"' in source
    assert '--source-world "${installed_package_share}/worlds/formal_vehicle_validation.sdf"' in source
    assert '--vehicle-urdf "${repo_root}/reports/engineering/formal_competition_vehicle.urdf"' in source
    assert '--controller-config "${installed_package_share}/config/formal_vehicle_controllers.yaml"' in source
    assert '--runtime-install-root "${runtime_install_root}"' in source
    assert '--output-world "${preembedded_world}"' in source
    assert '--report "${preembedded_report}"' in source
    assert '--model-pose "${preembedded_model_pose}"' in source
    assert source.index("prepare_formal_preembedded_sensor_world.py") < source.index(
        'world:="${preembedded_world}" spawn_robot:=false'
    )


def test_formal_water_scenarios_bind_the_preembedded_contact_world() -> None:
    runner = (ROOT / "scripts/run_formal_water_recovery_runtime.sh").read_text(
        encoding="utf-8"
    )
    validator = (ROOT / "scripts/validate_formal_water_recovery_runtime.py").read_text(
        encoding="utf-8"
    )

    assert '--preembedded-report "${preembedded_report}"' in runner
    assert '--preembedded-world "${preembedded_world}"' in runner
    assert '--preembedded-model-pose "${preembedded_model_pose}"' in runner
    assert "from formal_preembedded_sensor_world_binding import validate_preembedded_sensor_world" in validator
    assert "formal water acceptance requires --preembedded-report" in validator
    assert 'result["preembedded_sensor_world_binding"] = preembedded_world_binding' in validator


def test_finalizer_rejects_tampered_or_wrong_runtime_preembedded_binding(
    tmp_path: Path,
) -> None:
    report = tmp_path / "world.json"
    world = tmp_path / "world.sdf"
    source_world = tmp_path / "source.sdf"
    source_urdf = tmp_path / "vehicle.urdf"
    controller = tmp_path / "install/share/sanitation_vehicle_description/config/controller.yaml"
    controller.parent.mkdir(parents=True)
    for path, content in (
        (report, "report\n"),
        (world, "world\n"),
        (source_world, "source\n"),
        (source_urdf, "urdf\n"),
        (controller, "controller\n"),
    ):
        path.write_text(content, encoding="utf-8")

    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    source_binding = {
        "snapshot_manifest_sha256": "a" * 64,
        "source_inventory_sha256": "b" * 64,
        "expanded_urdf_sha256": sha(source_urdf),
    }
    session = {"session_manifest_sha256": "c" * 64}
    runtime = {"runtime_closure_binding": {"runtime_install_root": str(tmp_path / "install")}}
    binding = {
        "preembedded_report_path": str(report),
        "preembedded_report_sha256": sha(report),
        "preembedded_world_path": str(world),
        "preembedded_world_sha256": sha(world),
        "source_world_path": str(source_world),
        "source_world_sha256": sha(source_world),
        "source_urdf_path": str(source_urdf),
        "source_urdf_sha256": sha(source_urdf),
        "controller_config_path": str(controller),
        "controller_config_sha256": sha(controller),
        "runtime_install_root": str(tmp_path / "install"),
        "sensor_count": 1,
        "model_name": "tzcup_formal_sanitation_vehicle",
        "spawn_mode": "preembedded_before_gazebo_sensors_system",
        "acceptance_session_sha256": session["session_manifest_sha256"],
        "snapshot": source_binding,
    }
    assert _preembedded_world_binding_valid(
        binding,
        source_binding=source_binding,
        acceptance_session_binding=session,
        runtime_binding=runtime,
    )

    world.write_text("tampered\n", encoding="utf-8")
    assert not _preembedded_world_binding_valid(
        binding,
        source_binding=source_binding,
        acceptance_session_binding=session,
        runtime_binding=runtime,
    )

    binding["preembedded_world_sha256"] = sha(world)
    binding["runtime_install_root"] = str(tmp_path / "other-install")
    assert not _preembedded_world_binding_valid(
        binding,
        source_binding=source_binding,
        acceptance_session_binding=session,
        runtime_binding=runtime,
    )


def test_aggregate_fails_closed_unless_both_runtime_episodes_pass(tmp_path: Path) -> None:
    normal = tmp_path / "normal.json"
    full = tmp_path / "full.json"
    normal_surface = tmp_path / "normal_surface.json"
    full_surface = tmp_path / "full_surface.json"
    for path in (normal_surface, full_surface):
        path.write_text(
            json.dumps(_surface_evidence()),
            encoding="utf-8",
        )
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
    typed = _typed_evidence(tmp_path)
    report = combine(normal, full, normal_surface, full_surface, *typed)
    assert report["passed"] is False
    assert report["status"] == "FAILED"

    full.write_text(
        json.dumps({
            "scenario": "full_tank_fail_closed",
            "passed": True,
        }),
        encoding="utf-8",
    )
    report = combine(normal, full, normal_surface, full_surface, *typed)
    assert report["passed"] is False
    assert report["checks"]["normal_physics_checks_valid"] is False
    assert report["checks"]["full_interlock_checks_valid"] is False

    normal.write_text(
        json.dumps({
            "scenario": "normal_recovery",
            "passed": True,
            "checks": {name: True for name in NORMAL_REQUIRED_CHECKS},
            "metrics": {
                "initial_ground_volume_l": 2.88,
                "final_ground_volume_l": 0.1,
                "ground_removed_l": 2.78,
                "tank_mass_gain_kg": 2.78,
                "dynamic_payload_applied_mass_kg": 2.78,
                "recovery_rate": 2.78 / 2.88,
            },
        }),
        encoding="utf-8",
    )
    full.write_text(
        json.dumps({
            "scenario": "full_tank_fail_closed",
            "passed": True,
            "checks": {name: True for name in FULL_REQUIRED_CHECKS},
            "active_recovery_drain_interlock_terminal": {
                "service_drain_requested_open": True,
                "service_drain_open": False,
            },
        }),
        encoding="utf-8",
    )
    report = combine(normal, full, normal_surface, full_surface, *typed)
    assert report["passed"] is True
    assert report["report_id"] == "tzcup_formal_water_recovery_acceptance_v1"
    assert report["status"] == "FORMAL_WATER_RECOVERY_ACCEPTANCE_PASSED"
    assert report["summary"]["normal_ground_removed_l"] == 2.78
    assert report["summary"]["normal_tank_mass_gain_kg"] == 2.78
    assert report["summary"]["active_recovery_drain_requested_open"] is True
    assert report["summary"]["active_recovery_actual_drain_open"] is False
    assert report["checks"]["side_brush_expanded_sdf_surface_valid"] is True
    typed_transport = report["evidence"]["typed_transport"]
    assert typed_transport["contract"] == {
        "ros_type": "std_msgs/msg/Float64MultiArray",
        "gz_type": "gz.msgs.Double_V",
        "snapshot_length": 63,
        "status_transport": "gazebo_only_diagnostic",
    }
    assert typed_transport["typed_diag_json"] == str(typed[0].resolve())
    assert typed_transport["raw_trace_jsonl"] == str(typed[1].resolve())
    assert typed_transport["runner_script"] == str(typed[2].resolve())
    assert typed_transport["collector_script"] == str(typed[3].resolve())
    assert typed_transport["critical_source_manifest_json"] == str(
        typed[4].resolve()
    )
    for key in (
        "typed_diag_sha256",
        "raw_trace_sha256",
        "runner_sha256",
        "collector_sha256",
        "critical_source_manifest_sha256",
        "typed_cleaning_telemetry_source_sha256",
    ):
        assert len(typed_transport[key]) == 64

    full_surface.write_text(
        json.dumps(_surface_evidence(status="FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_FAILED")),
        encoding="utf-8",
    )
    report = combine(normal, full, normal_surface, full_surface, *typed)
    assert report["passed"] is False
    assert report["checks"]["side_brush_expanded_sdf_surface_valid"] is False


def test_aggregate_rejects_typed_transport_that_final_orchestrator_would_reject(
    tmp_path: Path,
) -> None:
    normal = tmp_path / "normal.json"
    full = tmp_path / "full.json"
    normal_surface = tmp_path / "normal_surface.json"
    full_surface = tmp_path / "full_surface.json"
    normal.write_text(
        json.dumps(
            {
                "scenario": "normal_recovery",
                "passed": True,
                "checks": {name: True for name in NORMAL_REQUIRED_CHECKS},
            }
        ),
        encoding="utf-8",
    )
    full.write_text(
        json.dumps(
            {
                "scenario": "full_tank_fail_closed",
                "passed": True,
                "checks": {name: True for name in FULL_REQUIRED_CHECKS},
            }
        ),
        encoding="utf-8",
    )
    for path in (normal_surface, full_surface):
        path.write_text(
            json.dumps(_surface_evidence()),
            encoding="utf-8",
        )
    typed = _typed_evidence(tmp_path)

    typed[1].write_text('{"frame":1}\n[]\n', encoding="utf-8")
    report = combine(normal, full, normal_surface, full_surface, *typed)
    assert report["passed"] is False
    assert report["checks"]["typed_transport_evidence_valid"] is False

    typed = _typed_evidence(tmp_path)
    report = combine(
        normal,
        full,
        normal_surface,
        full_surface,
        *typed[:-1],
        "A" * 64,
    )
    assert report["passed"] is False
    assert report["checks"]["typed_transport_evidence_valid"] is False

    typed = _typed_evidence(tmp_path)
    typed[4].write_text(
        json.dumps(
            {
                "schema_version": 1,
                "source_package_files_match_frozen_copy": True,
                "install_symlink_count": 1,
            }
        ),
        encoding="utf-8",
    )
    report = combine(normal, full, normal_surface, full_surface, *typed)
    assert report["passed"] is False
    assert report["checks"]["typed_transport_evidence_valid"] is False
