from __future__ import annotations

import json
import re
import subprocess
import time
import types
from pathlib import Path

import pytest
import yaml

import run_formal_final_acceptance as orchestration


ROOT = Path(__file__).resolve().parents[1]


def test_watchdog_marks_child_environment_as_one_outer_step_session() -> None:
    source = (ROOT / "scripts/run_formal_final_acceptance.py").read_text(
        encoding="utf-8"
    )
    assert 'child_environment["FORMAL_ORCHESTRATED_STEP_SESSION"] = "1"' in source
    assert "start_new_session=True" in source
    assert "(signal.SIGINT, 8.0)" in source
    assert "(signal.SIGTERM, 5.0)" in source
    assert "(signal.SIGKILL, 2.0)" in source


def test_heavy_stages_recheck_resources_and_four_chains_are_strictly_serial() -> None:
    assert orchestration.WINDOWS_DRY_RUN_FOUR_CHAIN_STEPS == (
        "chassis", "ground_dirt", "water_recovery", "physical_grasp"
    )
    assert {
        step.step_id for step in orchestration.STEP_SPECS if step.mode == "gazebo"
    }.issubset(orchestration.HEAVY_RUNTIME_STEP_IDS)
    assert "rl_policy" in orchestration.HEAVY_RUNTIME_STEP_IDS
    assert all(
        orchestration._requires_resource_gate(
            next(step for step in orchestration.STEP_SPECS if step.step_id == step_id)
        )
        for step_id in orchestration.WINDOWS_DRY_RUN_FOUR_CHAIN_STEPS
    )
    source = (ROOT / "scripts/run_formal_final_acceptance.py").read_text(encoding="utf-8")
    assert "SKIPPED_AFTER_INDEPENDENT_FOUR_CHAIN_FAILURE" not in source
    assert "stop_on_first_step_failure" in source


def test_orchestrator_required_value_comparison_rejects_bool_integer_coercion() -> None:
    assert orchestration._strict_json_equal(True, True)
    assert orchestration._strict_json_equal([True], [True])
    assert not orchestration._strict_json_equal(1, True)
    assert not orchestration._strict_json_equal(0, False)
    assert not orchestration._strict_json_equal({"passed": 1}, {"passed": True})


def test_orchestrator_ros_domain_range_stops_at_231() -> None:
    assert orchestration._linux_safe_domain(0, 102)
    assert not orchestration._linux_safe_domain(101, 2)
    assert orchestration._linux_safe_domain(215, 17)
    assert not orchestration._linux_safe_domain(215, 18)


def test_orchestrator_defaults_to_formal_perception_matrix_and_rejects_smoke_scale() -> None:
    args = orchestration.build_parser().parse_args(["--static-audit"])
    assert args.perception_episodes == 30
    assert args.base_domain == 60
    assert orchestration._linux_safe_domain(args.base_domain, args.perception_episodes)
    assert (
        args.integrated_source_build_preflight_timeout_seconds
        == orchestration.DEFAULT_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS
        == 300
    )
    source = (ROOT / "scripts/run_formal_final_acceptance.py").read_text(
        encoding="utf-8"
    )
    assert "context.episode_count >= 30" in source
    assert "timeout=context.integrated_source_build_preflight_timeout_seconds" in source
    assert "timeout=55" not in source


@pytest.mark.parametrize("value", [60, 300, 900])
def test_integrated_source_build_preflight_timeout_accepts_bounded_integer(
    value: int,
) -> None:
    args = orchestration.build_parser().parse_args(
        [
            "--static-audit",
            "--integrated-source-build-preflight-timeout-seconds",
            str(value),
        ]
    )
    assert args.integrated_source_build_preflight_timeout_seconds == value


@pytest.mark.parametrize("value", ["59", "901", "not-an-integer"])
def test_integrated_source_build_preflight_timeout_rejects_out_of_contract_value(
    value: str,
) -> None:
    with pytest.raises(SystemExit):
        orchestration.build_parser().parse_args(
            [
                "--static-audit",
                "--integrated-source-build-preflight-timeout-seconds",
                value,
            ]
        )


@pytest.mark.parametrize("value", [True, 59, 901])
def test_context_rejects_invalid_integrated_source_build_preflight_timeout(
    value: object,
) -> None:
    with pytest.raises(ValueError, match="preflight timeout"):
        orchestration.Context(
            root=ROOT,
            runtime_ws=ROOT / ".work/final_runtime",
            integrated_build_manifest=ROOT / ".work/final_runtime/build_manifest.json",
            perception_artifacts=ROOT / ".work/formal_perception_assets",
            onnx_pythonpath=Path("/home/zhexu/tzcup-ros-onnx"),
            run_root=ROOT / ".work/formal_final_acceptance/test-invalid-timeout",
            base_domain=60,
            episode_count=30,
            integrated_source_build_preflight_timeout_seconds=value,  # type: ignore[arg-type]
        )


def test_windows_preflight_never_treats_system_bash_exe_as_formal_linux_runtime() -> None:
    source = (ROOT / "scripts/run_formal_final_acceptance.py").read_text(
        encoding="utf-8"
    )
    assert 'native_posix = os.name == "posix"' in source
    assert "shutil.which(command) if native_posix else None" in source
    assert "not probed outside native Linux/WSL" in source
    assert "run preflight inside WSL/Linux; Windows bash.exe is not a formal runtime" in source


def test_central_formal_rl_budget_audit_locks_both_task_scales(tmp_path: Path) -> None:
    payload = yaml.safe_load(orchestration.FORMAL_RL_BUDGET_CONTRACT.read_text(encoding="utf-8"))
    path = tmp_path / "formal_rl_budget_contract.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    report, failures = orchestration._formal_rl_budget_contract_audit(path)
    assert failures == []
    assert report["stage_a_task_counts"] == {"train": 10000, "validation": 500, "hidden": 1000}
    assert report["multimap_task_counts"] == {"train": 6400, "validation": 800, "hidden": 1200}

    payload["stage_a_fixed_map"]["task_counts"]["train"] = 52
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    report, failures = orchestration._formal_rl_budget_contract_audit(path)
    assert report["valid"] is False
    assert failures == [
        "formal_rl_budget_contract_drift:stage_a_fixed_map.task_counts"
    ]


def test_orchestrator_gate_rejects_integer_for_required_boolean(tmp_path: Path) -> None:
    contract = yaml.safe_load(orchestration.CONTRACT.read_text(encoding="utf-8"))
    gate = contract["evidence_gates"]["whole_vehicle_interlock"]
    gate["path"] = "artifacts/interlock.json"
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True, exist_ok=True)
    contract_path.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")

    payload = {
        "status": gate["success_statuses"][0],
        "checks": {
            dotted.removeprefix("checks."): expected
            for dotted, expected in gate["required_values"].items()
        },
    }
    payload["checks"]["managed_command_topics_have_single_gateway_writer"] = 1
    evidence = tmp_path / gate["path"]
    evidence.parent.mkdir(parents=True, exist_ok=True)
    evidence.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(orchestration.OrchestrationError, match="required value mismatch"):
        orchestration._validate_gate(_context(tmp_path), "whole_vehicle_interlock", 0)


def _context(root: Path = ROOT) -> orchestration.Context:
    return orchestration.Context(
        root=root,
        runtime_ws=root / ".work/final_runtime",
        integrated_build_manifest=root / ".work/final_runtime/build_manifest.json",
        perception_artifacts=root / ".work/formal_perception_assets",
        onnx_pythonpath=root / ".work/onnx",
        run_root=root / ".work/formal_final_acceptance/fresh-session",
        base_domain=90,
        episode_count=30,
        session=root / "artifacts/formal_final_acceptance_session.json",
        snapshot=root / "reports/engineering/formal_vehicle_snapshot_manifest.json",
    )


def _verified_closure_identity() -> dict[str, object]:
    return {
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "passed": True,
        "typed_cleaning_telemetry_source_sha256": "e" * 64,
    }


@pytest.mark.parametrize(
    ("gate_id", "report_id", "status"),
    (
        (
            "a300_drivetrain_runtime",
            "tzcup_formal_a300_drivetrain_runtime_v1",
            "FORMAL_A300_DRIVETRAIN_FORWARD_STOP_RUNTIME_PASSED",
        ),
        (
            "manipulator_trajectory",
            "tzcup_formal_manipulator_runtime_v2",
            "UR5E_AND_ROBOTIQ_GAZEBO_TRAJECTORY_EXECUTION_PASSED",
        ),
        (
            "physical_grasp_and_bin",
            "tzcup_formal_product_grasp_executor_runtime_v1",
            "FORMAL_PRODUCT_GRASP_AND_DRY_BIN_ACCEPTANCE_PASSED",
        ),
    ),
)
def test_runtime_gate_requires_current_matching_runtime_binding(
    tmp_path: Path, gate_id: str, report_id: str, status: str
) -> None:
    contract = yaml.safe_load(orchestration.CONTRACT.read_text(encoding="utf-8"))
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True, exist_ok=True)
    contract_path.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    context = _context(tmp_path)
    snapshot = {
        "source_inventory_sha256": "a" * 64,
        "outputs": {
            "reports/engineering/formal_competition_vehicle.urdf": {"sha256": "b" * 64}
        },
    }
    context.snapshot.parent.mkdir(parents=True, exist_ok=True)
    context.snapshot.write_text(json.dumps(snapshot), encoding="utf-8")
    identity = orchestration._snapshot_identity(context)
    started_ns = time.time_ns() - 1_000_000
    session = {
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "started_epoch_ns": started_ns,
        "snapshot": identity,
    }
    context.session.parent.mkdir(parents=True, exist_ok=True)
    context.session.write_text(json.dumps(session), encoding="utf-8")
    closure = {"closure_sha256": "c" * 64}
    assert context.runtime_closure_manifest is not None
    context.runtime_closure_manifest.parent.mkdir(parents=True, exist_ok=True)
    context.runtime_closure_manifest.write_text(json.dumps(closure), encoding="utf-8")
    gate = contract["evidence_gates"][gate_id]
    evidence = context.root / gate["path"]
    evidence.parent.mkdir(parents=True, exist_ok=True)
    binding = {
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "verified_epoch_ns": time.time_ns(),
        "acceptance_session_binding": {
            "session_manifest_sha256": orchestration._sha256(context.session),
            "session_started_epoch_ns": started_ns,
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
            "snapshot": identity,
            "snapshot_current_source_verified": True,
        },
        "runtime_closure_binding": {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
            "manifest": str(context.runtime_closure_manifest.resolve()),
            "manifest_sha256": orchestration._sha256(context.runtime_closure_manifest),
            "closure_sha256": closure["closure_sha256"],
            "runtime_install_root": str(context.overlay.resolve()),
            "symbolic_link_count": 0,
        },
    }
    sidecar = evidence.with_name(evidence.name + ".runtime_binding.json")
    sidecar.write_text(json.dumps(binding), encoding="utf-8")
    payload = {
        "report_id": report_id,
        "status": status,
        "source_binding": identity,
        "acceptance_session_binding": binding["acceptance_session_binding"],
        "runtime_gate_binding": binding,
    }
    if gate_id == "physical_grasp_and_bin":
        payload.update(passed=True, truth_used_for_control=False)
    evidence.write_text(json.dumps(payload), encoding="utf-8")

    orchestration._validate_gate(context, gate_id, started_ns)

    payload["report_id"] = "tzcup_formal_manipulator_runtime_v1"
    evidence.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(orchestration.OrchestrationError, match="report_id mismatch"):
        orchestration._validate_gate(context, gate_id, started_ns)

    payload["report_id"] = report_id
    payload["runtime_gate_binding"]["runtime_closure_binding"]["closure_sha256"] = "d" * 64
    evidence.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(orchestration.OrchestrationError, match="differs from its sidecar"):
        orchestration._validate_gate(context, gate_id, started_ns)


def _blocked_resume_report(context: orchestration.Context) -> dict[str, object]:
    steps = []
    for spec in orchestration.STEP_SPECS:
        status = "PASSED"
        if spec.step_id == "s100_live":
            status = "BLOCKED_EXTERNAL_HARD_GATE"
        elif spec.step_id == "functional_aggregate":
            status = "PASSED_WITH_EXTERNAL_S100_BLOCK"
        steps.append(
            {
                "id": spec.step_id,
                "mode": spec.mode,
                "status": status,
                "evidence": [],
            }
        )
    return {
        "status": "FORMAL_FINAL_ACCEPTANCE_LOCAL_GATES_PASSED_S100_EXTERNAL_BLOCKED",
        "run_root": str(context.run_root),
        "session_path": str(context.session),
        "snapshot_path": str(context.snapshot),
        "steps": steps,
    }


def test_resume_rejects_runtime_closure_drift_from_session_start(tmp_path: Path) -> None:
    context = _context(tmp_path)
    closure = {
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "manifest": "/tmp/frozen-closure.json",
        "manifest_sha256": "a" * 64,
        "closure_sha256": "b" * 64,
        "symbolic_link_count": 0,
    }
    session = {
        "runtime_closure_binding": {
            **closure,
            "runtime_install_root": str(context.overlay.resolve()),
        }
    }

    orchestration._require_session_runtime_closure_binding(context, session, closure)

    drifted = {**closure, "closure_sha256": "c" * 64}
    with pytest.raises(
        orchestration.OrchestrationError,
        match="differs from the acceptance session binding",
    ):
        orchestration._require_session_runtime_closure_binding(context, session, drifted)


def _install_fake_resume_lock(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        orchestration,
        "fcntl",
        types.SimpleNamespace(LOCK_EX=1, LOCK_NB=2, LOCK_UN=8, flock=lambda *args: None),
    )


def test_static_audit_covers_each_contract_gate_exactly_once() -> None:
    report = orchestration.static_audit()
    contract = yaml.safe_load(orchestration.CONTRACT.read_text(encoding="utf-8"))
    assert report["passed"] is True
    assert report["contract_gate_count"] == len(contract["evidence_gates"])
    assert set(report["gate_producers"]) == set(contract["evidence_gates"])
    assert all(len(producers) == 1 for producers in report["gate_producers"].values())
    assert report["gate_producers"]["s100_live_runtime"] == ["s100_live"]
    assert report["registered_function_position_count"] == 38
    assert report["crosswalk_function_position_count"] == 38
    assert report["unique_gate_output_path_count"] == len(contract["evidence_gates"])
    assert all(
        "static" in modes and ({"gazebo", "external"} & set(modes))
        for modes in report["function_position_evidence_modes"].values()
    )
    assert report["s100_live_gate"] == {
        "gate": "s100_live_runtime",
        "mode": "external",
        "hardware_only": True,
        "operator_trusted_evidence_required": True,
        "cryptographic_hardware_attestation": False,
        "malicious_pc_forgery_resistant": False,
        "pc_substitution_allowed": False,
    }
    assert report["runtime_closure"] == {
        "manifest_required": True,
        "merged_overlay_required": True,
        "symlink_install_allowed": False,
        "runtime_package_count": 16,
        "side_brush_surface_preflight_required": True,
        "typed_cleaning_telemetry_source_manifest_required": True,
        "water_normal_full_surface_hash_reverification_required": True,
            "water_typed_transport_evidence_required": True,
            "verified_before_and_after_every_step": True,
            "functional_aggregate_revalidates_runtime_binding_sidecars": True,
            "runtime_gate_bindings_required": [
            "a300_drivetrain_runtime",
            "auxiliary_power_lighting",
            "cleaning_actuators",
            "cleaning_motor_runtime",
            "dynamic_obstacle_avoidance",
            "end_to_end_cleaning_mission",
            "first_map_then_clean",
            "formal_20_cube_grasp_and_dynamic_mass",
            "ground_dirt_cleaning",
            "integrated_basic_physics",
            "manipulator_trajectory",
            "multi_site_product_generalization",
            "physical_grasp_and_bin",
            "product_visual_acceptance",
            "random_scene_perception",
            "sensor_runtime",
            "service_door_runtime",
            "service_interface_acceptance",
            "service_visual_acceptance",
            "water_recovery",
            "whole_vehicle_interlock",
        ],
    }


def test_runtime_binding_gate_contract_and_retained_sidecars_have_one_authority(
    tmp_path: Path,
) -> None:
    contract = yaml.safe_load(orchestration.CONTRACT.read_text(encoding="utf-8"))
    contracted = {
        gate_id
        for gate_id, row in contract["evidence_gates"].items()
        if isinstance(row, dict) and row.get("runtime_binding") is not None
    }
    assert contracted == orchestration.RUNTIME_GATE_BINDING_GATES

    context = orchestration.Context(
        root=orchestration.ROOT,
        runtime_ws=tmp_path / "runtime/install",
        integrated_build_manifest=tmp_path / "integrated-build.json",
        perception_artifacts=tmp_path / "perception",
        onnx_pythonpath=tmp_path / "onnx",
        run_root=tmp_path / "run",
        base_domain=210,
        episode_count=30,
        runtime_closure_manifest=tmp_path / "closure.json",
    )
    retained = {
        path.relative_to(context.root).as_posix()
        for path in orchestration._runtime_binding_auxiliary_paths(context)
    }
    expected = {
        str(contract["evidence_gates"][gate_id]["path"]) + ".runtime_binding.json"
        for gate_id in contracted
    }
    assert retained == expected


def test_requested_whole_vehicle_order_is_preserved() -> None:
    order = [step.step_id for step in orchestration.STEP_SPECS]
    assert len(order) == 31
    required = [
        "freeze_snapshot",
        "start_session",
        "visual",
        "inertia",
        "sensor",
        "chassis",
        "safety_interlock",
        "cleaning_motors",
        "ground_dirt",
        "water_recovery",
        "service_door",
        "charge_and_drain",
        "manipulator",
        "twenty_cubes",
        "first_map",
        "saved_map_reuse",
        "perception",
        "dynamic_obstacle",
        "rl_policy",
        "single_episode",
        "multisite_product",
        "s100_live",
        "finalize_session",
        "functional_aggregate",
    ]
    assert [order.index(step) for step in required] == sorted(order.index(step) for step in required)


def test_every_gazebo_step_has_one_shared_lock_strategy() -> None:
    report = orchestration.static_audit()
    rows = report["runner_inventory"]
    for step in orchestration.STEP_SPECS:
        if step.mode != "gazebo":
            continue
        assert rows[step.step_id]["gazebo_lock_strategy"] in {
            "runner_internal",
            "orchestrator_outer_flock",
        }
        assert not (
            rows[step.step_id]["gazebo_lock_strategy"] == "runner_internal"
            and step.requires_outer_gazebo_lock
        )
    assert rows["safety_interlock"]["gazebo_lock_strategy"] == "runner_internal"
    assert rows["integrated_basic_physics"]["gazebo_lock_strategy"] == "orchestrator_outer_flock"


def test_commands_use_one_fresh_episode_map_rl_and_e2e_root() -> None:
    context = _context()
    materialize, _ = orchestration._step_command("episode_materialization", context)
    assert str(context.episode_root) in " ".join(materialize)

    first_map, first_env = orchestration._step_command("first_map", context)
    assert first_map[-1].endswith("run_formal_first_map_dynamic_prerequisite.sh")
    assert first_env["FORMAL_DYNAMIC_EPISODE_ROOT"] == str(context.episode_root)
    assert first_env["FORMAL_DYNAMIC_SAVED_MAP_ROOT"] == str(context.map_root)

    rl, _ = orchestration._step_command("rl_policy", context)
    assert rl[0] == "__sequence__"
    sequences = json.loads(rl[1])
    rendered = " ".join(value for command in sequences for value in command)
    assert str(context.rl_evidence_root) in rendered
    assert "formal_active_cleaning_train" in rendered
    for flag, last in (("--train", "31:199"), ("--validation", "7:99"), ("--test", "11:99")):
        assert flag in rendered
        assert last in rendered
    assert "formal_stage_a_active_cleaning_train" in rendered
    assert "formal_rl_budget_contract.yaml" in rendered
    assert "--policy-seeds 7,17,29,43,61" in rendered

    e2e, _ = orchestration._step_command("single_episode", context)
    rendered_e2e = " ".join(e2e)
    assert str(context.episode_root) in rendered_e2e
    assert str(context.map_root) in rendered_e2e
    assert str(context.same_map_baseline) in rendered_e2e
    assert str(context.rl_evidence_root / "formal_planning/q_policy.json") in rendered_e2e

    multisite, _ = orchestration._step_command("multisite_product", context)
    rendered_multisite = " ".join(multisite)
    assert "formal_multisite_product_acceptance.py" in rendered_multisite
    assert "--execute" in rendered_multisite
    assert str(context.run_root / "multisite_product_sites") in rendered_multisite
    assert str(context.run_root / "multisite_product_runtime") in rendered_multisite
    assert orchestration.STEP_SPECS[[row.step_id for row in orchestration.STEP_SPECS].index("multisite_product")].mode == "gazebo"


def test_final_orchestrator_rejects_reduced_frozen_multimap_scenario(
    tmp_path: Path,
) -> None:
    scenario = tmp_path / "default_scenario.yaml"
    scenario.write_text(
        "split:\n"
        "  train: {map_count: 2}\n"
        "  val: {map_count: 2}\n"
        "  hidden: {map_count: 1}\n",
        encoding="utf-8",
    )
    with pytest.raises(orchestration.OrchestrationError, match="32/8/12"):
        orchestration._formal_multimap_training_arguments(scenario)


def test_static_audit_recursively_binds_multisite_pc_edgesam_and_follow_path_interfaces() -> None:
    checks, failures = orchestration._recursive_interface_contract_audit(ROOT)

    assert failures == []
    assert checks == {
        "validation_split_maps_to_generator_val": True,
        "pc_edgesam_ground_dirt_mask_is_image_end_to_end": True,
        "active_cleaning_follow_path_action_chain_is_consistent": True,
    }


def test_static_audit_rejects_pc_edgesam_image_interface_drift(tmp_path: Path) -> None:
    root = tmp_path
    files = {
        "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml": (
            "split: {train: {map_count: 32}, val: {map_count: 8}, hidden: {map_count: 12}}\n"
        ),
        "scripts/run_formal_final_acceptance.py": 'source_keys = {"validation": "val"}\n',
        "scripts/formal_multisite_product_acceptance.py": (
            'generator_split = "val" if site["split"] == "validation" else site["split"]\n'
            '"--split", generator_split\n'
        ),
        "scripts/collect_formal_single_episode_cleaning_mission.py": (
            "MULTISITE_INTERFACES = {\n"
            '    "edgesam": {"name": "/perception/ground_dirt/masks", '
            '"type": "sensor_msgs/msg/Image", "interface_kind": "topic", '
            '"observed_topic": "/perception/ground_dirt/masks"},\n'
            '    "nav2": {"name": "/follow_path", '
            '"type": "nav2_msgs/action/FollowPath", "interface_kind": "action", '
            '"observed_topic": "/follow_path/_action/status"},\n'
            "}\n"
            "def wire(self):\n"
            '    self.create_subscription(Image, MULTISITE_INTERFACES["edgesam"]["observed_topic"], None)\n'
            '    self.create_subscription(GoalStatusArray, MULTISITE_INTERFACES["nav2"]["observed_topic"], None)\n'
        ),
        "starter_ws/src/sanitation_perception/config/formal_open_vocab_perception.yaml": (
            "product_outputs: {masks: /perception/ground_dirt/masks}\n"
            "product_output_types: {/perception/ground_dirt/masks: std_msgs/msg/String}\n"
        ),
        "config/high_fidelity_vehicle/formal_multisite_product_acceptance_contract.yaml": (
            "site_evidence:\n  required_topics:\n"
            "    edgesam: {name: /perception/ground_dirt/masks, type: sensor_msgs/msg/Image, interface_kind: topic}\n"
            "    nav2: {name: /follow_path, type: nav2_msgs/action/FollowPath, interface_kind: action}\n"
        ),
        "starter_ws/src/sanitation_perception/sanitation_perception/pc_open_vocab_adapter.py": (
            'EdgeSamOnnxSegmenter\nImage, "/perception/ground_dirt/masks", 10\n'
        ),
        "starter_ws/src/sanitation_active_cleaning/sanitation_active_cleaning/formal_observation_bridge.py": (
            '"/perception/ground_dirt/masks"\nself.create_subscription(\n                Image,\n'
        ),
        "starter_ws/src/sanitation_active_cleaning/config/formal_runtime.yaml": (
            "formal_active_cleaning_trajectory_executor:\n"
            "  ros__parameters: {follow_path_action: /follow_path, controller_id: FollowPath}\n"
        ),
        "starter_ws/src/sanitation_navigation/config/nav2.yaml": (
            "controller_server:\n  ros__parameters:\n"
            "    controller_plugins: [FollowPath]\n"
            "    FollowPath: {plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController}\n"
        ),
        "starter_ws/src/sanitation_active_cleaning/sanitation_active_cleaning/formal_trajectory_executor.py": (
            'NAVIGATION_ACTION = "/follow_path"\nActionClient(\nFollowPath,\n'
            "goal = FollowPath.Goal()\ngoal.controller_id\n"
        ),
    }
    for relative, content in files.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    checks, failures = orchestration._recursive_interface_contract_audit(root)

    assert checks["pc_edgesam_ground_dirt_mask_is_image_end_to_end"] is False
    assert failures == [
        "recursive_interface_contract_drift:pc_edgesam_ground_dirt_mask_is_image_end_to_end"
    ]


def test_visual_step_passes_setup_file_not_install_directory() -> None:
    context = _context()
    _, environment = orchestration._step_command("visual", context)
    assert environment["FORMAL_VEHICLE_VISUAL_RUNTIME_SETUP"] == str(
        context.overlay / "setup.bash"
    )


def test_integrated_step_refreshes_only_git_binding_after_prior_evidence_changes(
    tmp_path: Path,
) -> None:
    context = _context(tmp_path)
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True, exist_ok=True)
    contract_path.write_bytes(orchestration.CONTRACT.read_bytes())
    context.integrated_build_manifest.parent.mkdir(parents=True, exist_ok=True)
    context.integrated_build_manifest.write_text(
        json.dumps({"build_started_epoch_ns": 123456789}), encoding="utf-8"
    )
    command, environment = orchestration._step_command("integrated_basic_physics", context)
    assert command[0] == "__sequence__"
    sequences = json.loads(command[1])
    assert "record-build" in sequences[0]
    assert sequences[1][-1].endswith("run_integrated_functional_acceptance.sh")
    refreshed = context.run_root / "integrated_build_manifest.json"
    assert environment["INTEGRATED_ACCEPTANCE_BUILD_MANIFEST"] == str(refreshed)
    assert str(refreshed) in sequences[0]


def test_step_command_reads_gate_output_from_the_isolated_context_contract(
    tmp_path: Path,
) -> None:
    context = _context(tmp_path)
    isolated_contract_path = tmp_path / orchestration.CONTRACT.relative_to(
        orchestration.ROOT
    )
    isolated_contract_path.parent.mkdir(parents=True, exist_ok=True)
    isolated_contract = yaml.safe_load(
        orchestration.CONTRACT.read_text(encoding="utf-8")
    )
    isolated_contract["evidence_gates"]["a300_drivetrain_runtime"]["path"] = (
        "artifacts/isolated/mobility.json"
    )
    isolated_contract_path.write_text(
        yaml.safe_dump(isolated_contract, sort_keys=False), encoding="utf-8"
    )

    command, environment = orchestration._step_command("chassis", context)

    isolated_output = str(tmp_path / "artifacts/isolated/mobility.json")
    global_output = str(
        orchestration.ROOT
        / yaml.safe_load(orchestration.CONTRACT.read_text(encoding="utf-8"))["evidence_gates"][
            "a300_drivetrain_runtime"
        ]["path"]
    )
    assert environment["FORMAL_VEHICLE_MOBILITY_OUTPUT"] == isolated_output
    assert global_output not in command
    assert global_output not in environment.values()


def test_freshness_includes_component_register_and_marks_planned_archives(
    tmp_path: Path,
) -> None:
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True)
    contract_path.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "component_register": {"path": "reports/component.json"},
                    "runtime": {"path": "artifacts/runtime.json"},
                }
            }
        ),
        encoding="utf-8",
    )
    context = _context(tmp_path)
    rows = {row["path"]: row for row in orchestration.freshness_rows(context)}
    assert rows["artifacts/runtime.json"]["passed"] is True
    runtime = tmp_path / "artifacts/runtime.json"
    runtime.parent.mkdir(parents=True, exist_ok=True)
    runtime.write_text("{}\n", encoding="utf-8")
    rows = {row["path"]: row for row in orchestration.freshness_rows(context)}
    assert rows["artifacts/runtime.json"] == {
        "path": "artifacts/runtime.json",
        "passed": False,
        "detail": "existing evidence would be refused",
    }
    component = tmp_path / "reports/component.json"
    component.parent.mkdir(parents=True, exist_ok=True)
    component.write_text("component\n", encoding="utf-8")
    rows = {row["path"]: row for row in orchestration.freshness_rows(context)}
    assert rows["reports/component.json"] == {
        "path": "reports/component.json",
        "passed": False,
        "detail": "existing evidence would be refused",
    }

    rows = {
        row["path"]: row
        for row in orchestration.freshness_rows(
            context, planned_archive_sources=[component]
        )
    }
    assert rows["reports/component.json"] == {
        "path": "reports/component.json",
        "passed": True,
        "detail": "existing evidence scheduled for preservation",
    }


def test_archive_plan_preserves_component_and_global_aggregate_reports(
    tmp_path: Path,
) -> None:
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True)
    contract_path.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "component_register": {"path": "reports/component.json"},
                    "water_recovery": {
                        "path": "artifacts/formal_water_recovery_acceptance.json"
                    },
                }
            }
        ),
        encoding="utf-8",
    )
    context = _context(tmp_path)
    retained = {
        tmp_path / "reports/component.json": "component\n",
        tmp_path / "artifacts/formal_water_recovery/water_normal.json": "raw\n",
        tmp_path / "artifacts/formal_water_recovery_acceptance.json": "final\n",
        tmp_path / orchestration.ORCHESTRATION_REPORT.relative_to(orchestration.ROOT): (
            "orchestration\n"
        ),
        tmp_path / orchestration.FUNCTIONAL_AUDIT.relative_to(orchestration.ROOT): (
            "aggregate\n"
        ),
    }
    for path, contents in retained.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")

    plan = orchestration.plan_final_output_archive(
        context, timestamp="20260829T120000000000Z"
    )
    assert plan["validated"] is True
    assert plan["executed"] is False
    assert all(path.is_file() for path in retained)
    assert not Path(plan["destination"]).exists()
    planned_sources = [Path(str(row["source"])) for row in plan["entries"]]
    for path in retained:
        assert any(source == path or source in path.parents for source in planned_sources)
    rows = {
        row["path"]: row
        for row in orchestration.freshness_rows(
            context, planned_archive_sources=planned_sources
        )
    }
    freshness_paths = [
        tmp_path / "reports/component.json",
        tmp_path / "artifacts/formal_water_recovery",
        tmp_path / "artifacts/formal_water_recovery_acceptance.json",
        tmp_path / orchestration.ORCHESTRATION_REPORT.relative_to(orchestration.ROOT),
        tmp_path / orchestration.FUNCTIONAL_AUDIT.relative_to(orchestration.ROOT),
    ]
    for path in freshness_paths:
        relative = path.relative_to(tmp_path).as_posix()
        assert rows[relative] == {
            "path": relative,
            "passed": True,
            "detail": "existing evidence scheduled for preservation",
        }

    preserved = orchestration.execute_final_output_archive(context, plan)
    assert preserved["executed"] is True
    assert all(not path.exists() for path in retained)
    archive = Path(plan["destination"])
    for path, contents in retained.items():
        archived = archive / path.relative_to(tmp_path)
        assert archived.read_text(encoding="utf-8") == contents
    with pytest.raises(orchestration.OrchestrationError, match="unique and unused"):
        orchestration.execute_final_output_archive(context, plan)


def test_sensor_runtime_sidecars_are_archived_and_refused_as_stale_attempt_evidence(
    tmp_path: Path,
) -> None:
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True)
    contract_path.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "sensor_runtime": {"path": "reports/sensor-runtime.json"},
                    "sensor_fov_and_occlusion": {"path": "reports/sensor-fov.json"},
                }
            }
        ),
        encoding="utf-8",
    )
    context = _context(tmp_path)
    sidecars = {
        tmp_path / "reports/sensor-runtime.json.runtime_binding.json": "binding\n",
        tmp_path / "reports/sensor-runtime.preembedded_sensor_world.sdf": "world\n",
        tmp_path / "reports/sensor-runtime.preembedded_sensor_world.json": "report\n",
    }
    for path, contents in sidecars.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")

    stale_rows = {
        row["path"]: row for row in orchestration.freshness_rows(context)
    }
    for path in sidecars:
        relative = path.relative_to(tmp_path).as_posix()
        assert stale_rows[relative] == {
            "path": relative,
            "passed": False,
            "detail": "existing evidence would be refused",
        }

    plan = orchestration.plan_final_output_archive(
        context, timestamp="20260830T130000000000Z"
    )
    planned_sources = [Path(str(row["source"])) for row in plan["entries"]]
    assert set(sidecars).issubset(planned_sources)
    rows = {
        row["path"]: row
        for row in orchestration.freshness_rows(
            context, planned_archive_sources=planned_sources
        )
    }
    for path in sidecars:
        relative = path.relative_to(tmp_path).as_posix()
        assert rows[relative] == {
            "path": relative,
            "passed": True,
            "detail": "existing evidence scheduled for preservation",
        }

    preserved = orchestration.execute_final_output_archive(context, plan)
    assert preserved["executed"] is True
    for path, contents in sidecars.items():
        archived = Path(plan["destination"]) / path.relative_to(tmp_path)
        assert archived.read_text(encoding="utf-8") == contents


def test_sensor_step_binds_preembedded_outputs_to_the_contract_runtime_path(
    tmp_path: Path,
) -> None:
    context = _context(tmp_path)
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True, exist_ok=True)
    contract_path.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "sensor_runtime": {"path": "reports/custom-sensor.json"},
                    "sensor_fov_and_occlusion": {"path": "reports/custom-fov.json"},
                }
            }
        ),
        encoding="utf-8",
    )

    _, environment = orchestration._step_command("sensor", context)

    assert environment["FORMAL_SENSOR_PREEMBEDDED_WORLD"] == str(
        tmp_path / "reports/custom-sensor.preembedded_sensor_world.sdf"
    )
    assert environment["FORMAL_SENSOR_PREEMBEDDED_REPORT"] == str(
        tmp_path / "reports/custom-sensor.preembedded_sensor_world.json"
    )


def test_gate_runtime_bindings_are_archived_as_attempt_evidence(tmp_path: Path) -> None:
    gate_paths = {
        "sensor_runtime": "reports/sensor.json",
        "a300_drivetrain_runtime": "artifacts/chassis.json",
        "whole_vehicle_interlock": "artifacts/interlock.json",
        "auxiliary_power_lighting": "artifacts/auxiliary.json",
        "water_recovery": "artifacts/water.json",
        "service_door_runtime": "artifacts/service-door.json",
        "service_interface_acceptance": "artifacts/service-interface.json",
        "manipulator_trajectory": "artifacts/manipulator.json",
        "physical_grasp_and_bin": "artifacts/physical-grasp.json",
        "formal_20_cube_grasp_and_dynamic_mass": "artifacts/twenty-cubes.json",
    }
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True, exist_ok=True)
    contract_path.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    gate: {"path": path} for gate, path in gate_paths.items()
                }
            }
        ),
        encoding="utf-8",
    )
    context = _context(tmp_path)
    bindings = []
    for path in gate_paths.values():
        output = tmp_path / path
        binding = output.with_name(output.name + ".runtime_binding.json")
        binding.parent.mkdir(parents=True, exist_ok=True)
        binding.write_text(path + "\n", encoding="utf-8")
        bindings.append(binding)

    rows = {row["path"]: row for row in orchestration.freshness_rows(context)}
    for binding in bindings:
        relative = binding.relative_to(tmp_path).as_posix()
        assert rows[relative]["passed"] is False

    plan = orchestration.plan_final_output_archive(
        context, timestamp="20260830T140000000000Z"
    )
    planned_sources = {Path(str(row["source"])) for row in plan["entries"]}
    assert set(bindings).issubset(planned_sources)

    preserved = orchestration.execute_final_output_archive(context, plan)
    assert preserved["executed"] is True
    for binding in bindings:
        archived = Path(plan["destination"]) / binding.relative_to(tmp_path)
        assert archived.read_text(encoding="utf-8").endswith(".json\n")


def test_execute_refuses_before_creating_run_root_when_preflight_blocks(
    tmp_path: Path, monkeypatch
) -> None:
    context = _context(tmp_path)
    monkeypatch.setattr(
        orchestration,
        "preflight",
        lambda unused: {
            "report_id": "test",
            "status": "FORMAL_FINAL_ACCEPTANCE_PREFLIGHT_BLOCKED",
            "passed": False,
            "blockers": ["fresh_path:artifacts/runtime.json"],
        },
    )
    report, return_code = orchestration.execute(context)
    assert return_code == 2
    assert report["status"] == "FORMAL_FINAL_ACCEPTANCE_EXECUTION_REFUSED_BY_PREFLIGHT"
    assert not context.run_root.exists()


def _configure_execute_until_s100(
    context: orchestration.Context,
    s100: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    context.runtime_closure_manifest.parent.mkdir(parents=True, exist_ok=True)
    context.runtime_closure_manifest.write_text("closure\n", encoding="utf-8")

    monkeypatch.setattr(
        orchestration,
        "preflight",
        lambda unused: {"passed": True, "archive_plan": {}},
    )
    monkeypatch.setattr(
        orchestration,
        "execute_final_output_archive",
        lambda unused_context, unused_plan: {},
    )
    monkeypatch.setattr(
        orchestration,
        "_verify_runtime_closure",
        lambda unused_context, phase: {"phase": phase, "passed": True},
    )
    monkeypatch.setattr(orchestration, "_snapshot_check", lambda *unused_args: None)
    monkeypatch.setattr(orchestration, "_lock_available", lambda: (True, None))
    monkeypatch.setattr(
        orchestration,
        "_step_command",
        lambda step_id, *unused_args, **unused_kwargs: ([step_id], {}),
    )

    def fake_run(command, *unused_args, **unused_kwargs) -> None:
        if command == ["start_session"]:
            context.session.parent.mkdir(parents=True, exist_ok=True)
            context.session.write_text(
                json.dumps(
                    {
                        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                        "started_epoch_ns": 1,
                        "snapshot": {},
                    }
                ),
                encoding="utf-8",
            )

    monkeypatch.setattr(orchestration, "_run_process", fake_run)
    monkeypatch.setattr(
        orchestration,
        "_validate_gate",
        lambda unused_context, gate_id, *unused_args, **unused_kwargs: {
            "gate": gate_id,
            "sha256": "a" * 64,
        },
    )
    monkeypatch.setattr(
        orchestration,
        "_local_gate_digests_from_report",
        lambda *unused_args, **unused_kwargs: {},
    )
    monkeypatch.setattr(orchestration, "_gate_path", lambda *unused_args: s100)


def test_execute_refuses_discovered_s100_final_without_operator_trust(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    s100 = tmp_path / "artifacts/formal_s100_live_acceptance.json"
    s100.parent.mkdir(parents=True)
    s100.write_text("{}\n", encoding="utf-8")
    _configure_execute_until_s100(context, s100, monkeypatch)
    monkeypatch.setattr(
        orchestration,
        "_validate_s100_external_evidence",
        lambda *unused_args: pytest.fail("untrusted S100 final must not be validated"),
    )
    monkeypatch.setattr(
        orchestration,
        "_run_session_finalize",
        lambda *unused_args: pytest.fail("untrusted S100 final must not finalize"),
    )

    report, code = orchestration.execute(context)
    assert code == 3
    assert report["status"] == "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_FAILED"
    assert "requires explicit acceptance" in report["error"]
    assert report["s100_evidence_trust_boundary"] == orchestration.S100_EVIDENCE_TRUST_BOUNDARY


def test_execute_records_the_failed_runner_step_before_failing_closed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    s100 = tmp_path / "artifacts/formal_s100_live_acceptance.json"
    _configure_execute_until_s100(context, s100, monkeypatch)

    def fail_sensor(command, *unused_args, **unused_kwargs) -> None:
        if command == ["start_session"]:
            context.session.parent.mkdir(parents=True, exist_ok=True)
            context.session.write_text(
                json.dumps(
                    {
                        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                        "started_epoch_ns": 1,
                        "snapshot": {},
                    }
                ),
                encoding="utf-8",
            )
        if command == ["sensor"]:
            raise orchestration.OrchestrationError(
                "command failed rc=125; log=simulated-sensor.log"
            )

    monkeypatch.setattr(orchestration, "_run_process", fail_sensor)

    report, code = orchestration.execute(context)

    assert code == 3
    assert report["status"] == "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_FAILED"
    failed = report["steps"][-1]
    expected_log = context.run_root / "orchestration_logs/05_sensor.log"
    assert failed["id"] == "sensor"
    assert failed["status"] == "FAILED"
    assert failed["blocker"] == "command"
    assert failed["exit_status"] == "COMMAND_FAILED"
    assert failed["exit_code"] == 125
    assert failed["reason"] == "command failed rc=125; log=simulated-sensor.log"
    assert failed["evidence_paths"] == [str(expected_log)]
    assert failed["runtime_closure_after"]["phase"] == "after:sensor"
    persisted = json.loads(
        (context.run_root / "orchestration_report.json").read_text(encoding="utf-8")
    )
    assert persisted["steps"][-1] == failed


def test_execute_stops_at_first_four_chain_failure_and_preserves_running_session(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    s100 = tmp_path / "artifacts/formal_s100_live_acceptance.json"
    _configure_execute_until_s100(context, s100, monkeypatch)
    invoked: list[str] = []

    def fail_chassis(command, *unused_args, **unused_kwargs) -> None:
        step_id = command[0]
        invoked.append(step_id)
        if step_id == "start_session":
            context.session.parent.mkdir(parents=True, exist_ok=True)
            context.session.write_text(
                json.dumps(
                    {
                        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                        "started_epoch_ns": 1,
                        "snapshot": {},
                    }
                ),
                encoding="utf-8",
            )
        if step_id == "chassis":
            raise orchestration.OrchestrationError("simulated chassis resource-gate failure")

    monkeypatch.setattr(orchestration, "_run_process", fail_chassis)

    report, code = orchestration.execute(context)

    assert code == 3
    assert report["status"] == "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_FAILED"
    assert invoked[-1] == "chassis"
    assert "ground_dirt" not in invoked
    assert [row["id"] for row in report["steps"]][-1] == "chassis"
    assert report["steps"][-1]["status"] == "FAILED"
    assert report["stop_on_first_step_failure"] is True
    assert json.loads(context.session.read_text(encoding="utf-8"))["status"] == (
        "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
    )


def test_execute_initial_complete_records_s100_trust_boundary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    context.accept_operator_trusted_s100 = True
    s100 = tmp_path / "artifacts/formal_s100_live_acceptance.json"
    s100.parent.mkdir(parents=True)
    s100.write_text("{}\n", encoding="utf-8")
    _configure_execute_until_s100(context, s100, monkeypatch)
    monkeypatch.setattr(
        orchestration,
        "_validate_s100_external_evidence",
        lambda *unused_args: {"gate": orchestration.EXTERNAL_GATE, "sha256": "a" * 64},
    )
    monkeypatch.setattr(
        orchestration,
        "_run_session_finalize",
        lambda *unused_args: {"status": "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"},
    )
    monkeypatch.setattr(
        orchestration,
        "_run_functional_aggregate",
        lambda *unused_args, **unused_kwargs: {
            "status": "FORMAL_FUNCTIONAL_ACCEPTANCE_COMPLETE",
            "passed_position_count": 38,
            "pending_position_count": 0,
        },
    )

    report, code = orchestration.execute(context)

    assert code == 0
    assert report["status"] == "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_COMPLETE"
    assert report["operator_trusted_s100_acknowledged"] is True
    assert report["s100_evidence_trust_boundary"] == orchestration.S100_EVIDENCE_TRUST_BOUNDARY


def test_initial_s100_commit_aggregate_failure_is_recovered_without_local_rerun(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    context.accept_operator_trusted_s100 = True
    s100_path = tmp_path / "artifacts/formal_s100_live_acceptance.json"
    s100_path.parent.mkdir(parents=True)
    s100_path.write_text("{}\n", encoding="utf-8")
    _configure_execute_until_s100(context, s100_path, monkeypatch)
    monkeypatch.setattr(
        orchestration,
        "_validate_s100_external_evidence",
        lambda *unused_args: {"gate": orchestration.EXTERNAL_GATE, "sha256": "a" * 64},
    )
    complete_session = {
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE",
        "failures": {},
        "started_epoch_ns": 1,
        "finished_epoch_ns": 2,
        "snapshot": {},
        "evidence": {f"gate_{index}": {} for index in range(25)},
    }

    def finalize(*unused_args):
        context.session.write_text(json.dumps(complete_session), encoding="utf-8")
        return complete_session

    monkeypatch.setattr(orchestration, "_run_session_finalize", finalize)
    monkeypatch.setattr(
        orchestration,
        "_run_functional_aggregate",
        lambda *unused_args, **unused_kwargs: (_ for _ in ()).throw(
            orchestration.OrchestrationError("simulated initial aggregate failure")
        ),
    )

    pending_report, pending_code = orchestration.execute(context)

    assert pending_code == 3
    assert pending_report["status"] == orchestration.S100_COMMITTED_AGGREGATE_PENDING
    assert [row["id"] for row in pending_report["steps"]] == [
        spec.step_id for spec in orchestration.STEP_SPECS
    ]
    assert pending_report["steps"][-1]["id"] == "functional_aggregate"
    assert pending_report["steps"][-1]["status"] == "FAILED"
    assert pending_report["steps"][-1]["blocker"] == "functional_aggregate"
    assert pending_report["local_gate_digests"] == {}

    _install_fake_resume_lock(monkeypatch)
    local_results = {f"gate_{index}": {"gate": f"gate_{index}"} for index in range(24)}
    complete_verifications = []
    monkeypatch.setattr(
        orchestration,
        "_resume_preconditions",
        lambda unused_context, unused_logs: (
            pending_report,
            complete_session,
            1,
            _verified_closure_identity(),
        ),
    )
    monkeypatch.setattr(orchestration, "_gate_path", lambda *unused_args: s100_path)
    monkeypatch.setattr(
        orchestration,
        "_verify_runtime_closure",
        lambda unused_context, phase: {"phase": phase, "passed": True},
    )
    monkeypatch.setattr(
        orchestration,
        "_revalidate_local_gate_digests",
        lambda *unused_args, **unused_kwargs: local_results,
    )
    monkeypatch.setattr(
        orchestration,
        "_verify_complete_session_evidence",
        lambda *unused_args: complete_verifications.append(True),
    )
    monkeypatch.setattr(
        orchestration,
        "_run_session_finalize",
        lambda *unused_args: pytest.fail("COMPLETE session must not finalize again"),
    )
    monkeypatch.setattr(
        orchestration,
        "_run_functional_aggregate",
        lambda *unused_args, **unused_kwargs: {
            "status": "FORMAL_FUNCTIONAL_ACCEPTANCE_COMPLETE",
            "passed_position_count": 38,
            "pending_position_count": 0,
            "complete": True,
        },
    )
    monkeypatch.setattr(
        orchestration,
        "_step_command",
        lambda *unused_args, **unused_kwargs: pytest.fail("resume must not build local commands"),
    )
    monkeypatch.setattr(
        orchestration,
        "_run_process",
        lambda *unused_args, **unused_kwargs: pytest.fail("resume must not run local/Gazebo work"),
    )

    result, code = orchestration.resume_s100(context)

    assert code == 0
    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_COMPLETE"
    assert [row["id"] for row in result["steps"]] == [
        spec.step_id for spec in orchestration.STEP_SPECS
    ]
    assert complete_verifications == [True]
    aggregate_history = result["resume_history"][-1]["terminal_rows"]["functional_aggregate"]
    assert aggregate_history["before"]["status"] == "FAILED"
    assert aggregate_history["before"]["blocker"] == "functional_aggregate"
    assert aggregate_history["before_sha256"] == orchestration._json_digest(
        aggregate_history["before"]
    )
    assert aggregate_history["after_sha256"] == orchestration._json_digest(
        aggregate_history["after"]
    )
    assert result["resume_history"][-1]["previous_orchestration_status"] == (
        orchestration.S100_COMMITTED_AGGREGATE_PENDING
    )


def test_resume_preconditions_rejects_generic_orchestration_failure(tmp_path: Path) -> None:
    context = _context(tmp_path)
    context.accept_operator_trusted_s100 = True
    context.run_root.mkdir(parents=True)
    context.session.parent.mkdir(parents=True)
    context.session.write_text("{}\n", encoding="utf-8")
    context.snapshot.parent.mkdir(parents=True)
    context.snapshot.write_text("{}\n", encoding="utf-8")
    (context.run_root / "orchestration_report.json").write_text(
        json.dumps({"status": "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_FAILED"}),
        encoding="utf-8",
    )

    with pytest.raises(orchestration.OrchestrationError, match="exact S100-external-blocked"):
        orchestration._resume_preconditions(context, context.run_root / "orchestration_logs")


def test_initial_s100_availability_allows_missing_without_trust_but_rejects_final(
    tmp_path: Path,
) -> None:
    context = _context(tmp_path)
    final = tmp_path / "artifacts/formal_s100_live_acceptance.json"
    assert orchestration._s100_final_available(context, final) is False
    final.parent.mkdir(parents=True)
    final.write_text("{}\n", encoding="utf-8")
    with pytest.raises(orchestration.OrchestrationError, match="operator-trusted"):
        orchestration._s100_final_available(context, final)
    context.accept_operator_trusted_s100 = True
    assert orchestration._s100_final_available(context, final) is True


def test_resume_s100_updates_only_the_three_terminal_rows_without_gazebo_runners(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    context.accept_operator_trusted_s100 = True
    context.run_root.mkdir(parents=True)
    _install_fake_resume_lock(monkeypatch)
    report = _blocked_resume_report(context)
    s100_path = tmp_path / "artifacts/formal_s100_live_acceptance.json"
    s100_path.parent.mkdir(parents=True)
    s100_path.write_text("{}\n", encoding="utf-8")
    calls: list[str] = []
    monkeypatch.setattr(orchestration, "_gate_path", lambda *unused_args: s100_path)

    monkeypatch.setattr(
        orchestration,
        "_resume_preconditions",
        lambda unused_context, unused_logs: (report, {}, 1, _verified_closure_identity()),
    )
    monkeypatch.setattr(
        orchestration,
        "_verify_runtime_closure",
        lambda unused_context, phase: {"phase": phase, "passed": True},
    )
    monkeypatch.setattr(
        orchestration,
        "_validate_s100_external_evidence",
        lambda *unused_args: {"gate": orchestration.EXTERNAL_GATE, "sha256": "a" * 64},
    )
    monkeypatch.setattr(
        orchestration,
        "_run_session_finalize",
        lambda *unused_args: {"status": "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"},
    )
    monkeypatch.setattr(
        orchestration,
        "_run_functional_aggregate",
        lambda *unused_args, **unused_kwargs: {
            "status": "FORMAL_FUNCTIONAL_ACCEPTANCE_COMPLETE",
            "passed_position_count": 38,
            "pending_position_count": 0,
            "complete": True,
        },
    )
    monkeypatch.setattr(
        orchestration,
        "_step_command",
        lambda *unused_args, **unused_kwargs: pytest.fail("resume must not build a local step command"),
    )
    monkeypatch.setattr(
        orchestration,
        "_run_process",
        lambda *unused_args, **unused_kwargs: pytest.fail("resume must not run Gazebo/local runners"),
    )

    result, code = orchestration.resume_s100(context)
    assert code == 0
    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_COMPLETE"
    assert len(result["steps"]) == 31
    assert [row["id"] for row in result["steps"]] == [
        spec.step_id for spec in orchestration.STEP_SPECS
    ]
    assert all("previous_status" not in row for row in result["steps"][:-3])
    assert [row["previous_status"] for row in result["steps"][-3:]] == [
        "BLOCKED_EXTERNAL_HARD_GATE",
        "PASSED",
        "PASSED_WITH_EXTERNAL_S100_BLOCK",
    ]
    history = result["resume_history"]
    assert len(history) == 1
    assert history[0]["attempt"] == 1
    assert set(history[0]["terminal_rows"]) == {
        "s100_live", "finalize_session", "functional_aggregate"
    }
    for row in history[0]["terminal_rows"].values():
        assert row["before_sha256"] == orchestration._json_digest(row["before"])
        assert row["after_sha256"] == orchestration._json_digest(row["after"])
    assert result["operator_trusted_s100_acknowledged"] is True
    assert result["s100_evidence_trust_boundary"] == orchestration.S100_EVIDENCE_TRUST_BOUNDARY
    assert calls == []


def test_resume_s100_missing_artifact_returns_four_without_finalizing_session(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    context.accept_operator_trusted_s100 = True
    context.run_root.mkdir(parents=True)
    _install_fake_resume_lock(monkeypatch)
    report = _blocked_resume_report(context)
    finalized = []
    monkeypatch.setattr(
        orchestration,
        "_gate_path",
        lambda *unused_args: tmp_path / "artifacts/formal_s100_live_acceptance.json",
    )
    monkeypatch.setattr(
        orchestration,
        "_resume_preconditions",
        lambda unused_context, unused_logs: (report, {}, 1, _verified_closure_identity()),
    )
    monkeypatch.setattr(
        orchestration,
        "_run_session_finalize",
        lambda *unused_args: finalized.append(True),
    )

    result, code = orchestration.resume_s100(context)
    assert code == 4
    assert result is report
    assert finalized == []


def test_resume_s100_rejects_stale_or_forged_external_evidence_before_finalize(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    context.accept_operator_trusted_s100 = True
    context.run_root.mkdir(parents=True)
    _install_fake_resume_lock(monkeypatch)
    report = _blocked_resume_report(context)
    s100_path = tmp_path / "artifacts/formal_s100_live_acceptance.json"
    s100_path.parent.mkdir(parents=True)
    s100_path.write_text("{}\n", encoding="utf-8")
    finalized = []
    monkeypatch.setattr(orchestration, "_gate_path", lambda *unused_args: s100_path)
    monkeypatch.setattr(
        orchestration,
        "_resume_preconditions",
        lambda unused_context, unused_logs: (report, {}, 123, _verified_closure_identity()),
    )
    monkeypatch.setattr(
        orchestration,
        "_verify_runtime_closure",
        lambda unused_context, phase: {"phase": phase, "passed": True},
    )
    monkeypatch.setattr(
        orchestration,
        "_validate_s100_external_evidence",
        lambda *unused_args: (_ for _ in ()).throw(
            orchestration.OrchestrationError("S100 final artifact predates the acceptance session")
        ),
    )
    monkeypatch.setattr(
        orchestration,
        "_run_session_finalize",
        lambda *unused_args: finalized.append(True),
    )

    result, code = orchestration.resume_s100(context)
    assert code == 3
    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_S100_RESUME_REFUSED"
    assert "predates" in result["error"]
    assert finalized == []


def test_resume_s100_recovers_phase_two_after_prior_finalize_then_aggregate_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    context.accept_operator_trusted_s100 = True
    context.run_root.mkdir(parents=True)
    _install_fake_resume_lock(monkeypatch)
    report = _blocked_resume_report(context)
    completed_session = {
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE",
        "failures": {},
        "evidence": {f"gate_{index}": {} for index in range(25)},
    }
    s100_path = tmp_path / "artifacts/formal_s100_live_acceptance.json"
    s100_path.parent.mkdir(parents=True)
    s100_path.write_text("{}\n", encoding="utf-8")
    local_results = {f"gate_{index}": {"gate": f"gate_{index}"} for index in range(24)}
    finalize_calls = []
    complete_verifications = []
    aggregate_attempts = []

    monkeypatch.setattr(
        orchestration,
        "_resume_preconditions",
        lambda unused_context, unused_logs: (
            report,
            completed_session,
            1,
            _verified_closure_identity(),
        ),
    )
    monkeypatch.setattr(orchestration, "_gate_path", lambda *unused_args: s100_path)
    monkeypatch.setattr(
        orchestration,
        "_verify_runtime_closure",
        lambda unused_context, phase: {"phase": phase, "passed": True},
    )
    monkeypatch.setattr(
        orchestration,
        "_validate_s100_external_evidence",
        lambda *unused_args: {"gate": orchestration.EXTERNAL_GATE, "sha256": "a" * 64},
    )
    monkeypatch.setattr(
        orchestration,
        "_revalidate_local_gate_digests",
        lambda *unused_args: local_results,
    )
    monkeypatch.setattr(
        orchestration,
        "_verify_complete_session_evidence",
        lambda unused_context, unused_session, unused_started, gate_results: complete_verifications.append(gate_results),
    )
    monkeypatch.setattr(
        orchestration,
        "_run_session_finalize",
        lambda *unused_args: finalize_calls.append(True),
    )

    def aggregate(*unused_args, **unused_kwargs):
        aggregate_attempts.append(True)
        if len(aggregate_attempts) == 1:
            raise orchestration.OrchestrationError("simulated aggregate write failure")
        return {
            "status": "FORMAL_FUNCTIONAL_ACCEPTANCE_COMPLETE",
            "passed_position_count": 38,
            "pending_position_count": 0,
            "complete": True,
        }

    monkeypatch.setattr(orchestration, "_run_functional_aggregate", aggregate)

    first, first_code = orchestration.resume_s100(context)
    assert first_code == 3
    assert first["status"] == "FORMAL_FINAL_ACCEPTANCE_S100_RESUME_REFUSED"
    assert report["status"] == "FORMAL_FINAL_ACCEPTANCE_LOCAL_GATES_PASSED_S100_EXTERNAL_BLOCKED"

    second, second_code = orchestration.resume_s100(context)
    assert second_code == 0
    assert second["status"] == "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_COMPLETE"
    assert finalize_calls == []
    assert len(aggregate_attempts) == 2
    assert len(complete_verifications) == 2
    assert all(len(gate_results) == 25 for gate_results in complete_verifications)
    assert len(second["steps"]) == 31


def test_run_root_must_be_fresh_and_inside_the_formal_run_namespace(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    assert orchestration._validate_run_root(context, require_exists=False) == context.run_root

    context.run_root.mkdir(parents=True)
    with pytest.raises(orchestration.OrchestrationError, match="fresh nonexistent"):
        orchestration._validate_run_root(context, require_exists=False)
    assert orchestration._validate_run_root(context, require_exists=True) == context.run_root

    escaped = tmp_path / "outside_formal_run_root"
    context.requested_run_root = escaped
    context.run_root = escaped.resolve()
    with pytest.raises(orchestration.OrchestrationError, match="must be inside"):
        orchestration._validate_run_root(context, require_exists=False)

    monkeypatch.setattr(
        orchestration,
        "preflight",
        lambda unused: pytest.fail("out-of-namespace run_root must be refused before preflight"),
    )
    report, code = orchestration.execute(context)
    assert code == 2
    assert report["status"] == "FORMAL_FINAL_ACCEPTANCE_EXECUTION_REFUSED_RUN_ROOT"


def test_resume_preconditions_reject_local_digest_drift(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    context.accept_operator_trusted_s100 = True
    context.run_root.mkdir(parents=True)
    context.snapshot.parent.mkdir(parents=True)
    context.snapshot.write_text(
        json.dumps(
            {
                "source_inventory_sha256": "a" * 64,
                "outputs": {
                    "reports/engineering/formal_competition_vehicle.urdf": {
                        "sha256": "b" * 64
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    identity = orchestration._snapshot_identity(context)
    resume_closure = {
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "manifest": "/tmp/frozen-closure.json",
        "manifest_sha256": "c" * 64,
        "closure_sha256": "d" * 64,
        "symbolic_link_count": 0,
    }
    session = {
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING",
        "failures": {orchestration.EXTERNAL_GATE: "missing"},
        "started_epoch_ns": 1,
        "snapshot": identity,
        "runtime_closure_binding": {
            **resume_closure,
            "runtime_install_root": str(context.overlay.resolve()),
        },
    }
    context.session.parent.mkdir(parents=True)
    context.session.write_text(json.dumps(session), encoding="utf-8")
    report = _blocked_resume_report(context)
    report["snapshot_identity"] = identity
    (context.run_root / "orchestration_report.json").write_text(
        json.dumps(report), encoding="utf-8"
    )
    monkeypatch.setattr(orchestration, "_snapshot_check", lambda *unused_args: None)
    monkeypatch.setattr(
        orchestration,
        "_verify_runtime_closure",
        lambda *unused_args: resume_closure,
    )
    monkeypatch.setattr(
        orchestration,
        "_revalidate_local_gate_digests",
        lambda *unused_args: (_ for _ in ()).throw(
            orchestration.OrchestrationError("local gate artifact or digest drifted: water_recovery")
        ),
    )
    with pytest.raises(orchestration.OrchestrationError, match="local gate artifact or digest drifted"):
        orchestration._resume_preconditions(context, context.run_root / "orchestration_logs")


def test_resume_preconditions_reject_wrong_prior_failures(tmp_path: Path) -> None:
    context = _context(tmp_path)
    context.accept_operator_trusted_s100 = True
    context.run_root.mkdir(parents=True)
    context.snapshot.parent.mkdir(parents=True)
    context.snapshot.write_text(
        json.dumps(
            {
                "source_inventory_sha256": "a" * 64,
                "outputs": {
                    "reports/engineering/formal_competition_vehicle.urdf": {
                        "sha256": "b" * 64
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    report = _blocked_resume_report(context)
    (context.run_root / "orchestration_report.json").write_text(
        json.dumps(report), encoding="utf-8"
    )
    context.session.parent.mkdir(parents=True)
    context.session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING",
                "failures": {"water_recovery": "missing"},
                "started_epoch_ns": 1,
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(orchestration.OrchestrationError, match="exactly the missing S100"):
        orchestration._resume_preconditions(context, context.run_root / "orchestration_logs")


def test_resume_preconditions_require_explicit_operator_trust(tmp_path: Path) -> None:
    context = _context(tmp_path)
    with pytest.raises(orchestration.OrchestrationError, match="operator-trusted"):
        orchestration._resume_preconditions(context, tmp_path / "logs")


def test_s100_final_rejects_absolute_raw_handoff_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    context = _context(tmp_path)
    final = tmp_path / "artifacts/formal_s100_live_acceptance.json"
    final.parent.mkdir(parents=True)
    final.write_text(
        json.dumps(
            {
                "raw_evidence": {
                    "path": str(tmp_path / "outside-raw.json"),
                    "sha256": "a" * 64,
                }
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(orchestration, "_gate_path", lambda *unused_args: final)
    with pytest.raises(orchestration.OrchestrationError, match="repository-root-relative"):
        orchestration._validate_s100_external_evidence(
            context,
            0,
            context.run_root / "revalidation.json",
            context.run_root / "revalidation.log",
        )


def test_cli_requires_explicit_mode_and_runtime_inputs() -> None:
    parser = orchestration.build_parser()
    args = parser.parse_args(["--static-audit"])
    assert args.static_audit is True
    assert args.preflight is False
    assert args.execute is False
    assert args.resume_s100 is False
    with pytest.raises(SystemExit):
        parser.parse_args(["--execute", "--resume-s100"])


def test_context_defaults_to_runtime_local_unified_closure_manifest() -> None:
    context = _context()
    assert context.runtime_closure_manifest == (
        context.runtime_ws / "final_runtime_closure_manifest.json"
    )


def test_windows_memory_query_failure_refuses_launch_before_process_start(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    calls: list[list[str]] = []

    def failed_probe(argv, **unused_kwargs):
        calls.append(list(argv))
        return subprocess.CompletedProcess(argv, 125)

    monkeypatch.setattr(orchestration.subprocess, "run", failed_probe)
    log_path = tmp_path / "gazebo_step.log"
    with pytest.raises(orchestration.OrchestrationError, match="failed closed rc=125"):
        orchestration._run_windows_memory_preflight({}, log_path)
    assert len(calls) == 1
    assert "--check-start" in calls[0]
    assert log_path.with_name("gazebo_step.windows_memory_preflight.log").is_file()


def test_windows_memory_threshold_breach_is_classified_as_memory_limit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        orchestration.subprocess,
        "run",
        lambda argv, **unused_kwargs: subprocess.CompletedProcess(argv, 86),
    )
    with pytest.raises(orchestration.MemoryLimitError, match="refused launch rc=86"):
        orchestration._run_windows_memory_preflight({}, tmp_path / "gazebo_step.log")


def test_child_memory_breach_exit_is_preserved_after_watchdog_exits_cleanly(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class FakeCommand:
        pid = 4321

        def poll(self):
            return orchestration.MEMORY_BREACH_EXIT_CODE

        def wait(self, timeout=None):
            return orchestration.MEMORY_BREACH_EXIT_CODE

    class FakeWatchdog:
        def __init__(self) -> None:
            self.signalled = False

        def poll(self):
            return None

        def send_signal(self, unused_signal) -> None:
            self.signalled = True

        def wait(self, timeout=None):
            assert self.signalled
            return 0

        def kill(self) -> None:
            raise AssertionError("clean watchdog shutdown must not require kill")

    command = FakeCommand()
    watchdog = FakeWatchdog()
    popen_results = iter((command, watchdog))
    monkeypatch.setattr(orchestration.os, "name", "posix")
    monkeypatch.setattr(orchestration, "_run_windows_memory_preflight", lambda *args: None)
    monkeypatch.setattr(
        orchestration.subprocess,
        "Popen",
        lambda *args, **kwargs: next(popen_results),
    )

    with pytest.raises(
        orchestration.MemoryLimitError,
        match="child command reported memory limit rc=86",
    ):
        orchestration._run_process(
            ["fake-formal-step"],
            {},
            tmp_path / "gazebo_step.log",
            memory_watchdog=True,
        )


@pytest.mark.parametrize(
    ("key", "probe"),
    (
        ("FORMAL_MEMORY_WATCHDOG_ENABLED", orchestration._memory_watchdog_enabled),
        (
            "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED",
            orchestration._windows_memory_guard_enabled,
        ),
    ),
)
def test_final_orchestrator_refuses_disabled_memory_protection(key, probe) -> None:
    with pytest.raises(orchestration.OrchestrationError, match="cannot be disabled"):
        probe({key: "0"})


def test_twenty_cube_step_receives_the_unified_closure_manifest() -> None:
    context = _context()
    command, environment = orchestration._step_command("twenty_cubes", context)
    assert command[-1].endswith("run_formal_20_cube_grasp_acceptance.sh")
    assert environment["FORMAL_MANIPULATION_RUNTIME_WS"] == str(context.overlay)
    assert environment["FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST"] == str(
        context.runtime_closure_manifest
    )


@pytest.mark.parametrize(
    ("step_id", "output_gate", "binding_key"),
    (
        (
            "chassis",
            "a300_drivetrain_runtime",
            "FORMAL_VEHICLE_MOBILITY_RUNTIME_BINDING",
        ),
        (
            "cleaning_motors",
            "cleaning_motor_runtime",
            "FORMAL_CLEANING_MOTOR_RUNTIME_BINDING",
        ),
        (
            "cleaning_positions",
            "cleaning_actuators",
            "FORMAL_FUNCTION_POSITIONS_RUNTIME_BINDING",
        ),
        (
            "manipulator",
            "manipulator_trajectory",
            "FORMAL_MANIPULATOR_TRAJECTORY_RUNTIME_BINDING",
        ),
        (
            "physical_grasp",
            "physical_grasp_and_bin",
            "FORMAL_GRASP_EXECUTOR_RUNTIME_BINDING",
        ),
        (
            "dynamic_obstacle",
            "dynamic_obstacle_avoidance",
            "FORMAL_DYNAMIC_RUNTIME_BINDING",
        ),
        (
            "single_episode",
            "end_to_end_cleaning_mission",
            "FORMAL_E2E_RUNTIME_BINDING",
        ),
    ),
)
def test_direct_physical_steps_route_fresh_runtime_bindings(
    step_id: str, output_gate: str, binding_key: str
) -> None:
    context = _context()
    _, environment = orchestration._step_command(step_id, context)
    contract = orchestration._read_contract(context.root)
    output = context.root / contract["evidence_gates"][output_gate]["path"]

    assert environment[binding_key] == str(
        output.with_name(output.name + ".runtime_binding.json")
    )
    assert environment["FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST"] == str(
        context.runtime_closure_manifest
    )


def test_water_step_generates_manifest_then_typed_diagnostic_then_all_scenarios() -> None:
    context = _context()
    command, environment = orchestration._step_command(
        "water_recovery",
        context,
        runtime_closure=_verified_closure_identity(),
    )
    assert command[0] == "__sequence__"
    sequence = json.loads(command[1])
    assert len(sequence) == 3
    assert sequence[0][1].endswith(
        "generate_formal_water_critical_source_manifest.py"
    )
    assert sequence[1][-1].endswith(
        "run_formal_typed_cleaning_motor_diagnostic.sh"
    )
    assert sequence[2][-5].endswith("run_formal_water_recovery_runtime.sh")
    assert sequence[2][-4:] == [
        "--scenario",
        "all",
        "--output-dir",
        str(context.run_root / "water_recovery/scenarios"),
    ]
    assert environment["FORMAL_VEHICLE_RUNTIME_WS"] == str(context.overlay)
    rendered = " ".join(value for row in sequence for value in row)
    assert " touch " not in f" {rendered} "
    assert " cp " not in f" {rendered} "

    water_root = context.run_root / "water_recovery"
    expected = {
        "FORMAL_WATER_OUTPUT_DIR": str(water_root / "scenarios"),
        "FORMAL_WATER_TYPED_OUTPUT_DIR": str(water_root / "typed_transport"),
        "FORMAL_WATER_TYPED_RUNTIME_WS": str(context.runtime_ws),
        "FORMAL_WATER_TYPED_DURATION_S": "10",
        "FORMAL_WATER_TYPED_DIAG_JSON": str(
            water_root / "typed_transport/typed_diag.json"
        ),
        "FORMAL_WATER_TYPED_RAW_TRACE": str(
            water_root / "typed_transport/raw_frames.jsonl"
        ),
        "FORMAL_WATER_TYPED_RUNNER": str(
            context.root / "scripts/run_formal_typed_cleaning_motor_diagnostic.sh"
        ),
        "FORMAL_WATER_TYPED_COLLECTOR": str(
            context.root / "scripts/collect_formal_typed_cleaning_motor_diagnostic.py"
        ),
        "FORMAL_WATER_CRITICAL_SOURCE_MANIFEST": str(
            water_root / "critical_source_manifest.json"
        ),
        "FORMAL_WATER_TYPED_SUBCLOSURE_SHA256": "e" * 64,
    }
    for name, value in expected.items():
        assert environment[name] == value
    assert {
        name
        for name in environment
        if name.startswith(("FORMAL_WATER_TYPED", "FORMAL_WATER_CRITICAL"))
    } == {
        name
        for name in expected
        if name.startswith(("FORMAL_WATER_TYPED", "FORMAL_WATER_CRITICAL"))
    }
    assert sequence[0][-1] == expected["FORMAL_WATER_CRITICAL_SOURCE_MANIFEST"]


def test_water_step_refuses_missing_or_invalid_typed_subclosure() -> None:
    context = _context()
    with pytest.raises(orchestration.OrchestrationError, match="verified runtime closure"):
        orchestration._step_command("water_recovery", context)
    with pytest.raises(orchestration.OrchestrationError, match="subclosure digest"):
        orchestration._step_command(
            "water_recovery",
            context,
            runtime_closure={"typed_cleaning_telemetry_source_sha256": "not-a-digest"},
        )


def test_each_local_gate_is_routed_to_its_contract_output_before_execution(
    tmp_path: Path,
) -> None:
    context = _context(tmp_path)
    context.integrated_build_manifest.parent.mkdir(parents=True, exist_ok=True)
    context.integrated_build_manifest.write_text(
        json.dumps({"build_started_epoch_ns": 123456789}), encoding="utf-8"
    )
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True, exist_ok=True)
    contract_path.write_bytes(orchestration.CONTRACT.read_bytes())
    scenario_source = (
        orchestration.ROOT
        / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml"
    )
    scenario_target = tmp_path / scenario_source.relative_to(orchestration.ROOT)
    scenario_target.parent.mkdir(parents=True, exist_ok=True)
    scenario_target.write_bytes(scenario_source.read_bytes())
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    output_environment_keys = {
        "visual": ("FORMAL_VEHICLE_VISUAL_PUBLISH_ROOT",),
        "sensor": ("FORMAL_SENSOR_RUNTIME_OUTPUT", "FORMAL_SENSOR_FOV_OUTPUT"),
        "chassis": ("FORMAL_VEHICLE_MOBILITY_OUTPUT",),
        "safety_power_lighting": ("FORMAL_AUXILIARY_OUTPUT",),
        "cleaning_positions": ("FORMAL_FUNCTION_POSITIONS_OUTPUT",),
        "cleaning_motors": ("FORMAL_CLEANING_MOTOR_OUTPUT",),
        "ground_dirt": ("FORMAL_DIRT_OUTPUT_DIR",),
        "water_recovery": ("FORMAL_WATER_FINAL_ARTIFACT",),
        "service_door": ("FORMAL_SERVICE_DOOR_RUNTIME_OUTPUT",),
        "manipulator": ("FORMAL_MANIPULATOR_TRAJECTORY_OUTPUT",),
        "physical_grasp": ("FORMAL_GRASP_EXECUTOR_OUTPUT",),
        "twenty_cubes": ("FORMAL_20_CUBE_OUTPUT",),
        "integrated_basic_physics": ("INTEGRATED_ACCEPTANCE_CONTRACT_SUMMARY",),
        "saved_map_reuse": ("FORMAL_MAP_LIFECYCLE_OUTPUT",),
        "perception": ("FORMAL_PERCEPTION_FINAL_ARTIFACT",),
        "dynamic_obstacle": ("FORMAL_DYNAMIC_OUTPUT",),
    }
    for step in orchestration.STEP_SPECS:
        if not step.produces_gates or step.step_id == "s100_live":
            continue
        command, environment = orchestration._step_command(
            step.step_id,
            context,
            runtime_closure=(
                _verified_closure_identity()
                if step.step_id == "water_recovery"
                else None
            ),
        )
        routed = [Path(environment[key]) for key in output_environment_keys.get(step.step_id, ())]
        command_values = (
            [value for row in json.loads(command[1]) for value in row]
            if command[:1] == ["__sequence__"]
            else list(command)
        )
        for gate in step.produces_gates:
            expected = context.root / contract["evidence_gates"][gate]["path"]
            assert any(str(expected) in value for value in command_values) or any(
                expected == root or root in expected.parents for root in routed
            ), f"{step.step_id} does not route {gate} to {expected}"


def test_static_audit_rejects_crosswalk_drift_before_long_runtime(tmp_path: Path) -> None:
    contract = yaml.safe_load(orchestration.CONTRACT.read_text(encoding="utf-8"))
    contract["functional_positions"].pop("front_contact_safety")
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True)
    contract_path.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    register_source = orchestration.ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
    register_target = tmp_path / register_source.relative_to(orchestration.ROOT)
    register_target.parent.mkdir(parents=True, exist_ok=True)
    register_target.write_bytes(register_source.read_bytes())
    scripts_target = tmp_path / "scripts"
    scripts_target.mkdir(exist_ok=True)
    for step in orchestration.STEP_SPECS:
        if step.runner:
            (scripts_target / step.runner).write_text(
                "# run_formal_runtime_isolation.sh\n", encoding="utf-8"
            )
    report = orchestration.static_audit(tmp_path)
    assert report["passed"] is False
    assert any(
        row.startswith("functional_position_crosswalk_mismatch:")
        for row in report["failures"]
    )


def test_static_audit_rejects_runtime_binding_gate_set_drift(tmp_path: Path) -> None:
    contract = yaml.safe_load(orchestration.CONTRACT.read_text(encoding="utf-8"))
    contract["evidence_gates"]["sensor_runtime"].pop("runtime_binding")
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True)
    contract_path.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    register_source = orchestration.ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
    register_target = tmp_path / register_source.relative_to(orchestration.ROOT)
    register_target.parent.mkdir(parents=True, exist_ok=True)
    register_target.write_bytes(register_source.read_bytes())
    scripts_target = tmp_path / "scripts"
    scripts_target.mkdir(exist_ok=True)
    for step in orchestration.STEP_SPECS:
        if step.runner:
            (scripts_target / step.runner).write_text(
                "# run_formal_runtime_isolation.sh\n", encoding="utf-8"
            )

    report = orchestration.static_audit(tmp_path)

    assert report["passed"] is False
    assert "gate_has_invalid_runtime_binding_contract:sensor_runtime" in report["failures"]
    assert any(
        row.startswith("runtime_binding_gate_set_mismatch:missing=['sensor_runtime']:")
        for row in report["failures"]
    )


def test_runtime_closure_verifier_binds_the_full_context(monkeypatch) -> None:
    context = _context()
    calls = []

    def fake_verify(manifest, root, runtime_ws, models, onnx):
        calls.append((manifest, root, runtime_ws, models, onnx))
        return {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
            "passed": True,
            "closure_sha256": "a" * 64,
            "runtime_package_count": 16,
            "side_brush_installed_xacro": "/tmp/frozen/install/vehicle.xacro",
            "side_brush_installed_xacro_sha256": "b" * 64,
            "side_brush_expanded_sdf_sha256": "c" * 64,
            "symbolic_link_count": 0,
            "windows_cold_start_evidence_bound": True,
            "windows_cold_start_evidence_sha256": "d" * 64,
        }

    monkeypatch.setattr(orchestration, "verify_runtime_closure_manifest", fake_verify)
    result = orchestration._verify_runtime_closure(context, "before:test")
    assert calls == [
        (
            context.runtime_closure_manifest,
            context.root,
            context.runtime_ws,
            context.perception_artifacts,
            context.onnx_pythonpath,
        )
    ]
    assert result["phase"] == "before:test"
    assert result["runtime_package_count"] == 16


def test_water_gate_rehashes_normal_and_full_surface_evidence_against_closure(
    tmp_path: Path,
) -> None:
    context = _context(tmp_path)
    contract_path = tmp_path / orchestration.CONTRACT.relative_to(orchestration.ROOT)
    contract_path.parent.mkdir(parents=True)
    contract_path.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "water_recovery": {
                        "path": "artifacts/formal_water_recovery_acceptance.json",
                        "success_statuses": ["FORMAL_WATER_RECOVERY_ACCEPTANCE_PASSED"],
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    context.snapshot.parent.mkdir(parents=True, exist_ok=True)
    context.snapshot.write_text(
        json.dumps({"source_inventory_sha256": "", "outputs": {}}),
        encoding="utf-8",
    )
    installed_xacro = context.overlay / "share/vehicle.xacro"
    installed_xacro.parent.mkdir(parents=True)
    installed_xacro.write_text("<robot/>\n", encoding="utf-8")
    xacro_hash = orchestration._sha256(installed_xacro)
    expanded_hash = "d" * 64
    evidence_dir = context.run_root / "water_recovery/scenarios"
    evidence_dir.mkdir(parents=True)
    raw_reports = {}
    for scenario, scenario_name in (
        ("normal", "normal_recovery"),
        ("full", "full_tank_fail_closed"),
    ):
        raw = evidence_dir / f"water_{scenario}.json"
        raw.write_text(
            json.dumps({"scenario": scenario_name, "passed": True}),
            encoding="utf-8",
        )
        raw_reports[scenario] = raw
    surfaces = {}
    for scenario in ("normal", "full"):
        surface = evidence_dir / f"water_{scenario}_side_brush_sdf_surface.json"
        surface.write_text(
            json.dumps(
                {
                    "status": "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED",
                    "source": {
                        "mode": "xacro_to_gz_sdf",
                        "path": str(installed_xacro.resolve()),
                        "sha256": xacro_hash,
                    },
                    "expanded_sdf_sha256": expanded_hash,
                }
            ),
            encoding="utf-8",
        )
        surfaces[scenario] = surface
    typed_dir = context.run_root / "water_recovery/typed_transport"
    typed_dir.mkdir(parents=True)
    typed_diag = typed_dir / "typed_diag.json"
    typed_launch = typed_dir / "typed_launch.log"
    typed_launch_audit = typed_dir / "typed_launch_audit.json"
    typed_gz_info = typed_dir / "typed_gz_info.txt"
    typed_ros_info = typed_dir / "typed_ros_info.txt"
    typed_launch.write_text("healthy\n", encoding="utf-8")
    typed_launch_audit.write_text(json.dumps({"passed": True}), encoding="utf-8")
    typed_gz_info.write_text("gz.msgs.Double_V\n", encoding="utf-8")
    typed_ros_info.write_text(
        "std_msgs/msg/Float64MultiArray\nPublisher count: 1\n", encoding="utf-8"
    )
    typed_diag.write_text(
        json.dumps(
            {
                "status": "FORMAL_TYPED_CLEANING_MOTOR_DIAG_PASSED",
                "passed": True,
                "checks": {
                    name: True for name in orchestration.TYPED_DIAG_REQUIRED_CHECKS
                },
                "metrics": {"raw_trace_frame_count": 2},
                "transport_audit": {
                    "passed": True,
                    "checks": {
                        name: True
                        for name in orchestration.TYPED_TRANSPORT_AUDIT_REQUIRED_CHECKS
                    },
                    "node_shared_publish_errors": [],
                    "topic_tagged_publish_failures": [],
                    "launch_log": str(typed_launch),
                    "launch_log_sha256": orchestration._sha256(typed_launch),
                    "launch_audit_json": str(typed_launch_audit),
                    "launch_audit_sha256": orchestration._sha256(typed_launch_audit),
                    "gazebo_topic_info": str(typed_gz_info),
                    "gazebo_topic_info_sha256": orchestration._sha256(typed_gz_info),
                    "ros_topic_info": str(typed_ros_info),
                    "ros_topic_info_sha256": orchestration._sha256(typed_ros_info),
                },
            }
        ),
        encoding="utf-8",
    )
    raw_trace = typed_dir / "raw_frames.jsonl"
    raw_trace.write_text("{\"frame\": 1}\n{\"frame\": 2}\n", encoding="utf-8")
    typed_runner = evidence_dir / "typed_run.sh"
    typed_runner.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    typed_collector = evidence_dir / "collect_typed.py"
    typed_collector.write_text("# typed collector\n", encoding="utf-8")
    source_manifest = context.run_root / "water_recovery/critical_source_manifest.json"
    source_manifest.write_text("{}\n", encoding="utf-8")
    typed_source_digest = "e" * 64
    final = tmp_path / "artifacts/formal_water_recovery_acceptance.json"
    final.parent.mkdir(parents=True, exist_ok=True)
    final.write_text(
        json.dumps(
            {
                "status": "FORMAL_WATER_RECOVERY_ACCEPTANCE_PASSED",
                "checks": {"side_brush_expanded_sdf_surface_valid": True},
                "evidence": {
                    "normal_json": str(raw_reports["normal"]),
                    "normal_sha256": orchestration._sha256(raw_reports["normal"]),
                    "full_json": str(raw_reports["full"]),
                    "full_sha256": orchestration._sha256(raw_reports["full"]),
                    "normal_side_brush_surface_json": str(surfaces["normal"]),
                    "normal_side_brush_surface_sha256": orchestration._sha256(
                        surfaces["normal"]
                    ),
                    "full_side_brush_surface_json": str(surfaces["full"]),
                    "full_side_brush_surface_sha256": orchestration._sha256(
                        surfaces["full"]
                    ),
                    "expanded_side_brush_sdf_sha256": expanded_hash,
                    "typed_transport": {
                        "contract": dict(orchestration.TYPED_WATER_TRANSPORT_CONTRACT),
                        "typed_diag_json": str(typed_diag),
                        "typed_diag_sha256": orchestration._sha256(typed_diag),
                        "raw_trace_jsonl": str(raw_trace),
                        "raw_trace_sha256": orchestration._sha256(raw_trace),
                        "runner_script": str(typed_runner),
                        "runner_sha256": orchestration._sha256(typed_runner),
                        "collector_script": str(typed_collector),
                        "collector_sha256": orchestration._sha256(typed_collector),
                        "critical_source_manifest_json": str(source_manifest),
                        "critical_source_manifest_sha256": orchestration._sha256(
                            source_manifest
                        ),
                        "typed_cleaning_telemetry_source_sha256": typed_source_digest,
                    },
                },
            }
        ),
        encoding="utf-8",
    )
    closure_identity = {
        "side_brush_installed_xacro": str(installed_xacro.resolve()),
        "side_brush_installed_xacro_sha256": xacro_hash,
        "side_brush_expanded_sdf_sha256": expanded_hash,
        "typed_cleaning_telemetry_source_sha256": typed_source_digest,
    }
    result = orchestration._validate_gate(
        context, "water_recovery", 0, runtime_closure=closure_identity
    )
    assert result["normal_side_brush_surface_sha256"] == orchestration._sha256(
        surfaces["normal"]
    )
    assert result["full_side_brush_surface_sha256"] == orchestration._sha256(
        surfaces["full"]
    )
    assert result["typed_diag_sha256"] == orchestration._sha256(typed_diag)
    assert result["raw_trace_sha256"] == orchestration._sha256(raw_trace)
    assert result["typed_cleaning_telemetry_source_sha256"] == typed_source_digest

    final_payload = json.loads(final.read_text(encoding="utf-8"))
    final_payload["evidence"]["typed_transport"][
        "typed_cleaning_telemetry_source_sha256"
    ] = "f" * 64
    final.write_text(json.dumps(final_payload), encoding="utf-8")
    with pytest.raises(orchestration.OrchestrationError, match="source digest"):
        orchestration._validate_gate(
            context, "water_recovery", 0, runtime_closure=closure_identity
        )
    final_payload["evidence"]["typed_transport"][
        "typed_cleaning_telemetry_source_sha256"
    ] = typed_source_digest
    final.write_text(json.dumps(final_payload), encoding="utf-8")

    surfaces["full"].write_text("{}\n", encoding="utf-8")
    with pytest.raises(orchestration.OrchestrationError, match="hash/freshness mismatch"):
        orchestration._validate_gate(
            context, "water_recovery", 0, runtime_closure=closure_identity
        )


def test_every_direct_internal_ros_runtime_package_is_in_unified_closure() -> None:
    direct_packages = set()
    for step in orchestration.STEP_SPECS:
        if step.runner and step.runner.endswith(".sh"):
            source = (ROOT / "scripts" / step.runner).read_text(encoding="utf-8")
            direct_packages.update(
                re.findall(r"ros2\s+(?:run|launch)\s+(sanitation_[a-z0-9_]+)", source)
            )
    for step_id in ("episode_materialization", "rl_policy"):
        command, _ = orchestration._step_command(step_id, _context())
        direct_packages.update(
            re.findall(
                r"ros2\s+(?:run|launch)\s+(sanitation_[a-z0-9_]+)",
                " ".join(command),
            )
        )
    assert direct_packages <= set(orchestration.FINAL_RUNTIME_PACKAGES)
