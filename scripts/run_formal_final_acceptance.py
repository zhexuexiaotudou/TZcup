#!/usr/bin/env python3
"""Fail-closed serial orchestrator for the formal whole-vehicle acceptance.

The entry point is deliberately opt-in: ``--preflight`` never mutates the
workspace or starts Gazebo, while ``--execute`` creates one fresh session and
runs every local gate serially.  The RDK S100P / Journey 6P live-board gate is
external by definition and is never synthesized on a PC.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
import re
import secrets
import signal
import shlex
import shutil
import subprocess
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml

from formal_final_runtime_closure import (
    ClosureError,
    FINAL_RUNTIME_PACKAGES,
    verify_manifest as verify_runtime_closure_manifest,
)
from formal_preembedded_grasp_world_binding import (
    PreembeddedGraspWorldBindingError,
    validate_preembedded_grasp_world,
)

try:  # Windows runs the static audit; Linux owns the actual runtime lock.
    import fcntl
except ImportError:  # pragma: no cover - exercised by Windows CI import
    fcntl = None  # type: ignore[assignment]


ROOT = Path(__file__).resolve().parents[1]
FORMAL_RUN_ROOT_PARENT = ROOT / ".work/formal_final_acceptance"
CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml"
SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"
FUNCTIONAL_AUDIT = ROOT / "reports/engineering/formal_functional_acceptance_audit.json"
STATIC_AUDIT = ROOT / "reports/engineering/formal_final_acceptance_orchestration_audit.json"
ORCHESTRATION_REPORT = ROOT / "reports/engineering/formal_final_acceptance_orchestration_report.json"
LOCK_FILE = Path("/tmp/tzcup_formal_gazebo.lock")
MEMORY_WATCHDOG = ROOT / "scripts/formal_memory_watchdog.sh"
MEMORY_BREACH_EXIT_CODE = 86
DEFAULT_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS = 300
MIN_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS = 60
MAX_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS = 900
EXTERNAL_GATE = "s100_live_runtime"
S100_COMMITTED_AGGREGATE_PENDING = (
    "FORMAL_FINAL_ACCEPTANCE_S100_COMMITTED_AGGREGATE_PENDING"
)
S100_EVIDENCE_TRUST_BOUNDARY = (
    "Operator-trusted, tamper-evident, non-cryptographic S100 evidence chain; "
    "it is not TPM/signed remote attestation and cannot authenticate against a malicious PC operator."
)
TYPED_WATER_TRANSPORT_CONTRACT = {
    "ros_type": "std_msgs/msg/Float64MultiArray",
    "gz_type": "gz.msgs.Double_V",
    "snapshot_length": 63,
    "status_transport": "gazebo_only_diagnostic",
}
TYPED_DIAG_REQUIRED_CHECKS = {
    "all_snapshots_parse_as_63_finite_values",
    "first_frame_below_0_5_s",
    "maximum_gap_at_most_75_ms",
    "no_burst_gap_below_20_ms",
    "physics_revision_advances",
    "physics_revision_never_moves_backwards",
    "physics_revision_stagnation_below_0_75_s",
    "rate_18_to_22_hz",
    "raw_trace_contains_every_received_frame",
    "ros_status_json_has_zero_publishers",
    "steady_samples_not_physics_stale",
    "telemetry_sequence_strictly_increasing",
}
TYPED_TRANSPORT_AUDIT_REQUIRED_CHECKS = {
    "gazebo_topic_type_is_double_v",
    "launch_log_audit_passed",
    "ros_topic_type_is_float64_multi_array",
    "ros_typed_topic_has_one_publisher",
    "zero_nodeshared_publish_errors",
    "zero_topic_tagged_publish_failures",
}
FORMAL_FULL_MAP_COUNTS = {"train": 32, "validation": 8, "hidden": 12}
FORMAL_MULTIMAP_MISSIONS_PER_MAP = {
    "train": 200,
    "validation": 100,
    "hidden": 100,
}
FORMAL_MULTIMAP_TASK_COUNTS = {"train": 6400, "validation": 800, "hidden": 1200}
FORMAL_STAGE_A_TASK_COUNTS = {"train": 10000, "validation": 500, "hidden": 1000}
FORMAL_POLICY_SEEDS = [7, 17, 29, 43, 61]
FORMAL_RL_BUDGET_CONTRACT = (
    ROOT / "starter_ws/src/sanitation_active_cleaning/config/formal_rl_budget_contract.yaml"
)
RUNTIME_GATE_BINDING_CONTRACT = {
    "report_field": "runtime_gate_binding",
    "sidecar_suffix": ".runtime_binding.json",
}
RUNTIME_GATE_BINDING_GATES = {
    # Every physical runtime gate that launches against the frozen overlay must
    # carry the exact sidecar re-hashed against the active final session and
    # closure.  Keep this as the single authority for contract enforcement and
    # retained-sidecar rotation; a source hash copied into a report is weaker.
    "sensor_runtime",
    "product_visual_acceptance",
    "service_visual_acceptance",
    "integrated_basic_physics",
    "a300_drivetrain_runtime",
    "whole_vehicle_interlock",
    "auxiliary_power_lighting",
    "cleaning_actuators",
    "cleaning_motor_runtime",
    "ground_dirt_cleaning",
    "first_map_then_clean",
    "random_scene_perception",
    "dynamic_obstacle_avoidance",
    "end_to_end_cleaning_mission",
    "multi_site_product_generalization",
    "water_recovery",
    "service_door_runtime",
    "service_interface_acceptance",
    "manipulator_trajectory",
    "physical_grasp_and_bin",
    "formal_20_cube_grasp_and_dynamic_mass",
}
WINDOWS_DRY_RUN_FOUR_CHAIN_STEPS = (
    "chassis",
    "ground_dirt",
    "water_recovery",
    "physical_grasp",
)


class OrchestrationError(RuntimeError):
    """Raised when a final acceptance prerequisite or gate fails closed."""


class MemoryLimitError(OrchestrationError):
    """Raised after the dedicated watchdog terminates one exact step PGID."""


def _strict_json_equal(actual: Any, expected: Any) -> bool:
    """Compare JSON-compatible values without Python's bool/int coercion."""
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _strict_json_equal(actual[key], value) for key, value in expected.items()
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _strict_json_equal(actual_item, expected_item)
            for actual_item, expected_item in zip(actual, expected)
        )
    return actual == expected


def _finite_contract_number(value: Any) -> bool:
    return type(value) in (int, float) and (
        not isinstance(value, float) or math.isfinite(value)
    )


@dataclass(frozen=True)
class StepSpec:
    step_id: str
    mode: str
    description: str
    runner: str | None = None
    produces_gates: tuple[str, ...] = ()
    requires_outer_gazebo_lock: bool = False


STEP_SPECS: tuple[StepSpec, ...] = (
    StepSpec("freeze_snapshot", "static", "regenerate and freeze the authoritative vehicle snapshot", "generate_formal_vehicle_snapshot.py"),
    StepSpec("start_session", "static", "start one source-bound final acceptance session", "formal_acceptance_session.py"),
    StepSpec("component_register", "static", "refresh the post-session component-register evidence", "refresh_formal_component_register_evidence.py", ("component_register",)),
    StepSpec("visual", "gazebo", "capture product and service nineteen-view acceptance", "run_formal_vehicle_visual_acceptance.sh", ("product_visual_acceptance", "service_visual_acceptance")),
    StepSpec("inertia", "static", "scan inertia, CoG, collision meshes and swept volumes", "scan_formal_vehicle_inertia_and_swept_volume.py", ("inertia_cog_and_swept_volume",)),
    StepSpec("sensor", "gazebo", "validate every declared sensor stream and FOV/occlusion", "run_formal_vehicle_sensor_runtime.sh", ("sensor_fov_and_occlusion", "sensor_runtime")),
    StepSpec("chassis", "gazebo", "validate A300 forward motion and stop", "run_formal_vehicle_mobility_runtime.sh", ("a300_drivetrain_runtime",)),
    StepSpec("safety_interlock", "gazebo", "validate whole-vehicle actuator interlocks", "run_whole_vehicle_actuator_interlock_runtime.sh", ("whole_vehicle_interlock",)),
    StepSpec("safety_power_lighting", "gazebo", "validate power, E-stop and lighting", "run_formal_auxiliary_runtime.sh", ("auxiliary_power_lighting",)),
    StepSpec("cleaning_positions", "gazebo", "validate cleaning, storage and recovery mechanisms", "run_formal_function_positions_runtime.sh", ("cleaning_actuators",)),
    StepSpec("cleaning_motors", "gazebo", "validate cleaning motor dynamics and feedback", "run_formal_cleaning_actuator_motor_runtime.sh", ("cleaning_motor_runtime",)),
    StepSpec("ground_dirt", "gazebo", "validate physical ground-dirt removal", "run_formal_ground_dirt_cleaning_runtime.sh", ("ground_dirt_cleaning",)),
    StepSpec("water_recovery", "gazebo", "validate normal and near-full water recovery", "run_formal_water_recovery_runtime.sh", ("water_recovery",)),
    StepSpec("service_door", "gazebo", "validate all bodywork service-door joints", "run_formal_service_door_runtime.sh", ("service_door_runtime",)),
    StepSpec("charge_and_drain", "gazebo", "validate charge and wastewater-drain interfaces", "run_formal_service_interface_acceptance.sh", ("service_interface_acceptance",)),
    StepSpec("manipulator", "gazebo", "validate UR5e and Robotiq trajectories", "run_formal_manipulator_trajectory_runtime.sh", ("manipulator_trajectory",)),
    StepSpec("physical_grasp", "gazebo", "validate contact-gated grasp and physical bin deposit", "run_formal_grasp_executor_runtime.sh", ("physical_grasp_and_bin",)),
    StepSpec("twenty_cubes", "gazebo", "validate all twenty material cubes and dynamic bin mass", "run_formal_20_cube_grasp_acceptance.sh", ("formal_20_cube_grasp_and_dynamic_mass",)),
    StepSpec("integrated_basic_physics", "gazebo", "repeat the source-bound basic physics bundle", "run_integrated_functional_acceptance.sh", ("integrated_basic_physics",), True),
    StepSpec("rl_policy", "static", "freeze and evaluate the belief-only cross-map policy before any held-out final episode", "generate_formal_rl_multimap_report.py", ("rl_cross_map_policy",)),
    StepSpec("episode_materialization", "static", "materialize a fresh formal hidden episode"),
    StepSpec("first_map", "gazebo", "explore once and seal the first-task SLAM map", "run_formal_first_map_dynamic_prerequisite.sh"),
    StepSpec("saved_map_reuse", "gazebo", "hard-restart and clean using only the saved map", "run_formal_saved_map_cleaning_lifecycle.sh", ("first_map_then_clean",)),
    StepSpec("same_map_baseline", "gazebo", "measure the same-episode FullCoverage distance baseline", "run_formal_same_map_full_coverage_baseline.sh"),
    StepSpec("perception", "gazebo", "run fresh random-scene DOSOD plus EdgeSAM episodes", "run_formal_random_scene_perception.sh", ("random_scene_perception",)),
    StepSpec("dynamic_obstacle", "gazebo", "validate saved-map pedestrian avoidance", "run_formal_dynamic_obstacle_avoidance.sh", ("dynamic_obstacle_avoidance",)),
    StepSpec("single_episode", "gazebo", "run the complete product single-episode mission", "run_formal_single_episode_cleaning_mission.sh", ("end_to_end_cleaning_mission",)),
    StepSpec(
        "multisite_product",
        "gazebo",
        "serially materialize and run all eight validation and twelve hidden product sites",
        "formal_multisite_product_acceptance.py",
        ("multi_site_product_generalization",),
    ),
    StepSpec("s100_live", "external", "validate only real RDK S100P / Journey 6P runtime evidence", "validate_formal_s100_live_runtime.py", (EXTERNAL_GATE,)),
    StepSpec("finalize_session", "static", "seal evidence digests into the frozen session", "formal_acceptance_session.py"),
    StepSpec("functional_aggregate", "static", "aggregate all 38 positions and mission gates", "validate_formal_functional_acceptance_contract.py"),
)

# Every Gazebo launch and the cross-map RL trainer can materially consume the
# recovered Windows/WSL memory budget. They must each receive a fresh start
# gate; an earlier passing phase is not evidence that the next one is safe.
HEAVY_RUNTIME_STEP_IDS = frozenset(
    step.step_id for step in STEP_SPECS if step.mode == "gazebo"
) | {"rl_policy"}


def _requires_resource_gate(step: StepSpec) -> bool:
    """Return whether this step must recheck recovered memory before launch."""

    return step.step_id in HEAVY_RUNTIME_STEP_IDS


@dataclass
class Context:
    root: Path
    runtime_ws: Path
    integrated_build_manifest: Path
    perception_artifacts: Path
    onnx_pythonpath: Path
    run_root: Path
    base_domain: int
    episode_count: int
    runtime_closure_manifest: Path | None = None
    session: Path = SESSION
    snapshot: Path = SNAPSHOT
    accept_operator_trusted_s100: bool = False
    integrated_source_build_preflight_timeout_seconds: int = (
        DEFAULT_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS
    )
    requested_run_root: Path = field(init=False)
    requested_session: Path = field(init=False)
    requested_snapshot: Path = field(init=False)
    overlay: Path = field(init=False)
    episode_root: Path = field(init=False)
    map_root: Path = field(init=False)
    rl_evidence_root: Path = field(init=False)
    same_map_baseline: Path = field(init=False)

    def __post_init__(self) -> None:
        timeout_seconds = self.integrated_source_build_preflight_timeout_seconds
        if type(timeout_seconds) is not int or not (
            MIN_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS
            <= timeout_seconds
            <= MAX_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS
        ):
            raise ValueError(
                "integrated source/build preflight timeout must be an integer in "
                f"[{MIN_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS}, "
                f"{MAX_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS}] seconds"
            )
        # Preserve the lexical paths first: resolve() alone would hide a
        # symlinked session/snapshot hand-off before resume can reject it.
        self.requested_run_root = self.run_root
        self.requested_session = self.session
        self.requested_snapshot = self.snapshot
        self.root = self.root.resolve()
        self.runtime_ws = self.runtime_ws.resolve()
        self.integrated_build_manifest = self.integrated_build_manifest.resolve()
        self.perception_artifacts = self.perception_artifacts.resolve()
        self.onnx_pythonpath = self.onnx_pythonpath.resolve()
        self.run_root = self.run_root.resolve()
        self.session = self.session.resolve()
        self.snapshot = self.snapshot.resolve()
        self.overlay = self.runtime_ws / "install"
        self.runtime_closure_manifest = (
            self.runtime_ws / "final_runtime_closure_manifest.json"
            if self.runtime_closure_manifest is None
            else self.runtime_closure_manifest.resolve()
        )
        self.episode_root = self.run_root / "episode"
        self.map_root = self.run_root / "saved_map"
        self.rl_evidence_root = self.run_root / "rl_evidence"
        self.same_map_baseline = self.run_root / "same_map_full_coverage_baseline.json"


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise OrchestrationError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise OrchestrationError(f"JSON root is not an object: {path}")
    return value


def _read_contract(root: Path = ROOT) -> dict[str, Any]:
    path = root / CONTRACT.relative_to(ROOT)
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise OrchestrationError(f"cannot read functional contract: {exc}") from exc
    if not isinstance(value, dict) or not isinstance(value.get("evidence_gates"), dict):
        raise OrchestrationError("functional acceptance contract has no evidence_gates mapping")
    return value


def _advise_drop_cache(stream: Any) -> None:
    """Release clean pages after orchestrator identity hashing when supported."""

    fadvise = getattr(os, "posix_fadvise", None)
    dontneed = getattr(os, "POSIX_FADV_DONTNEED", None)
    if fadvise is None or dontneed is None:
        return
    try:
        fadvise(stream.fileno(), 0, 0, dontneed)
    except (OSError, ValueError):
        return


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
        # Only a fully consumed file is eligible for an advisory cache drop.
        _advise_drop_cache(stream)
    return digest.hexdigest()


def _nested(payload: Mapping[str, Any], dotted: str) -> Any:
    value: Any = payload
    for key in dotted.split("."):
        if not isinstance(value, Mapping) or key not in value:
            return None
        value = value[key]
    return value


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.replace(path)


def _formal_multimap_training_arguments(scenario_config: Path) -> list[str]:
    """Bind the final runner to every frozen map before any long runtime starts."""
    try:
        scenario = yaml.safe_load(scenario_config.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise OrchestrationError(
            f"unable to read frozen multi-map scenario: {scenario_config}"
        ) from exc
    split = scenario.get("split") if isinstance(scenario, dict) else None
    source_keys = {"train": "train", "validation": "val", "hidden": "hidden"}
    actual: dict[str, Any] = {}
    missions: dict[str, Any] = {}
    if isinstance(split, dict):
        for name, source_key in source_keys.items():
            row = split.get(source_key)
            actual[name] = row.get("map_count") if isinstance(row, dict) else None
            missions[name] = row.get("missions_per_map") if isinstance(row, dict) else None
    if actual != FORMAL_FULL_MAP_COUNTS:
        raise OrchestrationError(
            "formal RL gate requires frozen 32/8/12 multi-map counts; "
            f"got {actual}"
        )
    required_missions = FORMAL_MULTIMAP_MISSIONS_PER_MAP
    if missions != required_missions:
        raise OrchestrationError(
            "formal RL gate requires frozen 200/100/100 missions per map; "
            f"got {missions}"
        )
    selection = lambda name: ",".join(
        f"{map_index}:{mission_index}"
        for map_index in range(FORMAL_FULL_MAP_COUNTS[name])
        for mission_index in range(required_missions[name])
    )
    return [
        "--train",
        selection("train"),
        "--validation",
        selection("validation"),
        "--test",
        selection("hidden"),
    ]


def _formal_rl_budget_contract_audit(path: Path) -> tuple[dict[str, Any], list[str]]:
    """Read and lock both formal RL budgets before any runtime can start."""

    failures: list[str] = []
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        return {"path": str(path), "valid": False}, [
            f"invalid_formal_rl_budget_contract:{exc}"
        ]
    if not isinstance(payload, Mapping):
        return {"path": str(path), "valid": False}, [
            "invalid_formal_rl_budget_contract:not_a_mapping"
        ]

    required = {
        "schema_version": 1,
        "contract_id": "tzcup_formal_rl_budget_v1",
        "semantics.max_steps_is_episode_truncation_guard_not_task_or_episode_budget": True,
        "stage_a_fixed_map.fixed_map_id": "stage-a-fixed-formal-map-000",
        "stage_a_fixed_map.task_counts": FORMAL_STAGE_A_TASK_COUNTS,
        "stage_a_fixed_map.phase_order": [
            "train",
            "validation",
            "freeze_configuration",
            "hidden",
        ],
        "stage_a_fixed_map.hidden_tasks_visible_before_freeze": False,
        "multimap_generalization.map_counts": FORMAL_FULL_MAP_COUNTS,
        "multimap_generalization.missions_per_map": FORMAL_MULTIMAP_MISSIONS_PER_MAP,
        "multimap_generalization.task_counts": FORMAL_MULTIMAP_TASK_COUNTS,
        "multimap_generalization.hidden_tasks_visible_before_freeze": False,
        "policy.training_seeds": FORMAL_POLICY_SEEDS,
        "policy.required_seed_count": len(FORMAL_POLICY_SEEDS),
        "policy.selection_source": "validation_only_before_hidden",
        "execution.max_steps_per_episode": 400,
        "execution.max_steps_is_not_a_substitute_for_task_budget": True,
        "execution.mode": "serial_one_policy_seed_at_a_time",
        "execution.smoke_budget_can_pass_formal_acceptance": False,
    }
    mismatches = [
        dotted
        for dotted, expected in required.items()
        if not _strict_json_equal(_nested(payload, dotted), expected)
    ]
    if mismatches:
        failures.append(
            "formal_rl_budget_contract_drift:" + ",".join(sorted(mismatches))
        )
    return {
        "path": str(path),
        "valid": not mismatches,
        "stage_a_task_counts": _nested(payload, "stage_a_fixed_map.task_counts"),
        "multimap_task_counts": _nested(
            payload, "multimap_generalization.task_counts"
        ),
        "policy_seeds": _nested(payload, "policy.training_seeds"),
        "max_steps_per_episode": _nested(payload, "execution.max_steps_per_episode"),
        "max_steps_is_episode_truncation_guard_not_task_or_episode_budget": _nested(
            payload,
            "semantics.max_steps_is_episode_truncation_guard_not_task_or_episode_budget",
        ),
    }, failures


def _recursive_interface_contract_audit(root: Path) -> tuple[dict[str, bool], list[str]]:
    """Verify the static interfaces traversed by the final product runner.

    This deliberately reads only contracts/configuration and the PC product
    adapter / active-cleaning executor sources.  It does not import ROS
    packages, start a graph, or treat a static match as runtime evidence.
    """

    checks: dict[str, bool] = {}
    failures: list[str] = []

    def read(relative: str) -> str:
        path = root / relative
        try:
            return path.read_text(encoding="utf-8") if path.is_file() else ""
        except (OSError, UnicodeError):
            return ""

    def load_yaml(relative: str) -> Mapping[str, Any]:
        try:
            value = yaml.safe_load(read(relative))
        except yaml.YAMLError:
            return {}
        return value if isinstance(value, Mapping) else {}

    def python_constant(relative: str, name: str) -> Any:
        try:
            tree = ast.parse(read(relative), filename=relative)
        except SyntaxError:
            return None
        for node in tree.body:
            if not isinstance(node, (ast.Assign, ast.AnnAssign)):
                continue
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            if not any(isinstance(target, ast.Name) and target.id == name for target in targets):
                continue
            try:
                return ast.literal_eval(node.value)
            except (ValueError, TypeError):
                return None
        return None

    scenario = load_yaml(
        "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml"
    )
    scenario_split = scenario.get("split")
    scenario_val = scenario_split.get("val") if isinstance(scenario_split, Mapping) else None
    final_runner = read("scripts/run_formal_final_acceptance.py")
    multisite_runner = read("scripts/formal_multisite_product_acceptance.py")
    checks["validation_split_maps_to_generator_val"] = bool(
        isinstance(scenario_val, Mapping)
        and scenario_val.get("map_count") == FORMAL_FULL_MAP_COUNTS["validation"]
        and isinstance(scenario_split, Mapping)
        and "validation" not in scenario_split
        and '"validation": "val"' in final_runner
        and 'generator_split = "val" if site["split"] == "validation" else site["split"]'
        in multisite_runner
        and '"--split", generator_split' in multisite_runner
    )

    perception_contract = load_yaml(
        "starter_ws/src/sanitation_perception/config/formal_open_vocab_perception.yaml"
    )
    product_outputs = perception_contract.get("product_outputs")
    product_output_types = perception_contract.get("product_output_types")
    multisite_contract = load_yaml(
        "config/high_fidelity_vehicle/formal_multisite_product_acceptance_contract.yaml"
    )
    site_evidence = multisite_contract.get("site_evidence")
    required_topics = (
        site_evidence.get("required_topics") if isinstance(site_evidence, Mapping) else None
    )
    edgesam = required_topics.get("edgesam") if isinstance(required_topics, Mapping) else None
    pc_adapter = read(
        "starter_ws/src/sanitation_perception/sanitation_perception/pc_open_vocab_adapter.py"
    )
    collector_path = "scripts/collect_formal_single_episode_cleaning_mission.py"
    collector = read(collector_path)
    collector_interfaces = python_constant(collector_path, "MULTISITE_INTERFACES")
    collector_edgesam = (
        collector_interfaces.get("edgesam")
        if isinstance(collector_interfaces, Mapping)
        else None
    )
    observation_bridge = read(
        "starter_ws/src/sanitation_active_cleaning/"
        "sanitation_active_cleaning/formal_observation_bridge.py"
    )
    checks["pc_edgesam_ground_dirt_mask_is_image_end_to_end"] = bool(
        isinstance(product_outputs, Mapping)
        and product_outputs.get("masks") == "/perception/ground_dirt/masks"
        and isinstance(product_output_types, Mapping)
        and product_output_types.get("/perception/ground_dirt/masks")
        == "sensor_msgs/msg/Image"
        and edgesam
        == {
            "name": "/perception/ground_dirt/masks",
            "type": "sensor_msgs/msg/Image",
            "interface_kind": "topic",
        }
        and collector_edgesam
        == {
            "name": "/perception/ground_dirt/masks",
            "type": "sensor_msgs/msg/Image",
            "interface_kind": "topic",
            "observed_topic": "/perception/ground_dirt/masks",
        }
        and 'self.create_subscription(Image, MULTISITE_INTERFACES["edgesam"]["observed_topic"]'
        in collector
        and 'Image, "/perception/ground_dirt/masks", 10' in pc_adapter
        and "EdgeSamOnnxSegmenter" in pc_adapter
        and '"/perception/ground_dirt/masks"' in observation_bridge
        and "self.create_subscription(\n                Image," in observation_bridge
    )

    active_cleaning = load_yaml(
        "starter_ws/src/sanitation_active_cleaning/config/formal_runtime.yaml"
    )
    executor_parameters = active_cleaning.get("formal_active_cleaning_trajectory_executor")
    executor_parameters = (
        executor_parameters.get("ros__parameters")
        if isinstance(executor_parameters, Mapping)
        else None
    )
    nav2 = load_yaml("starter_ws/src/sanitation_navigation/config/nav2.yaml")
    controller = nav2.get("controller_server")
    controller = controller.get("ros__parameters") if isinstance(controller, Mapping) else None
    nav2_follow = controller.get("FollowPath") if isinstance(controller, Mapping) else None
    nav2_contract = required_topics.get("nav2") if isinstance(required_topics, Mapping) else None
    collector_nav2 = (
        collector_interfaces.get("nav2")
        if isinstance(collector_interfaces, Mapping)
        else None
    )
    trajectory_executor = read(
        "starter_ws/src/sanitation_active_cleaning/"
        "sanitation_active_cleaning/formal_trajectory_executor.py"
    )
    checks["active_cleaning_follow_path_action_chain_is_consistent"] = bool(
        nav2_contract
        == {
            "name": "/follow_path",
            "type": "nav2_msgs/action/FollowPath",
            "interface_kind": "action",
        }
        and collector_nav2
        == {
            "name": "/follow_path",
            "type": "nav2_msgs/action/FollowPath",
            "interface_kind": "action",
            "observed_topic": "/follow_path/_action/status",
        }
        and 'self.create_subscription(GoalStatusArray, MULTISITE_INTERFACES["nav2"]["observed_topic"]'
        in collector
        and isinstance(executor_parameters, Mapping)
        and executor_parameters.get("follow_path_action") == "/follow_path"
        and executor_parameters.get("controller_id") == "FollowPath"
        and isinstance(controller, Mapping)
        and "FollowPath" in controller.get("controller_plugins", [])
        and isinstance(nav2_follow, Mapping)
        and nav2_follow.get("plugin")
        == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        and 'NAVIGATION_ACTION = "/follow_path"' in trajectory_executor
        and "ActionClient(" in trajectory_executor
        and "FollowPath," in trajectory_executor
        and "goal = FollowPath.Goal()" in trajectory_executor
        and "goal.controller_id" in trajectory_executor
    )

    for name, passed in checks.items():
        if not passed:
            failures.append(f"recursive_interface_contract_drift:{name}")
    return checks, failures


def static_audit(root: Path = ROOT) -> dict[str, Any]:
    contract = _read_contract(root)
    gates = set(map(str, contract["evidence_gates"]))
    producers: dict[str, list[str]] = {gate: [] for gate in gates}
    runner_rows: dict[str, dict[str, Any]] = {}
    failures: list[str] = []
    scenario_config = root / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml"
    try:
        formal_multimap_arguments = _formal_multimap_training_arguments(scenario_config)
    except OrchestrationError as exc:
        formal_multimap_arguments = []
        failures.append(f"invalid_formal_multimap_split_contract:{exc}")
    formal_rl_budget, formal_rl_budget_failures = _formal_rl_budget_contract_audit(
        root
        / "starter_ws/src/sanitation_active_cleaning/config/formal_rl_budget_contract.yaml"
    )
    failures.extend(formal_rl_budget_failures)
    recursive_interface_checks, recursive_interface_failures = (
        _recursive_interface_contract_audit(root)
    )
    failures.extend(recursive_interface_failures)
    seen_steps: set[str] = set()
    for index, step in enumerate(STEP_SPECS):
        if step.step_id in seen_steps:
            failures.append(f"duplicate_step:{step.step_id}")
        seen_steps.add(step.step_id)
        if step.mode not in {"static", "gazebo", "external"}:
            failures.append(f"invalid_mode:{step.step_id}")
        if step.runner:
            runner = root / "scripts" / step.runner
            runner_source = runner.read_text(encoding="utf-8") if runner.is_file() else ""
            lock_strategy = None
            if step.mode == "gazebo":
                if "formal_runtime_configure" in runner_source:
                    lock_strategy = "runner_internal"
                    if step.requires_outer_gazebo_lock:
                        failures.append(f"duplicate_gazebo_lock_strategy:{step.step_id}")
                elif (
                    step.step_id == "multisite_product"
                    and "FIRST_MAP_RUNNER" in runner_source
                    and "SINGLE_SITE_RUNNER" in runner_source
                    and "for ordinal, site in enumerate(sites):" in runner_source
                ):
                    # The Python coordinator starts only isolation-owning
                    # runners, one after another. An outer flock here would
                    # deadlock those runners' own exclusive lock acquisition.
                    lock_strategy = "runner_internal"
                elif step.requires_outer_gazebo_lock:
                    lock_strategy = "orchestrator_outer_flock"
                else:
                    failures.append(f"missing_gazebo_lock_strategy:{step.step_id}")
            runner_rows[step.step_id] = {
                "path": str(runner.relative_to(root)).replace("\\", "/"),
                "exists": runner.is_file(),
                "gazebo_lock_strategy": lock_strategy,
            }
            if not runner.is_file():
                failures.append(f"missing_runner:{step.step_id}:{step.runner}")
        for gate in step.produces_gates:
            if gate not in producers:
                failures.append(f"unknown_gate:{step.step_id}:{gate}")
            else:
                producers[gate].append(step.step_id)
    for gate, rows in sorted(producers.items()):
        if len(rows) != 1:
            failures.append(f"gate_producer_count:{gate}:{len(rows)}")
    register_path = root / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
    try:
        register = yaml.safe_load(register_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        register = None
        failures.append(f"invalid_component_register:{exc}")
    registered_positions = (
        [str(row.get("id", "")) for row in register.get("functional_positions", [])]
        if isinstance(register, dict)
        else []
    )
    if (
        len(registered_positions) != 38
        or any(not position for position in registered_positions)
        or len(set(registered_positions)) != len(registered_positions)
    ):
        failures.append(f"registered_function_position_count:{len(registered_positions)}")
    position_contract = contract.get("functional_positions")
    if not isinstance(position_contract, dict):
        position_contract = {}
        failures.append("functional_position_crosswalk_missing")
    missing_positions = sorted(set(registered_positions) - set(position_contract))
    extra_positions = sorted(set(position_contract) - set(registered_positions))
    if missing_positions or extra_positions:
        failures.append(
            f"functional_position_crosswalk_mismatch:missing={missing_positions}:extra={extra_positions}"
        )
    producer_modes = {
        gate: {next(step.mode for step in STEP_SPECS if step.step_id == step_id) for step_id in step_ids}
        for gate, step_ids in producers.items()
        if step_ids
    }
    position_evidence_modes: dict[str, list[str]] = {}
    for position in registered_positions:
        required = position_contract.get(position)
        if not isinstance(required, list) or not required:
            failures.append(f"function_position_has_no_evidence:{position}")
            continue
        unknown = sorted(set(map(str, required)) - gates)
        if unknown:
            failures.append(f"function_position_unknown_gates:{position}:{unknown}")
            continue
        modes = sorted({mode for gate in required for mode in producer_modes.get(str(gate), set())})
        position_evidence_modes[position] = modes
        if "static" not in modes or not ({"gazebo", "external"} & set(modes)):
            failures.append(f"function_position_missing_static_or_runtime_evidence:{position}:{modes}")
    evidence_paths: dict[str, list[str]] = {}
    contracted_runtime_gates: set[str] = set()
    for gate, row in contract["evidence_gates"].items():
        if not isinstance(row, dict):
            failures.append(f"invalid_gate_contract:{gate}")
            continue
        relative = row.get("path")
        if (
            not isinstance(relative, str)
            or not relative
            or Path(relative).is_absolute()
            or ".." in Path(relative).parts
        ):
            failures.append(f"invalid_gate_output_path:{gate}:{relative}")
        else:
            evidence_paths.setdefault(Path(relative).as_posix(), []).append(str(gate))
        if row.get("session_bound") is not True:
            failures.append(f"gate_not_session_bound:{gate}")
        if row.get("runtime_binding") is not None:
            contracted_runtime_gates.add(str(gate))
        if gate in RUNTIME_GATE_BINDING_GATES and row.get(
            "runtime_binding"
        ) != RUNTIME_GATE_BINDING_CONTRACT:
            failures.append(f"gate_has_invalid_runtime_binding_contract:{gate}")
    if contracted_runtime_gates != RUNTIME_GATE_BINDING_GATES:
        failures.append(
            "runtime_binding_gate_set_mismatch:"
            f"missing={sorted(RUNTIME_GATE_BINDING_GATES - contracted_runtime_gates)}:"
            f"extra={sorted(contracted_runtime_gates - RUNTIME_GATE_BINDING_GATES)}"
        )
    for relative, path_gates in sorted(evidence_paths.items()):
        if len(path_gates) != 1:
            failures.append(f"duplicate_gate_output_path:{relative}:{path_gates}")
    mission_gates = contract.get("mission_level_gates")
    if not isinstance(mission_gates, list) or set(map(str, mission_gates or [])) - gates:
        failures.append("invalid_mission_level_gates")
    local_gates = sorted(gates - {EXTERNAL_GATE})
    external = contract["evidence_gates"].get(EXTERNAL_GATE, {})
    if external.get("evidence_origin") != "external_rdk_s100_live_hardware_only":
        failures.append("s100_gate_not_declared_external_hardware_only")
    modes = [step.mode for step in STEP_SPECS]
    if modes.count("external") != 1:
        failures.append("external_step_count_must_be_one")
    gazebo_order = [step.step_id for step in STEP_SPECS if step.mode == "gazebo"]
    required_order = [
        "visual", "sensor", "chassis", "safety_interlock",
        "cleaning_motors", "ground_dirt", "water_recovery", "service_door",
        "charge_and_drain", "manipulator", "twenty_cubes", "first_map",
        "saved_map_reuse", "perception", "dynamic_obstacle", "single_episode",
    ]
    positions = {name: gazebo_order.index(name) for name in required_order if name in gazebo_order}
    if len(positions) != len(required_order) or list(positions.values()) != sorted(positions.values()):
        failures.append("requested_gazebo_order_is_not_preserved")
    return {
        "report_id": "tzcup_formal_final_acceptance_orchestration_static_audit_v1",
        "status": (
            "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_STATIC_AUDIT_PASSED"
            if not failures else "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_STATIC_AUDIT_FAILED"
        ),
        "passed": not failures,
        "step_count": len(STEP_SPECS),
        "gazebo_step_count": sum(step.mode == "gazebo" for step in STEP_SPECS),
        "local_gate_count": len(local_gates),
        "contract_gate_count": len(gates),
        "registered_function_position_count": len(registered_positions),
        "crosswalk_function_position_count": len(position_contract),
        "function_position_evidence_modes": position_evidence_modes,
        "unique_gate_output_path_count": len(evidence_paths),
        "gate_producers": producers,
        "runner_inventory": runner_rows,
        "gazebo_execution_order": gazebo_order,
        "serial_execution": True,
        "shared_gazebo_lock": LOCK_FILE.as_posix(),
        "snapshot_checked_after_every_post_session_step": True,
        "old_evidence_reuse_allowed": False,
        "formal_multimap_training_contract": {
            "required_map_counts": FORMAL_FULL_MAP_COUNTS,
            "required_missions_per_map": FORMAL_MULTIMAP_MISSIONS_PER_MAP,
            "required_task_counts": FORMAL_MULTIMAP_TASK_COUNTS,
            "explicit_runner_arguments": formal_multimap_arguments,
            "smoke_subset_accepted_as_generalization": False,
        },
        "formal_rl_budget_contract": formal_rl_budget,
        "recursive_interface_contract": {
            "static_only": True,
            "checks": recursive_interface_checks,
        },
        "s100_live_gate": {
            "gate": EXTERNAL_GATE,
            "mode": "external",
            "hardware_only": True,
            "operator_trusted_evidence_required": True,
            "cryptographic_hardware_attestation": False,
            "malicious_pc_forgery_resistant": False,
            # This is an acceptance-process prohibition, not a claim that the
            # non-cryptographic evidence chain cannot be forged by a malicious PC.
            "pc_substitution_allowed": False,
        },
        "runtime_closure": {
            "manifest_required": True,
            "merged_overlay_required": True,
            "symlink_install_allowed": False,
            "runtime_package_count": len(FINAL_RUNTIME_PACKAGES),
            "side_brush_surface_preflight_required": True,
            "typed_cleaning_telemetry_source_manifest_required": True,
            "water_normal_full_surface_hash_reverification_required": True,
            "water_typed_transport_evidence_required": True,
            "verified_before_and_after_every_step": True,
            "runtime_gate_bindings_required": sorted(RUNTIME_GATE_BINDING_GATES),
            "functional_aggregate_revalidates_runtime_binding_sidecars": True,
        },
        "failures": failures,
    }


def _contract_evidence_paths(context: Context) -> list[Path]:
    contract = _read_contract(context.root)
    result = []
    for row in contract["evidence_gates"].values():
        relative = row.get("path") if isinstance(row, dict) else None
        if isinstance(relative, str):
            result.append(context.root / relative)
    return result


def _sensor_runtime_auxiliary_paths(context: Context) -> list[Path]:
    """Return every non-contract artifact owned by the sensor/FOV runner.

    The prepared world and memory sidecars are created before Gazebo starts and
    are meaningful only for that exact sensor attempt.  Keeping them outside
    the final-output archive left an incomplete attempt behind: a later run
    could neither prove it was fresh nor preserve it alongside the FOV/runtime
    reports.  Treat all sensor sidecars as one no-reuse unit.
    """

    contract = _read_contract(context.root)
    sensor_gate = contract["evidence_gates"].get("sensor_runtime")
    if not isinstance(sensor_gate, dict) or not isinstance(sensor_gate.get("path"), str):
        return []
    runtime_output = context.root / str(sensor_gate["path"])
    memory_base = runtime_output.with_suffix("")
    return [
        memory_base.with_name(memory_base.name + ".preembedded_sensor_world.sdf"),
        memory_base.with_name(memory_base.name + ".preembedded_sensor_world.json"),
        memory_base.with_name(memory_base.name + ".windows_memory_preflight.json"),
        memory_base.with_name(memory_base.name + ".windows_memory_preflight.log"),
        memory_base.with_name(memory_base.name + ".memory_watchdog.json"),
        memory_base.with_name(memory_base.name + ".memory_watchdog.log"),
        memory_base.with_name(memory_base.name + ".loopback_attestation.json"),
    ]


def _grasp_runtime_auxiliary_paths(context: Context) -> list[Path]:
    """Keep the contact-bearing grasp world with its final output attempt."""

    contract = _read_contract(context.root)
    gate = contract["evidence_gates"].get("physical_grasp_and_bin")
    if not isinstance(gate, dict) or not isinstance(gate.get("path"), str):
        return []
    output = context.root / str(gate["path"])
    base = output.with_suffix("")
    return [
        base.with_name(base.name + ".preembedded_grasp_world.sdf"),
        base.with_name(base.name + ".preembedded_grasp_world.json"),
        base.with_name(base.name + ".preembedded_vehicle.urdf"),
        base.with_name(base.name + ".preembedded_cube.urdf"),
    ]


def _runtime_binding_auxiliary_paths(context: Context) -> list[Path]:
    """Return the attempt-bound gate bindings written beside final outputs."""

    contract = _read_contract(context.root)
    result = []
    for gate_id in sorted(RUNTIME_GATE_BINDING_GATES):
        gate = contract["evidence_gates"].get(gate_id)
        if not isinstance(gate, dict) or not isinstance(gate.get("path"), str):
            continue
        output = context.root / str(gate["path"])
        result.append(output.with_name(output.name + ".runtime_binding.json"))
    return result


def _extra_fresh_paths(context: Context) -> list[Path]:
    return [
        context.session,
        context.run_root,
        context.root / ORCHESTRATION_REPORT.relative_to(ROOT),
        context.root / FUNCTIONAL_AUDIT.relative_to(ROOT),
        context.root / "artifacts/formal_vehicle_sensor_runtime.launch.log",
        context.root / "artifacts/formal_a300_drivetrain_runtime.launch.log",
        context.root / "artifacts/formal_vehicle_safety/launch.log",
        context.root / "artifacts/formal_auxiliary_power_lighting_runtime.launch.log",
        context.root / "artifacts/formal_function_positions_runtime.launch.log",
        context.root / "artifacts/formal_cleaning_actuator_motor_runtime.capture.json",
        context.root / "artifacts/formal_cleaning_actuator_motor_runtime.launch.log",
        context.root / "artifacts/formal_ground_dirt_cleaning_final_retry",
        context.root / "artifacts/formal_water_recovery",
        context.root / "artifacts/formal_service_door_runtime.launch.log",
        context.root / "artifacts/formal_service_interface_episodes",
        context.root / "artifacts/formal_manipulator_trajectory_runtime.launch.log",
        context.root / "artifacts/formal_grasp_executor_runtime.launch.log",
        context.root / "artifacts/formal_20_cube_grasp_manifest.json",
        context.root / "artifacts/formal_20_cube_grasp_runtime.launch.log",
        context.root / "artifacts/formal_same_map_full_coverage_baseline.json",
        *_runtime_binding_auxiliary_paths(context),
        *_sensor_runtime_auxiliary_paths(context),
        *_grasp_runtime_auxiliary_paths(context),
    ]


def _archive_timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")


def plan_final_output_archive(
    context: Context, *, timestamp: str | None = None
) -> dict[str, Any]:
    """Plan preservation of existing final outputs without moving any bytes."""

    stamp = timestamp or _archive_timestamp()
    if not re.fullmatch(r"[0-9]{8}T[0-9]{12}Z", stamp):
        raise OrchestrationError(f"invalid final-output archive timestamp: {stamp}")
    archive_root = context.root / ".work/formal_final_acceptance/archives"
    destination = archive_root / stamp
    if destination.exists() or destination.is_symlink():
        raise OrchestrationError(f"final-output archive destination already exists: {destination}")
    candidates = []
    for path in dict.fromkeys(_contract_evidence_paths(context) + _extra_fresh_paths(context)):
        if not path.exists() and not path.is_symlink():
            continue
        if path.is_symlink():
            raise OrchestrationError(f"final output archive source is a symbolic link: {path}")
        resolved = path.resolve()
        try:
            relative = resolved.relative_to(context.root)
        except ValueError as exc:
            raise OrchestrationError(f"final output escapes repository root: {path}") from exc
        if relative == Path(".") or not relative.parts:
            raise OrchestrationError("repository root cannot be archived as a final output")
        if archive_root.resolve() == resolved or archive_root.resolve() in resolved.parents:
            raise OrchestrationError(f"archive source is already inside archive root: {path}")
        if resolved in destination.resolve().parents:
            raise OrchestrationError(
                f"archive destination would be nested inside source: {path}"
            )
        candidates.append((resolved, relative))
    selected: list[tuple[Path, Path]] = []
    for source, relative in sorted(candidates, key=lambda row: len(row[1].parts)):
        if any(parent == source or parent in source.parents for parent, _ in selected):
            continue
        selected.append((source, relative))
    entries = [
        {
            "source": str(source),
            "relative": relative.as_posix(),
            "destination": str(destination / relative),
        }
        for source, relative in selected
    ]
    if len({row["destination"] for row in entries}) != len(entries):
        raise OrchestrationError("final-output archive plan has duplicate destinations")
    return {
        "status": "FORMAL_FINAL_OUTPUT_ARCHIVE_PLANNED",
        "validated": True,
        "executed": False,
        "timestamp": stamp,
        "destination": str(destination),
        "entry_count": len(entries),
        "entries": entries,
    }


def execute_final_output_archive(
    context: Context, plan: Mapping[str, Any]
) -> dict[str, Any]:
    """Execute only an already validated plan, failing closed on any drift."""

    if plan.get("validated") is not True or plan.get("executed") is not False:
        raise OrchestrationError("final-output archive plan is not executable")
    destination = Path(str(plan.get("destination", ""))).resolve()
    expected_root = (
        context.root / ".work/formal_final_acceptance/archives"
    ).resolve()
    if destination.parent != expected_root or destination.exists() or destination.is_symlink():
        raise OrchestrationError("final-output archive destination is not unique and unused")
    entries = plan.get("entries")
    if not isinstance(entries, list) or len(entries) != plan.get("entry_count"):
        raise OrchestrationError("final-output archive plan entries are invalid")
    checked: list[tuple[Path, Path]] = []
    for row in entries:
        if not isinstance(row, dict):
            raise OrchestrationError("final-output archive entry is invalid")
        source = Path(str(row.get("source", ""))).resolve()
        target = Path(str(row.get("destination", ""))).resolve()
        try:
            relative = source.relative_to(context.root)
        except ValueError as exc:
            raise OrchestrationError(f"archive source escapes repository root: {source}") from exc
        if not source.exists() or target != destination / relative:
            raise OrchestrationError(f"archive plan drifted before execution: {source}")
        checked.append((source, target))
    if not checked:
        return {**dict(plan), "status": "FORMAL_FINAL_OUTPUT_ARCHIVE_NOT_NEEDED"}
    destination.mkdir(parents=True, exist_ok=False)
    moved: list[str] = []
    try:
        for source, target in checked:
            target.parent.mkdir(parents=True, exist_ok=True)
            source.replace(target)
            moved.append(str(source))
    except OSError as exc:
        raise OrchestrationError(
            f"final-output archive failed closed after moving {len(moved)} entries: {exc}"
        ) from exc
    return {
        **dict(plan),
        "status": "FORMAL_FINAL_OUTPUT_ARCHIVE_PRESERVED",
        "executed": True,
        "moved_sources": moved,
    }


def freshness_rows(
    context: Context, *, planned_archive_sources: Iterable[Path] = ()
) -> list[dict[str, Any]]:
    """Return the complete no-reuse decision for every evidence/output path."""
    rows = []
    planned = {path.resolve() for path in planned_archive_sources}
    for path in dict.fromkeys(_contract_evidence_paths(context) + _extra_fresh_paths(context)):
        exists = path.exists()
        resolved = path.resolve()
        archived = exists and any(
            source == resolved or source in resolved.parents for source in planned
        )
        rows.append(
            {
                "path": str(path.relative_to(context.root)).replace("\\", "/"),
                "passed": not exists or archived,
                "detail": (
                    "existing evidence scheduled for preservation"
                    if archived
                    else "existing evidence would be refused"
                    if exists
                    else "absent"
                ),
            }
        )
    return rows


def _linux_safe_domain(base: int, count: int) -> bool:
    return all((0 <= value <= 101) or (215 <= value <= 231) for value in range(base, base + count))


def _lock_available() -> tuple[bool, str | None]:
    if os.name != "posix" or fcntl is None:
        return True, None
    LOCK_FILE.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(LOCK_FILE, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False, f"another formal Gazebo run owns {LOCK_FILE}"
        fcntl.flock(descriptor, fcntl.LOCK_UN)
        return True, None
    finally:
        os.close(descriptor)


def preflight(context: Context) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def add(check_id: str, passed: bool, detail: str) -> None:
        checks.append({"id": check_id, "passed": bool(passed), "detail": detail})

    try:
        archive_plan = plan_final_output_archive(context)
    except OrchestrationError as exc:
        archive_plan = {
            "status": "FORMAL_FINAL_OUTPUT_ARCHIVE_PLAN_INVALID",
            "validated": False,
            "executed": False,
            "error": str(exc),
            "entries": [],
        }
    audit = static_audit(context.root)
    add("static_audit", audit["passed"], audit["status"])
    add("repository_root", context.root == ROOT.resolve(), str(context.root))
    try:
        run_root = _validate_run_root(context, require_exists=False)
    except OrchestrationError as exc:
        add("fresh_run_root", False, str(exc))
    else:
        add("fresh_run_root", True, str(run_root))
    add("runtime_overlay", (context.overlay / "setup.bash").is_file(), str(context.overlay / "setup.bash"))
    add(
        "runtime_closure_manifest",
        bool(context.runtime_closure_manifest and context.runtime_closure_manifest.is_file()),
        str(context.runtime_closure_manifest),
    )
    add("integrated_build_manifest", context.integrated_build_manifest.is_file(), str(context.integrated_build_manifest))
    add("perception_artifact_manifest", (context.perception_artifacts / "artifact_manifest.json").is_file(), str(context.perception_artifacts / "artifact_manifest.json"))
    add("onnxruntime_overlay", (context.onnx_pythonpath / "onnxruntime/__init__.py").is_file(), str(context.onnx_pythonpath))
    add("episode_count", context.episode_count >= 30, f"episode_count={context.episode_count}; formal_minimum=30")
    add(
        "final_output_archive_plan",
        archive_plan.get("validated") is True,
        str(archive_plan.get("destination", archive_plan.get("error", "invalid"))),
    )
    add("ros_domain_range", _linux_safe_domain(context.base_domain, max(8, context.episode_count)), f"base={context.base_domain}")
    native_posix = os.name == "posix"
    add(
        "native_posix_runtime",
        native_posix,
        (
            "native Linux/WSL process"
            if native_posix
            else "run preflight inside WSL/Linux; Windows bash.exe is not a formal runtime"
        ),
    )
    resolved_commands: dict[str, str | None] = {}
    for command in ("bash", "python3", "flock", "setsid", "timeout"):
        resolved = shutil.which(command) if native_posix else None
        resolved_commands[command] = resolved
        add(
            f"command:{command}",
            resolved is not None,
            resolved or ("missing" if native_posix else "not probed outside Linux/WSL"),
        )
    ros_setup = Path("/opt/ros/jazzy/setup.bash")
    add("ros_jazzy_setup", native_posix and ros_setup.is_file(), str(ros_setup))
    ros_gz_image_bridge = Path("/opt/ros/jazzy/lib/ros_gz_image/image_bridge")
    add(
        "ros_gz_image_bridge",
        native_posix
        and ros_gz_image_bridge.is_file()
        and os.access(ros_gz_image_bridge, os.X_OK),
        str(ros_gz_image_bridge),
    )
    sourced_commands: dict[str, str] = {}
    if (
        native_posix
        and ros_setup.is_file()
        and (context.overlay / "setup.bash").is_file()
        and resolved_commands["bash"]
    ):
        probe_script = (
            f"source {shlex.quote(str(ros_setup))}; "
            f"source {shlex.quote(str(context.overlay / 'setup.bash'))}; "
            "for name in xacro ros2 gz; do printf '%s=%s\\n' \"${name}\" \"$(command -v \"${name}\" || true)\"; done"
        )
        try:
            command_probe = subprocess.run(
                ["bash", "-lc", probe_script], cwd=context.root,
                capture_output=True, text=True, timeout=15,
            )
        except subprocess.TimeoutExpired:
            pass
        else:
            sourced_commands = dict(
                line.split("=", 1) for line in command_probe.stdout.splitlines() if "=" in line
            )
    for command in ("xacro", "ros2", "gz"):
        resolved = sourced_commands.get(command, "")
        add(f"sourced_command:{command}", bool(resolved), resolved or "missing after ROS/runtime setup")
    lock_ok, lock_error = _lock_available()
    add("gazebo_global_lock_available", lock_ok, lock_error or str(LOCK_FILE))
    for step in STEP_SPECS:
        if not step.runner or not step.runner.endswith(".sh"):
            continue
        runner = context.root / "scripts" / step.runner
        if not runner.is_file():
            continue
        if not native_posix or resolved_commands["bash"] is None:
            add(
                f"bash_syntax:{step.step_id}",
                False,
                "not probed outside native Linux/WSL",
            )
            continue
        try:
            probe = subprocess.run(
                ["bash", "-n", str(runner)], cwd=context.root,
                capture_output=True, text=True, timeout=10,
            )
        except subprocess.TimeoutExpired:
            add(f"bash_syntax:{step.step_id}", False, "syntax check exceeded 10 seconds")
        else:
            add(f"bash_syntax:{step.step_id}", probe.returncode == 0, probe.stderr.strip() or str(runner))
    try:
        snapshot_probe = subprocess.run(
            [sys.executable, str(context.root / "scripts/generate_formal_vehicle_snapshot.py"), "--check"],
            cwd=context.root,
            capture_output=True,
            text=True,
            timeout=30,
        )
        snapshot_current = snapshot_probe.returncode == 0
    except subprocess.TimeoutExpired:
        snapshot_current = False
    add(
        "snapshot_regeneration_ready",
        True,
        "current=true" if snapshot_current else "current=false; --execute regenerates before session start",
    )
    planned_sources = [
        Path(str(row["source"]))
        for row in archive_plan.get("entries", [])
        if isinstance(row, dict) and "source" in row
    ]
    for row in freshness_rows(context, planned_archive_sources=planned_sources):
        add(f"fresh_path:{row['path']}", row["passed"], row["detail"])
    for path in (
        context.root / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml",
        context.root / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml",
    ):
        add(f"input:{path.relative_to(context.root)}", path.is_file(), str(path))
    if context.runtime_closure_manifest and context.runtime_closure_manifest.is_file():
        try:
            closure = _verify_runtime_closure(context, "preflight")
        except OrchestrationError as exc:
            add("unified_runtime_closure", False, str(exc))
        else:
            add(
                "unified_runtime_closure",
                True,
                (
                    f"closure={closure['closure_sha256']} "
                    f"packages={closure['runtime_package_count']} "
                    f"symlinks={closure['symbolic_link_count']}"
                ),
            )
    if context.integrated_build_manifest.is_file() and (context.overlay / "setup.bash").is_file():
        try:
            integrated = subprocess.run(
                [
                    sys.executable,
                    str(context.root / "scripts/aggregate_integrated_functional_acceptance.py"),
                    "preflight",
                    "--repo-root", str(context.root),
                    "--runtime-ws", str(context.runtime_ws),
                    "--build-manifest", str(context.integrated_build_manifest),
                ],
                cwd=context.root,
                capture_output=True,
                text=True,
                timeout=context.integrated_source_build_preflight_timeout_seconds,
            )
        except subprocess.TimeoutExpired:
            add(
                "integrated_source_build_binding",
                False,
                (
                    "source/install verification exceeded "
                    f"{context.integrated_source_build_preflight_timeout_seconds} seconds"
                ),
            )
        else:
            add("integrated_source_build_binding", integrated.returncode == 0, (integrated.stderr or integrated.stdout).strip()[-1500:])
    blockers = [row["id"] for row in checks if not row["passed"]]
    return {
        "report_id": "tzcup_formal_final_acceptance_preflight_v1",
        "status": "FORMAL_FINAL_ACCEPTANCE_PREFLIGHT_PASSED" if not blockers else "FORMAL_FINAL_ACCEPTANCE_PREFLIGHT_BLOCKED",
        "passed": not blockers,
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "mutations_performed": False,
        "gazebo_started": False,
        "old_evidence_reuse_allowed": False,
        "integrated_source_build_preflight_timeout_seconds": (
            context.integrated_source_build_preflight_timeout_seconds
        ),
        "archive_plan": archive_plan,
        "checks": checks,
        "blockers": blockers,
        "s100_live_gate": {
            "status": "EXTERNAL_HARD_GATE_PENDING",
            "pc_substitution_allowed": False,
        },
    }


def _windows_dry_run_requirement(path: Path, category: str) -> dict[str, str]:
    """Describe one required frozen-runtime path without probing a runtime."""

    if path.is_symlink():
        status = "SYMBOLIC_LINK_REJECTED"
    elif path.exists():
        status = "PRESENT"
    else:
        status = "MISSING"
    return {"category": category, "path": str(path), "status": status}


def windows_dry_run(context: Context) -> dict[str, Any]:
    """Plan the four formal chains on Windows without launching any runtime.

    This is deliberately narrower than :func:`preflight`: Windows cannot
    validate a Linux/WSL ROS overlay, but it can safely inventory the frozen
    workspace hand-off and render the exact runner order.  In particular, it
    never resolves or invokes ``bash.exe``/``wsl.exe``.  A READY result means
    only that the local hand-off paths exist; native preflight and a verified
    closure remain mandatory before execution.
    """

    runtime_ws = context.runtime_ws
    requirements = [
        _windows_dry_run_requirement(runtime_ws, "runtime_workspace"),
        _windows_dry_run_requirement(runtime_ws / "src", "frozen_source"),
        _windows_dry_run_requirement(runtime_ws / "build", "build"),
        _windows_dry_run_requirement(runtime_ws / "install", "install"),
        _windows_dry_run_requirement(runtime_ws / "log", "build_log_directory"),
        _windows_dry_run_requirement(runtime_ws / "install/setup.bash", "install_setup"),
        _windows_dry_run_requirement(
            context.integrated_build_manifest, "integrated_build_manifest"
        ),
        _windows_dry_run_requirement(
            context.runtime_closure_manifest, "closure_manifest"
        ),
        _windows_dry_run_requirement(
            runtime_ws / "INSTALL_SYMLINKS.txt", "install_symlink_manifest"
        ),
        _windows_dry_run_requirement(
            runtime_ws / "side_brush_sdf_surface_preflight.json",
            "installed_surface_preflight",
        ),
        _windows_dry_run_requirement(
            runtime_ws / "formal_windows_cold_start_evidence.json",
            "windows_cold_start_manifest",
        ),
        _windows_dry_run_requirement(
            context.perception_artifacts / "artifact_manifest.json",
            "perception_artifact_manifest",
        ),
        _windows_dry_run_requirement(
            context.onnx_pythonpath / "onnxruntime/__init__.py",
            "onnxruntime_manifest",
        ),
    ]
    missing = [row for row in requirements if row["status"] != "PRESENT"]

    # Rendering the water command needs the typed-telemetry subclosure.  A
    # shape-valid placeholder is used only to expose its final three-command
    # sequence when no frozen manifest exists.  It is marked below and can
    # never become evidence or an executable authorization.
    command_placeholder = {"typed_cleaning_telemetry_source_sha256": "0" * 64}
    sequence_index = {step.step_id: index for index, step in enumerate(STEP_SPECS)}
    runners: list[dict[str, Any]] = []
    for step_id in WINDOWS_DRY_RUN_FOUR_CHAIN_STEPS:
        step = next(step for step in STEP_SPECS if step.step_id == step_id)
        command, environment = _step_command(
            step_id, context, runtime_closure=command_placeholder
        )
        if command[0] == "__sequence__":
            command_rows: list[list[str]] = json.loads(command[1])
        else:
            command_rows = [command]
        runners.append(
            {
                "sequence_index": sequence_index[step_id],
                "step_id": step_id,
                "runner": step.runner,
                "mode": step.mode,
                "commands": command_rows,
                "environment": dict(sorted(environment.items())),
                "requires_verified_typed_subclosure": step_id == "water_recovery",
                "requires_fresh_resource_gate_before_launch": True,
                "windows_execution_permitted": False,
            }
        )

    return {
        "report_id": "tzcup_formal_final_runtime_windows_dry_run_v1",
        "status": (
            "FORMAL_FINAL_RUNTIME_WINDOWS_DRY_RUN_READY_FOR_NATIVE_PREFLIGHT"
            if not missing
            else "FORMAL_FINAL_RUNTIME_WINDOWS_DRY_RUN_BLOCKED"
        ),
        "passed": not missing,
        "ready_for_native_preflight": not missing,
        "ready_for_runtime_execution": False,
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "execution_scope": {
            "windows_python_read_only": True,
            "started_wsl": False,
            "started_gazebo": False,
            "started_cadquery": False,
            "started_freecad": False,
            "system_bash_exe_invoked": False,
            "runtime_evidence_created": False,
        },
        "requirements": requirements,
        "missing_requirements": missing,
        "four_chain_runner_order": runners,
        "strict_memory_recovery_contract": {
            "phase_order": [
                "windows_cold_memory_gate_and_single_worker_frozen_build",
                "record_and_verify_frozen_runtime_closure",
                *WINDOWS_DRY_RUN_FOUR_CHAIN_STEPS,
            ],
            "frozen_runtime_must_preexist": True,
            "native_preflight_and_closure_verification_required_before_execute": True,
            "resource_gate": {
                "required_before_each_heavy_phase": True,
                "heavy_step_ids": sorted(HEAVY_RUNTIME_STEP_IDS),
                "windows_probe": "formal_windows_memory_probe.py --check-start",
                "dry_run_does_not_probe_or_start_a_runtime": True,
            },
            "four_chain": {
                "strict_serial_execution": True,
                "stop_on_first_failure": True,
                "preserve_unfinalized_session_on_failure": True,
            },
            "parallelism": {
                "gazebo_max_parallel_processes": 1,
                "cad_execution_permitted": False,
                "board_execution_automatic": False,
            },
        },
        "water_command_placeholder_only": True,
        "claim_boundary": (
            "This Windows dry-run inventories hand-off paths and renders commands only; "
            "it does not verify the Linux closure, build, install, or any runtime evidence."
        ),
    }


def _shell_source_command(context: Context, argv: Sequence[str]) -> list[str]:
    command = " ".join(shlex.quote(str(value)) for value in argv)
    script = (
        "source /opt/ros/jazzy/setup.bash; "
        f"source {shlex.quote(str(context.overlay / 'setup.bash'))}; "
        f"exec {command}"
    )
    return ["bash", "-lc", script]


def _bound_nvidia_egl_environment(
    context: Context, runtime_closure: Mapping[str, Any]
) -> dict[str, str]:
    """Return the only EGL environment accepted from a verified closure.

    The final runtime must never inherit a host EGL vendor path.  Keeping this
    contract here lets preflight reject an incomplete closure without launching
    a renderer, then gives every real child process the same frozen pair.
    """

    if runtime_closure.get("nvidia_egl_runtime_bound") is not True:
        raise OrchestrationError("runtime closure has no bound NVIDIA EGL runtime")
    nvidia_egl_runtime = runtime_closure.get("nvidia_egl_runtime")
    if not isinstance(nvidia_egl_runtime, Mapping):
        raise OrchestrationError("runtime closure has no NVIDIA EGL runtime object")
    if nvidia_egl_runtime.get("status") != "NVIDIA_EGL_RUNTIME_BOUND":
        raise OrchestrationError("runtime closure NVIDIA EGL runtime is not bound")
    expected = {
        "__EGL_VENDOR_LIBRARY_FILENAMES": str(
            context.runtime_ws / "egl_vendor.d/10_nvidia.json"
        ),
        "EGL_PLATFORM": "surfaceless",
    }
    if nvidia_egl_runtime.get("environment") != expected:
        raise OrchestrationError(
            "runtime closure NVIDIA EGL environment does not match the frozen runtime"
        )
    return expected


def _step_command(
    step_id: str,
    context: Context,
    *,
    runtime_closure: Mapping[str, Any] | None = None,
    execution_environment: bool = False,
) -> tuple[list[str], dict[str, str]]:
    root = context.root
    scripts = root / "scripts"
    # Destination paths are read from this execution context's contract.  This
    # keeps an isolated acceptance root from accidentally emitting evidence to
    # the repository-global contract's locations.
    contract = _read_contract(context.root)
    gate_output = lambda gate: root / contract["evidence_gates"][gate]["path"]
    environment = {
        "TZCUP_REPOSITORY_ROOT": str(root),
        "ROS_DOMAIN_ID": str(context.base_domain),
        "PYTHONDONTWRITEBYTECODE": "1",
        "FORMAL_VEHICLE_SNAPSHOT_MANIFEST": str(context.snapshot),
        "FORMAL_ACCEPTANCE_SESSION": str(context.session),
        "FORMAL_ACCEPTANCE_SESSION_STATUS": str(context.session),
        "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST": str(
            context.runtime_closure_manifest
        ),
    }
    if execution_environment:
        if runtime_closure is None:
            raise OrchestrationError(
                "formal execution requires the verified runtime closure identity"
            )
        environment.update(_bound_nvidia_egl_environment(context, runtime_closure))
    bash = lambda name: ["bash", str(scripts / name)]
    python = lambda name, *args: [sys.executable, str(scripts / name), *map(str, args)]
    if step_id == "freeze_snapshot":
        return _shell_source_command(context, [sys.executable, scripts / "generate_formal_vehicle_snapshot.py"]), environment
    if step_id == "start_session":
        return python(
            "formal_acceptance_session.py",
            "start",
            "--snapshot",
            context.snapshot,
            "--output",
            context.session,
            "--runtime-closure-manifest",
            context.runtime_closure_manifest,
            "--runtime-install-root",
            context.overlay,
            "--repository-root",
            context.root,
        ), environment
    if step_id == "component_register":
        return python("refresh_formal_component_register_evidence.py", "--output", gate_output("component_register")), environment
    if step_id == "visual":
        environment.update(FORMAL_VEHICLE_VISUAL_RUNTIME_SETUP=str(context.overlay / "setup.bash"), FORMAL_VEHICLE_VISUAL_RUN_ROOT=str(context.run_root / "visual"), FORMAL_VEHICLE_VISUAL_PUBLISH_ROOT=str(root / "reports/engineering"))
        return bash("run_formal_vehicle_visual_acceptance.sh"), environment
    if step_id == "inertia":
        return python("scan_formal_vehicle_inertia_and_swept_volume.py", "--output", root / "reports/engineering/formal_vehicle_inertia_and_swept_volume_report.json"), environment
    if step_id == "sensor":
        sensor_output = gate_output("sensor_runtime")
        sensor_base = sensor_output.with_suffix("")
        environment.update(
            FORMAL_SENSOR_RUNTIME_SETUP=str(context.overlay / "setup.bash"),
            FORMAL_SENSOR_RUNTIME_OUTPUT=str(sensor_output),
            FORMAL_SENSOR_FOV_OUTPUT=str(gate_output("sensor_fov_and_occlusion")),
            FORMAL_SENSOR_PREEMBEDDED_WORLD=str(
                sensor_base.with_name(sensor_base.name + ".preembedded_sensor_world.sdf")
            ),
            FORMAL_SENSOR_PREEMBEDDED_REPORT=str(
                sensor_base.with_name(sensor_base.name + ".preembedded_sensor_world.json")
            ),
        )
        return bash("run_formal_vehicle_sensor_runtime.sh"), environment
    if step_id in {"chassis", "safety_interlock", "cleaning_positions", "cleaning_motors", "manipulator"}:
        environment["FORMAL_VEHICLE_RUNTIME_WS"] = str(context.runtime_ws)
        output_variables = {
            "chassis": ("FORMAL_VEHICLE_MOBILITY_OUTPUT", "a300_drivetrain_runtime"),
            "cleaning_positions": ("FORMAL_FUNCTION_POSITIONS_OUTPUT", "cleaning_actuators"),
            "cleaning_motors": ("FORMAL_CLEANING_MOTOR_OUTPUT", "cleaning_motor_runtime"),
            "manipulator": ("FORMAL_MANIPULATOR_TRAJECTORY_OUTPUT", "manipulator_trajectory"),
        }
        if step_id in output_variables:
            variable, gate = output_variables[step_id]
            environment[variable] = str(gate_output(gate))
        if step_id == "chassis":
            mobility_output = gate_output("a300_drivetrain_runtime")
            environment["FORMAL_VEHICLE_MOBILITY_RUNTIME_BINDING"] = str(
                mobility_output.with_name(mobility_output.name + ".runtime_binding.json")
            )
        if step_id == "cleaning_positions":
            function_positions_output = gate_output("cleaning_actuators")
            environment["FORMAL_FUNCTION_POSITIONS_RUNTIME_BINDING"] = str(
                function_positions_output.with_name(
                    function_positions_output.name + ".runtime_binding.json"
                )
            )
        if step_id == "cleaning_motors":
            cleaning_motor_output = gate_output("cleaning_motor_runtime")
            environment["FORMAL_CLEANING_MOTOR_RUNTIME_BINDING"] = str(
                cleaning_motor_output.with_name(
                    cleaning_motor_output.name + ".runtime_binding.json"
                )
            )
        if step_id == "manipulator":
            manipulator_output = gate_output("manipulator_trajectory")
            environment["FORMAL_MANIPULATOR_TRAJECTORY_RUNTIME_BINDING"] = str(
                manipulator_output.with_name(
                    manipulator_output.name + ".runtime_binding.json"
                )
            )
        names = {
            "chassis": "run_formal_vehicle_mobility_runtime.sh",
            "safety_interlock": "run_whole_vehicle_actuator_interlock_runtime.sh",
            "cleaning_positions": "run_formal_function_positions_runtime.sh",
            "cleaning_motors": "run_formal_cleaning_actuator_motor_runtime.sh",
            "manipulator": "run_formal_manipulator_trajectory_runtime.sh",
        }
        command = bash(names[step_id])
        if step_id == "safety_interlock":
            command.append(str(gate_output("whole_vehicle_interlock")))
        return command, environment
    if step_id == "safety_power_lighting":
        environment.update(
            FORMAL_AUXILIARY_UNDERLAY=str(context.overlay),
            FORMAL_AUXILIARY_OUTPUT=str(gate_output("auxiliary_power_lighting")),
        )
        return bash("run_formal_auxiliary_runtime.sh"), environment
    if step_id == "ground_dirt":
        environment.update(
            FORMAL_DIRT_RUNTIME_WS=str(context.runtime_ws),
            FORMAL_DIRT_OUTPUT_DIR=str(gate_output("ground_dirt_cleaning").parent),
        )
        return bash("run_formal_ground_dirt_cleaning_runtime.sh"), environment
    if step_id == "water_recovery":
        if runtime_closure is None:
            raise OrchestrationError(
                "water recovery requires the verified runtime closure identity"
            )
        typed_subclosure = runtime_closure.get(
            "typed_cleaning_telemetry_source_sha256"
        )
        if not isinstance(typed_subclosure, str) or re.fullmatch(
            r"[0-9a-f]{64}", typed_subclosure
        ) is None:
            raise OrchestrationError(
                "verified runtime closure has no valid typed telemetry subclosure digest"
            )
        water_root = context.run_root / "water_recovery"
        critical_manifest = water_root / "critical_source_manifest.json"
        typed_output = water_root / "typed_transport"
        scenario_output = water_root / "scenarios"
        environment.update(
            # The water runner's variable names the frozen install root, unlike
            # the workspace-root contract used by the other vehicle runners.
            FORMAL_VEHICLE_RUNTIME_WS=str(context.overlay),
            FORMAL_WATER_OUTPUT_DIR=str(scenario_output),
            FORMAL_WATER_FINAL_ARTIFACT=str(gate_output("water_recovery")),
            FORMAL_WATER_RUNTIME_BINDING=str(
                gate_output("water_recovery").with_name(
                    gate_output("water_recovery").name + ".runtime_binding.json"
                )
            ),
            FORMAL_WATER_TYPED_OUTPUT_DIR=str(typed_output),
            FORMAL_WATER_TYPED_RUNTIME_WS=str(context.runtime_ws),
            FORMAL_WATER_TYPED_DURATION_S="10",
            FORMAL_WATER_TYPED_DIAG_JSON=str(typed_output / "typed_diag.json"),
            FORMAL_WATER_TYPED_RAW_TRACE=str(typed_output / "raw_frames.jsonl"),
            FORMAL_WATER_TYPED_RUNNER=str(
                scripts / "run_formal_typed_cleaning_motor_diagnostic.sh"
            ),
            FORMAL_WATER_TYPED_COLLECTOR=str(
                scripts / "collect_formal_typed_cleaning_motor_diagnostic.py"
            ),
            FORMAL_WATER_CRITICAL_SOURCE_MANIFEST=str(critical_manifest),
            FORMAL_WATER_TYPED_SUBCLOSURE_SHA256=typed_subclosure,
        )
        commands = [
            python(
                "generate_formal_water_critical_source_manifest.py",
                "--repo",
                root,
                "--workspace",
                context.runtime_ws,
                "--output",
                critical_manifest,
            ),
            bash("run_formal_typed_cleaning_motor_diagnostic.sh"),
            bash("run_formal_water_recovery_runtime.sh")
            + ["--scenario", "all", "--output-dir", str(scenario_output)],
        ]
        return ["__sequence__", json.dumps(commands)], environment
    if step_id == "service_door":
        environment.update(
            FORMAL_SERVICE_DOOR_RUNTIME_SETUP=str(context.overlay / "setup.bash"),
            FORMAL_SERVICE_DOOR_RUNTIME_OUTPUT=str(gate_output("service_door_runtime")),
        )
        return bash("run_formal_service_door_runtime.sh"), environment
    if step_id == "charge_and_drain":
        environment.update(FORMAL_SERVICE_SETUP=str(context.overlay / "setup.bash"), FORMAL_SERVICE_DOMAIN_BASE=str(context.base_domain))
        return bash("run_formal_service_interface_acceptance.sh") + [
            str(context.run_root / "service_interface_episodes"),
            str(gate_output("service_interface_acceptance")),
        ], environment
    if step_id in {"physical_grasp", "twenty_cubes"}:
        environment["FORMAL_MANIPULATION_RUNTIME_WS"] = str(context.overlay)
        environment["FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST"] = str(
            context.runtime_closure_manifest
        )
        name = "run_formal_grasp_executor_runtime.sh" if step_id == "physical_grasp" else "run_formal_20_cube_grasp_acceptance.sh"
        environment[
            "FORMAL_GRASP_EXECUTOR_OUTPUT" if step_id == "physical_grasp" else "FORMAL_20_CUBE_OUTPUT"
        ] = str(gate_output(
            "physical_grasp_and_bin" if step_id == "physical_grasp" else "formal_20_cube_grasp_and_dynamic_mass"
        ))
        if step_id == "physical_grasp":
            grasp_output = gate_output("physical_grasp_and_bin")
            environment["FORMAL_GRASP_EXECUTOR_RUNTIME_BINDING"] = str(
                grasp_output.with_name(grasp_output.name + ".runtime_binding.json")
            )
            grasp_base = grasp_output.with_suffix("")
            environment.update(
                FORMAL_GRASP_PREEMBEDDED_WORLD=str(
                    grasp_base.with_name(grasp_base.name + ".preembedded_grasp_world.sdf")
                ),
                FORMAL_GRASP_PREEMBEDDED_REPORT=str(
                    grasp_base.with_name(grasp_base.name + ".preembedded_grasp_world.json")
                ),
                FORMAL_GRASP_PREEMBEDDED_VEHICLE_URDF=str(
                    grasp_base.with_name(grasp_base.name + ".preembedded_vehicle.urdf")
                ),
                FORMAL_GRASP_PREEMBEDDED_CUBE_URDF=str(
                    grasp_base.with_name(grasp_base.name + ".preembedded_cube.urdf")
                ),
            )
        else:
            cube_output = gate_output("formal_20_cube_grasp_and_dynamic_mass")
            environment["FORMAL_20_CUBE_RUNTIME_BINDING"] = str(
                cube_output.with_name(cube_output.name + ".runtime_binding.json")
            )
        return bash(name), environment
    if step_id == "integrated_basic_physics":
        supplied_build = _read_json(context.integrated_build_manifest)
        build_started_ns = supplied_build.get("build_started_epoch_ns")
        if not isinstance(build_started_ns, int) or build_started_ns <= 0:
            raise OrchestrationError("integrated build manifest has no valid build_started_epoch_ns")
        refreshed_build_manifest = context.run_root / "integrated_build_manifest.json"
        environment.update(
            INTEGRATED_ACCEPTANCE_RUNTIME_WS=str(context.runtime_ws),
            INTEGRATED_ACCEPTANCE_BUILD_MANIFEST=str(refreshed_build_manifest),
            INTEGRATED_ACCEPTANCE_OUTPUT_DIR=str(context.run_root / "integrated_basic_physics"),
            INTEGRATED_ACCEPTANCE_DOMAIN_BASE=str(context.base_domain),
            INTEGRATED_ACCEPTANCE_SNAPSHOT_MANIFEST=str(context.snapshot),
            INTEGRATED_ACCEPTANCE_CONTRACT_SUMMARY=str(gate_output("integrated_basic_physics")),
            INTEGRATED_ACCEPTANCE_RUNTIME_BINDING=str(
                gate_output("integrated_basic_physics").with_name(
                    gate_output("integrated_basic_physics").name
                    + ".runtime_binding.json"
                )
            ),
        )
        refresh = python(
            "aggregate_integrated_functional_acceptance.py", "record-build",
            "--repo-root", context.root, "--runtime-ws", context.runtime_ws,
            "--build-started-epoch-ns", build_started_ns,
            "--output", refreshed_build_manifest,
        )
        return ["__sequence__", json.dumps([refresh, bash("run_integrated_functional_acceptance.sh")])], environment
    if step_id == "episode_materialization":
        return _shell_source_command(context, [
            "ros2", "run", "sanitation_campus_scenario", "sanitation-campus-scenario", "materialize-hidden",
            "--config", root / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml",
            "--snapshot", context.snapshot, "--session", context.session,
            "--run-root", context.run_root,
            "--freeze-producer", "formal_rl_multimap",
            "--map-index", "0", "--mission-index", "0",
            "--output", context.episode_root,
        ]), environment
    if step_id == "first_map":
        environment.update(
            FORMAL_VEHICLE_RUNTIME_WS=str(context.runtime_ws),
            FORMAL_DYNAMIC_EPISODE_ROOT=str(context.episode_root),
            FORMAL_DYNAMIC_SAVED_MAP_ROOT=str(context.map_root),
            FORMAL_MAP_RUNTIME_OVERLAY=str(context.overlay),
        )
        return bash("run_formal_first_map_dynamic_prerequisite.sh"), environment
    if step_id == "saved_map_reuse":
        environment.update(
            FORMAL_VEHICLE_RUNTIME_WS=str(context.runtime_ws),
            FORMAL_DYNAMIC_EPISODE_ROOT=str(context.episode_root),
            FORMAL_DYNAMIC_SAVED_MAP_ROOT=str(context.map_root),
            FORMAL_MAP_RUNTIME_OVERLAY=str(context.overlay),
            FORMAL_MAP_LIFECYCLE_OUTPUT=str(gate_output("first_map_then_clean")),
        )
        return bash("run_formal_saved_map_cleaning_lifecycle.sh"), environment
    if step_id == "same_map_baseline":
        environment["FORMAL_VEHICLE_RUNTIME_WS"] = str(context.runtime_ws)
        return bash("run_formal_same_map_full_coverage_baseline.sh") + [
            "--episode-root", str(context.episode_root), "--map-root", str(context.map_root),
            "--session", str(context.session), "--runtime-overlay", str(context.overlay),
            "--output", str(context.run_root / "same_map_full_coverage"),
            "--formal-output", str(context.same_map_baseline), "--ros-domain-id", str(context.base_domain),
        ], environment
    if step_id == "perception":
        environment.update(
            FORMAL_VEHICLE_RUNTIME_WS=str(context.runtime_ws),
            FORMAL_CAMPUS_STAGE1_SETUP=str(context.overlay / "setup.bash"),
            FORMAL_CAMPUS_RUNTIME_SETUP=str(context.overlay / "setup.bash"),
            FORMAL_CAMPUS_AGENT_SETUP=str(context.overlay / "setup.bash"),
            FORMAL_PERCEPTION_ACCEPTANCE_SETUP=str(context.overlay / "setup.bash"),
            FORMAL_PERCEPTION_ARTIFACT_ROOT=str(context.perception_artifacts),
            FORMAL_PERCEPTION_ONNXRUNTIME_PYTHONPATH=str(context.onnx_pythonpath),
            FORMAL_PERCEPTION_OUTPUT_ROOT=str(context.run_root / "perception"),
            FORMAL_PERCEPTION_FINAL_ARTIFACT=str(gate_output("random_scene_perception")),
            FORMAL_PERCEPTION_EPISODE_COUNT=str(context.episode_count),
            FORMAL_PERCEPTION_BASE_DOMAIN=str(context.base_domain),
        )
        return bash("run_formal_random_scene_perception.sh"), environment
    if step_id == "dynamic_obstacle":
        dynamic_output = gate_output("dynamic_obstacle_avoidance")
        environment.update(
            FORMAL_VEHICLE_RUNTIME_WS=str(context.runtime_ws),
            FORMAL_DYNAMIC_EPISODE_ROOT=str(context.episode_root),
            FORMAL_DYNAMIC_SAVED_MAP_ROOT=str(context.map_root),
            FORMAL_DYNAMIC_TELEMETRY=str(context.run_root / "dynamic_obstacle/runtime_telemetry.json"),
            FORMAL_DYNAMIC_OUTPUT=str(dynamic_output),
            FORMAL_DYNAMIC_RUNTIME_BINDING=str(
                dynamic_output.with_name(dynamic_output.name + ".runtime_binding.json")
            ),
        )
        return bash("run_formal_dynamic_obstacle_avoidance.sh"), environment
    if step_id == "rl_policy":
        scenario_config = (
            root
            / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml"
        )
        stage_a = _shell_source_command(context, [
            "ros2", "run", "sanitation_active_cleaning", "formal_stage_a_active_cleaning_train",
            "--scenario-config", scenario_config,
            "--motion-profile", root / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml",
            "--budget-contract", FORMAL_RL_BUDGET_CONTRACT,
            "--work-root", context.run_root / "rl_stage_a_work",
            "--snapshot", context.snapshot, "--session", context.session,
            "--hidden-receipt-root", context.run_root,
            "--output", context.rl_evidence_root / "stage_a_budget_report.json",
            "--map-resolution", "0.5", "--planning-resolution", "2.0",
        ])
        training = _shell_source_command(context, [
            "ros2", "run", "sanitation_active_cleaning", "formal_active_cleaning_train",
            "--scenario-config", scenario_config,
            "--motion-profile", root / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml",
            "--work-root", context.run_root / "rl_work", "--evidence-root", context.rl_evidence_root,
            "--snapshot", context.snapshot, "--session", context.session,
            "--hidden-receipt-root", context.run_root,
            "--map-resolution", "0.5", "--planning-resolution", "2.0", "--epochs", "1", "--max-steps", "400",
            "--budget-contract", FORMAL_RL_BUDGET_CONTRACT, "--policy-seeds", "7,17,29,43,61",
            *_formal_multimap_training_arguments(scenario_config),
        ])
        report = python(
            "generate_formal_rl_multimap_report.py", "--evidence-root", context.rl_evidence_root / "formal_planning",
            "--stage-a-evidence", context.rl_evidence_root / "stage_a_budget_report.json",
            "--snapshot", context.snapshot, "--session", context.session,
            "--runtime-overlay", context.overlay,
            "--runtime-closure", context.runtime_closure_manifest,
            "--output", gate_output("rl_cross_map_policy"),
        )
        # The executor recognizes this sentinel and runs both commands without a shell string.
        return ["__sequence__", json.dumps([stage_a, training, report])], environment
    if step_id == "single_episode":
        e2e_output = gate_output("end_to_end_cleaning_mission")
        environment["FORMAL_E2E_RUNTIME_BINDING"] = str(
            e2e_output.with_name(e2e_output.name + ".runtime_binding.json")
        )
        return bash("run_formal_single_episode_cleaning_mission.sh") + [
            "--episode-root", str(context.episode_root), "--session-status", str(context.session),
            "--saved-map", str(context.map_root), "--perception-artifacts", str(context.perception_artifacts),
            "--policy-checkpoint", str(context.rl_evidence_root / "formal_planning/q_policy.json"),
            "--same-map-baseline", str(context.same_map_baseline), "--output", str(context.run_root / "single_episode"),
            "--runtime-overlay", str(context.overlay), "--formal-output", str(e2e_output),
            "--ros-domain-id", str(context.base_domain),
        ], environment
    if step_id == "multisite_product":
        multisite_output = gate_output("multi_site_product_generalization")
        return _shell_source_command(context, [
            "python3", scripts / "formal_multisite_product_acceptance.py", "--execute",
            "--evidence-root", context.run_root / "multisite_product_sites",
            "--work-root", context.run_root / "multisite_product_runtime",
            "--snapshot", context.snapshot,
            "--session", context.session,
            "--runtime-closure", context.runtime_closure_manifest,
            "--runtime-ws", context.runtime_ws,
            "--runtime-overlay", context.overlay,
            "--runtime-binding", multisite_output.with_name(
                multisite_output.name + ".runtime_binding.json"
            ),
            "--perception-artifacts", context.perception_artifacts,
            "--policy-checkpoint", context.rl_evidence_root / "formal_planning/q_policy.json",
            "--base-domain", context.base_domain,
            "--output", multisite_output,
        ]), environment
    raise OrchestrationError(f"no executable command for step {step_id}")


def _stop_exact_process_group(process: subprocess.Popen[str]) -> None:
    """Stop only the session created for one orchestrated step."""

    if process.poll() is not None:
        return
    for sig, timeout_s in (
        (signal.SIGINT, 8.0),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 2.0),
    ):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout_s)
            return
        except subprocess.TimeoutExpired:
            continue


def _memory_watchdog_enabled(environment: Mapping[str, str]) -> bool:
    raw = environment.get(
        "FORMAL_MEMORY_WATCHDOG_ENABLED",
        os.environ.get("FORMAL_MEMORY_WATCHDOG_ENABLED", "1"),
    )
    if raw in {"1", "true", "TRUE", "yes", "YES"}:
        return True
    if raw in {"0", "false", "FALSE", "no", "NO"}:
        raise OrchestrationError(
            "FORMAL_MEMORY_WATCHDOG_ENABLED cannot be disabled for formal acceptance"
        )
    raise OrchestrationError("FORMAL_MEMORY_WATCHDOG_ENABLED must be a boolean")


def _windows_memory_guard_enabled(environment: Mapping[str, str]) -> bool:
    raw = environment.get(
        "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED",
        os.environ.get("FORMAL_WINDOWS_MEMORY_GUARD_ENABLED", "1"),
    )
    if raw in {"1", "true", "TRUE", "yes", "YES"}:
        return True
    if raw in {"0", "false", "FALSE", "no", "NO"}:
        raise OrchestrationError(
            "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED cannot be disabled for formal acceptance"
        )
    raise OrchestrationError("FORMAL_WINDOWS_MEMORY_GUARD_ENABLED must be a boolean")


def _run_windows_memory_preflight(
    child_environment: Mapping[str, str], log_path: Path
) -> None:
    if not _windows_memory_guard_enabled(child_environment):
        return
    evidence = log_path.with_name(
        f"{log_path.stem}.windows_memory_preflight.json"
    )
    probe_log = log_path.with_name(
        f"{log_path.stem}.windows_memory_preflight.log"
    )
    if evidence.exists() or probe_log.exists():
        raise OrchestrationError(
            f"refusing stale Windows memory preflight evidence beside {log_path}"
        )
    with probe_log.open("w", encoding="utf-8", errors="backslashreplace") as stream:
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/formal_windows_memory_probe.py"),
                "--check-start",
                "--output",
                str(evidence),
            ],
            cwd=ROOT,
            env=dict(child_environment),
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
        )
    if result.returncode == MEMORY_BREACH_EXIT_CODE:
        raise MemoryLimitError(
            f"Windows commit/Docker memory gate refused launch rc={MEMORY_BREACH_EXIT_CODE}; "
            f"evidence={evidence}"
        )
    if result.returncode != 0:
        raise OrchestrationError(
            f"Windows memory preflight failed closed rc={result.returncode}; "
            f"log={probe_log}"
        )


def _run_process(
    argv: Sequence[str],
    environment: Mapping[str, str],
    log_path: Path,
    *,
    wrap_lock: bool = False,
    memory_watchdog: bool = False,
) -> None:
    actual = list(argv)
    if wrap_lock:
        actual = ["flock", "-n", str(LOCK_FILE), *actual]
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8", errors="backslashreplace") as stream:
        child_environment = {**os.environ, **environment}
        if not memory_watchdog or not _memory_watchdog_enabled(environment):
            result = subprocess.run(
                actual,
                cwd=ROOT,
                env=child_environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                text=True,
            )
            if result.returncode != 0:
                raise OrchestrationError(
                    f"command failed rc={result.returncode}; log={log_path}"
                )
            return

        if os.name != "posix":
            raise OrchestrationError("formal memory watchdog requires Linux /proc")
        # The orchestrated command already owns a fresh session / PGID below.
        # Runners which support this flag must keep their heavy launch children
        # in that group, otherwise a nested `setsid` escapes both RSS accounting
        # and the watchdog's exact-group shutdown.
        child_environment["FORMAL_ORCHESTRATED_STEP_SESSION"] = "1"
        child_environment["FORMAL_ORCHESTRATED_STEP_SESSION_TOKEN"] = secrets.token_hex(32)
        _run_windows_memory_preflight(child_environment, log_path)
        watchdog_json = log_path.with_name(f"{log_path.stem}.memory_watchdog.json")
        watchdog_log = log_path.with_name(f"{log_path.stem}.memory_watchdog.log")
        if watchdog_json.exists() or watchdog_log.exists():
            raise OrchestrationError(
                f"refusing stale memory-watchdog evidence beside {log_path}"
            )
        process = subprocess.Popen(
            actual,
            cwd=ROOT,
            env=child_environment,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        watchdog = subprocess.Popen(
            [
                "bash",
                str(MEMORY_WATCHDOG),
                "--leader-pid",
                str(process.pid),
                "--pgid",
                str(process.pid),
                "--json",
                str(watchdog_json),
                "--log",
                str(watchdog_log),
            ],
            cwd=ROOT,
            env=child_environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            start_new_session=True,
        )
        while process.poll() is None and watchdog.poll() is None:
            time.sleep(0.25)
        if process.poll() is not None and watchdog.poll() is None:
            watchdog.send_signal(signal.SIGTERM)
        try:
            watchdog_returncode = watchdog.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            watchdog.kill()
            watchdog_returncode = watchdog.wait(timeout=2.0)
        if watchdog_returncode == MEMORY_BREACH_EXIT_CODE:
            _stop_exact_process_group(process)
            process.wait(timeout=15.0)
            raise MemoryLimitError(
                f"memory watchdog stopped command rc={MEMORY_BREACH_EXIT_CODE}; "
                f"evidence={watchdog_json}; log={log_path}"
            )
        if watchdog_returncode != 0:
            _stop_exact_process_group(process)
            raise OrchestrationError(
                f"memory watchdog failed closed rc={watchdog_returncode}; "
                f"evidence={watchdog_json}"
            )
        returncode = process.wait()
        if returncode == MEMORY_BREACH_EXIT_CODE:
            raise MemoryLimitError(
                f"child command reported memory limit rc={MEMORY_BREACH_EXIT_CODE}; "
                f"log={log_path}"
            )
        if returncode != 0:
            raise OrchestrationError(f"command failed rc={returncode}; log={log_path}")


def _snapshot_identity(context: Context) -> dict[str, str]:
    snapshot = _read_json(context.snapshot)
    outputs = snapshot.get("outputs", {})
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf", {}) if isinstance(outputs, dict) else {}
    return {
        "snapshot_manifest_sha256": _sha256(context.snapshot),
        "source_inventory_sha256": str(snapshot.get("source_inventory_sha256", "")),
        "expanded_urdf_sha256": str(urdf.get("sha256", "")) if isinstance(urdf, dict) else "",
    }


def _validate_runtime_binding(
    context: Context,
    gate_id: str,
    row: Mapping[str, Any],
    payload: Mapping[str, Any],
    evidence_path: Path,
    session_started_ns: int,
) -> None:
    """Reject a runtime report detached from its frozen runner binding."""

    binding_contract = row.get("runtime_binding")
    if binding_contract is None:
        return
    if binding_contract != RUNTIME_GATE_BINDING_CONTRACT:
        raise OrchestrationError(
            f"gate {gate_id} has an invalid runtime-binding contract"
        )
    report_field = str(binding_contract["report_field"])
    sidecar = evidence_path.with_name(
        evidence_path.name + str(binding_contract["sidecar_suffix"])
    )
    if (
        sidecar.is_symlink()
        or not sidecar.is_file()
        or sidecar.stat().st_mtime_ns < session_started_ns
        or sidecar.stat().st_mtime_ns > evidence_path.stat().st_mtime_ns
    ):
        raise OrchestrationError(
            f"gate {gate_id} runtime binding sidecar is missing, stale, or reordered"
        )
    binding = _read_json(sidecar)
    if not _strict_json_equal(_nested(payload, report_field), binding):
        raise OrchestrationError(
            f"gate {gate_id} report runtime binding differs from its sidecar"
        )
    session = _read_json(context.session)
    identity = _snapshot_identity(context)
    session_binding = binding.get("acceptance_session_binding")
    if not isinstance(session_binding, dict):
        raise OrchestrationError(f"gate {gate_id} runtime binding has no session binding")
    if (
        binding.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
        or session_binding.get("snapshot") != identity
        or session_binding.get("session_started_epoch_ns") != session_started_ns
        or session_binding.get("session_status_at_gate")
        != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or session_binding.get("snapshot_current_source_verified") is not True
    ):
        raise OrchestrationError(
            f"gate {gate_id} runtime binding is not bound to the active source session"
        )
    verified_ns = binding.get("verified_epoch_ns")
    if (
        not isinstance(verified_ns, int)
        or verified_ns < session_started_ns
        or verified_ns > time.time_ns()
    ):
        raise OrchestrationError(f"gate {gate_id} runtime binding verification time is invalid")
    if session.get("started_epoch_ns") != session_started_ns or session.get(
        "snapshot"
    ) != identity:
        raise OrchestrationError(
            f"gate {gate_id} current acceptance session differs from its runtime binding"
        )
    if session.get("status") == "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING" and (
        session_binding.get("session_manifest_sha256") != _sha256(context.session)
    ):
        raise OrchestrationError(
            f"gate {gate_id} runtime binding does not match the running session manifest"
        )
    closure = binding.get("runtime_closure_binding")
    closure_manifest = _read_json(context.runtime_closure_manifest)
    if (
        not isinstance(closure, dict)
        or closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
        or closure.get("manifest") != str(context.runtime_closure_manifest.resolve())
        or closure.get("manifest_sha256") != _sha256(context.runtime_closure_manifest)
        or closure.get("closure_sha256") != closure_manifest.get("closure_sha256")
        or closure.get("runtime_install_root") != str(context.overlay.resolve())
        or closure.get("symbolic_link_count") != 0
    ):
        raise OrchestrationError(
            f"gate {gate_id} runtime binding does not match the frozen runtime closure"
        )


def _validate_preembedded_grasp_evidence(
    context: Context,
    row: Mapping[str, Any],
    payload: Mapping[str, Any],
    evidence_path: Path,
    session_started_ns: int,
) -> None:
    """Re-hash and semantically revalidate the contact world retained by grasp."""

    contract = row.get("preembedded_grasp_binding")
    expected_contract = {"report_field": "preembedded_grasp_world_binding"}
    if contract != expected_contract:
        raise OrchestrationError("physical grasp has an invalid preembedded-world contract")
    binding = _nested(payload, str(contract["report_field"]))
    if not isinstance(binding, dict):
        raise OrchestrationError("physical grasp has no preembedded-world binding")

    base = evidence_path.with_suffix("")
    expected_paths = {
        "preembedded_report_path": base.with_name(
            base.name + ".preembedded_grasp_world.json"
        ),
        "preembedded_world_path": base.with_name(
            base.name + ".preembedded_grasp_world.sdf"
        ),
        "vehicle_urdf_path": base.with_name(base.name + ".preembedded_vehicle.urdf"),
        "cube_urdf_path": base.with_name(base.name + ".preembedded_cube.urdf"),
    }
    resolved: dict[str, Path] = {}
    for field, expected in expected_paths.items():
        candidate = _repo_regular_file(context.root, expected, f"physical grasp {field}")
        if candidate != expected.resolve() or binding.get(field) != str(candidate):
            raise OrchestrationError(f"physical grasp {field} differs from routed auxiliary evidence")
        if candidate.stat().st_mtime_ns < session_started_ns:
            raise OrchestrationError(f"physical grasp {field} predates the current session")
        hash_field = field.removesuffix("_path") + "_sha256"
        if binding.get(hash_field) != _sha256(candidate):
            raise OrchestrationError(f"physical grasp {field} digest differs from retained evidence")
        resolved[field] = candidate

    runtime_binding = payload.get("runtime_gate_binding")
    if not isinstance(runtime_binding, dict):
        raise OrchestrationError("physical grasp has no runtime gate binding")
    closure = runtime_binding.get("runtime_closure_binding")
    if not isinstance(closure, dict):
        raise OrchestrationError("physical grasp runtime binding has no closure")
    session = _read_json(context.session)
    session_hash = _sha256(context.session)
    try:
        verified = validate_preembedded_grasp_world(
            report_path=resolved["preembedded_report_path"],
            world_path=resolved["preembedded_world_path"],
            vehicle_urdf_path=resolved["vehicle_urdf_path"],
            cube_urdf_path=resolved["cube_urdf_path"],
            source_world_path=(
                context.overlay
                / "share/sanitation_manipulation/worlds/formal_cube_manipulation.sdf"
            ),
            acceptance_session={
                "started_epoch_ns": session_started_ns,
                "session_manifest_sha256": session_hash,
            },
            snapshot_identity=_snapshot_identity(context),
            expected_runtime_install_root=context.overlay,
        )
    except (OSError, ValueError, RuntimeError, PreembeddedGraspWorldBindingError) as error:
        raise OrchestrationError(f"physical grasp preembedded-world binding is invalid: {error}") from error
    if session.get("started_epoch_ns") != session_started_ns or not _strict_json_equal(
        binding, verified
    ):
        raise OrchestrationError("physical grasp preembedded-world binding differs from current session evidence")


def _validate_typed_water_transport_evidence(
    context: Context,
    payload: Mapping[str, Any],
    session_started_ns: int,
    runtime_closure: Mapping[str, Any],
) -> dict[str, str]:
    evidence = payload.get("evidence")
    typed = evidence.get("typed_transport") if isinstance(evidence, dict) else None
    if not isinstance(typed, dict):
        raise OrchestrationError("water recovery has no typed transport evidence")
    if typed.get("contract") != TYPED_WATER_TRANSPORT_CONTRACT:
        raise OrchestrationError("water typed transport contract is not frozen")
    closure_digest = runtime_closure.get(
        "typed_cleaning_telemetry_source_sha256"
    )
    if (
        not isinstance(closure_digest, str)
        or typed.get("typed_cleaning_telemetry_source_sha256") != closure_digest
    ):
        raise OrchestrationError(
            "water typed transport source digest does not match runtime closure"
        )
    file_fields = (
        ("typed_diag_json", "typed_diag_sha256", True),
        ("raw_trace_jsonl", "raw_trace_sha256", True),
        ("runner_script", "runner_sha256", False),
        ("collector_script", "collector_sha256", False),
        ("critical_source_manifest_json", "critical_source_manifest_sha256", True),
    )
    paths: dict[str, Path] = {}
    hashes: dict[str, str] = {}
    for path_key, hash_key, fresh in file_fields:
        raw_path = typed.get(path_key)
        expected_hash = typed.get(hash_key)
        if not isinstance(raw_path, str) or not isinstance(expected_hash, str):
            raise OrchestrationError(f"water typed transport field is missing: {path_key}")
        candidate = Path(raw_path)
        if candidate.is_symlink() or not candidate.is_file():
            raise OrchestrationError(f"water typed transport file is not regular: {path_key}")
        candidate = candidate.resolve()
        if fresh and candidate.stat().st_mtime_ns < session_started_ns:
            raise OrchestrationError(f"water typed transport file is stale: {path_key}")
        if _sha256(candidate) != expected_hash:
            raise OrchestrationError(f"water typed transport hash mismatch: {path_key}")
        paths[path_key] = candidate
        hashes[hash_key] = expected_hash
    if len(set(paths.values())) != len(paths):
        raise OrchestrationError("water typed transport evidence files must be distinct")
    expected_water_root = (context.run_root / "water_recovery").resolve()
    expected_typed_root = (expected_water_root / "typed_transport").resolve()
    if paths["critical_source_manifest_json"].parent != expected_water_root:
        raise OrchestrationError(
            "water critical source manifest is outside the current run root"
        )
    if any(
        paths[name].parent != expected_typed_root
        for name in ("typed_diag_json", "raw_trace_jsonl")
    ):
        raise OrchestrationError(
            "water typed runtime evidence is outside the current run root"
        )
    _read_json(paths["critical_source_manifest_json"])
    diagnostic = _read_json(paths["typed_diag_json"])
    checks = diagnostic.get("checks")
    if (
        diagnostic.get("status") != "FORMAL_TYPED_CLEANING_MOTOR_DIAG_PASSED"
        or diagnostic.get("passed") is not True
        or not isinstance(checks, dict)
        or set(checks) != TYPED_DIAG_REQUIRED_CHECKS
        or any(checks[name] is not True for name in TYPED_DIAG_REQUIRED_CHECKS)
    ):
        raise OrchestrationError("water typed diagnostic did not pass every frozen check")
    metrics = diagnostic.get("metrics")
    if not isinstance(metrics, dict):
        raise OrchestrationError("water typed diagnostic has no metrics")
    raw_lines = [
        line
        for line in paths["raw_trace_jsonl"].read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    try:
        raw_frames = [json.loads(line) for line in raw_lines]
    except json.JSONDecodeError as exc:
        raise OrchestrationError(f"water typed raw trace is invalid JSONL: {exc}") from exc
    if any(not isinstance(frame, dict) for frame in raw_frames):
        raise OrchestrationError("water typed raw trace rows must be JSON objects")
    raw_trace_count = len(raw_frames)
    if raw_trace_count <= 0 or raw_trace_count != metrics.get("raw_trace_frame_count"):
        raise OrchestrationError("water typed raw trace does not match diagnostic metrics")
    transport_audit = diagnostic.get("transport_audit")
    transport_checks = (
        transport_audit.get("checks") if isinstance(transport_audit, dict) else None
    )
    if (
        not isinstance(transport_audit, dict)
        or transport_audit.get("passed") is not True
        or not isinstance(transport_checks, dict)
        or set(transport_checks) != TYPED_TRANSPORT_AUDIT_REQUIRED_CHECKS
        or any(
            transport_checks[name] is not True
            for name in TYPED_TRANSPORT_AUDIT_REQUIRED_CHECKS
        )
        or transport_audit.get("node_shared_publish_errors") != []
        or transport_audit.get("topic_tagged_publish_failures") != []
    ):
        raise OrchestrationError("water typed transport runtime audit did not pass")
    transport_file_fields = (
        ("launch_log", "launch_log_sha256"),
        ("launch_audit_json", "launch_audit_sha256"),
        ("gazebo_topic_info", "gazebo_topic_info_sha256"),
        ("ros_topic_info", "ros_topic_info_sha256"),
    )
    transport_paths: list[Path] = []
    for path_key, hash_key in transport_file_fields:
        raw_path = transport_audit.get(path_key)
        expected_hash = transport_audit.get(hash_key)
        if not isinstance(raw_path, str) or not isinstance(expected_hash, str):
            raise OrchestrationError(
                f"water typed transport runtime audit field is missing: {path_key}"
            )
        candidate_path = Path(raw_path)
        if candidate_path.is_symlink() or not candidate_path.is_file():
            raise OrchestrationError(
                f"water typed transport runtime audit file is not regular: {path_key}"
            )
        candidate = candidate_path.resolve()
        if (
            candidate.parent != paths["typed_diag_json"].parent
            or candidate.stat().st_mtime_ns < session_started_ns
            or _sha256(candidate) != expected_hash
        ):
            raise OrchestrationError(
                f"water typed transport runtime audit hash/freshness mismatch: {path_key}"
            )
        transport_paths.append(candidate)
        hashes[hash_key] = expected_hash
    if len(set(transport_paths)) != len(transport_paths):
        raise OrchestrationError(
            "water typed transport runtime audit files must be distinct"
        )
    return {
        **hashes,
        "typed_cleaning_telemetry_source_sha256": closure_digest,
    }


def _validate_water_surface_evidence(
    context: Context,
    payload: Mapping[str, Any],
    session_started_ns: int,
    runtime_closure: Mapping[str, Any] | None,
) -> dict[str, str]:
    if runtime_closure is None:
        raise OrchestrationError("water surface evidence has no verified runtime closure")
    expected_sdf = runtime_closure.get("side_brush_expanded_sdf_sha256")
    expected_xacro = runtime_closure.get("side_brush_installed_xacro_sha256")
    expected_xacro_path = runtime_closure.get("side_brush_installed_xacro")
    if (
        not isinstance(expected_sdf, str)
        or not isinstance(expected_xacro, str)
        or not isinstance(expected_xacro_path, str)
    ):
        raise OrchestrationError("runtime closure has no side-brush surface identity")
    evidence = payload.get("evidence")
    if not isinstance(evidence, dict):
        raise OrchestrationError("water recovery evidence has no evidence mapping")
    results: dict[str, str] = {}
    expanded_hashes = set()
    if _nested(payload, "checks.side_brush_expanded_sdf_surface_valid") is not True:
        raise OrchestrationError("water recovery did not pass side-brush surface checks")
    water_evidence_root = (
        context.run_root / "water_recovery" / "scenarios"
    ).resolve()
    raw_paths: dict[str, Path] = {}
    for scenario, expected_scenario in (
        ("normal", "normal_recovery"),
        ("full", "full_tank_fail_closed"),
    ):
        path_key = f"{scenario}_json"
        hash_key = f"{scenario}_sha256"
        candidate_path = Path(str(evidence.get(path_key, "")))
        if candidate_path.is_symlink():
            raise OrchestrationError(f"water {scenario} raw evidence is a symbolic link")
        candidate = candidate_path.resolve()
        try:
            candidate.relative_to(water_evidence_root)
        except ValueError as exc:
            raise OrchestrationError(
                f"water {scenario} raw evidence escapes the current run root"
            ) from exc
        expected_hash = evidence.get(hash_key)
        if (
            not candidate.is_file()
            or candidate.stat().st_mtime_ns < session_started_ns
            or not isinstance(expected_hash, str)
            or _sha256(candidate) != expected_hash
        ):
            raise OrchestrationError(f"water {scenario} raw evidence hash/freshness mismatch")
        raw = _read_json(candidate)
        if raw.get("scenario") != expected_scenario or raw.get("passed") is not True:
            raise OrchestrationError(f"water {scenario} raw evidence is not passing")
        raw_paths[scenario] = candidate
        results[hash_key] = expected_hash
    if raw_paths["normal"] == raw_paths["full"]:
        raise OrchestrationError("water normal/full raw evidence must be distinct")
    for scenario in ("normal", "full"):
        path_key = f"{scenario}_side_brush_surface_json"
        hash_key = f"{scenario}_side_brush_surface_sha256"
        candidate_path = Path(str(evidence.get(path_key, "")))
        if candidate_path.is_symlink():
            raise OrchestrationError(
                f"water {scenario} side-brush evidence is a symbolic link"
            )
        candidate = candidate_path.resolve()
        try:
            candidate.relative_to(water_evidence_root)
        except ValueError as exc:
            raise OrchestrationError(
                f"water {scenario} side-brush evidence escapes the current run root"
            ) from exc
        expected_file_hash = evidence.get(hash_key)
        if (
            not candidate.is_file()
            or candidate.stat().st_mtime_ns < session_started_ns
            or not isinstance(expected_file_hash, str)
            or _sha256(candidate) != expected_file_hash
        ):
            raise OrchestrationError(
                f"water {scenario} side-brush evidence hash/freshness mismatch"
            )
        surface = _read_json(candidate)
        source = surface.get("source")
        if (
            surface.get("status")
            != "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED"
            or not isinstance(source, dict)
            or source.get("mode") != "xacro_to_gz_sdf"
            or Path(str(source.get("path", ""))).resolve()
            != Path(expected_xacro_path).resolve()
            or source.get("sha256") != expected_xacro
            or surface.get("expanded_sdf_sha256") != expected_sdf
        ):
            raise OrchestrationError(
                f"water {scenario} side-brush evidence is not bound to frozen closure"
            )
        results[hash_key] = expected_file_hash
        expanded_hashes.add(str(surface.get("expanded_sdf_sha256")))
    if evidence.get("normal_side_brush_surface_json") == evidence.get(
        "full_side_brush_surface_json"
    ):
        raise OrchestrationError("water normal/full side-brush evidence must be distinct")
    if expanded_hashes != {expected_sdf} or evidence.get(
        "expanded_side_brush_sdf_sha256"
    ) != expected_sdf:
        raise OrchestrationError("water side-brush expanded-SDF identity mismatch")
    results.update(
        _validate_typed_water_transport_evidence(
            context, payload, session_started_ns, runtime_closure
        )
    )
    return results


def _validate_gate(
    context: Context,
    gate_id: str,
    session_started_ns: int,
    runtime_closure: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    contract = _read_contract(context.root)
    row = contract["evidence_gates"][gate_id]
    path = context.root / row["path"]
    if not path.is_file():
        raise OrchestrationError(f"gate {gate_id} did not produce {path}")
    if path.stat().st_mtime_ns < session_started_ns:
        raise OrchestrationError(f"gate {gate_id} evidence predates the current session")
    payload = _read_json(path)
    if payload.get("status") not in row["success_statuses"]:
        raise OrchestrationError(f"gate {gate_id} has non-passing status {payload.get('status')}")
    if row.get("report_id") is not None and payload.get("report_id") != row["report_id"]:
        raise OrchestrationError(f"gate {gate_id} report_id mismatch")
    for dotted, expected in row.get("required_values", {}).items():
        if not _strict_json_equal(_nested(payload, dotted), expected):
            raise OrchestrationError(f"gate {gate_id} required value mismatch: {dotted}")
    for dotted, expected in row.get("required_mapping_keys", {}).items():
        actual = _nested(payload, dotted)
        if not isinstance(actual, dict) or set(actual) != set(map(str, expected)):
            raise OrchestrationError(f"gate {gate_id} mapping-key mismatch: {dotted}")
    for dotted, requirements in row.get("required_mapping_item_values", {}).items():
        actual = _nested(payload, dotted)
        if not isinstance(actual, dict) or not isinstance(requirements, dict):
            raise OrchestrationError(f"gate {gate_id} mapping-item contract invalid: {dotted}")
        for item_name, item in actual.items():
            if not isinstance(item, dict) or any(
                not _strict_json_equal(_nested(item, str(field)), expected)
                for field, expected in requirements.items()
            ):
                raise OrchestrationError(
                    f"gate {gate_id} mapping-item mismatch: {dotted}.{item_name}"
                )
    list_contracts = (
        ("required_list_item_values", "equal"),
        ("required_list_item_minimums", "minimum"),
        ("required_list_item_maximums", "maximum"),
    )
    for contract_key, comparison in list_contracts:
        requirements_by_list = row.get(contract_key, {})
        if not isinstance(requirements_by_list, dict):
            raise OrchestrationError(
                f"gate {gate_id} {contract_key} is not a mapping"
            )
        for dotted, requirements in requirements_by_list.items():
            actual = _nested(payload, str(dotted))
            if not isinstance(actual, list) or not actual or not isinstance(requirements, dict):
                raise OrchestrationError(
                    f"gate {gate_id} list-item contract invalid: {dotted}"
                )
            for index, item in enumerate(actual):
                if not isinstance(item, dict):
                    raise OrchestrationError(
                        f"gate {gate_id} list-item mismatch: {dotted}[{index}]"
                    )
                for field, expected in requirements.items():
                    value = _nested(item, str(field))
                    if comparison == "equal":
                        matched = _strict_json_equal(value, expected)
                    else:
                        matched = (
                            _finite_contract_number(value)
                            and _finite_contract_number(expected)
                            and (
                                value >= expected
                                if comparison == "minimum"
                                else value <= expected
                            )
                        )
                    if not matched:
                        raise OrchestrationError(
                            f"gate {gate_id} list-item mismatch: {dotted}[{index}]"
                        )
    bound_file_mapping = row.get("bound_file_mapping")
    if bound_file_mapping is not None:
        mapping = _nested(payload, str(bound_file_mapping))
        if not isinstance(mapping, dict) or not mapping:
            raise OrchestrationError(f"gate {gate_id} bound-file mapping is empty")
        evidence_root = path.parent.resolve()
        for item_name, item in mapping.items():
            if not isinstance(item, dict):
                raise OrchestrationError(f"gate {gate_id} invalid bound file: {item_name}")
            relative = item.get("path")
            candidate = (evidence_root / str(relative)).resolve()
            if (
                not isinstance(relative, str)
                or candidate.parent != evidence_root
                or not candidate.is_file()
                or candidate.stat().st_mtime_ns < session_started_ns
                or candidate.stat().st_size != item.get("png_size_bytes")
                or _sha256(candidate) != item.get("png_sha256")
            ):
                raise OrchestrationError(f"gate {gate_id} bound file mismatch: {item_name}")
    identity = _snapshot_identity(context)
    fields = (
        ("snapshot_manifest_hash_field", "snapshot_manifest_sha256"),
        ("snapshot_urdf_hash_field", "expanded_urdf_sha256"),
        ("snapshot_source_hash_field", "source_inventory_sha256"),
    )
    for field_name, identity_key in fields:
        dotted = row.get(field_name)
        if dotted is not None and _nested(payload, dotted) != identity[identity_key]:
            raise OrchestrationError(f"gate {gate_id} is not bound to the frozen {identity_key}")
    _validate_runtime_binding(
        context, gate_id, row, payload, path, session_started_ns
    )
    if gate_id == "physical_grasp_and_bin" and row.get(
        "preembedded_grasp_binding"
    ) is not None:
        _validate_preembedded_grasp_evidence(
            context, row, payload, path, session_started_ns
        )
    surface_hashes = (
        _validate_water_surface_evidence(
            context, payload, session_started_ns, runtime_closure
        )
        if gate_id == "water_recovery"
        else {}
    )
    return {
        "gate": gate_id,
        "path": str(path.relative_to(context.root)).replace("\\", "/"),
        "status": payload.get("status"),
        "sha256": _sha256(path),
        **surface_hashes,
    }


def _gate_path(context: Context, gate_id: str) -> Path:
    contract = _read_contract(context.root)
    row = contract["evidence_gates"].get(gate_id)
    if not isinstance(row, dict) or not isinstance(row.get("path"), str):
        raise OrchestrationError(f"gate {gate_id} has no valid contract path")
    return context.root / row["path"]


def _s100_final_available(context: Context, path: Path) -> bool:
    """Return whether a regular S100 final exists, requiring operator trust to pass it."""

    if path.is_symlink():
        raise OrchestrationError("S100 final artifact path must not be a symbolic link")
    if not path.exists():
        return False
    if not path.is_file():
        raise OrchestrationError("S100 final artifact path is not a regular file")
    if not context.accept_operator_trusted_s100:
        raise OrchestrationError(
            "S100 final artifact requires explicit acceptance of operator-trusted evidence"
        )
    return True


def _local_gate_ids(context: Context) -> set[str]:
    return set(map(str, _read_contract(context.root)["evidence_gates"])) - {
        EXTERNAL_GATE
    }


def _repo_regular_file(root: Path, candidate: Path, label: str) -> Path:
    """Resolve one retained evidence file without allowing links or root escape."""

    root = root.resolve()
    lexical = candidate.absolute()
    current = lexical
    while True:
        if current.is_symlink():
            raise OrchestrationError(f"{label} contains a symbolic-link path component")
        if current == current.parent:
            break
        current = current.parent
    if not lexical.is_file():
        raise OrchestrationError(f"{label} is missing or not a regular file")
    resolved = lexical.resolve()
    try:
        relative = resolved.relative_to(root)
    except ValueError as exc:
        raise OrchestrationError(f"{label} escapes the repository-controlled evidence root") from exc
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise OrchestrationError(f"{label} contains a symbolic-link path component")
    return resolved


def _validate_run_root(context: Context, *, require_exists: bool) -> Path:
    """Keep every formal run in one non-symlinked, repository-local namespace."""

    lexical = context.requested_run_root.absolute()
    current = lexical
    while True:
        if current.is_symlink():
            raise OrchestrationError("formal run_root contains a symbolic-link path component")
        if current == current.parent:
            break
        current = current.parent
    resolved = lexical.resolve()
    parent = (context.root / FORMAL_RUN_ROOT_PARENT.relative_to(ROOT)).resolve()
    try:
        relative = resolved.relative_to(parent)
    except ValueError as exc:
        raise OrchestrationError(
            f"formal run_root must be inside {parent}"
        ) from exc
    if not relative.parts or resolved == parent:
        raise OrchestrationError("formal run_root must name a child directory")
    if resolved != context.run_root:
        raise OrchestrationError("formal run_root resolution changed after context setup")
    if require_exists:
        if not resolved.is_dir():
            raise OrchestrationError("resume run_root does not exist as a regular directory")
    elif resolved.exists():
        raise OrchestrationError("execute run_root must be a fresh nonexistent directory")
    return resolved


def _repo_relative_evidence_path(root: Path, value: str, label: str) -> tuple[Path, str]:
    """Accept only one canonical repo-relative hand-off path."""

    candidate = Path(value)
    if candidate.is_absolute() or not value or ".." in candidate.parts:
        raise OrchestrationError(f"{label} path must be repository-root-relative")
    resolved = _repo_regular_file(root, root / candidate, label)
    return resolved, resolved.relative_to(root.resolve()).as_posix()


def _validate_s100_external_evidence(
    context: Context,
    session_started_ns: int,
    revalidation_output: Path,
    log_path: Path,
) -> dict[str, Any]:
    """Revalidate retained board evidence; a final JSON alone is never trusted."""

    final_path = _repo_regular_file(
        context.root, _gate_path(context, EXTERNAL_GATE), "S100 final artifact"
    )
    if final_path.stat().st_mtime_ns < session_started_ns:
        raise OrchestrationError("S100 final artifact predates the acceptance session")
    payload = _read_json(final_path)
    raw_evidence = payload.get("raw_evidence")
    if not isinstance(raw_evidence, dict):
        raise OrchestrationError("S100 final artifact has no raw evidence binding")
    raw_value = raw_evidence.get("path")
    raw_hash = raw_evidence.get("sha256")
    if not isinstance(raw_value, str) or not isinstance(raw_hash, str):
        raise OrchestrationError("S100 raw evidence path or digest is invalid")
    raw_path, raw_relative = _repo_relative_evidence_path(
        context.root, raw_value, "S100 raw collector artifact"
    )
    if raw_path.stat().st_mtime_ns < session_started_ns:
        raise OrchestrationError("S100 raw collector artifact predates the acceptance session")
    if _sha256(raw_path) != raw_hash:
        raise OrchestrationError("S100 raw collector artifact digest changed")
    if revalidation_output.exists() or revalidation_output.is_symlink():
        raise OrchestrationError(
            f"refusing to overwrite retained S100 revalidation report: {revalidation_output}"
        )
    revalidation_output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        str(context.root / "scripts/validate_formal_s100_live_runtime.py"),
        "--raw",
        raw_relative,
        "--snapshot",
        str(context.snapshot),
        "--acceptance-session",
        str(context.session),
        "--runtime-closure",
        str(context.runtime_closure_manifest),
        "--output",
        str(revalidation_output),
    ]
    with log_path.open("w", encoding="utf-8") as stream:
        result = subprocess.run(
            command,
            cwd=context.root,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
        )
    if result.returncode != 0:
        raise OrchestrationError(
            f"S100 raw collector revalidation failed rc={result.returncode}; log={log_path}"
        )
    revalidated_path = _repo_regular_file(
        context.root, revalidation_output, "S100 revalidation report"
    )
    revalidated = _read_json(revalidated_path)
    revalidated_raw = _nested(revalidated, "raw_evidence.path")
    if revalidated_raw != raw_relative or not _strict_json_equal(payload, revalidated):
        raise OrchestrationError(
            "S100 final artifact is not the exact derived result of a live collector artifact"
        )
    return {
        **_validate_gate(context, EXTERNAL_GATE, session_started_ns),
        "raw_path": raw_relative,
        "raw_sha256": _sha256(raw_path),
        "revalidation_path": str(revalidated_path.relative_to(context.root)).replace("\\", "/"),
        "revalidation_sha256": _sha256(revalidated_path),
    }


def _local_gate_digests_from_report(
    context: Context,
    report: Mapping[str, Any],
    *,
    allow_incomplete_terminal_steps: bool = False,
) -> dict[str, dict[str, Any]]:
    """Return the one recorded digest for each local contract gate, fail closed."""

    steps = report.get("steps")
    if not isinstance(steps, list):
        raise OrchestrationError("orchestration report has no steps list")
    by_id: dict[str, Mapping[str, Any]] = {}
    for row in steps:
        if not isinstance(row, dict) or not isinstance(row.get("id"), str):
            raise OrchestrationError("orchestration report has an invalid step row")
        step_id = row["id"]
        if step_id in by_id:
            raise OrchestrationError(f"orchestration report duplicates step {step_id}")
        by_id[step_id] = row
    expected_order = [step.step_id for step in STEP_SPECS]
    actual_order = [row.get("id") for row in steps]
    if allow_incomplete_terminal_steps:
        terminal_start = expected_order.index("s100_live")
        if (
            len(actual_order) < terminal_start
            or actual_order != expected_order[:len(actual_order)]
        ):
            raise OrchestrationError(
                "recoverable S100-committed report does not preserve the local-step prefix"
            )
    elif actual_order != expected_order:
        raise OrchestrationError("orchestration report does not preserve the 31-step contract")
    digests: dict[str, dict[str, Any]] = {}
    for step in STEP_SPECS:
        local_gates = tuple(gate for gate in step.produces_gates if gate != EXTERNAL_GATE)
        if not local_gates:
            continue
        row = by_id[step.step_id]
        if row.get("status") != "PASSED":
            raise OrchestrationError(f"local step is not retained as passed: {step.step_id}")
        evidence = row.get("evidence")
        if not isinstance(evidence, list):
            raise OrchestrationError(f"local step has no evidence list: {step.step_id}")
        for item in evidence:
            if not isinstance(item, dict) or not isinstance(item.get("gate"), str):
                raise OrchestrationError(f"local step has an invalid digest: {step.step_id}")
            gate = item["gate"]
            if gate in digests:
                raise OrchestrationError(f"orchestration report duplicates gate digest {gate}")
            digests[gate] = item
    expected_gates = _local_gate_ids(context)
    if set(digests) != expected_gates:
        raise OrchestrationError(
            "orchestration report does not retain exactly "
            f"{len(expected_gates)} local gate digests"
        )
    recorded = report.get("local_gate_digests")
    if recorded is not None and not _strict_json_equal(recorded, digests):
        raise OrchestrationError("orchestration report local gate digest index drifted")
    return digests


def _revalidate_local_gate_digests(
    context: Context,
    report: Mapping[str, Any],
    session_started_ns: int,
    runtime_closure: Mapping[str, Any],
    *,
    allow_incomplete_terminal_steps: bool = False,
) -> dict[str, dict[str, Any]]:
    expected = _local_gate_digests_from_report(
        context,
        report,
        allow_incomplete_terminal_steps=allow_incomplete_terminal_steps,
    )
    actual = {
        gate: _validate_gate(
            context,
            gate,
            session_started_ns,
            runtime_closure=runtime_closure if gate == "water_recovery" else None,
        )
        for gate in sorted(expected)
    }
    for gate in sorted(expected):
        if not _strict_json_equal(actual[gate], expected[gate]):
            raise OrchestrationError(f"local gate artifact or digest drifted: {gate}")
    return actual


def _current_session_bound_files(
    evidence_path: Path,
    payload: Mapping[str, Any],
    gate_contract: Mapping[str, Any],
    session_started_ns: int,
) -> list[dict[str, Any]]:
    mapping_field = gate_contract.get("bound_file_mapping")
    if mapping_field is None:
        return []
    mapping = _nested(payload, str(mapping_field))
    if not isinstance(mapping, dict) or not mapping:
        raise OrchestrationError("session-bound file mapping is empty during complete-session verification")
    base = evidence_path.parent.resolve()
    rows: list[dict[str, Any]] = []
    for name, item in sorted(mapping.items()):
        if not isinstance(item, dict):
            raise OrchestrationError("session-bound file mapping has an invalid row")
        relative = item.get("path")
        expected_hash = item.get("png_sha256")
        expected_size = item.get("png_size_bytes")
        if not isinstance(relative, str) or not isinstance(expected_hash, str) or not isinstance(expected_size, int):
            raise OrchestrationError("session-bound file mapping lacks a complete binding")
        candidate = (base / relative).resolve()
        if (
            candidate.parent != base
            or not candidate.is_file()
            or candidate.stat().st_mtime_ns < session_started_ns
            or candidate.stat().st_size != expected_size
            or _sha256(candidate) != expected_hash
        ):
            raise OrchestrationError("session-bound file changed after complete session finalization")
        rows.append(
            {
                "name": str(name),
                "path": relative,
                "sha256": expected_hash,
                "size_bytes": expected_size,
            }
        )
    return rows


def _verify_complete_session_evidence(
    context: Context,
    session: Mapping[str, Any],
    session_started_ns: int,
    gate_results: Mapping[str, Mapping[str, Any]],
) -> None:
    """Re-hash all 25 retained session-bound gate rows before phase-two closeout."""

    contract = _read_contract(context.root)
    expected_gates = {
        str(gate_id)
        for gate_id, row in contract["evidence_gates"].items()
        if isinstance(row, dict) and row.get("session_bound") is True
    }
    evidence = session.get("evidence")
    if (
        session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
        or session.get("failures") != {}
        or not isinstance(evidence, dict)
        or set(evidence) != expected_gates
        or set(gate_results) != expected_gates
    ):
        raise OrchestrationError("complete session does not retain exactly the 25 passing gate rows")
    for gate_id in sorted(expected_gates):
        gate_contract = contract["evidence_gates"][gate_id]
        entry = evidence.get(gate_id)
        result = gate_results[gate_id]
        if not isinstance(entry, dict):
            raise OrchestrationError(f"complete session evidence row is invalid: {gate_id}")
        path = _gate_path(context, gate_id)
        payload = _read_json(path)
        expected = {
            "path": str(gate_contract["path"]),
            "sha256": result.get("sha256"),
            "status": result.get("status"),
            "mtime_epoch_ns": path.stat().st_mtime_ns,
            "bound_files": _current_session_bound_files(
                path, payload, gate_contract, session_started_ns
            ),
        }
        if not _strict_json_equal(entry, expected):
            raise OrchestrationError(
                f"complete session evidence digest or binding drifted: {gate_id}"
            )


def _upsert_terminal_step(
    report: dict[str, Any],
    row: dict[str, Any],
    *,
    resuming: bool,
    allow_missing_terminal_step: bool = False,
) -> None:
    """Keep the 31-step report semantic stable while replacing a terminal row."""

    steps = report.get("steps")
    if not isinstance(steps, list):
        raise OrchestrationError("orchestration report has no mutable steps list")
    matches = [index for index, item in enumerate(steps) if item.get("id") == row["id"]]
    if len(matches) > 1:
        raise OrchestrationError(f"orchestration report duplicates terminal step {row['id']}")
    if not matches:
        if resuming and not allow_missing_terminal_step:
            raise OrchestrationError(f"resume report is missing terminal step {row['id']}")
        if resuming:
            expected_order = [step.step_id for step in STEP_SPECS]
            actual_order = [item.get("id") for item in steps]
            if (
                actual_order != expected_order[:len(actual_order)]
                or len(actual_order) >= len(expected_order)
                or expected_order[len(actual_order)] != row["id"]
            ):
                raise OrchestrationError(
                    f"resume cannot append terminal step out of contract order: {row['id']}"
                )
        steps.append(row)
        return
    previous = steps[matches[0]]
    if not isinstance(previous, dict):
        raise OrchestrationError(f"orchestration report has invalid terminal step {row['id']}")
    if not resuming:
        raise OrchestrationError(f"new execution unexpectedly already has terminal step {row['id']}")
    row["previous_status"] = previous.get("status")
    steps[matches[0]] = row


def _run_session_finalize(
    context: Context,
    log_path: Path,
    expected_failures: Mapping[str, str],
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(context.root / "scripts/formal_acceptance_session.py"),
        "finalize",
        "--contract",
        str(context.root / CONTRACT.relative_to(ROOT)),
        "--snapshot",
        str(context.snapshot),
        "--output",
        str(context.session),
        "--root",
        str(context.root),
    ]
    with log_path.open("w", encoding="utf-8") as stream:
        result = subprocess.run(
            command,
            cwd=context.root,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
        )
    if result.returncode not in {0, 3}:
        raise OrchestrationError(
            f"session finalization failed rc={result.returncode}; log={log_path}"
        )
    session = _read_json(context.session)
    if session.get("failures") != dict(expected_failures):
        raise OrchestrationError(
            f"session has unexpected failures: {session.get('failures')}"
        )
    expected_status = (
        "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
        if not expected_failures
        else "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    )
    if session.get("status") != expected_status:
        raise OrchestrationError(
            f"session finalization returned unexpected status: {session.get('status')}"
        )
    return session


def _functional_audit_path(context: Context) -> Path:
    return context.root / FUNCTIONAL_AUDIT.relative_to(ROOT)


def _run_functional_aggregate(
    context: Context, log_path: Path, *, external_present: bool
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(context.root / "scripts/validate_formal_functional_acceptance_contract.py"),
        "--output",
        str(_functional_audit_path(context)),
        "--require-all",
    ]
    with log_path.open("w", encoding="utf-8") as stream:
        result = subprocess.run(
            command,
            cwd=context.root,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
        )
    expected_rc = 0 if external_present else 3
    if result.returncode != expected_rc:
        raise OrchestrationError(
            f"functional aggregate failed rc={result.returncode}; log={log_path}"
        )
    functional = _read_json(_functional_audit_path(context))
    if external_present:
        if functional.get("complete") is not True:
            raise OrchestrationError("all evidence exists but the functional aggregate is not complete")
    elif functional.get("unresolved_mission_gates") != [EXTERNAL_GATE]:
        raise OrchestrationError(
            "local closure left mission blockers other than the external S100 gate"
        )
    return functional


def _snapshot_check(context: Context, log_path: Path) -> None:
    _run_process(
        [sys.executable, str(context.root / "scripts/generate_formal_vehicle_snapshot.py"), "--check"],
        {},
        log_path,
    )


def _verify_runtime_closure(context: Context, phase: str) -> dict[str, Any]:
    """Verify the same immutable merged runtime around every formal step."""

    if context.runtime_closure_manifest is None:
        raise OrchestrationError("final runtime closure manifest is not configured")
    try:
        result = verify_runtime_closure_manifest(
            context.runtime_closure_manifest,
            context.root,
            context.runtime_ws,
            context.perception_artifacts,
            context.onnx_pythonpath,
        )
    except (ClosureError, OSError, ValueError) as exc:
        raise OrchestrationError(f"runtime closure verification failed at {phase}: {exc}") from exc
    if result.get("windows_cold_start_evidence_bound") is not True:
        raise OrchestrationError(
            f"runtime closure has no bound Windows cold-start evidence at {phase}"
        )
    _bound_nvidia_egl_environment(context, result)
    return {**result, "phase": phase, "verified_epoch_ns": time.time_ns()}


def _initial_s100_commit_is_recoverable(
    context: Context, report: Mapping[str, Any]
) -> bool:
    """Return true only after the initial run durably committed the S100 session.

    This intentionally does not treat a generic orchestration exception as
    recoverable.  The retained session must already be COMPLETE with no
    failures, and the report must retain the separately validated S100 PASS
    row.  Resume revalidates all of that evidence again before any closeout.
    """

    s100_row = _terminal_row_optional(report, "s100_live")
    if s100_row is None or s100_row.get("status") != "PASSED":
        return False
    try:
        session = _read_json(context.session)
    except (OSError, ValueError):
        return False
    return (
        session.get("status") == "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
        and session.get("failures") == {}
    )


def _failed_step_row(
    *,
    step_id: str,
    mode: str,
    started_epoch_ns: int,
    log: Path | None,
    evidence_paths: Sequence[Path],
    blocker: str,
    reason: Exception,
    runtime_closure_before: Mapping[str, Any] | None,
    runtime_closure_after: Mapping[str, Any] | None,
    post_failure_closure_error: Exception | None = None,
) -> dict[str, Any]:
    """Create an explicit, durable fail-closed record for one attempted step."""

    message = str(reason)
    match = re.search(r"\brc=(-?\d+)\b", message)
    exit_code = int(match.group(1)) if match else None
    if isinstance(reason, MemoryLimitError):
        exit_status = "MEMORY_LIMIT_BREACHED"
    elif exit_code is not None:
        exit_status = "COMMAND_FAILED"
    else:
        exit_status = "ORCHESTRATION_ERROR"
    row: dict[str, Any] = {
        "id": step_id,
        "mode": mode,
        "status": "FAILED",
        "started_epoch_ns": started_epoch_ns,
        "finished_epoch_ns": time.time_ns(),
        "log": str(log) if log is not None else None,
        "evidence_paths": [str(path) for path in evidence_paths],
        "exit_status": exit_status,
        "exit_code": exit_code,
        "blocker": blocker,
        "reason": message,
        "evidence": [],
        "runtime_closure_before": runtime_closure_before,
        "runtime_closure_after": runtime_closure_after,
    }
    if post_failure_closure_error is not None:
        row["post_failure_closure_error"] = str(post_failure_closure_error)
    return row


def execute(context: Context) -> tuple[dict[str, Any], int]:
    try:
        _validate_run_root(context, require_exists=False)
    except OrchestrationError as exc:
        return {
            "status": "FORMAL_FINAL_ACCEPTANCE_EXECUTION_REFUSED_RUN_ROOT",
            "run_root": str(context.run_root),
            "error": str(exc),
        }, 2
    gate = preflight(context)
    if not gate["passed"]:
        return {**gate, "status": "FORMAL_FINAL_ACCEPTANCE_EXECUTION_REFUSED_BY_PREFLIGHT"}, 2
    try:
        archive_result = execute_final_output_archive(context, gate["archive_plan"])
    except (KeyError, OSError, OrchestrationError, ValueError) as exc:
        return {
            **gate,
            "status": "FORMAL_FINAL_ACCEPTANCE_ARCHIVE_FAILED_CLOSED",
            "error": str(exc),
        }, 2
    context.run_root.mkdir(parents=True)
    logs = context.run_root / "orchestration_logs"
    report: dict[str, Any] = {
        "report_id": "tzcup_formal_final_acceptance_orchestration_runtime_v1",
        "status": "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_RUNNING",
        "started_epoch_ns": time.time_ns(),
        "run_root": str(context.run_root),
        "serial_execution": True,
        "old_evidence_reuse_allowed": False,
        "runtime_closure_manifest": str(context.runtime_closure_manifest),
        "runtime_closure_manifest_sha256": _sha256(context.runtime_closure_manifest),
        "runtime_closure_verified_before_and_after_every_step": True,
        "resource_gate_rechecked_before_each_heavy_phase": True,
        "heavy_runtime_step_ids": sorted(HEAVY_RUNTIME_STEP_IDS),
        "integrated_source_build_preflight_timeout_seconds": (
            context.integrated_source_build_preflight_timeout_seconds
        ),
        "strict_serial_execution": True,
        "stop_on_first_step_failure": True,
        "cad_execution_permitted": False,
        "board_execution_automatic": False,
        "s100_evidence_trust_boundary": S100_EVIDENCE_TRUST_BOUNDARY,
        "previous_final_outputs": archive_result,
        "steps": [],
    }
    _atomic_json(context.run_root / "orchestration_report.json", report)
    session_started_ns = 0
    try:
        for step in STEP_SPECS:
            if step.step_id in {"s100_live", "finalize_session", "functional_aggregate"}:
                continue
            started = time.time_ns()
            log = logs / f"{len(report['steps']):02d}_{step.step_id}.log"
            evidence_paths: list[Path] = []
            closure_before: dict[str, Any] | None = None
            closure_after: dict[str, Any] | None = None
            step_error: Exception | None = None
            step_blocker = "runtime_closure_before"
            post_failure_closure_error: Exception | None = None
            evidence: list[dict[str, Any]] = []
            try:
                closure_before = _verify_runtime_closure(context, f"before:{step.step_id}")
                step_blocker = "command"
                command, environment = _step_command(
                    step.step_id,
                    context,
                    runtime_closure=closure_before,
                    execution_environment=True,
                )
                if command and command[0] == "__sequence__":
                    commands = json.loads(command[1])
                    for sequence_index, sequence_command in enumerate(commands):
                        sequence_log = log.with_name(
                            f"{log.stem}_{sequence_index}{log.suffix}"
                        )
                        evidence_paths.append(sequence_log)
                        step_blocker = f"command:{sequence_index}"
                        _run_process(
                            sequence_command,
                            environment,
                            sequence_log,
                            wrap_lock=step.requires_outer_gazebo_lock,
                            memory_watchdog=_requires_resource_gate(step),
                        )
                else:
                    evidence_paths.append(log)
                    _run_process(
                        command,
                        environment,
                        log,
                        wrap_lock=step.requires_outer_gazebo_lock,
                        memory_watchdog=_requires_resource_gate(step),
                        )
                if step.step_id == "start_session":
                    step_blocker = "session_start"
                    session_payload = _read_json(context.session)
                    session_started_ns = int(session_payload.get("started_epoch_ns", 0))
                    if session_payload.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING" or session_started_ns <= 0:
                        raise OrchestrationError("session start did not produce a valid RUNNING session")
                    report["session_path"] = str(context.session)
                    report["snapshot_path"] = str(context.snapshot)
                    report["snapshot_identity"] = session_payload.get("snapshot")
                if step.mode == "gazebo":
                    step_blocker = "gazebo_lock_release"
                    lock_ok, lock_error = _lock_available()
                    if not lock_ok:
                        raise OrchestrationError(f"Gazebo lock leaked after {step.step_id}: {lock_error}")
                if session_started_ns:
                    step_blocker = "snapshot_check"
                    _snapshot_check(context, logs / f"{len(report['steps']):02d}_{step.step_id}_snapshot_check.log")
                step_blocker = "gate_validation"
                evidence = [
                    _validate_gate(
                        context,
                        gate_id,
                        session_started_ns,
                        runtime_closure=closure_before,
                    )
                    for gate_id in step.produces_gates
                ]
            except Exception as exc:  # verify post-state even after a failed runner
                step_error = exc
            try:
                closure_after = _verify_runtime_closure(context, f"after:{step.step_id}")
            except Exception as exc:  # preserve the runner error, but record closure drift too
                if step_error is None:
                    step_error = exc
                    step_blocker = "runtime_closure_after"
                else:
                    post_failure_closure_error = exc
            if step_error is not None:
                report["steps"].append(
                    _failed_step_row(
                        step_id=step.step_id,
                        mode=step.mode,
                        started_epoch_ns=started,
                        log=log,
                        evidence_paths=evidence_paths,
                        blocker=step_blocker,
                        reason=step_error,
                        runtime_closure_before=closure_before,
                        runtime_closure_after=closure_after,
                        post_failure_closure_error=post_failure_closure_error,
                    )
                )
                _atomic_json(context.run_root / "orchestration_report.json", report)
                raise step_error
            report["steps"].append({
                "id": step.step_id,
                "mode": step.mode,
                "status": "PASSED",
                "started_epoch_ns": started,
                "finished_epoch_ns": time.time_ns(),
                "log": str(log),
                "evidence": evidence,
                "runtime_closure_before": closure_before,
                "runtime_closure_after": closure_after,
            })
            _atomic_json(context.run_root / "orchestration_report.json", report)

        s100_started = time.time_ns()
        s100_closure_before: dict[str, Any] | None = None
        s100_closure_after: dict[str, Any] | None = None
        s100_path: Path | None = None
        s100_log = logs / "s100_revalidation_initial.log"
        s100_blocker = "runtime_closure_before"
        s100_error: Exception | None = None
        s100_post_failure_closure_error: Exception | None = None
        try:
            s100_closure_before = _verify_runtime_closure(context, "before:s100_live")
            s100_blocker = "external_evidence_availability"
            s100_path = _gate_path(context, EXTERNAL_GATE)
            s100_available = _s100_final_available(context, s100_path)
            if s100_available:
                s100_blocker = "external_evidence_validation"
                s100_evidence = [_validate_s100_external_evidence(
                    context,
                    session_started_ns,
                    context.run_root / "s100_revalidation_initial.json",
                    s100_log,
                )]
                s100_status = "PASSED"
            else:
                s100_evidence = []
                s100_status = "BLOCKED_EXTERNAL_HARD_GATE"
        except Exception as exc:
            s100_error = exc
        try:
            s100_closure_after = _verify_runtime_closure(context, "after:s100_live")
        except Exception as exc:
            if s100_error is None:
                s100_error = exc
                s100_blocker = "runtime_closure_after"
            else:
                s100_post_failure_closure_error = exc
        if s100_error is not None:
            _upsert_terminal_step(report, _failed_step_row(
                step_id="s100_live",
                mode="external",
                started_epoch_ns=s100_started,
                log=s100_log,
                evidence_paths=[path for path in (s100_path, s100_log) if path is not None],
                blocker=s100_blocker,
                reason=s100_error,
                runtime_closure_before=s100_closure_before,
                runtime_closure_after=s100_closure_after,
                post_failure_closure_error=s100_post_failure_closure_error,
            ), resuming=False)
            _atomic_json(context.run_root / "orchestration_report.json", report)
            raise s100_error
        s100_row = {
            "id": "s100_live",
            "mode": "external",
            "status": s100_status,
            "started_epoch_ns": s100_started,
            "finished_epoch_ns": time.time_ns(),
            "evidence": s100_evidence,
            "runtime_closure_before": s100_closure_before,
            "runtime_closure_after": s100_closure_after,
        }
        if not s100_available:
            s100_row["reason"] = "No fresh live RDK S100P / Journey 6P board artifact was produced after session start; PC substitution is prohibited."
        _upsert_terminal_step(report, s100_row, resuming=False)

        finalize_started = time.time_ns()
        finalize_closure_before: dict[str, Any] | None = None
        finalize_closure_after: dict[str, Any] | None = None
        finalize_log = logs / "finalize_session.log"
        finalize_blocker = "runtime_closure_before"
        finalize_error: Exception | None = None
        finalize_post_failure_closure_error: Exception | None = None
        try:
            finalize_closure_before = _verify_runtime_closure(context, "before:finalize_session")
            finalize_blocker = "session_finalization"
            session = _run_session_finalize(
                context,
                finalize_log,
                {} if s100_available else {EXTERNAL_GATE: "missing"},
            )
        except Exception as exc:
            finalize_error = exc
        try:
            finalize_closure_after = _verify_runtime_closure(context, "after:finalize_session")
        except Exception as exc:
            if finalize_error is None:
                finalize_error = exc
                finalize_blocker = "runtime_closure_after"
            else:
                finalize_post_failure_closure_error = exc
        if finalize_error is not None:
            _upsert_terminal_step(report, _failed_step_row(
                step_id="finalize_session",
                mode="static",
                started_epoch_ns=finalize_started,
                log=finalize_log,
                evidence_paths=[finalize_log, context.session],
                blocker=finalize_blocker,
                reason=finalize_error,
                runtime_closure_before=finalize_closure_before,
                runtime_closure_after=finalize_closure_after,
                post_failure_closure_error=finalize_post_failure_closure_error,
            ), resuming=False)
            _atomic_json(context.run_root / "orchestration_report.json", report)
            raise finalize_error
        _upsert_terminal_step(report, {
            "id": "finalize_session",
            "mode": "static",
            "status": "PASSED",
            "started_epoch_ns": finalize_started,
            "finished_epoch_ns": time.time_ns(),
            "log": str(finalize_log),
            "evidence": [],
            "runtime_closure_before": finalize_closure_before,
            "runtime_closure_after": finalize_closure_after,
        }, resuming=False)

        aggregate_started = time.time_ns()
        aggregate_closure_before: dict[str, Any] | None = None
        aggregate_closure_after: dict[str, Any] | None = None
        audit_log = logs / "functional_aggregate.log"
        aggregate_blocker = "runtime_closure_before"
        aggregate_error: Exception | None = None
        aggregate_post_failure_closure_error: Exception | None = None
        try:
            aggregate_closure_before = _verify_runtime_closure(
                context, "before:functional_aggregate"
            )
            aggregate_blocker = "functional_aggregate"
            functional = _run_functional_aggregate(
                context, audit_log, external_present=s100_available
            )
        except Exception as exc:
            aggregate_error = exc
        try:
            aggregate_closure_after = _verify_runtime_closure(
                context, "after:functional_aggregate"
            )
        except Exception as exc:
            if aggregate_error is None:
                aggregate_error = exc
                aggregate_blocker = "runtime_closure_after"
            else:
                aggregate_post_failure_closure_error = exc
        if aggregate_error is not None:
            _upsert_terminal_step(report, _failed_step_row(
                step_id="functional_aggregate",
                mode="static",
                started_epoch_ns=aggregate_started,
                log=audit_log,
                evidence_paths=[audit_log, _functional_audit_path(context)],
                blocker=aggregate_blocker,
                reason=aggregate_error,
                runtime_closure_before=aggregate_closure_before,
                runtime_closure_after=aggregate_closure_after,
                post_failure_closure_error=aggregate_post_failure_closure_error,
            ), resuming=False)
            _atomic_json(context.run_root / "orchestration_report.json", report)
            raise aggregate_error
        _upsert_terminal_step(report, {
            "id": "functional_aggregate",
            "mode": "static",
            "status": "PASSED" if s100_available else "PASSED_WITH_EXTERNAL_S100_BLOCK",
            "started_epoch_ns": aggregate_started,
            "finished_epoch_ns": time.time_ns(),
            "log": str(audit_log),
            "evidence": [],
            "runtime_closure_before": aggregate_closure_before,
            "runtime_closure_after": aggregate_closure_after,
        }, resuming=False)
        report["local_gate_digests"] = _local_gate_digests_from_report(context, report)
        report.update(
            finished_epoch_ns=time.time_ns(),
            session_status=session.get("status"),
            functional_status=functional.get("status"),
            passed_position_count=functional.get("passed_position_count"),
            pending_position_count=functional.get("pending_position_count"),
        )
        if s100_available:
            report["status"] = "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_COMPLETE"
            report["operator_trusted_s100_acknowledged"] = True
            exit_code = 0
        else:
            report["status"] = "FORMAL_FINAL_ACCEPTANCE_LOCAL_GATES_PASSED_S100_EXTERNAL_BLOCKED"
            exit_code = 4
        _atomic_json(context.run_root / "orchestration_report.json", report)
        _atomic_json(context.root / ORCHESTRATION_REPORT.relative_to(ROOT), report)
        return report, exit_code
    except MemoryLimitError as exc:
        report.update(
            status="FORMAL_FINAL_ACCEPTANCE_MEMORY_LIMIT_BREACHED",
            finished_epoch_ns=time.time_ns(),
            error=str(exc),
        )
        _atomic_json(context.run_root / "orchestration_report.json", report)
        _atomic_json(context.root / ORCHESTRATION_REPORT.relative_to(ROOT), report)
        return report, MEMORY_BREACH_EXIT_CODE
    except (OSError, OrchestrationError, subprocess.SubprocessError, ValueError) as exc:
        status = "FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_FAILED"
        if _initial_s100_commit_is_recoverable(context, report):
            try:
                report["local_gate_digests"] = _local_gate_digests_from_report(
                    context, report, allow_incomplete_terminal_steps=True
                )
            except OrchestrationError:
                # A report whose retained local prefix cannot be re-read is not
                # safe to resume merely because the session happened to finish.
                pass
            else:
                status = S100_COMMITTED_AGGREGATE_PENDING
        report.update(status=status, finished_epoch_ns=time.time_ns(), error=str(exc))
        _atomic_json(context.run_root / "orchestration_report.json", report)
        _atomic_json(context.root / ORCHESTRATION_REPORT.relative_to(ROOT), report)
        return report, 3


@contextmanager
def _session_resume_lock(context: Context) -> Iterable[None]:
    """Serialize terminal S100 hand-offs for one retained acceptance session."""

    if fcntl is None:
        raise OrchestrationError("S100 resume requires a POSIX session lock")
    lock_path = context.session.with_name(context.session.name + ".resume.lock")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+", encoding="utf-8") as stream:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise OrchestrationError("another S100 resume already holds this session") from exc
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def _terminal_row_optional(
    report: Mapping[str, Any], step_id: str
) -> Mapping[str, Any] | None:
    steps = report.get("steps")
    if not isinstance(steps, list):
        raise OrchestrationError("orchestration report has no steps list")
    matches = [row for row in steps if isinstance(row, dict) and row.get("id") == step_id]
    if len(matches) > 1:
        raise OrchestrationError(f"orchestration report has no unique terminal step {step_id}")
    return matches[0] if matches else None


def _terminal_row(report: Mapping[str, Any], step_id: str) -> Mapping[str, Any]:
    row = _terminal_row_optional(report, step_id)
    if row is None:
        raise OrchestrationError(f"orchestration report has no unique terminal step {step_id}")
    return row


def _json_digest(payload: Mapping[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def _json_copy(payload: Mapping[str, Any]) -> dict[str, Any]:
    return json.loads(json.dumps(payload, sort_keys=True))


def _require_session_runtime_closure_binding(
    context: Context,
    session: Mapping[str, Any],
    runtime_closure: Mapping[str, Any],
) -> None:
    """Reject a resume hand-off whose active closure differs from session start."""

    session_closure = session.get("runtime_closure_binding")
    if not isinstance(session_closure, dict):
        raise OrchestrationError("resume session has no runtime closure binding")
    required_fields = (
        "status",
        "manifest",
        "manifest_sha256",
        "closure_sha256",
        "symbolic_link_count",
    )
    if any(
        field not in session_closure or field not in runtime_closure
        for field in required_fields
    ):
        raise OrchestrationError("resume runtime closure binding is incomplete")
    if (
        any(
            not _strict_json_equal(session_closure[field], runtime_closure[field])
            for field in required_fields
        )
        or session_closure.get("runtime_install_root")
        != str(context.overlay.resolve())
    ):
        raise OrchestrationError(
            "resume runtime closure differs from the acceptance session binding"
        )


def _resume_preconditions(
    context: Context, logs: Path
) -> tuple[dict[str, Any], dict[str, Any], int, dict[str, Any]]:
    """Validate the retained local acceptance closure without executing it again."""

    if not context.accept_operator_trusted_s100:
        raise OrchestrationError(
            "resume requires explicit acceptance of operator-trusted S100 evidence"
        )
    session_file = _repo_regular_file(
        context.root, context.requested_session, "requested acceptance session"
    )
    snapshot_file = _repo_regular_file(
        context.root, context.requested_snapshot, "requested vehicle snapshot"
    )
    if session_file != context.session or snapshot_file != context.snapshot:
        raise OrchestrationError("requested session/snapshot path resolution changed")
    _validate_run_root(context, require_exists=True)
    report_path = _repo_regular_file(
        context.root,
        context.run_root / "orchestration_report.json",
        "resume orchestration report",
    )
    report = _read_json(report_path)
    report_status = report.get("status")
    if report_status not in {
        "FORMAL_FINAL_ACCEPTANCE_LOCAL_GATES_PASSED_S100_EXTERNAL_BLOCKED",
        S100_COMMITTED_AGGREGATE_PENDING,
    }:
        raise OrchestrationError(
            "resume requires the exact S100-external-blocked or S100-committed aggregate-pending orchestration status"
        )
    committed_aggregate_pending = report_status == S100_COMMITTED_AGGREGATE_PENDING
    session_path = report.get("session_path")
    snapshot_path = report.get("snapshot_path")
    if not isinstance(session_path, str) or not isinstance(snapshot_path, str):
        raise OrchestrationError("resume report lacks retained session/snapshot paths")
    report_session = _repo_regular_file(
        context.root, Path(session_path), "resume report session path"
    )
    report_snapshot = _repo_regular_file(
        context.root, Path(snapshot_path), "resume report snapshot path"
    )
    if report_session != context.session or report_snapshot != context.snapshot:
        raise OrchestrationError("resume context does not match the retained session/snapshot paths")
    session = _read_json(context.session)
    if committed_aggregate_pending:
        if (
            session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
            or session.get("failures") != {}
        ):
            raise OrchestrationError(
                "S100-committed aggregate-pending resume requires a COMPLETE session with no failures"
            )
    elif session.get("status") == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING":
        if session.get("failures") != {EXTERNAL_GATE: "missing"}:
            raise OrchestrationError("resume requires prior failures to be exactly the missing S100 gate")
    elif session.get("status") == "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE":
        if session.get("failures") != {}:
            raise OrchestrationError("complete session has non-empty failures")
    else:
        raise OrchestrationError("resume requires a PENDING or phase-two COMPLETE acceptance session")
    started_ns = session.get("started_epoch_ns")
    if not isinstance(started_ns, int) or started_ns <= 0:
        raise OrchestrationError("resume session has no valid start time")
    identity = _snapshot_identity(context)
    if not _strict_json_equal(session.get("snapshot"), identity):
        raise OrchestrationError("vehicle source snapshot changed after the retained session started")
    if not _strict_json_equal(report.get("snapshot_identity"), identity):
        raise OrchestrationError("retained orchestration snapshot identity changed")
    _snapshot_check(context, logs / "resume_snapshot_current.log")
    runtime_closure = _verify_runtime_closure(context, "resume:preconditions")
    _require_session_runtime_closure_binding(context, session, runtime_closure)
    if committed_aggregate_pending:
        _revalidate_local_gate_digests(
            context,
            report,
            started_ns,
            runtime_closure,
            allow_incomplete_terminal_steps=True,
        )
    else:
        _revalidate_local_gate_digests(context, report, started_ns, runtime_closure)
    if committed_aggregate_pending:
        if _terminal_row(report, "s100_live").get("status") != "PASSED":
            raise OrchestrationError("S100-committed report does not retain a passed S100 step")
        finalize_row = _terminal_row_optional(report, "finalize_session")
        if finalize_row is not None and finalize_row.get("status") != "PASSED":
            raise OrchestrationError("S100-committed report has a non-passed finalize row")
        aggregate_row = _terminal_row_optional(report, "functional_aggregate")
        if aggregate_row is not None and aggregate_row.get("status") not in {"PASSED", "FAILED"}:
            raise OrchestrationError("S100-committed report has a non-passed aggregate row")
    else:
        if _terminal_row(report, "s100_live").get("status") != "BLOCKED_EXTERNAL_HARD_GATE":
            raise OrchestrationError("retained S100 step is not in the external-blocked state")
        if _terminal_row(report, "finalize_session").get("status") != "PASSED":
            raise OrchestrationError("retained session-finalize step is not passed")
        if _terminal_row(report, "functional_aggregate").get("status") != "PASSED_WITH_EXTERNAL_S100_BLOCK":
            raise OrchestrationError("retained functional aggregate is not S100-blocked")
    return report, session, started_ns, runtime_closure


def resume_s100(context: Context) -> tuple[dict[str, Any], int]:
    """Close only a retained S100-blocked session; never rerun local/Gazebo work."""

    try:
        with _session_resume_lock(context):
            logs = context.run_root / "orchestration_logs"
            report, retained_session, session_started_ns, retained_closure = _resume_preconditions(context, logs)
            prior_orchestration_status = report.get("status")
            committed_aggregate_pending = (
                prior_orchestration_status == S100_COMMITTED_AGGREGATE_PENDING
            )
            resume_from_complete = (
                retained_session.get("status")
                == "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
            )
            history = report.get("resume_history", [])
            if not isinstance(history, list) or any(
                not isinstance(item, dict) for item in history
            ):
                raise OrchestrationError("resume report has an invalid resume_history")
            terminal_ids = ("s100_live", "finalize_session", "functional_aggregate")
            before_rows: dict[str, dict[str, Any] | None] = {}
            for step_id in terminal_ids:
                retained_row = _terminal_row_optional(report, step_id)
                before_rows[step_id] = (
                    _json_copy(retained_row) if retained_row is not None else None
                )
            s100_path = _gate_path(context, EXTERNAL_GATE)
            if not _s100_final_available(context, s100_path):
                return report, 4

            resume_started_ns = time.time_ns()
            s100_closure_before = _verify_runtime_closure(context, "resume:before:s100_live")
            s100_evidence = [_validate_s100_external_evidence(
                context,
                session_started_ns,
                context.run_root / f"s100_revalidation_resume_{resume_started_ns}.json",
                logs / f"s100_revalidation_resume_{resume_started_ns}.log",
            )]
            s100_closure_after = _verify_runtime_closure(context, "resume:after:s100_live")

            if resume_from_complete:
                if committed_aggregate_pending:
                    local_evidence = _revalidate_local_gate_digests(
                        context,
                        report,
                        session_started_ns,
                        retained_closure,
                        allow_incomplete_terminal_steps=True,
                    )
                else:
                    local_evidence = _revalidate_local_gate_digests(
                        context, report, session_started_ns, retained_closure
                    )
                _verify_complete_session_evidence(
                    context,
                    retained_session,
                    session_started_ns,
                    {**local_evidence, EXTERNAL_GATE: s100_evidence[0]},
                )
                session = retained_session
                if _terminal_row_optional(report, "finalize_session") is None:
                    session_finished_ns = session.get("finished_epoch_ns")
                    if not isinstance(session_finished_ns, int) or session_finished_ns <= 0:
                        raise OrchestrationError(
                            "complete session has no finish time for reconstructed finalize row"
                        )
                    _upsert_terminal_step(report, {
                        "id": "finalize_session",
                        "mode": "static",
                        "status": "PASSED",
                        "started_epoch_ns": session_started_ns,
                        "finished_epoch_ns": session_finished_ns,
                        "log": None,
                        "evidence": [],
                        "reconstructed_from_complete_session": True,
                        "reconstructed_session_status": session.get("status"),
                        "reconstructed_session_finished_epoch_ns": session_finished_ns,
                        "resume_started_epoch_ns": resume_started_ns,
                    }, resuming=True, allow_missing_terminal_step=committed_aggregate_pending)
            else:
                finalize_started_ns = time.time_ns()
                finalize_closure_before = _verify_runtime_closure(
                    context, "resume:before:finalize_session"
                )
                session = _run_session_finalize(
                    context, logs / f"finalize_session_resume_{resume_started_ns}.log", {}
                )
                finalize_closure_after = _verify_runtime_closure(
                    context, "resume:after:finalize_session"
                )

            aggregate_started_ns = time.time_ns()
            aggregate_closure_before = _verify_runtime_closure(
                context, "resume:before:functional_aggregate"
            )
            functional = _run_functional_aggregate(
                context,
                logs / f"functional_aggregate_resume_{resume_started_ns}.log",
                external_present=True,
            )
            aggregate_closure_after = _verify_runtime_closure(
                context, "resume:after:functional_aggregate"
            )
            resume_finished_ns = time.time_ns()
            resume_fields = {
                "resume_started_epoch_ns": resume_started_ns,
                "resume_finished_epoch_ns": resume_finished_ns,
            }
            _upsert_terminal_step(report, {
                "id": "s100_live",
                "mode": "external",
                "status": "PASSED",
                "started_epoch_ns": resume_started_ns,
                "finished_epoch_ns": resume_finished_ns,
                "evidence": s100_evidence,
                "runtime_closure_before": s100_closure_before,
                "runtime_closure_after": s100_closure_after,
                **resume_fields,
            }, resuming=True)
            if not resume_from_complete:
                _upsert_terminal_step(report, {
                    "id": "finalize_session",
                    "mode": "static",
                    "status": "PASSED",
                    "started_epoch_ns": finalize_started_ns,
                    "finished_epoch_ns": resume_finished_ns,
                    "log": str(logs / f"finalize_session_resume_{resume_started_ns}.log"),
                    "evidence": [],
                    "runtime_closure_before": finalize_closure_before,
                    "runtime_closure_after": finalize_closure_after,
                    **resume_fields,
                }, resuming=True)
            _upsert_terminal_step(report, {
                "id": "functional_aggregate",
                "mode": "static",
                "status": "PASSED",
                "started_epoch_ns": aggregate_started_ns,
                "finished_epoch_ns": resume_finished_ns,
                "log": str(logs / f"functional_aggregate_resume_{resume_started_ns}.log"),
                "evidence": [],
                "runtime_closure_before": aggregate_closure_before,
                "runtime_closure_after": aggregate_closure_after,
                **resume_fields,
            }, resuming=True, allow_missing_terminal_step=committed_aggregate_pending)
            after_rows = {
                step_id: _json_copy(_terminal_row(report, step_id))
                for step_id in terminal_ids
            }
            history.append({
                "attempt": len(history) + 1,
                "resume_started_epoch_ns": resume_started_ns,
                "resume_finished_epoch_ns": resume_finished_ns,
                "previous_orchestration_status": prior_orchestration_status,
                "terminal_rows": {
                    step_id: {
                        "before": before_rows[step_id],
                        "before_sha256": (
                            _json_digest(before_rows[step_id])
                            if before_rows[step_id] is not None else None
                        ),
                        "after": after_rows[step_id],
                        "after_sha256": _json_digest(after_rows[step_id]),
                    }
                    for step_id in terminal_ids
                },
            })
            report.update(
                status="FORMAL_FINAL_ACCEPTANCE_ORCHESTRATION_COMPLETE",
                previous_status=prior_orchestration_status,
                resume_started_epoch_ns=resume_started_ns,
                resume_finished_epoch_ns=resume_finished_ns,
                finished_epoch_ns=resume_finished_ns,
                session_status=session.get("status"),
                functional_status=functional.get("status"),
                passed_position_count=functional.get("passed_position_count"),
                pending_position_count=functional.get("pending_position_count"),
                resume_history=history,
                operator_trusted_s100_acknowledged=True,
                s100_evidence_trust_boundary=S100_EVIDENCE_TRUST_BOUNDARY,
            )
            _atomic_json(context.run_root / "orchestration_report.json", report)
            _atomic_json(context.root / ORCHESTRATION_REPORT.relative_to(ROOT), report)
            return report, 0
    except (OSError, OrchestrationError, subprocess.SubprocessError, ValueError) as exc:
        return {
            "status": "FORMAL_FINAL_ACCEPTANCE_S100_RESUME_REFUSED",
            "run_root": str(context.run_root),
            "error": str(exc),
        }, 3


def _default_run_root() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return ROOT / ".work/formal_final_acceptance" / f"session_{stamp}_{os.getpid()}"


def _integrated_source_build_preflight_timeout_seconds(value: str) -> int:
    try:
        timeout_seconds = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("timeout must be an integer number of seconds") from exc
    if not (
        MIN_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS
        <= timeout_seconds
        <= MAX_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS
    ):
        raise argparse.ArgumentTypeError(
            "timeout must be in "
            f"[{MIN_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS}, "
            f"{MAX_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS}] seconds"
        )
    return timeout_seconds


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--static-audit", action="store_true")
    mode.add_argument("--windows-dry-run", action="store_true")
    mode.add_argument("--preflight", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--resume-s100", action="store_true")
    parser.add_argument("--runtime-ws", type=Path)
    parser.add_argument("--integrated-build-manifest", type=Path)
    parser.add_argument(
        "--runtime-closure-manifest",
        type=Path,
        help="frozen non-symlink merged-overlay closure; defaults inside --runtime-ws",
    )
    parser.add_argument("--perception-artifacts", type=Path, default=ROOT / ".work/formal_perception_assets")
    parser.add_argument("--onnx-pythonpath", type=Path, default=Path("/home/zhexu/tzcup-ros-onnx"))
    parser.add_argument("--run-root", type=Path)
    parser.add_argument("--base-domain", type=int, default=60)
    parser.add_argument("--perception-episodes", type=int, default=30)
    parser.add_argument(
        "--integrated-source-build-preflight-timeout-seconds",
        type=_integrated_source_build_preflight_timeout_seconds,
        default=DEFAULT_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS,
        help=(
            "bounded timeout for the full source/install binding audit "
            f"({MIN_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS}-"
            f"{MAX_INTEGRATED_SOURCE_BUILD_PREFLIGHT_TIMEOUT_SECONDS} seconds)"
        ),
    )
    parser.add_argument(
        "--accept-operator-trusted-s100",
        action="store_true",
        help="acknowledge that the retained S100 raw/final chain is operator-trusted, not TPM/signed attestation",
    )
    parser.add_argument("--output", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.static_audit:
        result = static_audit()
        output = args.output or STATIC_AUDIT
        _atomic_json(output, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["passed"] else 2
    if args.runtime_ws is None or args.integrated_build_manifest is None:
        print(json.dumps({"status": "INVALID", "error": "--runtime-ws and --integrated-build-manifest are required"}, indent=2))
        return 2
    if args.resume_s100 and args.run_root is None:
        print(json.dumps({"status": "INVALID", "error": "--resume-s100 requires an explicit --run-root"}, indent=2))
        return 2
    context = Context(
        root=ROOT,
        runtime_ws=args.runtime_ws,
        integrated_build_manifest=args.integrated_build_manifest,
        perception_artifacts=args.perception_artifacts,
        onnx_pythonpath=args.onnx_pythonpath,
        run_root=args.run_root or _default_run_root(),
        base_domain=args.base_domain,
        episode_count=args.perception_episodes,
        runtime_closure_manifest=args.runtime_closure_manifest,
        accept_operator_trusted_s100=args.accept_operator_trusted_s100,
        integrated_source_build_preflight_timeout_seconds=(
            args.integrated_source_build_preflight_timeout_seconds
        ),
    )
    if args.windows_dry_run:
        result = windows_dry_run(context)
        if args.output:
            _atomic_json(args.output, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["passed"] else 2
    if args.preflight:
        result = preflight(context)
        if args.output:
            _atomic_json(args.output, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["passed"] else 2
    result, exit_code = resume_s100(context) if args.resume_s100 else execute(context)
    if args.output:
        _atomic_json(args.output, result)
    print(json.dumps(result, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
