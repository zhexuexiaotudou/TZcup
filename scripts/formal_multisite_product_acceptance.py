#!/usr/bin/env python3
"""Fail-closed serial executor and aggregator for 8+12 product-site evidence.

``--emit-plan`` is deliberately static.  ``--execute`` is the only live mode:
it materializes, maps, baselines and runs exactly one product mission before
moving to the next frozen site.  It never treats the retained single-site
report as a substitute for the twenty fresh site records.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
from typing import Any, Iterable, Mapping

import yaml

from formal_runtime_gate_binding import (
    RuntimeGateError,
    build_binding,
    load_binding,
)
from sanitation_campus_scenario.hidden_materializer import (
    commit_hidden_configuration_freeze,
    verify_hidden_consumption_records,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_multisite_product_acceptance_contract.yaml"
DEFAULT_OUTPUT = ROOT / "artifacts/formal_multisite_product_acceptance.json"
SINGLE_SITE_RUNNER = ROOT / "scripts/run_formal_single_episode_cleaning_mission.sh"
FIRST_MAP_RUNNER = ROOT / "scripts/run_formal_first_map_dynamic_prerequisite.sh"
BASELINE_RUNNER = ROOT / "scripts/run_formal_same_map_full_coverage_baseline.sh"
SCENARIO_CONFIG = ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml"


class MultiSiteAcceptanceError(RuntimeError):
    """A contract or site evidence violation that must not be waived silently."""


def _read_mapping(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise MultiSiteAcceptanceError(f"cannot read structured file {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise MultiSiteAcceptanceError(f"structured root must be an object: {path}")
    return value


def _json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise MultiSiteAcceptanceError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise MultiSiteAcceptanceError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _strict_equal(actual: Any, expected: Any) -> bool:
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _strict_equal(actual[key], value) for key, value in expected.items()
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _strict_equal(left, right) for left, right in zip(actual, expected)
        )
    return actual == expected


def _require_regular(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise MultiSiteAcceptanceError(f"{label} must be a regular file: {path}")


def _snapshot_identity(path: Path) -> dict[str, str]:
    _require_regular(path, "snapshot manifest")
    snapshot = _json_object(path)
    outputs = snapshot.get("outputs")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf") if isinstance(outputs, dict) else None
    source_hash = snapshot.get("source_inventory_sha256")
    if not isinstance(source_hash, str) or not source_hash or not isinstance(urdf, dict):
        raise MultiSiteAcceptanceError("snapshot has no immutable source or expanded-URDF identity")
    urdf_hash = urdf.get("sha256")
    if not isinstance(urdf_hash, str) or not urdf_hash:
        raise MultiSiteAcceptanceError("snapshot has no expanded-URDF hash")
    return {
        "snapshot_manifest_sha256": _sha256(path),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
    }


def _session_identity(path: Path, snapshot: dict[str, str]) -> dict[str, Any]:
    _require_regular(path, "acceptance session")
    session = _json_object(path)
    if session.get("status") not in {
        "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING",
        "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE",
    }:
        raise MultiSiteAcceptanceError("acceptance session does not have an allowed status")
    started = session.get("started_epoch_ns")
    if not isinstance(started, int) or started <= 0:
        raise MultiSiteAcceptanceError("acceptance session has no valid start time")
    if not _strict_equal(session.get("snapshot"), snapshot):
        raise MultiSiteAcceptanceError("acceptance session is not bound to the current frozen snapshot")
    return {"session_started_epoch_ns": started, "snapshot": snapshot}


def _closure_identity(path: Path) -> dict[str, str]:
    _require_regular(path, "runtime closure manifest")
    # Parsing is intentional: a random byte blob with a matching supplied digest
    # is not a runtime closure manifest.
    _json_object(path)
    return {"manifest_sha256": _sha256(path)}


def _runtime_gate_evidence(
    binding_path: Path,
    snapshot_path: Path,
    session_path: Path,
    runtime_closure_path: Path,
    runtime_overlay: Path,
) -> dict[str, Any]:
    """Revalidate the exact sidecar against current frozen inputs."""

    try:
        binding = load_binding(binding_path)
    except (OSError, RuntimeGateError, ValueError) as exc:
        raise MultiSiteAcceptanceError(f"invalid runtime gate binding: {exc}") from exc
    snapshot = _snapshot_identity(snapshot_path)
    session = _json_object(session_path)
    if session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise MultiSiteAcceptanceError("multi-site aggregation requires a RUNNING session")
    started_epoch_ns = session.get("started_epoch_ns")
    bound_session = binding.get("acceptance_session_binding")
    if not isinstance(bound_session, dict):
        raise MultiSiteAcceptanceError("runtime gate binding has no session identity")
    if not _strict_equal(bound_session.get("snapshot"), snapshot):
        raise MultiSiteAcceptanceError("runtime gate binding snapshot drifted")
    if (
        bound_session.get("session_manifest_sha256") != _sha256(session_path)
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
        or bound_session.get("session_status_at_gate")
        != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or bound_session.get("snapshot_current_source_verified") is not True
    ):
        raise MultiSiteAcceptanceError("runtime gate binding session drifted")
    closure = binding.get("runtime_closure_binding")
    if not isinstance(closure, dict):
        raise MultiSiteAcceptanceError("runtime gate binding has no closure identity")
    session_closure = session.get("runtime_closure_binding")
    if not isinstance(session_closure, dict) or not _strict_equal(session_closure, closure):
        raise MultiSiteAcceptanceError(
            "runtime gate binding closure does not match the acceptance session"
        )
    if closure.get("manifest_sha256") != _sha256(runtime_closure_path):
        raise MultiSiteAcceptanceError("runtime gate binding closure manifest drifted")
    if closure.get("runtime_install_root") != str(runtime_overlay.resolve()):
        raise MultiSiteAcceptanceError("runtime gate binding install root drifted")
    return binding


def expected_sites(contract: dict[str, Any]) -> Iterable[dict[str, Any]]:
    execution = contract.get("execution")
    splits = contract.get("frozen_splits")
    if not isinstance(execution, dict) or not isinstance(splits, dict):
        raise MultiSiteAcceptanceError("contract must define execution and frozen_splits")
    if execution.get("mode") != "serial_one_site_at_a_time" or execution.get("max_concurrent_sites") != 1:
        raise MultiSiteAcceptanceError("multi-site product contract must remain serial")
    if execution.get("single_site_e2e_counts_as_generalization") is not False:
        raise MultiSiteAcceptanceError("single-site E2E must not count as multi-site generalization")
    mission_index = execution.get("frozen_mission_index")
    if not isinstance(mission_index, int) or mission_index != 0:
        raise MultiSiteAcceptanceError("multi-site product contract must freeze mission index 0")
    for split, expected_count in (("validation", 8), ("hidden", 12)):
        row = splits.get(split)
        if not isinstance(row, dict) or row.get("map_count") != expected_count:
            raise MultiSiteAcceptanceError(f"{split} split must contain exactly {expected_count} frozen maps")
        prefix = row.get("map_id_prefix")
        if not isinstance(prefix, str) or not prefix:
            raise MultiSiteAcceptanceError(f"{split} split has no map id prefix")
        for index in range(expected_count):
            yield {
                "split": split,
                "map_index": index,
                "map_id": f"{prefix}{index:03d}",
                "mission_index": mission_index,
                "evidence_file": f"site-{split}-{index:02d}.json",
            }


def build_serial_plan(contract_path: Path = DEFAULT_CONTRACT) -> dict[str, Any]:
    contract = _read_mapping(contract_path)
    sites = list(expected_sites(contract))
    if len(sites) != 20 or len({row["map_id"] for row in sites}) != 20:
        raise MultiSiteAcceptanceError("contract did not materialize exactly 20 distinct frozen maps")
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_multisite_product_serial_plan_v1",
        "static_plan_only": True,
        "live_executor_available": True,
        "execution_mode": "serial_one_site_at_a_time",
        "single_site_e2e_counts_as_generalization": False,
        "site_runner_contract": {
            "runner": "scripts/run_formal_single_episode_cleaning_mission.sh",
            "required_live_components": ["DOSOD", "EdgeSAM", "Nav2", "dynamic_pedestrians", "cleaning_actuator"],
            "prohibited_actions": ["parallel_sites", "truth_control", "reuse_of_another_map_evidence"],
        },
        "sites": sites,
    }


def _require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise MultiSiteAcceptanceError(f"{label} must be an object")
    return value


def _require_bool_fields(value: Mapping[str, Any], fields: Iterable[str], label: str) -> None:
    for field in fields:
        if value.get(field) is not True:
            raise MultiSiteAcceptanceError(f"{label} is missing required true fact: {field}")


def _site_from_arguments(split: str, map_index: int, map_id: str, mission_index: int) -> dict[str, Any]:
    return {"split": split, "map_index": map_index, "map_id": map_id, "mission_index": mission_index}


def emit_site_evidence(
    *, validation_path: Path, raw_path: Path, topic_observations_path: Path,
    episode_manifest_path: Path, snapshot_path: Path, session_path: Path,
    runtime_closure_path: Path, site: dict[str, Any], contract_path: Path = DEFAULT_CONTRACT,
) -> dict[str, Any]:
    """Convert one fresh single-mission run into one canonical site record.

    This function is intentionally a converter, not a synthesizer: every
    asserted fact comes from the just-finished validation, raw collector or
    collector's read-only graph/topic observation file.
    """

    contract = _read_mapping(contract_path)
    expected = next((
        row for row in expected_sites(contract)
        if all(site.get(key) == row[key] for key in ("split", "map_index", "map_id", "mission_index"))
        and set(site) == {"split", "map_index", "map_id", "mission_index"}
    ), None)
    if expected is None:
        raise MultiSiteAcceptanceError("site arguments are not a frozen multi-site plan entry")
    validation = _json_object(validation_path)
    if validation.get("report_id") != "tzcup_formal_end_to_end_cleaning_mission_v3" or validation.get("passed") is not True:
        raise MultiSiteAcceptanceError("single-site validation did not pass")
    closed_loop = _require_mapping(validation.get("validated_closed_loop"), "validated closed-loop projection")
    _require_bool_fields(closed_loop, (
        "fixed_start_verified", "first_map_ignored_dirt", "saved_map_hard_restart_verified",
        "truth_isolated_from_product_control", "actual_brushed_area_at_least_95_percent",
        "all_20_discrete_targets_physically_deposited", "water_recovery_at_least_95_percent",
        "zero_collisions", "trajectory_output_verified", "single_snapshot_single_episode",
    ), "validated closed-loop projection")
    raw = _json_object(raw_path)
    if raw.get("artifact_kind") != "single_live_episode_raw_collection" or raw.get("timed_out") is not False:
        raise MultiSiteAcceptanceError("site raw collector is not a completed live mission")
    episode = _json_object(episode_manifest_path)
    if episode.get("map_id") != expected["map_id"]:
        raise MultiSiteAcceptanceError("materialized episode map id differs from frozen site")
    if episode.get("mission_index") != expected["mission_index"]:
        raise MultiSiteAcceptanceError("materialized episode mission index differs from frozen site")
    observations = _json_object(topic_observations_path)
    required_topics = _require_mapping(contract.get("site_evidence"), "site evidence").get("required_topics")
    if not isinstance(required_topics, dict) or set(observations.get("interfaces", {})) != set(required_topics):
        raise MultiSiteAcceptanceError("live collector did not record exactly the required product interfaces")
    topics: dict[str, dict[str, Any]] = {}
    for role, required in required_topics.items():
        observed = observations["interfaces"][role]
        if not isinstance(observed, dict) or not isinstance(required, dict):
            raise MultiSiteAcceptanceError(f"invalid live interface record: {role}")
        for key in ("name", "type", "interface_kind"):
            if observed.get(key) != required.get(key):
                raise MultiSiteAcceptanceError(f"live interface identity drifted: {role}.{key}")
        if observed.get("live_observed") is not True or not isinstance(observed.get("message_count"), int) or observed["message_count"] < 1:
            raise MultiSiteAcceptanceError(f"required live interface has no observed message: {role}")
        publishers = observed.get("publisher_nodes")
        if not isinstance(publishers, list) or not publishers or any(not isinstance(row, str) or not row.startswith("/") for row in publishers):
            raise MultiSiteAcceptanceError(f"required live interface has no attributable publisher: {role}")
        topics[role] = observed
    graph = _require_mapping(raw.get("runtime_graph"), "raw runtime graph")
    prohibited = graph.get("control_prohibited_truth_topic_subscribers")
    if not isinstance(prohibited, dict) or any(rows != ["/formal_single_episode_cleaning_collector"] for rows in prohibited.values()):
        raise MultiSiteAcceptanceError("product control truth-subscriber isolation failed")
    evaluator = _require_mapping(raw.get("evaluator"), "raw evaluator")
    terminal = _require_mapping(evaluator.get("terminal"), "terminal evaluator")
    pedestrians = _require_mapping(terminal.get("pedestrians"), "terminal pedestrian evaluator")
    map_hash = _sha256(episode_manifest_path.parent / "world.sdf")
    snapshot = _snapshot_identity(snapshot_path)
    session = _session_identity(session_path, snapshot)
    closure = _closure_identity(runtime_closure_path)
    collected = observations.get("collected_epoch_ns")
    if not isinstance(collected, int) or collected < session["session_started_epoch_ns"]:
        raise MultiSiteAcceptanceError("live interface observations predate acceptance session")
    return {
        "report_id": contract["site_evidence"]["report_id"],
        "status": contract["site_evidence"]["success_status"],
        "passed": True,
        "site": site,
        "map_snapshot": {"sha256": map_hash},
        "source_binding": snapshot,
        "acceptance_session_binding": session,
        "runtime_closure_binding": closure,
        "collected_epoch_ns": collected,
        "actual_runtime_topics": topics,
        "site_checks": {
            "product_closed_loop_passed": True,
            "nav2_goal_succeeded": observations["interfaces"]["nav2"].get("goal_succeeded") is True,
            "dynamic_pedestrian_interaction_observed": (
                observations["interfaces"]["dynamic_pedestrians"].get("message_count", 0) > 0
                and int(evaluator.get("collision_monitor_intervention_count", 0)) > 0
                and pedestrians.get("state") == "ACTIVE"
            ),
            "cleaning_actuator_effect_observed": (
                observations["interfaces"]["cleaning_actuator"].get("message_count", 0) > 0
                and closed_loop["actual_brushed_area_at_least_95_percent"] is True
                and closed_loop["water_recovery_at_least_95_percent"] is True
            ),
        },
        "truth_isolation": {
            "truth_used_for_product_control": False,
            "control_truth_topics_subscribed": [],
            "evaluator_truth_process_isolated": True,
        },
    }


def _run_live(command: list[str], environment: Mapping[str, str]) -> None:
    try:
        subprocess.run(command, cwd=ROOT, env=dict(environment), check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        raise MultiSiteAcceptanceError(f"serial site runner failed: {command[0]}: {exc}") from exc


def execute_live(
    *, evidence_root: Path, work_root: Path, snapshot_path: Path, session_path: Path,
    runtime_closure_path: Path, runtime_ws: Path, runtime_overlay: Path,
    perception_artifacts: Path, policy_checkpoint: Path, base_domain: int,
    scenario_config: Path = SCENARIO_CONFIG, contract_path: Path = DEFAULT_CONTRACT,
    output: Path = DEFAULT_OUTPUT, runtime_binding_path: Path | None = None,
) -> dict[str, Any]:
    """Run twenty *fresh* sites serially, then aggregate their canonical records."""

    contract = _read_mapping(contract_path)
    sites = list(expected_sites(contract))
    if runtime_binding_path is None:
        raise MultiSiteAcceptanceError("live executor requires a canonical runtime binding path")
    if (
        len(sites) != 20
        or evidence_root.exists()
        or work_root.exists()
        or output.exists()
        or runtime_binding_path.exists()
    ):
        raise MultiSiteAcceptanceError("live executor requires fresh evidence, work and aggregate output paths")
    for path, label in ((snapshot_path, "snapshot"), (session_path, "session"), (runtime_closure_path, "runtime closure"), (runtime_overlay / "setup.bash", "runtime overlay"), (scenario_config, "scenario config"), (policy_checkpoint, "policy checkpoint")):
        _require_regular(path, label)
    if not perception_artifacts.is_dir() or perception_artifacts.is_symlink():
        raise MultiSiteAcceptanceError("perception artifact root must be a regular directory")
    if not isinstance(base_domain, int) or base_domain < 0 or base_domain > 101:
        raise MultiSiteAcceptanceError("base ROS domain must be in the Linux-safe 0..101 range")
    try:
        binding = build_binding(
            repository_root=ROOT,
            install_root=runtime_overlay,
            closure_manifest=runtime_closure_path,
            session_path=session_path,
            snapshot_path=snapshot_path,
        )
    except (OSError, RuntimeGateError, ValueError) as exc:
        raise MultiSiteAcceptanceError(f"cannot bind multi-site runtime: {exc}") from exc
    _write_atomic(runtime_binding_path, binding)
    evidence_root.mkdir(parents=True)
    work_root.mkdir(parents=True)
    environment = dict(os.environ)
    environment.update({
        "FORMAL_VEHICLE_RUNTIME_WS": str(runtime_ws),
        "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST": str(runtime_closure_path),
        "FORMAL_ACCEPTANCE_SESSION": str(session_path),
        "FORMAL_VEHICLE_SNAPSHOT_MANIFEST": str(snapshot_path),
        "TZCUP_REPOSITORY_ROOT": str(ROOT),
        "TZCUP_FORMAL_HIDDEN_RUN_ROOT": str(work_root),
    })
    for ordinal, site in enumerate(sites):
        generator_split = "val" if site["split"] == "validation" else site["split"]
        site_root = work_root / f"site-{site['split']}-{site['map_index']:02d}"
        episode_root = site_root / "episode"
        map_root = site_root / "saved_map"
        baseline_root = site_root / "same_map_baseline"
        mission_root = site_root / "mission"
        baseline = site_root / "same_map_baseline.json"
        observation = mission_root / "multisite_live_observations.json"
        evidence = evidence_root / site["evidence_file"]
        if site["split"] == "hidden" and site["map_index"] == 0:
            validation_sites = [row for row in sites if row["split"] == "validation"]
            frozen_validation = {}
            for validation_site in validation_sites:
                validation_path = evidence_root / validation_site["evidence_file"]
                _check_site(
                    _json_object(validation_path), validation_site, contract,
                    _snapshot_identity(snapshot_path), _session_identity(
                        session_path, _snapshot_identity(snapshot_path)
                    ), _closure_identity(runtime_closure_path),
                )
                frozen_validation[validation_path.name] = _sha256(validation_path)
            commit_hidden_configuration_freeze(
                run_root=work_root, snapshot_path=snapshot_path,
                session_path=session_path, scenario_config=scenario_config,
                producer="formal_multisite_product_acceptance",
                frozen_configuration={
                    "validation_sites_completed": 8,
                    "validation_evidence_sha256": frozen_validation,
                    "selection_source": "validation_only_before_hidden",
                },
            )
        if site["split"] == "hidden":
            _run_live([
                "ros2", "run", "sanitation_campus_scenario", "sanitation-campus-scenario",
                "materialize-hidden", "--config", str(scenario_config),
                "--snapshot", str(snapshot_path), "--session", str(session_path),
                "--freeze-producer", "formal_multisite_product_acceptance",
                "--map-index", str(site["map_index"]),
                "--mission-index", str(site["mission_index"]), "--output", str(episode_root),
            ], environment)
        else:
            _run_live([
                "ros2", "run", "sanitation_campus_scenario", "sanitation-campus-scenario", "generate",
                "--config", str(scenario_config), "--profile", "formal", "--split", generator_split,
                "--map-index", str(site["map_index"]), "--mission-index", str(site["mission_index"]),
                "--output", str(episode_root),
            ], environment)
        map_environment = dict(environment)
        map_environment.update({
            "FORMAL_DYNAMIC_EPISODE_ROOT": str(episode_root),
            "FORMAL_DYNAMIC_SAVED_MAP_ROOT": str(map_root),
            "ROS_DOMAIN_ID": str(base_domain),
        })
        _run_live(["bash", str(FIRST_MAP_RUNNER)], map_environment)
        _run_live([
            "bash", str(BASELINE_RUNNER), "--episode-root", str(episode_root), "--map-root", str(map_root),
            "--session", str(session_path), "--runtime-overlay", str(runtime_overlay),
            "--output", str(baseline_root), "--formal-output", str(baseline),
            "--ros-domain-id", str(base_domain),
        ], environment)
        _run_live([
            "bash", str(SINGLE_SITE_RUNNER), "--episode-root", str(episode_root),
            "--session-status", str(session_path), "--saved-map", str(map_root),
            "--perception-artifacts", str(perception_artifacts), "--policy-checkpoint", str(policy_checkpoint),
            "--same-map-baseline", str(baseline), "--output", str(mission_root),
            "--runtime-overlay", str(runtime_overlay), "--formal-output", str(site_root / "validation.json"),
            "--ros-domain-id", str(base_domain), "--multisite-site-evidence", str(evidence),
            "--multisite-split", site["split"], "--multisite-map-index", str(site["map_index"]),
            "--multisite-map-id", site["map_id"], "--multisite-mission-index", str(site["mission_index"]),
        ], environment)
        if not evidence.is_file() or not observation.is_file():
            raise MultiSiteAcceptanceError(f"site {ordinal + 1}/20 did not publish its canonical live evidence")
    result = aggregate(
        evidence_root,
        snapshot_path,
        session_path,
        runtime_closure_path,
        runtime_binding_path,
        runtime_overlay,
        contract_path,
        work_root,
        scenario_config,
    )
    _write_atomic(output, result)
    return result


def _check_site(
    payload: dict[str, Any], expected: dict[str, Any], contract: dict[str, Any],
    snapshot: dict[str, str], session: dict[str, Any], closure: dict[str, str],
) -> dict[str, Any]:
    evidence = contract.get("site_evidence")
    if not isinstance(evidence, dict):
        raise MultiSiteAcceptanceError("contract.site_evidence must be an object")
    if payload.get("report_id") != evidence.get("report_id") or payload.get("status") != evidence.get("success_status") or payload.get("passed") is not True:
        raise MultiSiteAcceptanceError("site report does not declare the required passing identity")
    if not _strict_equal(payload.get("site"), {key: expected[key] for key in ("split", "map_index", "map_id", "mission_index")}):
        raise MultiSiteAcceptanceError("site identity does not match the frozen plan")
    map_snapshot = payload.get("map_snapshot")
    map_hash = map_snapshot.get("sha256") if isinstance(map_snapshot, dict) else None
    if not isinstance(map_hash, str) or len(map_hash) != 64:
        raise MultiSiteAcceptanceError("site has no immutable map snapshot hash")
    if not _strict_equal(payload.get("source_binding"), snapshot):
        raise MultiSiteAcceptanceError("site source/snapshot binding drifted")
    if not _strict_equal(payload.get("acceptance_session_binding"), session):
        raise MultiSiteAcceptanceError("site acceptance-session binding drifted")
    if not _strict_equal(payload.get("runtime_closure_binding"), closure):
        raise MultiSiteAcceptanceError("site frozen runtime closure binding drifted")
    collected = payload.get("collected_epoch_ns")
    if not isinstance(collected, int) or collected < session["session_started_epoch_ns"]:
        raise MultiSiteAcceptanceError("site evidence predates the frozen session")
    topics = payload.get("actual_runtime_topics")
    expected_topics = evidence.get("required_topics")
    if not isinstance(topics, dict) or not isinstance(expected_topics, dict) or set(topics) != set(expected_topics):
        raise MultiSiteAcceptanceError("site does not record exactly the required live runtime topics")
    for role, required in expected_topics.items():
        observed = topics.get(role)
        if not isinstance(required, dict) or not isinstance(observed, dict):
            raise MultiSiteAcceptanceError(f"invalid topic contract for {role}")
        for key in ("name", "type", "interface_kind"):
            if observed.get(key) != required.get(key):
                raise MultiSiteAcceptanceError(f"{role} did not use the required live ROS interface")
        if observed.get("live_observed") is not True or not isinstance(observed.get("message_count"), int) or observed["message_count"] < 1:
            raise MultiSiteAcceptanceError(f"{role} has no actual observed runtime messages")
        publishers = observed.get("publisher_nodes")
        if not isinstance(publishers, list) or not publishers or any(not isinstance(node, str) or not node.startswith("/") for node in publishers):
            raise MultiSiteAcceptanceError(f"{role} has no attributable live publisher")
    for field, value in evidence.get("required_site_checks", {}).items():
        checks = payload.get("site_checks")
        if not isinstance(checks, dict) or checks.get(field) is not value:
            raise MultiSiteAcceptanceError(f"site check failed or missing: {field}")
    truth = payload.get("truth_isolation")
    if not isinstance(truth, dict) or any(truth.get(field) != value for field, value in evidence.get("required_truth_isolation", {}).items()):
        raise MultiSiteAcceptanceError("site truth entered product control or isolation proof is missing")
    return {"map_id": expected["map_id"], "map_snapshot_sha256": map_hash}


def aggregate(
    evidence_root: Path, snapshot_path: Path, session_path: Path, runtime_closure_path: Path,
    runtime_binding_path: Path, runtime_overlay: Path,
    contract_path: Path = DEFAULT_CONTRACT,
    work_root: Path | None = None,
    scenario_config: Path = SCENARIO_CONFIG,
) -> dict[str, Any]:
    contract = _read_mapping(contract_path)
    snapshot = _snapshot_identity(snapshot_path)
    session = _session_identity(session_path, snapshot)
    closure = _closure_identity(runtime_closure_path)
    runtime_binding = _runtime_gate_evidence(
        runtime_binding_path,
        snapshot_path,
        session_path,
        runtime_closure_path,
        runtime_overlay,
    )
    expected = list(expected_sites(contract))
    aggregate_contract = contract.get("aggregate")
    if not isinstance(aggregate_contract, dict) or aggregate_contract.get("required_total_sites") != len(expected) or len(expected) != 20:
        raise MultiSiteAcceptanceError("aggregate contract must require all 20 frozen sites")
    if not evidence_root.is_dir() or evidence_root.is_symlink():
        raise MultiSiteAcceptanceError("site evidence root must be a real directory")
    expected_files = {row["evidence_file"] for row in expected}
    present_entries = {path.name for path in evidence_root.iterdir()}
    if present_entries != expected_files:
        raise MultiSiteAcceptanceError("site evidence set has missing, unexpected, or non-canonical files")
    seen_map_ids: set[str] = set()
    seen_map_hashes: set[str] = set()
    accepted: list[dict[str, Any]] = []
    for site in expected:  # exactly one record is read and released per site
        path = evidence_root / site["evidence_file"]
        _require_regular(path, "site evidence")
        checked = _check_site(_json_object(path), site, contract, snapshot, session, closure)
        if checked["map_id"] in seen_map_ids:
            raise MultiSiteAcceptanceError("duplicate frozen map id in site evidence")
        if checked["map_snapshot_sha256"] in seen_map_hashes:
            raise MultiSiteAcceptanceError("duplicate frozen map snapshot hash in site evidence")
        seen_map_ids.add(checked["map_id"])
        seen_map_hashes.add(checked["map_snapshot_sha256"])
        accepted.append({"site": site, "evidence_sha256": _sha256(path), **checked})
    hidden_consumption: list[dict[str, str]] = []
    if work_root is not None:
        hidden_records = [
            {
                "producer": "formal_hidden_episode",
                "request": {
                    "profile": "formal", "split": "hidden",
                    "map_index": site["map_index"], "mission_index": site["mission_index"],
                },
                "output": work_root / f"site-hidden-{site['map_index']:02d}" / "episode",
            }
            for site in expected if site["split"] == "hidden"
        ]
        try:
            hidden_consumption = verify_hidden_consumption_records(
                run_root=work_root, snapshot_path=snapshot_path, session_path=session_path,
                scenario_config=scenario_config, records=hidden_records,
            )
        except Exception as exc:
            raise MultiSiteAcceptanceError(f"hidden consumption ledger/output summary failed closed: {exc}") from exc
    return {
        "schema_version": 1,
        "report_id": aggregate_contract["report_id"],
        "status": aggregate_contract["passed_status"],
        "passed": True,
        "sites_total": len(accepted),
        "all_sites_passed": True,
        "single_site_e2e_counts_as_generalization": False,
        "split_counts": {"validation": 8, "hidden": 12},
        "failures": [],
        "source_binding": snapshot,
        "acceptance_session_binding": session,
        "runtime_closure_binding": closure,
        "runtime_gate_binding": runtime_binding,
        "hidden_consumption": hidden_consumption,
        "sites": accepted,
    }


def _write_atomic(path: Path, payload: dict[str, Any]) -> None:
    if path.exists():
        raise MultiSiteAcceptanceError(f"refusing to overwrite retained aggregate: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--emit-plan", type=Path, help="write only the static serial site plan")
    parser.add_argument("--emit-site-evidence", type=Path, help="convert one just-finished live mission into canonical site evidence")
    parser.add_argument("--execute", action="store_true", help="run all 20 frozen sites serially; requires a sourced ROS runtime")
    parser.add_argument("--evidence-root", type=Path)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--session", type=Path)
    parser.add_argument("--runtime-closure", type=Path)
    parser.add_argument("--runtime-binding", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--validation", type=Path)
    parser.add_argument("--raw", type=Path)
    parser.add_argument("--topic-observations", type=Path)
    parser.add_argument("--episode-manifest", type=Path)
    parser.add_argument("--split", choices=("val", "hidden"))
    parser.add_argument("--map-index", type=int)
    parser.add_argument("--map-id")
    parser.add_argument("--mission-index", type=int)
    parser.add_argument("--work-root", type=Path)
    parser.add_argument("--runtime-ws", type=Path)
    parser.add_argument("--runtime-overlay", type=Path)
    parser.add_argument("--perception-artifacts", type=Path)
    parser.add_argument("--policy-checkpoint", type=Path)
    parser.add_argument("--base-domain", type=int)
    parser.add_argument("--scenario-config", type=Path, default=SCENARIO_CONFIG)
    args = parser.parse_args()
    try:
        if args.emit_plan is not None:
            if args.execute or args.emit_site_evidence is not None or any(value is not None for value in (args.evidence_root, args.snapshot, args.session, args.runtime_closure)):
                raise MultiSiteAcceptanceError("--emit-plan cannot aggregate, convert evidence or launch runtime work")
            _write_atomic(args.emit_plan, build_serial_plan(args.contract))
            return 0
        site_args = (args.split, args.map_index, args.map_id, args.mission_index)
        if args.emit_site_evidence is not None:
            if args.execute or not all(value is not None for value in (
                args.validation, args.raw, args.topic_observations, args.episode_manifest,
                args.snapshot, args.session, args.runtime_closure, *site_args,
            )):
                raise MultiSiteAcceptanceError("--emit-site-evidence requires one complete live mission and frozen site identity")
            _write_atomic(args.emit_site_evidence, emit_site_evidence(
                validation_path=args.validation, raw_path=args.raw,
                topic_observations_path=args.topic_observations,
                episode_manifest_path=args.episode_manifest, snapshot_path=args.snapshot,
                session_path=args.session, runtime_closure_path=args.runtime_closure,
                site=_site_from_arguments(args.split, args.map_index, args.map_id, args.mission_index),
                contract_path=args.contract,
            ))
            return 0
        if args.execute:
            if not all(value is not None for value in (
                args.evidence_root, args.work_root, args.snapshot, args.session,
                args.runtime_closure, args.runtime_ws, args.runtime_overlay,
                args.runtime_binding, args.perception_artifacts,
                args.policy_checkpoint, args.base_domain,
            )):
                raise MultiSiteAcceptanceError("--execute requires every runtime input and fresh evidence/work roots")
            execute_live(
                evidence_root=args.evidence_root, work_root=args.work_root,
                snapshot_path=args.snapshot, session_path=args.session,
                runtime_closure_path=args.runtime_closure, runtime_ws=args.runtime_ws,
                runtime_overlay=args.runtime_overlay,
                perception_artifacts=args.perception_artifacts,
                policy_checkpoint=args.policy_checkpoint, base_domain=args.base_domain,
                scenario_config=args.scenario_config, contract_path=args.contract,
                output=args.output, runtime_binding_path=args.runtime_binding,
            )
            return 0
        if not all((
            args.evidence_root,
            args.snapshot,
            args.session,
            args.runtime_closure,
            args.runtime_binding,
            args.runtime_overlay,
            args.work_root,
        )):
            raise MultiSiteAcceptanceError(
                "aggregation requires evidence, snapshot, session, closure, binding and overlay"
            )
        _write_atomic(
            args.output,
            aggregate(
                args.evidence_root,
                args.snapshot,
                args.session,
                args.runtime_closure,
                args.runtime_binding,
                args.runtime_overlay,
                args.contract,
                args.work_root,
                args.scenario_config,
            ),
        )
        return 0
    except MultiSiteAcceptanceError as exc:
        print(f"formal multi-site product acceptance failed closed: {exc}", file=os.sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
