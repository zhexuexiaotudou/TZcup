#!/usr/bin/env python3
"""Static tests for the serial 8+12 product-generalization acceptance gate."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
import yaml

import formal_multisite_product_acceptance as multisite


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_multisite_product_acceptance_contract.yaml"
FUNCTIONAL_CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml"


def _write(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def _identity_files(tmp_path: Path) -> tuple[Path, Path, Path, dict, dict]:
    expanded = b"<robot name='frozen'/>\n"
    snapshot = tmp_path / "snapshot.json"
    _write(snapshot, {
        "source_inventory_sha256": "a" * 64,
        "outputs": {"reports/engineering/formal_competition_vehicle.urdf": {"sha256": hashlib.sha256(expanded).hexdigest()}},
    })
    snapshot_identity = multisite._snapshot_identity(snapshot)
    session = tmp_path / "session.json"
    session_identity = {"session_started_epoch_ns": 123, "snapshot": snapshot_identity}
    _write(session, {"status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "started_epoch_ns": 123, "snapshot": snapshot_identity, "evidence": {}})
    closure = tmp_path / "closure.json"
    _write(closure, {"kind": "frozen_runtime"})
    return snapshot, session, closure, snapshot_identity, session_identity


def _site_record(
    expected: dict, contract: dict, snapshot: dict, session: dict, closure: Path,
    *, map_hash: str,
) -> dict:
    required_topics = contract["site_evidence"]["required_topics"]
    return {
        "report_id": contract["site_evidence"]["report_id"],
        "status": contract["site_evidence"]["success_status"],
        "passed": True,
        "site": {key: expected[key] for key in ("split", "map_index", "map_id", "mission_index")},
        "map_snapshot": {"sha256": map_hash},
        "source_binding": snapshot,
        "acceptance_session_binding": session,
        "runtime_closure_binding": {"manifest_sha256": multisite._sha256(closure)},
        "collected_epoch_ns": 123,
        "actual_runtime_topics": {
            role: {
                **required,
                "live_observed": True,
                "message_count": 3,
                "publisher_nodes": [f"/{role}_live_node"],
            }
            for role, required in required_topics.items()
        },
        "site_checks": dict(contract["site_evidence"]["required_site_checks"]),
        "truth_isolation": dict(contract["site_evidence"]["required_truth_isolation"]),
    }


def _evidence_set(
    tmp_path: Path,
) -> tuple[Path, Path, Path, Path, Path, Path, dict]:
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    snapshot, session, closure, snapshot_id, session_id = _identity_files(tmp_path)
    evidence = tmp_path / "site_evidence"
    for ordinal, expected in enumerate(multisite.expected_sites(contract), start=1):
        _write(evidence / expected["evidence_file"], _site_record(
            expected, contract, snapshot_id, session_id, closure,
            map_hash=f"{ordinal:064x}",
        ))
    runtime_overlay = (tmp_path / "runtime/install").resolve()
    runtime_overlay.mkdir(parents=True)
    closure_binding = {
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "runtime_install_root": str(runtime_overlay),
        "manifest_sha256": multisite._sha256(closure),
    }
    session_payload = json.loads(session.read_text(encoding="utf-8"))
    session_payload["runtime_closure_binding"] = dict(closure_binding)
    _write(session, session_payload)
    runtime_binding = tmp_path / "runtime_binding.json"
    _write(runtime_binding, {
        "schema_version": 1,
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {
            "session_manifest_sha256": multisite._sha256(session),
            "session_started_epoch_ns": 123,
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
            "snapshot": snapshot_id,
            "snapshot_current_source_verified": True,
        },
        "runtime_closure_binding": closure_binding,
    })
    return (
        evidence,
        snapshot,
        session,
        closure,
        runtime_binding,
        runtime_overlay,
        contract,
    )


def test_static_plan_is_exactly_serial_validation_8_then_hidden_12() -> None:
    plan = multisite.build_serial_plan(CONTRACT)
    assert plan["static_plan_only"] is True
    assert plan["execution_mode"] == "serial_one_site_at_a_time"
    assert plan["single_site_e2e_counts_as_generalization"] is False
    assert len(plan["sites"]) == 20
    assert [row["split"] for row in plan["sites"]] == ["validation"] * 8 + ["hidden"] * 12
    assert plan["sites"][0]["map_id"] == "val-map-000"
    assert plan["sites"][7]["map_id"] == "val-map-007"
    assert plan["sites"][8]["map_id"] == "hidden-map-000"
    assert len({row["map_id"] for row in plan["sites"]}) == 20


def test_aggregate_requires_every_site_to_bind_one_frozen_runtime_session_snapshot(tmp_path: Path) -> None:
    evidence, snapshot, session, closure, binding, overlay, _ = _evidence_set(tmp_path)
    report = multisite.aggregate(
        evidence, snapshot, session, closure, binding, overlay, CONTRACT
    )
    assert report["passed"] is True
    assert report["sites_total"] == 20
    assert report["split_counts"] == {"validation": 8, "hidden": 12}
    assert report["single_site_e2e_counts_as_generalization"] is False
    assert report["runtime_gate_binding"]["status"] == "FORMAL_RUNTIME_GATE_BOUND"


def test_aggregate_fails_closed_when_one_of_the_twenty_sites_is_missing(tmp_path: Path) -> None:
    evidence, snapshot, session, closure, binding, overlay, _ = _evidence_set(tmp_path)
    (evidence / "site-hidden-11.json").unlink()
    with pytest.raises(multisite.MultiSiteAcceptanceError, match="missing, unexpected, or non-canonical"):
        multisite.aggregate(
            evidence, snapshot, session, closure, binding, overlay, CONTRACT
        )


def test_aggregate_fails_closed_when_hidden_consumption_ledger_is_missing(tmp_path: Path) -> None:
    evidence, snapshot, session, closure, binding, overlay, _ = _evidence_set(tmp_path)
    with pytest.raises(multisite.MultiSiteAcceptanceError, match="ledger/output summary failed closed"):
        multisite.aggregate(
            evidence, snapshot, session, closure, binding, overlay, CONTRACT,
            tmp_path / "formal-run-root", multisite.SCENARIO_CONFIG,
        )


def test_aggregate_rejects_runtime_binding_from_another_session_closure(tmp_path: Path) -> None:
    evidence, snapshot, session, closure, binding, overlay, _ = _evidence_set(tmp_path)
    session_payload = json.loads(session.read_text(encoding="utf-8"))
    session_payload["runtime_closure_binding"]["manifest_sha256"] = "f" * 64
    _write(session, session_payload)
    binding_payload = json.loads(binding.read_text(encoding="utf-8"))
    binding_payload["acceptance_session_binding"]["session_manifest_sha256"] = (
        multisite._sha256(session)
    )
    _write(binding, binding_payload)
    with pytest.raises(
        multisite.MultiSiteAcceptanceError,
        match="closure does not match the acceptance session",
    ):
        multisite.aggregate(evidence, snapshot, session, closure, binding, overlay, CONTRACT)


@pytest.mark.parametrize("mutation, error", [
    (lambda record: record["map_snapshot"].update(sha256=f"{1:064x}"), "duplicate frozen map snapshot hash"),
    (lambda record: record["truth_isolation"].update(truth_used_for_product_control=True), "truth entered product control"),
    (lambda record: record["runtime_closure_binding"].update(manifest_sha256="f" * 64), "runtime closure binding drifted"),
])
def test_aggregate_fails_closed_for_duplicate_map_truth_control_or_hash_drift(
    tmp_path: Path, mutation, error: str,
) -> None:
    evidence, snapshot, session, closure, binding, overlay, _ = _evidence_set(tmp_path)
    target = evidence / "site-validation-01.json"
    record = json.loads(target.read_text(encoding="utf-8"))
    mutation(record)
    _write(target, record)
    with pytest.raises(multisite.MultiSiteAcceptanceError, match=error):
        multisite.aggregate(
            evidence, snapshot, session, closure, binding, overlay, CONTRACT
        )


def test_functional_contract_keeps_single_site_e2e_and_multi_site_generalization_separate() -> None:
    contract = yaml.safe_load(FUNCTIONAL_CONTRACT.read_text(encoding="utf-8"))
    assert "end_to_end_cleaning_mission" in contract["mission_level_gates"]
    assert "multi_site_product_generalization" in contract["mission_level_gates"]
    gate = contract["evidence_gates"]["multi_site_product_generalization"]
    assert gate["required_values"]["sites_total"] == 20
    assert gate["required_values"]["single_site_e2e_counts_as_generalization"] is False
    assert gate["runtime_binding"] == {
        "report_field": "runtime_gate_binding",
        "sidecar_suffix": ".runtime_binding.json",
    }


def test_live_site_converter_requires_fresh_observed_interfaces_and_does_not_copy_single_site_report(tmp_path: Path) -> None:
    evidence, snapshot, session, closure, _, _, contract = _evidence_set(tmp_path)
    expected = next(multisite.expected_sites(contract))
    episode = tmp_path / "episode/public/episode_manifest.json"
    _write(episode, {"map_id": expected["map_id"], "mission_index": 0})
    (episode.parent / "world.sdf").write_text("<sdf version='1.10'/>", encoding="utf-8")
    validation = tmp_path / "validation.json"
    _write(validation, {
        "report_id": "tzcup_formal_end_to_end_cleaning_mission_v3", "passed": True,
        "validated_closed_loop": {
            "fixed_start_verified": True, "first_map_ignored_dirt": True,
            "saved_map_hard_restart_verified": True, "truth_isolated_from_product_control": True,
            "actual_brushed_area_at_least_95_percent": True,
            "all_20_discrete_targets_physically_deposited": True,
            "water_recovery_at_least_95_percent": True, "zero_collisions": True,
            "trajectory_output_verified": True, "single_snapshot_single_episode": True,
        },
    })
    raw = tmp_path / "raw.json"
    _write(raw, {
        "artifact_kind": "single_live_episode_raw_collection", "timed_out": False,
        "runtime_graph": {"control_prohibited_truth_topic_subscribers": {
            "/evaluation/single_episode/ground_dirt/status_json": ["/formal_single_episode_cleaning_collector"],
            "/evaluation/single_episode/water_recovery/status_json": ["/formal_single_episode_cleaning_collector"],
        }},
        "evaluator": {"collision_monitor_intervention_count": 1, "terminal": {"pedestrians": {"state": "ACTIVE"}}},
    })
    observations = tmp_path / "observations.json"
    _write(observations, {
        "collected_epoch_ns": 123,
        "interfaces": {
            role: {**required, "live_observed": True, "message_count": 1,
                   "publisher_nodes": [f"/{role}_node"], "goal_succeeded": role == "nav2"}
            for role, required in contract["site_evidence"]["required_topics"].items()
        },
    })
    result = multisite.emit_site_evidence(
        validation_path=validation, raw_path=raw, topic_observations_path=observations,
        episode_manifest_path=episode, snapshot_path=snapshot, session_path=session,
        runtime_closure_path=closure, site={key: expected[key] for key in ("split", "map_index", "map_id", "mission_index")},
        contract_path=CONTRACT,
    )
    assert result["site"] == {key: expected[key] for key in ("split", "map_index", "map_id", "mission_index")}
    assert result["actual_runtime_topics"]["dosod"]["message_count"] == 1
    assert result["site_checks"]["nav2_goal_succeeded"] is True


def test_live_executor_contract_is_serial_and_wires_fresh_per_site_paths() -> None:
    source = (ROOT / "scripts/formal_multisite_product_acceptance.py").read_text(encoding="utf-8")
    runner = (ROOT / "scripts/run_formal_single_episode_cleaning_mission.sh").read_text(encoding="utf-8")
    assert "for ordinal, site in enumerate(sites):" in source
    assert 'generator_split = "val" if site["split"] == "validation"' in source
    assert '"--config", str(scenario_config), "--profile", "formal", "--split", generator_split' in source
    assert '"materialize-hidden"' in source
    assert "commit_formal_hidden_run_context(" in source
    assert '"hidden-consumption-ledger"' in (ROOT / "starter_ws/src/sanitation_campus_scenario/sanitation_campus_scenario/hidden_materializer.py").read_text(encoding="utf-8")
    assert "verify_hidden_consumption_records(" in source
    assert "sanitation-campus-scenario" in source
    assert "run_formal_first_map_dynamic_prerequisite.sh" in source
    assert "run_formal_same_map_full_coverage_baseline.sh" in source
    assert "--multisite-site-evidence" in source
    assert "--multisite-topic-observations" in runner
    assert "--emit-site-evidence" in runner
    assert '"${MULTISITE_SPLIT}" == "validation"' in runner
    assert 'parser.add_argument("--runtime-binding", type=Path)' in source
    assert '"runtime_gate_binding": runtime_binding' in source


def test_multisite_interfaces_match_the_live_pc_product_and_follow_path_chain() -> None:
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    collector = (ROOT / "scripts/collect_formal_single_episode_cleaning_mission.py").read_text(
        encoding="utf-8"
    )
    pc_adapter = (
        ROOT
        / "starter_ws/src/sanitation_perception/sanitation_perception/pc_open_vocab_adapter.py"
    ).read_text(encoding="utf-8")
    trajectory_executor = (
        ROOT
        / "starter_ws/src/sanitation_active_cleaning/sanitation_active_cleaning/formal_trajectory_executor.py"
    ).read_text(encoding="utf-8")

    topics = contract["site_evidence"]["required_topics"]
    assert topics["edgesam"] == {
        "name": "/perception/ground_dirt/masks",
        "type": "sensor_msgs/msg/Image",
        "interface_kind": "topic",
    }
    assert topics["nav2"] == {
        "name": "/follow_path",
        "type": "nav2_msgs/action/FollowPath",
        "interface_kind": "action",
    }
    assert 'Image, "/perception/ground_dirt/masks", 10' in pc_adapter
    assert 'NAVIGATION_ACTION = "/follow_path"' in trajectory_executor
    assert "FollowPath" in trajectory_executor
    assert '"observed_topic": "/perception/ground_dirt/masks"' in collector
    assert '"observed_topic": "/follow_path/_action/status"' in collector
    assert 'self.create_subscription(Image, MULTISITE_INTERFACES["edgesam"]["observed_topic"]' in collector
