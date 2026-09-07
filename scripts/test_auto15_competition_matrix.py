from __future__ import annotations

import hashlib
import json
import os
import copy
from pathlib import Path

from auto15_competition_matrix import build_matrix
from coverage_mcap_replay_audit import REQUIRED_TOPICS
from validate_product_acceptance_contract import (
    MCAP_MAGIC,
    ProductAcceptanceContractError,
    load_contract,
    validate_auto15_execution_evidence,
    validate_static_contract,
)


ROOT = Path(__file__).resolve().parents[1]


def _write(path: Path, data: bytes) -> dict[str, str]:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return {"path": path.name, "sha256": hashlib.sha256(data).hexdigest()}


def _json(path: Path, payload: object) -> dict[str, str]:
    return _write(path, json.dumps(payload, sort_keys=True).encode())


def _fail(contract: dict, payload: dict, root: Path) -> None:
    try:
        validate_auto15_execution_evidence(contract, payload, root)
    except ProductAcceptanceContractError:
        return
    raise AssertionError("invalid AUTO-15 evidence must fail closed")


def _ledger(tmp_path: Path) -> tuple[dict, dict]:
    contract = load_contract()
    static = validate_static_contract(contract)
    run_root = tmp_path / "run"
    run_root.mkdir(exist_ok=True)
    snapshot = {
        "snapshot_manifest_sha256": "1" * 64,
        "source_inventory_sha256": "2" * 64,
        "expanded_urdf_sha256": "3" * 64,
    }
    session = {
        "report_id": "tzcup_formal_final_acceptance_session_v1",
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "started_epoch_ns": 1,
        "snapshot": snapshot,
    }
    session_ref = _json(tmp_path / "session.json", session)
    binding = {
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {
            "session_manifest_sha256": session_ref["sha256"],
            "snapshot": snapshot,
        },
    }
    binding_ref = _json(tmp_path / "binding.json", binding)
    provenance = {
        "formal_session": session_ref,
        "runtime_binding": binding_ref,
        "run_root": "run",
        "source_commit": "a" * 40,
        "source_tree": "b" * 40,
        "model_sha256": "c" * 64,
        "config_sha256": "d" * 64,
        "dataset_sha256": "e" * 64,
        "container_digest": "sha256:" + "f" * 64,
        "dependency_lock_sha256": "0" * 64,
    }
    execution_provenance = {
        "formal_session_sha256": session_ref["sha256"],
        "runtime_binding_sha256": binding_ref["sha256"],
        "snapshot": snapshot,
        "run_root": "run",
        **{key: provenance[key] for key in ("source_commit", "source_tree", "model_sha256", "config_sha256", "dataset_sha256", "container_digest", "dependency_lock_sha256")},
    }
    replay = {
        "schema": "tzcup.coverage_mcap_replay.v1",
        "bag_readable": True,
        "ros2_bag_play_exit_code": 0,
        "pass": True,
        "gates": {"ros2_bag_play_succeeded": True},
        "topic_message_counts": {topic: 1 for topic in REQUIRED_TOPICS},
    }
    executions, members = [], {f"group-{index}": [] for index in range(30)}
    for index, execution_id in enumerate(static["required_execution_ids"]):
        scenario_id, seed = execution_id.split(":seed-")
        media_dir = tmp_path / f"media-{index}"
        video_bytes = b"\x00\x00\x00\x18ftypisom" + str(index).encode() + b"v" * 100_000
        video = _write(media_dir / f"video-{index}.mp4", video_bytes)
        frame = _write(media_dir / f"frame-{index}.png", b"frame")
        video_audit = _json(media_dir / f"video-{index}.audit.json", {
            "stage": "AUTO-17", "status": "PASS", "machine_gate_pass": True,
            "video": {"path": video["path"], "bytes": len(video_bytes), "nonempty": True, "representative_frame": frame["path"]},
        })
        video["audit"] = {"path": f"media-{index}/{video_audit['path']}", "sha256": video_audit["sha256"]}
        video["path"] = f"media-{index}/{video['path']}"
        metadata = _write(media_dir / "metadata.yaml", f"# receipt {index}\nrosbag2_bagfile_information:\n  message_count: 1\n  duration:\n    nanoseconds: 1\n".encode())
        mcap_data = _write(media_dir / f"bag-{index}.mcap", MCAP_MAGIC + str(index).encode() + b"record" + MCAP_MAGIC)
        replay_ref = _json(media_dir / f"replay-{index}.json", {**replay, "receipt_index": index})
        mcap = {"data": {"path": f"media-{index}/{mcap_data['path']}", "sha256": mcap_data["sha256"]}, "metadata": {"path": f"media-{index}/{metadata['path']}", "sha256": metadata["sha256"]}, "replay": {"path": f"media-{index}/{replay_ref['path']}", "sha256": replay_ref["sha256"]}}
        group = f"group-{index % 30}"
        members[group].append(execution_id)
        executions.append({"scenario_id": scenario_id, "seed": int(seed), "mission_group_id": group, "status": "PASS", "provenance": execution_provenance, "command": ["ros2", "launch"], "exit_code": 0, "started_epoch_ns": 1, "finished_epoch_ns": 2, "video": video, "mcap": mcap, "replay": True})
    groups = []
    for group, group_members in members.items():
        receipt = _json(tmp_path / f"{group}.json", {"report_id": "tzcup_auto15_mission_group_receipt_v1", "status": "PASS", "mission_group_id": group, "members": group_members, "provenance": execution_provenance})
        groups.append({"mission_group_id": group, "members": group_members, "receipt": receipt})
    return contract, {"provenance": provenance, "executions": executions, "mission_groups": groups}


def test_matrix_is_complete_and_static_only() -> None:
    state = json.loads((ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json").read_text(encoding="utf-8"))
    matrix = build_matrix(state)
    assert matrix["scenario_count"] == 18
    assert matrix["required_unique_execution_count"] == 180
    assert len({item for row in matrix["scenarios"] for item in row["required_execution_ids"]}) == 180
    assert matrix["static_contract_pass"] is True
    assert matrix["simulation_competition_matrix_pass"] is False
    assert not any(matrix["product_runtime_states"].values())


def test_runtime_evidence_binds_media_replay_provenance_and_groups(tmp_path: Path) -> None:
    contract, payload = _ledger(tmp_path)
    result = validate_auto15_execution_evidence(contract, payload, tmp_path)
    assert result["retained_execution_count"] == 180
    assert result["mission_group_count"] == 30
    assert len(result["execution_to_mission_group"]) == 180
    assert all(value == {"executions": 10, "videos": 10, "mcaps": 10} for value in result["scenario_evidence_counts"].values())
    assert not any(result["product_runtime_states"].values())
    state = json.loads((ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json").read_text(encoding="utf-8"))
    matrix = build_matrix(state, payload, tmp_path)
    assert matrix["runtime_execution_evidence_pass"] is True
    assert all(row["scenario_seed_count"] == row["video_count"] == row["mcap_count"] == 10 for row in matrix["scenarios"])
    assert all(row["integrated_execution_status"] == "EVIDENCE_RETAINED_NOT_PRODUCT_ACCEPTED" for row in matrix["scenarios"])
    assert matrix["simulation_competition_matrix_pass"] is False

    payload["executions"].pop()
    _fail(contract, payload, tmp_path)


def test_rejects_alias_content_hardlink_traversal_fake_media_and_stale_provenance(tmp_path: Path) -> None:
    contract, payload = _ledger(tmp_path)
    variant = copy.deepcopy(payload)
    first, last = variant["executions"][0], variant["executions"][-1]
    last["video"] = first["video"]
    _fail(contract, variant, tmp_path)
    variant = copy.deepcopy(payload)
    first, last = variant["executions"][0], variant["executions"][-1]
    source = tmp_path / first["video"]["path"]
    linked = tmp_path / "hardlink.mp4"
    os.link(source, linked)
    last["video"]["path"] = "hardlink.mp4"
    last["video"]["sha256"] = first["video"]["sha256"]
    _fail(contract, variant, tmp_path)
    variant = copy.deepcopy(payload)
    copied = tmp_path / "copied.mp4"
    copied.write_bytes((tmp_path / variant["executions"][0]["video"]["path"]).read_bytes())
    variant["executions"][-1]["video"]["path"] = "copied.mp4"
    variant["executions"][-1]["video"]["sha256"] = variant["executions"][0]["video"]["sha256"]
    _fail(contract, variant, tmp_path)
    variant = copy.deepcopy(payload)
    variant["executions"][-1]["mcap"]["data"]["path"] = "../escape.mcap"
    _fail(contract, variant, tmp_path)
    variant = copy.deepcopy(payload)
    variant["executions"][-1]["mcap"]["data"]["sha256"] = "0" * 64
    _fail(contract, variant, tmp_path)
    variant = copy.deepcopy(payload)
    fake = tmp_path / "fake.mcap"
    fake_ref = _write(fake, b"eightbyte")
    variant["executions"][-1]["mcap"]["data"] = fake_ref
    _fail(contract, variant, tmp_path)
    variant = copy.deepcopy(payload)
    variant["executions"][-1]["mcap"]["replay"] = variant["executions"][0]["mcap"]["replay"]
    _fail(contract, variant, tmp_path)
    variant = copy.deepcopy(payload)
    variant["executions"][-1]["provenance"]["source_tree"] = "1" * 40
    _fail(contract, variant, tmp_path)
    variant = copy.deepcopy(payload)
    variant["mission_groups"] = variant["mission_groups"][:29]
    _fail(contract, variant, tmp_path)


def test_rejects_authoritative_source_drift(tmp_path: Path) -> None:
    contract = load_contract()
    source = tmp_path / "source.md"
    source.write_text((ROOT / "docs" / "a12-product-acceptance-specification.md").read_text(encoding="utf-8"), encoding="utf-8")
    assert validate_static_contract(contract, source)["static_contract_pass"] is True
    source.write_text(source.read_text(encoding="utf-8") + "drift\n", encoding="utf-8")
    try:
        validate_static_contract(contract, source)
    except ProductAcceptanceContractError:
        return
    raise AssertionError("authoritative source drift must fail closed")
