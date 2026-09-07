import copy
import json
from pathlib import Path

from a20_release_replay_receipt import HASH_FIELDS, SCHEMA, validate_receipt


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "config/high_fidelity_vehicle/a20_release_replay_receipt_contract.json"


def _hash(character: str) -> str:
    return character * 64


def _receipt() -> dict:
    hashes = dict(zip(HASH_FIELDS, (_hash(value) for value in "abcde")))
    snapshot = {
        "snapshot_manifest_sha256": _hash("f"),
        "source_inventory_sha256": hashes["source"],
        "expanded_urdf_sha256": _hash("0"),
    }
    closure = {"status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED", "passed": True, "closure_sha256": _hash("1")}
    replays = [
        {
            "bag_sha256": _hash(str(index + 2)),
            "product_replay": True,
            "audit": {"schema": "tzcup.coverage_mcap_replay.v1", "pass": True, "gates": {"ros2_bag_play_succeeded": True}},
            "input_hashes": hashes,
            "snapshot": snapshot,
            "closure_sha256": closure["closure_sha256"],
        }
        for index in range(5)
    ]
    return {
        "schema": SCHEMA,
        "input_hashes": hashes,
        "frozen_snapshot": snapshot,
        "sealed_final_session": {
            "report_id": "tzcup_formal_final_acceptance_session_v1",
            "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE",
            "snapshot": snapshot,
            "runtime_closure_binding": {"closure_sha256": closure["closure_sha256"]},
        },
        "current_runtime_closure": closure,
        "product_replays": replays,
        "release_artifact": {
            "status": "RELEASE_PACKAGE_ARTIFACT_RECORDED",
            "main_commit": "2" * 40,
            "rollback_commit": "3" * 40,
            "archive_sha256": _hash("4"),
            "sbom_sha256": _hash("5"),
        },
        "verified_rollback_exercise": {
            "status": "ROLLBACK_EXERCISE_VERIFIED",
            "verified": True,
            "rollback_commit": "3" * 40,
            "verification_report_sha256": _hash("6"),
        },
    }


def test_contract_and_pure_validator_are_fail_closed_and_non_promoting() -> None:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    report = validate_receipt(_receipt())

    assert contract["minimum_distinct_product_replay_bags"] == 5
    assert contract["required_input_hashes"] == list(HASH_FIELDS)
    assert report["valid"] is True
    assert report["status"] == "A20_RECEIPT_STATIC_VALID"
    assert report["release_runtime_pass"] is False


def test_receipt_rejects_copied_bags_missing_hashes_noncurrent_closure_and_rollback() -> None:
    receipt = copy.deepcopy(_receipt())
    receipt["input_hashes"].pop("dataset")
    for replay in receipt["product_replays"]:
        replay["bag_sha256"] = _hash("9")
    receipt["product_replays"][0]["closure_sha256"] = _hash("8")
    receipt["verified_rollback_exercise"]["verified"] = False

    report = validate_receipt(receipt)

    assert report["valid"] is False
    assert "missing or invalid dataset hash" in report["errors"]
    assert "fewer than five distinct product replay bags" in report["errors"]
    assert "replays[0] has a non-current closure" in report["errors"]
    assert "rollback exercise is not verified" in report["errors"]


def test_receipt_requires_current_sealed_session_and_passing_replay_audits() -> None:
    receipt = copy.deepcopy(_receipt())
    receipt["sealed_final_session"]["status"] = "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    receipt["product_replays"][0]["audit"]["gates"]["ros2_bag_play_succeeded"] = False

    report = validate_receipt(receipt)

    assert report["valid"] is False
    assert "sealed final session is not complete" in report["errors"]
    assert "replays[0] replay audit is not passing" in report["errors"]
