"""File-backed regression cases for independent mapping/cleaning handoff."""

import hashlib
import json

import pytest

from sanitation_formal_campus_integration.map_lifecycle_core import (
    hard_restart_record_valid,
)


def _write_json(root, name, value):
    path = root / name
    path.write_text(json.dumps(value), encoding="utf-8")
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def restart_evidence(tmp_path):
    manifest_hash = _write_json(tmp_path, "map_lifecycle_manifest.json", {
        "schema_version": 1,
        "status": "ready_for_localization_cleaning",
        "episode_id": "formal-life-001",
        "map_id": "train-map-000",
    })
    runtime_hash = _write_json(tmp_path, "mapping_runtime.json", {"passed": True})
    binding_hash = _write_json(tmp_path, "runtime_gate_binding.json", {
        "schema_version": 1, "session_id": "mapping-session-001",
        "source_sha256": "a" * 64,
    })
    handoff = {
        "schema_version": 2,
        "mapping_runner_completed": True,
        "mapping_runner_exit_code": 0,
        "mapping_process_groups_stopped": True,
        "mapping_runner_pid": 101,
        "mapping_launch_pid": 102,
        "mapping_collector_pid": 103,
        "mapping_ros_domain_id": 41,
        "mapping_gz_partition": "mapping_fixture",
        "mapping_completion_wall_time": "2026-08-28T10:00:00+00:00",
        "mapping_cleanup_wall_time": "2026-08-28T10:00:01+00:00",
        "map_lifecycle_manifest_sha256": manifest_hash,
        "mapping_runtime_sha256": runtime_hash,
        "mapping_runtime_gate_binding_sha256": binding_hash,
    }
    handoff_hash = _write_json(tmp_path, "mapping_handoff_record.json", handoff)
    record = {key: handoff[key] for key in (
        "schema_version", "mapping_runner_exit_code", "mapping_runner_pid",
        "mapping_launch_pid", "mapping_collector_pid",
        "mapping_completion_wall_time", "mapping_cleanup_wall_time",
        "map_lifecycle_manifest_sha256", "mapping_runtime_sha256",
        "mapping_runtime_gate_binding_sha256",
    )}
    record.update({
        "mapping_stopped_before_cleaning": True,
        "mapping_process_count_before_cleaning": 0,
        "mapping_pid_alive_count_before_cleaning": 0,
        "restart_type": "separate_process_hard_restart",
        "cleaning_runner_pid": 201,
        "cleaning_launch_pid": 202,
        "cleaning_start_wall_time": "2026-08-28T10:00:02+00:00",
        "cleaning_ros_domain_id": 42,
        "cleaning_gz_partition": "cleaning_fixture",
        "mapping_handoff_record_sha256": handoff_hash,
    })
    return tmp_path, handoff, record


def test_valid_file_backed_restart(restart_evidence):
    root, _, record = restart_evidence
    assert hard_restart_record_valid(record, root) is True


@pytest.mark.parametrize("handoff", [{}, [], None, "invalid"])
def test_handoff_requires_structured_completion_evidence(restart_evidence, handoff):
    root, _, record = restart_evidence
    record["mapping_handoff_record_sha256"] = _write_json(
        root, "mapping_handoff_record.json", handoff
    )
    assert hard_restart_record_valid(record, root) is False


@pytest.mark.parametrize(("field", "value"), [
    ("schema_version", 1),
    ("mapping_runner_completed", False),
    ("mapping_process_groups_stopped", False),
    ("mapping_runner_exit_code", 1),
    ("mapping_runner_exit_code", False),
    ("map_lifecycle_manifest_sha256", "0" * 64),
    ("mapping_runtime_sha256", "0" * 64),
    ("mapping_runtime_gate_binding_sha256", "0" * 64),
])
def test_rehashed_failed_handoff_is_rejected(restart_evidence, field, value):
    root, handoff, record = restart_evidence
    handoff[field] = value
    record["mapping_handoff_record_sha256"] = _write_json(
        root, "mapping_handoff_record.json", handoff
    )
    assert hard_restart_record_valid(record, root) is False


@pytest.mark.parametrize("action", ["missing", "tampered", "restart_rehashed", "handoff_rehashed"])
def test_mapping_runtime_binding_cannot_drift(restart_evidence, action):
    root, handoff, record = restart_evidence
    path = root / "runtime_gate_binding.json"
    if action == "missing":
        path.unlink()
    else:
        replacement_hash = _write_json(root, path.name, {
            "schema_version": 1, "session_id": "different-mapping-session",
            "source_sha256": "b" * 64,
        })
        if action == "restart_rehashed":
            record["mapping_runtime_gate_binding_sha256"] = replacement_hash
        elif action == "handoff_rehashed":
            handoff["mapping_runtime_gate_binding_sha256"] = replacement_hash
            record["mapping_handoff_record_sha256"] = _write_json(
                root, "mapping_handoff_record.json", handoff
            )
    assert hard_restart_record_valid(record, root) is False


@pytest.mark.parametrize(("field", "value"), [
    ("mapping_runner_pid", 501),
    ("mapping_launch_pid", 502),
    ("mapping_collector_pid", 503),
    ("mapping_completion_wall_time", "2026-08-28T09:59:59+00:00"),
    ("mapping_cleanup_wall_time", "2026-08-28T10:00:00+00:00"),
])
def test_restart_cannot_substitute_mapping_identity(restart_evidence, field, value):
    root, _, record = restart_evidence
    record[field] = value
    assert hard_restart_record_valid(record, root) is False


@pytest.mark.parametrize("field", [
    "mapping_runner_pid", "mapping_launch_pid", "mapping_collector_pid",
    "cleaning_runner_pid", "cleaning_launch_pid",
    "mapping_process_count_before_cleaning", "mapping_pid_alive_count_before_cleaning",
    "mapping_runner_exit_code",
])
def test_boolean_is_not_a_process_identifier_or_exit_count(restart_evidence, field):
    root, handoff, record = restart_evidence
    value = True if field.endswith("_pid") else False
    record[field] = value
    if field in handoff:
        handoff[field] = value
        record["mapping_handoff_record_sha256"] = _write_json(
            root, "mapping_handoff_record.json", handoff
        )
    assert hard_restart_record_valid(record, root) is False


@pytest.mark.parametrize("record", [None, [], ["bad"], "bad", {}, 3])
def test_malformed_restart_is_false_without_exception(restart_evidence, record):
    root, _, _ = restart_evidence
    assert hard_restart_record_valid(record, root) is False


@pytest.mark.parametrize("value", [[], {}, [101], {"pid": 101}])
def test_unhashable_pid_is_false_without_exception(restart_evidence, value):
    root, _, record = restart_evidence
    record["mapping_runner_pid"] = value
    assert hard_restart_record_valid(record, root) is False


@pytest.mark.parametrize("naive_fields", [
    ("mapping_completion_wall_time",),
    ("mapping_cleanup_wall_time",),
    ("cleaning_start_wall_time",),
    ("mapping_completion_wall_time", "mapping_cleanup_wall_time", "cleaning_start_wall_time"),
])
def test_naive_or_mixed_wall_times_are_rejected(restart_evidence, naive_fields):
    root, handoff, record = restart_evidence
    for field in naive_fields:
        record[field] = record[field].replace("+00:00", "")
        if field in handoff:
            handoff[field] = record[field]
    record["mapping_handoff_record_sha256"] = _write_json(
        root, "mapping_handoff_record.json", handoff
    )
    assert hard_restart_record_valid(record, root) is False
