from __future__ import annotations

import json
import os
import time
from array import array
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

import formal_product_mcap_replay as replay


ROOT = Path(__file__).resolve().parents[1]


def test_formal_diagnostics_and_verified_grasp_results_are_decoded_without_legacy_fields():
    chain = {"scheduler_states": set(), "grasp_target_ids": set(),
             "verified_grasp_target_ids": set(), "post_clean_success_count": 0}
    replay.update_product_chain(chain, "/active_cleaning/planner_status", NS(status=[
        NS(name="other", message="COMPLETE"),
        NS(name="formal_active_cleaning_policy_planner", message="WAITING_GRASP")]))
    assert chain["scheduler_states"] == {"WAITING_GRASP"}
    for verified in (False, True, True):
        replay.update_product_chain(chain, "/active_cleaning/grasp_result", NS(data=json.dumps({
            "schema_version": 2, "target_id": "cube-1", "verified_in_bin": verified})))
    assert chain["post_clean_success_count"] == 1
    assert chain["verified_grasp_target_ids"] == {"cube-1"}
    with pytest.raises(replay.ProductReplayError, match="malformed"):
        replay.update_product_chain(chain, "/active_cleaning/grasp_result", NS(data=json.dumps({
            "schema_version": 2, "target_id": "cube-2", "verified_in_bin": "true"})))


def _fixture_capture_guards(monkeypatch) -> None:
    monkeypatch.setattr(replay, "RAW_CAPTURE_PRODUCER_ID", replay.PRODUCER_ID)
    monkeypatch.setattr(replay, "_process_group_survivors", lambda _pgid: [])
    monkeypatch.setattr(replay, "mcap_semantic_sha256", lambda bag: replay.sha256(next(Path(bag).glob("*.mcap"))))


def _raw_capture_receipt(tmp_path: Path) -> tuple[Path, Path, Path, Path, dict]:
    bag = tmp_path / "capture" / "bag"
    bag.mkdir(parents=True)
    (bag / "capture_0.mcap").write_bytes(b"mcap-fixture")
    (bag / "metadata.yaml").write_text("""rosbag2_bagfile_information:
  storage_identifier: mcap
  relative_file_paths: [capture_0.mcap]
  files: [{path: capture_0.mcap}]
  topics_with_message_count: []
""", encoding="utf-8")
    video = tmp_path / "capture" / "product.mp4"
    video.write_bytes(b"real-video-fixture")
    metrics = tmp_path / "capture" / "source_metrics.json"
    metrics.write_text("{}", encoding="utf-8")
    now = time.time_ns()
    for artifact in (bag, video, metrics):
        os.utime(artifact, ns=(now - 200_000_000, now - 200_000_000))
    context = {
        "source_commit": "a" * 40,
        "source_tree": "b" * 40,
        "session": {"path": str(tmp_path / "session.json"), "sha256_at_replay": "c" * 64, "report_id": "tzcup_formal_final_acceptance_session_v1", "status_at_replay": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "started_epoch_ns": now - 1_000_000_000},
        "snapshot": {},
        "snapshot_manifest": {"path": str(tmp_path / "snapshot.json"), "sha256": "d" * 64},
        "runtime_closure_binding": {},
        "runtime_gate_binding": {"path": str(tmp_path / "binding.json"), "sha256": "e" * 64},
    }
    episode = tmp_path / "episode.json"
    episode.write_text(json.dumps({"a12_execution": {"scenario_id": "mapping", "seed": 0, "mission_id": "mission-00", "mission_group_id": "mission-group-00"}}), encoding="utf-8")
    evaluator = tmp_path / "evaluator.json"
    evaluator.write_text(json.dumps({"seeds": {"dirt": 0}}), encoding="utf-8")
    binding = tmp_path / "input_binding.json"
    binding.write_text(json.dumps({"artifacts": {
        "episode_manifest": {"kind": "file", "path": str(episode), "sha256": replay.sha256(episode)},
        "evaluator_manifest": {"kind": "file", "path": str(evaluator), "sha256": replay.sha256(evaluator)},
    }}), encoding="utf-8")
    receipt = tmp_path / "raw_capture_receipt.json"
    receipt.write_text(json.dumps({
        "schema": replay.RAW_CAPTURE_SCHEMA,
        "status": "A12_RAW_CAPTURE_COMPLETE",
        "capture_complete": True,
        "capture_id": "f" * 32,
        "producer": {"id": replay.PRODUCER_ID, "sha256": replay.sha256(ROOT / replay.PRODUCER_ID)},
        "publication": {"method": "atomic_link_no_replace", "overwrote_existing": False},
        "run_root": str(tmp_path),
        "formal_context": context,
        "scenario_id": "mapping",
        "seed": 0,
        "mission_id": "mission-00",
        "started_epoch_ns": now - 500_000_000,
        "finished_epoch_ns": now,
        "dds": {"ros_domain_id": 215, "ros_localhost_only": True, "rmw_implementation": "rmw_cyclonedds_cpp", "automatic_discovery_range": "LOCALHOST"},
        "process_group_cleanup": {"process_group_id": 123, "process_group_isolated": True, "zero_survivor": True, "surviving_group_processes": 0, "cleanup_completed_epoch_ns": now},
        "post_run_pgid_census": {"method": "procfs_post_cleanup_pgid_census_v1", "process_group_id": 123, "surviving_process_ids": [], "census_epoch_ns": now},
        "video": {"path": str(video), "sha256": replay.sha256(video)},
        "mcap": {"path": str(bag), "sha256": replay.mcap_sha256(bag)},
        "source_metrics": {"path": str(metrics), "sha256": replay.sha256(metrics)},
        "episode_input_binding": {"path": str(binding), "sha256": replay.sha256(binding)},
    }), encoding="utf-8")
    return receipt, bag, video, metrics, context


def test_raw_capture_receipt_binds_current_context_content_and_formal_dds(tmp_path: Path, monkeypatch) -> None:
    _fixture_capture_guards(monkeypatch)
    receipt, bag, video, metrics, context = _raw_capture_receipt(tmp_path)
    verified = replay.validate_raw_capture_receipt(
        repository_root=ROOT,
        raw_capture_receipt_path=receipt,
        formal_context=context,
        scenario_id="mapping",
        seed=0,
        mission_id="mission-00",
        run_root=tmp_path,
    )
    assert verified["mcap"]["path"] == str(bag)
    assert verified["video"]["path"] == str(video)
    assert verified["source_metrics"]["path"] == str(metrics)
    (bag / "metadata.yaml").write_text("mutated", encoding="utf-8")
    try:
        replay.validate_raw_capture_receipt(
            repository_root=ROOT,
            raw_capture_receipt_path=receipt,
            formal_context=context,
            scenario_id="mapping",
            seed=0,
            mission_id="mission-00",
            run_root=tmp_path,
        )
    except replay.ProductReplayError as exc:
        assert "MCAP" in str(exc) or "content hash mismatch" in str(exc)
    else:
        raise AssertionError("a modified MCAP must invalidate raw capture evidence")


def test_replay_refuses_nonformal_safe_dds_domain() -> None:
    try:
        replay.play_mcap(Path("bag"), domain_id=151, rate=1.0, timeout_seconds=1.0)
    except replay.ProductReplayError as exc:
        assert "formal-safe" in str(exc)
    else:
        raise AssertionError("replay must not run in a non-formal DDS domain")


def test_raw_capture_allows_primary_formal_domain_but_not_unsafe_domain(tmp_path: Path, monkeypatch) -> None:
    _fixture_capture_guards(monkeypatch)
    receipt, _bag, _video, _metrics, context = _raw_capture_receipt(tmp_path)
    payload = json.loads(receipt.read_text(encoding="utf-8"))
    payload["dds"]["ros_domain_id"] = 60
    receipt.write_text(json.dumps(payload), encoding="utf-8")
    assert replay.validate_raw_capture_receipt(
        repository_root=ROOT,
        raw_capture_receipt_path=receipt,
        formal_context=context,
        scenario_id="mapping",
        seed=0,
        mission_id="mission-00",
        run_root=tmp_path,
    )["dds"]["ros_domain_id"] == 60
    payload["dds"]["ros_domain_id"] = 151
    receipt.write_text(json.dumps(payload), encoding="utf-8")
    try:
        replay.validate_raw_capture_receipt(
            repository_root=ROOT,
            raw_capture_receipt_path=receipt,
            formal_context=context,
            scenario_id="mapping",
            seed=0,
            mission_id="mission-00",
            run_root=tmp_path,
        )
    except replay.ProductReplayError as exc:
        assert "DDS isolation" in str(exc)
    else:
        raise AssertionError("unsafe capture DDS domains must be rejected")


def test_raw_capture_rejects_arbitrary_producer_old_media_and_forged_identity(tmp_path: Path, monkeypatch) -> None:
    receipt, bag, _video, _metrics, context = _raw_capture_receipt(tmp_path)
    # A receipt naming the replay producer is not accepted until it is the
    # exact approved capture producer; no arbitrary repository file may attest.
    try:
        replay.validate_raw_capture_receipt(
            repository_root=ROOT, raw_capture_receipt_path=receipt, formal_context=context,
            scenario_id="mapping", seed=0, mission_id="mission-00", run_root=tmp_path,
        )
    except replay.ProductReplayError as exc:
        assert "producer" in str(exc)
    else:
        raise AssertionError("an arbitrary repository producer must not self-attest capture")
    _fixture_capture_guards(monkeypatch)
    payload = json.loads(receipt.read_text(encoding="utf-8"))
    payload["scenario_id"] = "forged-scenario"
    receipt.write_text(json.dumps(payload), encoding="utf-8")
    try:
        replay.validate_raw_capture_receipt(
            repository_root=ROOT, raw_capture_receipt_path=receipt, formal_context=context,
            scenario_id="mapping", seed=0, mission_id="mission-00", run_root=tmp_path,
        )
    except replay.ProductReplayError as exc:
        assert "immutable bound episode" in str(exc)
    else:
        raise AssertionError("receipt labels must not override the bound episode")
    payload["scenario_id"] = "mapping"
    receipt.write_text(json.dumps(payload), encoding="utf-8")
    # An old/copyable tree cannot smuggle arbitrary payload beside an MCAP.
    (bag / "copied-old-media.mcap").write_bytes(b"old-media")
    try:
        replay.validate_raw_capture_receipt(
            repository_root=ROOT, raw_capture_receipt_path=receipt, formal_context=context,
            scenario_id="mapping", seed=0, mission_id="mission-00", run_root=tmp_path,
        )
    except replay.ProductReplayError as exc:
        assert "undeclared" in str(exc)
    else:
        raise AssertionError("extra MCAP files must fail closed")


def test_mcap_semantic_digest_detects_renamed_duplicate_bag(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(replay, "mcap_semantic_sha256", lambda bag: replay.sha256(next(Path(bag).glob("*.mcap"))))
    def write_bag(path: Path, filename: str) -> None:
        path.mkdir()
        (path / filename).write_bytes(b"same-recorded-mcap")
        (path / "metadata.yaml").write_text(f"""rosbag2_bagfile_information:
  storage_identifier: mcap
  relative_file_paths: [{filename}]
  files: [{{path: {filename}}}]
  topics_with_message_count: []
""", encoding="utf-8")
    first, second = tmp_path / "first", tmp_path / "second"
    write_bag(first, "one.mcap")
    write_bag(second, "renamed.mcap")
    assert replay.mcap_sha256(first) != replay.mcap_sha256(second)
    assert replay.mcap_semantic_sha256(first) == replay.mcap_semantic_sha256(second)


def test_replay_timeout_is_failure_even_if_sigterm_exits_zero(monkeypatch) -> None:
    class Process:
        pid = 999
        returncode = 0
        calls = 0

        def communicate(self, timeout=None):
            self.calls += 1
            if self.calls == 1:
                raise replay.subprocess.TimeoutExpired("ros2", timeout)
            return "", ""

    monkeypatch.setattr(replay.os, "name", "posix")
    class AbsolutePath:
        def __init__(self, _value):
            pass

        def is_absolute(self):
            return True
    monkeypatch.setattr(replay, "Path", AbsolutePath)
    monkeypatch.setattr(replay, "_non_link_path", lambda _path, _label: ROOT / replay.PRODUCER_ID)
    monkeypatch.setenv("FORMAL_ROS2_EXECUTABLE", "/trusted/ros2")
    monkeypatch.setattr(replay.subprocess, "Popen", lambda *args, **kwargs: Process())
    monkeypatch.setattr(replay, "_terminate_process_group", lambda pgid, signals: {
        "process_group_id": pgid, "process_group_isolated": True,
        "surviving_group_processes": 0, "zero_survivor": True,
    })
    try:
        replay.play_mcap(Path("bag"), domain_id=215, rate=1.0, timeout_seconds=0.01)
    except replay.ProductReplayError as exc:
        assert "timeout" in str(exc)
    else:
        raise AssertionError("timeout followed by exit 0 must not pass")


def test_proc_stat_uses_final_parenthesis_and_fails_closed() -> None:
    assert replay._parse_proc_stat_process_group("42 (worker ) strange) S 1 321 4 5") == 321
    for payload in ("42 (broken S 1 321", "42 (worker) S 1 nope"):
        try:
            replay._parse_proc_stat_process_group(payload)
        except replay.ProductReplayError:
            pass
        else:
            raise AssertionError("unreadable/malformed proc stat must fail closed")


def test_producer_rejects_missing_actual_product_topics(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(replay, "mcap_semantic_sha256", lambda bag: replay.sha256(next(Path(bag).glob("*.mcap"))))
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "capture_0.mcap").write_bytes(b"mcap-fixture")
    (bag / "metadata.yaml").write_text("""rosbag2_bagfile_information:
  storage_identifier: mcap
  relative_file_paths: [capture_0.mcap]
  files: [{path: capture_0.mcap}]
  topics_with_message_count: []
""", encoding="utf-8")
    metrics = tmp_path / "metrics.json"
    metrics.write_text("{}", encoding="utf-8")
    context = {
        "session": {"started_epoch_ns": 1},
        "snapshot": {},
        "runtime_closure_binding": {},
    }
    monkeypatch.setattr(replay, "bind_formal_context", lambda *args: context)
    raw_capture = {
        "capture_id": "a" * 32,
        "run_root": str(tmp_path),
        "video": {"path": str(tmp_path / "video.mp4"), "sha256": "b" * 64},
        "mcap": {"path": str(bag), "sha256": replay.mcap_sha256(bag), "semantic_sha256": replay.mcap_semantic_sha256(bag)},
        "source_metrics": {"path": str(metrics), "sha256": replay.sha256(metrics)},
    }
    monkeypatch.setattr(replay, "validate_raw_capture_receipt", lambda **kwargs: raw_capture)
    monkeypatch.setattr(replay, "recalculate", lambda records, source: {
        "coverage": {"brush_on_sample_count": 1, "relative_delta": 0.0},
        "localization": {"matched_sample_count": 1, "relative_delta": 0.0},
    })
    records = {
        "topic_types": {"/coverage/evaluation_sample": "std_msgs/msg/String"},
        "product_topic_counts": {topic: 0 for topic in replay.PRODUCT_TOPICS},
        "coverage_states": ["COMPLETED"],
    }
    report = replay.produce_report(
        repository_root=ROOT,
        bag=bag,
        source_metrics_path=metrics,
        session_path=tmp_path / "session.json",
        snapshot_path=tmp_path / "snapshot.json",
        runtime_binding_path=tmp_path / "binding.json",
        raw_capture_receipt_path=tmp_path / "raw_capture_receipt.json",
        scenario_id="mapping",
        seed=0,
        mission_id="mission-00",
        run_root=tmp_path,
        input_hashes={name: "a" * 64 for name in ("model", "config", "dataset", "container", "dependency")},
        domain_id=215,
        playback_rate=1.0,
        timeout_seconds=1.0,
        reader=lambda path: records,
        player=lambda *args, **kwargs: {"exit_code": 0, "timed_out": False},
    )
    assert report["pass"] is False
    assert report["checks"]["required_product_and_metric_topics_present"] is False
    assert report["checks"]["product_chain_observed"] is False


def test_fresh_writer_never_replaces_retained_replay(tmp_path: Path) -> None:
    output = tmp_path / "replay.json"
    replay.write_fresh_json(output, {"first": True})
    try:
        replay.write_fresh_json(output, {"second": True})
    except replay.ProductReplayError:
        pass
    else:
        raise AssertionError("retained replay must not be replaced")
    assert json.loads(output.read_text(encoding="utf-8")) == {"first": True}


def test_cli_provenance_hashes_actual_artifacts_instead_of_accepting_claimed_hashes(tmp_path: Path) -> None:
    values = []
    for name in ("model", "config", "dataset", "dependency"):
        artifact = tmp_path / name
        artifact.write_text(name, encoding="utf-8")
        values.append(f"{name}={artifact}")
    hashes, references = replay._parse_input_artifacts(values, "sha256:" + "f" * 64)
    assert hashes["container"] == "f" * 64
    assert set(references) == {"model", "config", "dataset", "dependency"}
    assert all(hashes[name] == references[name]["sha256"] for name in references)


def test_full_width_brush_requires_both_lateral_brushes() -> None:
    assert replay.full_width_brush_active(array("d", [8.0, -8.0, 0.0])) is True
    assert replay.full_width_brush_active([0.0, 0.0, 0.0]) is False
    assert replay.full_width_brush_active([8.0, 0.0, 12.0]) is False
    assert replay.full_width_brush_active([0.0, -8.0, 12.0]) is False
    assert replay.full_width_brush_active([8.0, -8.0, 0.0]) is True
    assert replay.full_width_brush_active([-8.0, -8.0, 0.0]) is True
    for invalid in ("8,-8,0", b"8,-8,0", [8.0, -8.0], [float("nan"), -8.0, 0.0]):
        try:
            replay.full_width_brush_active(invalid)
        except replay.ProductReplayError:
            pass
        else:
            raise AssertionError("invalid brush array must fail closed")


def test_replay_uses_post_safety_commands_and_frozen_dual_brush_centers() -> None:
    source = (ROOT / "scripts/formal_product_mcap_replay.py").read_text(encoding="utf-8")
    capture = (ROOT / "scripts/formal_a12_single_execution_capture.py").read_text(encoding="utf-8")
    assert replay.FROZEN_DUAL_BRUSH_CENTERS_BASE_LINK_M == ((0.385, 0.545), (0.385, -0.545))
    assert '"/brush_controller/commands"' in source
    assert '"/safety/command/brush"' not in source
    assert '"/brush_controller/commands"' in capture
    assert '"/safety/command/brush"' not in capture
