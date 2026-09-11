from __future__ import annotations

import json
from pathlib import Path

import formal_product_mcap_replay as replay


ROOT = Path(__file__).resolve().parents[1]


def test_producer_rejects_missing_actual_product_topics(tmp_path: Path, monkeypatch) -> None:
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    metrics = tmp_path / "metrics.json"
    metrics.write_text("{}", encoding="utf-8")
    context = {
        "session": {"started_epoch_ns": 1},
        "snapshot": {},
        "runtime_closure_binding": {},
    }
    monkeypatch.setattr(replay, "bind_formal_context", lambda *args: context)
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
        input_hashes={name: "a" * 64 for name in ("model", "config", "dataset", "container", "dependency")},
        domain_id=151,
        playback_rate=1.0,
        timeout_seconds=1.0,
        reader=lambda path: records,
        player=lambda *args, **kwargs: {"exit_code": 0},
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
