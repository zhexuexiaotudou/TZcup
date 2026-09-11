import importlib.util
import json
from pathlib import Path

import yaml


SCRIPT = Path(__file__).with_name("finalize_formal_first_map_localization_diagnostic.py")
SPEC = importlib.util.spec_from_file_location("first_map_diagnostic", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _capture(root: Path, *, counts=None) -> tuple[Path, Path]:
    bag = root / "mapping_localization_diagnostic"
    bag.mkdir(parents=True)
    (bag / "mapping_localization_diagnostic_0.mcap").write_bytes(
        b"\x89MCAP0\r\nminimal-test-payload\x89MCAP0\r\n"
    )
    expected = {topic: 1 for topic in MODULE.REQUIRED_TOPICS}
    if counts:
        expected.update(counts)
    metadata = {
        "rosbag2_bagfile_information": {
            "storage_identifier": "mcap",
            "relative_file_paths": ["mapping_localization_diagnostic_0.mcap"],
            "starting_time": {"nanoseconds_since_epoch": 200},
            "duration": {"nanoseconds": 100},
            "message_count": sum(expected.values()),
            "topics_with_message_count": [
                {
                    "topic_metadata": {
                        "name": topic,
                        "type": MODULE.EXPECTED_TOPIC_TYPES[topic],
                        "serialization_format": "cdr",
                    },
                    "message_count": count,
                }
                for topic, count in expected.items()
            ],
        }
    }
    (bag / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    topics = root / "mapping_localization_diagnostic.topics"
    topics.write_text("\n".join(MODULE.REQUIRED_TOPICS) + "\n", encoding="utf-8")
    (root / "runtime_gate_binding.json").write_text(
        json.dumps({"verified_epoch_ns": 100}), encoding="utf-8"
    )
    return bag, topics


def test_finalize_seals_mcap_hashes_and_required_topic_counts(tmp_path):
    bag, topics = _capture(tmp_path)
    receipt = tmp_path / "mapping_localization_diagnostic.json"
    report = MODULE.finalize(
        run_root=tmp_path,
        bag_dir=bag,
        topic_manifest=topics,
        output=receipt,
        recorder_stop_rc=0,
        optional_topics=(),
        runtime_binding=tmp_path / "runtime_gate_binding.json",
        started_epoch_ns=200,
    )
    assert report["passed"] is True
    assert report["storage_identifier"] == "mcap"
    assert report["world_truth_used_for_control"] is False
    assert set(report["bag_files_sha256"]) == {
        "metadata.yaml", "mapping_localization_diagnostic_0.mcap"
    }
    assert json.loads(receipt.read_text(encoding="utf-8"))["passed"] is True


def test_finalize_fails_closed_for_missing_required_data_or_bad_recorder_rc(tmp_path):
    bag, topics = _capture(tmp_path, counts={"/odometry/gps": 0})
    receipt = tmp_path / "mapping_localization_diagnostic.json"
    report = MODULE.finalize(
        run_root=tmp_path,
        bag_dir=bag,
        topic_manifest=topics,
        output=receipt,
        recorder_stop_rc=0,
        optional_topics=(),
        runtime_binding=tmp_path / "runtime_gate_binding.json",
        started_epoch_ns=200,
    )
    assert report["passed"] is False
    assert "/odometry/gps" in report["blockers"][0]

    second_root = tmp_path / "second"
    bag, topics = _capture(second_root)
    report = MODULE.finalize(
        run_root=second_root,
        bag_dir=bag,
        topic_manifest=topics,
        output=second_root / "mapping_localization_diagnostic.json",
        recorder_stop_rc=1,
        optional_topics=(),
        runtime_binding=second_root / "runtime_gate_binding.json",
        started_epoch_ns=200,
    )
    assert report["passed"] is False
    assert "recorder stop failed" in report["blockers"][0]


def test_finalize_rejects_fake_mcap_and_inconsistent_metadata(tmp_path):
    bag, topics = _capture(tmp_path)
    receipt = tmp_path / "mapping_localization_diagnostic.json"
    (bag / "mapping_localization_diagnostic_0.mcap").write_bytes(b"mcap")
    report = MODULE.finalize(
        run_root=tmp_path,
        bag_dir=bag,
        topic_manifest=topics,
        output=receipt,
        recorder_stop_rc=0,
        optional_topics=(),
        runtime_binding=tmp_path / "runtime_gate_binding.json",
        started_epoch_ns=200,
    )
    assert report["passed"] is False
    assert "MCAP magic" in report["blockers"][0]

    second_root = tmp_path / "second"
    bag, topics = _capture(second_root)
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    metadata["rosbag2_bagfile_information"]["message_count"] = 1
    metadata_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
    report = MODULE.finalize(
        run_root=second_root,
        bag_dir=bag,
        topic_manifest=topics,
        output=second_root / "mapping_localization_diagnostic.json",
        recorder_stop_rc=0,
        optional_topics=(),
        runtime_binding=second_root / "runtime_gate_binding.json",
        started_epoch_ns=200,
    )
    assert report["passed"] is False
    assert "total message count" in report["blockers"][0]


def test_finalize_rejects_type_or_freshness_mismatch(tmp_path):
    bag, topics = _capture(tmp_path)
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    metadata["rosbag2_bagfile_information"]["topics_with_message_count"][0]["topic_metadata"]["type"] = "wrong/type"
    metadata_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
    report = MODULE.finalize(
        run_root=tmp_path,
        bag_dir=bag,
        topic_manifest=topics,
        output=tmp_path / "mapping_localization_diagnostic.json",
        recorder_stop_rc=0,
        optional_topics=(),
        runtime_binding=tmp_path / "runtime_gate_binding.json",
        started_epoch_ns=200,
    )
    assert report["passed"] is False
    assert "topic contract" in report["blockers"][0]

    second_root = tmp_path / "second"
    bag, topics = _capture(second_root)
    report = MODULE.finalize(
        run_root=second_root,
        bag_dir=bag,
        topic_manifest=topics,
        output=second_root / "mapping_localization_diagnostic.json",
        recorder_stop_rc=0,
        optional_topics=(),
        runtime_binding=second_root / "runtime_gate_binding.json",
        started_epoch_ns=6_000_000_000,
    )
    assert report["passed"] is False
    assert "time window" in report["blockers"][0]


def test_finalize_rejects_bag_or_receipt_outside_the_fresh_run_root(tmp_path):
    bag, topics = _capture(tmp_path)
    report = MODULE.finalize(
        run_root=tmp_path,
        bag_dir=bag,
        topic_manifest=topics,
        output=tmp_path.parent / "escaped.json",
        recorder_stop_rc=0,
        optional_topics=(),
        runtime_binding=tmp_path / "runtime_gate_binding.json",
        started_epoch_ns=200,
    )
    assert report["passed"] is False
    assert "direct child of run root" in report["blockers"][0]
