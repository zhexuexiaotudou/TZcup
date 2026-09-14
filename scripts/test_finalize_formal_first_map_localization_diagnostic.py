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
        require_writer_timing=False,  # Explicit historical fixture, not candidate acceptance.
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
        require_writer_timing=False,  # Explicit historical fixture, not candidate acceptance.
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
        require_writer_timing=False,  # Explicit historical fixture, not candidate acceptance.
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
        require_writer_timing=False,  # Explicit historical fixture, not candidate acceptance.
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
        require_writer_timing=False,  # Explicit historical fixture, not candidate acceptance.
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
        require_writer_timing=False,  # Explicit historical fixture, not candidate acceptance.
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
        require_writer_timing=False,  # Explicit historical fixture, not candidate acceptance.
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
        require_writer_timing=False,  # Explicit historical fixture, not candidate acceptance.
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


def test_sealed_writer_window_rejects_earlier_storage_without_slack(tmp_path):
    import hashlib
    import pytest
    bag = tmp_path / 'mapping_localization_diagnostic'; bag.mkdir()
    sample = {'epoch_ns': 1000, 'monotonic_ns': 2000}
    opened = {'timing_schema_version':1,'acceptance_window_start_epoch_ns':1000,
              'owner_identity':{'pid':1},'run_token':'token','formal_acceptance_session':'session',
              'writer_ready_sample':sample,'bag_dir':str(bag)}
    timing = {k:sample for k in ['process_start_sample','writer_ready_sample','open_receipt_sealed_sample','subscription_created_sample','spin_start_sample']}
    timing.update(owner_identity=opened['owner_identity'],run_token='token',formal_acceptance_session='session',acceptance_window_start_epoch_ns=1000)
    (tmp_path/'formal_localization_writer_open.json').write_text(json.dumps(opened))
    path=tmp_path/'formal_localization_timing_start.json';path.write_text(json.dumps(timing))
    (tmp_path/'formal_localization_timing_closed.json').write_text(json.dumps({'timing_start_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'closed_sample':{'epoch_ns':3000,'monotonic_ns':4000},'callbacks':[]}))
    with pytest.raises(ValueError,match='outside sealed writer-ready window'):
        MODULE._verify_writer_timing(tmp_path,bag,999,2000)


def test_required_marker_cannot_fall_back_to_legacy(tmp_path):
    import pytest
    bag = tmp_path / 'bag'; bag.mkdir()
    with pytest.raises(ValueError, match='marker missing'):
        MODULE._verify_writer_timing(tmp_path, bag, 1000, 2000)
    marker = tmp_path / 'formal_localization_writer_open.json'
    marker.write_text('{}')
    with pytest.raises(ValueError, match='downgraded'):
        MODULE._verify_writer_timing(tmp_path, bag, 1000, 2000)
    assert MODULE._verify_writer_timing(tmp_path, bag, 1000, 2000, require_writer_timing=False)['verified'] is False


def test_callback_sequence_and_close_bounds_fail_closed(tmp_path):
    import hashlib
    import pytest
    # Reuse the sealed-window fixture; it deliberately rejects before importing ROS.
    test_sealed_writer_window_rejects_earlier_storage_without_slack(tmp_path)
    path = tmp_path / 'formal_localization_timing_closed.json'
    base = json.loads(path.read_bytes())
    row = dict(sequence=1, topic='/x', receive_epoch_ns=1500, receive_monotonic_ns=2500,
               serialized_sha256=hashlib.sha256(b'x').hexdigest(), write_completed=True)
    cases = [
        ([dict(row, sequence=2)], base['closed_sample'], 'sequence'),
        ([dict(row, receive_monotonic_ns=4001)], base['closed_sample'], 'callback'),
        ([row], dict(epoch_ns=3000, monotonic_ns=1999), 'close clock'),
        ([row, dict(row, sequence=2, receive_monotonic_ns=2400)], base['closed_sample'], 'callback'),
    ]
    for rows, close, error in cases:
        data = dict(base, callbacks=rows, closed_sample=close)
        path.write_text(json.dumps(data))
        with pytest.raises(ValueError, match=error):
            MODULE._verify_writer_timing(tmp_path, tmp_path/'mapping_localization_diagnostic', 1000, 2000)
