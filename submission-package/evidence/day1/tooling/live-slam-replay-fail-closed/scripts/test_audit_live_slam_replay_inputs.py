import json
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from audit_live_slam_replay_inputs import (  # noqa: E402
    MCAP_MAGIC,
    audit_replay_inputs,
    main,
)


def _write_bag(
    root: Path,
    name: str,
    topics: dict[str, tuple[str, int]],
) -> Path:
    bag = root / name
    bag.mkdir(parents=True)
    data_name = f"{name}_0.mcap"
    (bag / data_name).write_bytes(MCAP_MAGIC + MCAP_MAGIC)
    rows = []
    for topic, (type_name, count) in sorted(topics.items()):
        rows.append(
            {
                "topic_metadata": {
                    "name": topic,
                    "type": type_name,
                    "serialization_format": "cdr",
                    "offered_qos_profiles": [],
                },
                "message_count": count,
            }
        )
    metadata = {
        "rosbag2_bagfile_information": {
            "version": 9,
            "storage_identifier": "mcap",
            "duration": {"nanoseconds": 1_000_000_000},
            "message_count": sum(count for _, count in topics.values()),
            "topics_with_message_count": rows,
            "relative_file_paths": [data_name],
            "files": [
                {
                    "path": data_name,
                    "starting_time": {"nanoseconds_since_epoch": 1},
                    "duration": {"nanoseconds": 1_000_000_000},
                    "message_count": sum(count for _, count in topics.values()),
                }
            ],
        }
    }
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False),
        encoding="utf-8",
    )
    return bag


def test_run06_shape_is_fail_closed(tmp_path: Path):
    bag = _write_bag(
        tmp_path,
        "run06",
        {
            "/clock": ("rosgraph_msgs/msg/Clock", 10),
            "/odom": ("nav_msgs/msg/Odometry", 5),
        },
    )

    report = audit_replay_inputs([bag])

    assert report["status"] == "LIVE_SLAM_REPLAY_INPUT_BLOCKED"
    assert report["replay_eligible"] is False
    assert report["slam_replay_claimed"] is False
    assert report["area_gate"] == "NOT_MEASURED"
    assert {row["topic"] for row in report["missing_requirements"]} == {
        "/scan",
        "/tf",
        "/tf_static",
    }


def test_complete_replay_contract_is_eligible(tmp_path: Path):
    bag = _write_bag(
        tmp_path,
        "complete",
        {
            "/scan": ("sensor_msgs/msg/LaserScan", 20),
            "/tf": ("tf2_msgs/msg/TFMessage", 20),
            "/tf_static": ("tf2_msgs/msg/TFMessage", 1),
            "/clock": ("rosgraph_msgs/msg/Clock", 20),
            "/odom": ("nav_msgs/msg/Odometry", 20),
        },
    )

    report = audit_replay_inputs([bag])

    assert report["status"] == "LIVE_SLAM_REPLAY_INPUT_READY"
    assert report["replay_eligible"] is True
    assert report["missing_requirements"] == []
    assert report["bags"][0]["data_files"][0]["sha256"]


def test_rejects_missing_mcap_footer(tmp_path: Path):
    bag = _write_bag(
        tmp_path,
        "truncated",
        {"/clock": ("rosgraph_msgs/msg/Clock", 1)},
    )
    (bag / "truncated_0.mcap").write_bytes(MCAP_MAGIC)

    report = audit_replay_inputs([bag])

    assert report["replay_eligible"] is False
    assert any("too short" in error for error in report["errors"])


def test_rejects_conflicting_types_across_bags(tmp_path: Path):
    first = _write_bag(
        tmp_path,
        "first",
        {"/clock": ("rosgraph_msgs/msg/Clock", 1)},
    )
    second = _write_bag(
        tmp_path,
        "second",
        {"/clock": ("std_msgs/msg/String", 1)},
    )

    report = audit_replay_inputs([first, second])

    assert report["replay_eligible"] is False
    assert any("conflicting types" in error for error in report["errors"])


def test_cli_require_pass_writes_receipt_and_returns_two(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
):
    bag = _write_bag(
        tmp_path,
        "run06",
        {"/odom": ("nav_msgs/msg/Odometry", 1)},
    )
    output = tmp_path / "receipt.json"

    assert (
        main(
            [
                "--bag-dir",
                str(bag),
                "--run-root",
                "/run-06",
                "--source-revision",
                "6021e7b",
                "--output",
                str(output),
                "--require-pass",
            ]
        )
        == 2
    )
    receipt = json.loads(output.read_text(encoding="utf-8"))
    assert receipt["run_root"] == "/run-06"
    assert receipt["source_revision"] == "6021e7b"
    assert receipt["replay_eligible"] is False
    assert capsys.readouterr().out == ""
