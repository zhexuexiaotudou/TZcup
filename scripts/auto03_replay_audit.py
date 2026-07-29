from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SPOT = ROOT / "starter_ws" / "src" / "sanitation_spot_cleaning"
sys.path.insert(0, str(SPOT))

from sanitation_spot_cleaning.auto03_contract import summarize_auto03  # noqa: E402


REQUIRED_TOPICS = {
    "/active_observation/candidate",
    "/active_observation/pose_plan",
    "/active_observation/selected_pose",
    "/auto03/capture_request",
    "/auto03/done",
    "/auto03/machine_ready_result",
    "/auto03/oracle_candidate",
    "/auto03/trial_result",
    "/brush_enabled",
    "/cmd_vel",
    "/cmd_vel_nav",
    "/compute_path_to_pose/_action/status",
    "/coverage/state",
    "/localization/fused_pose",
    "/navigate_to_pose/_action/status",
    "/odom",
    "/spin/_action/status",
    "/tf",
    "/tf_static",
}


def compare_runtime_payloads(
    *,
    matrix: dict,
    runtime: dict,
    replayed_results: list[dict],
    topic_counts: dict[str, int],
) -> dict:
    expected = runtime["trials"]
    expected_ids = [str(item["candidate_id"]) for item in expected]
    replayed_ids = [str(item["candidate_id"]) for item in replayed_results]
    duplicate_ids = sorted({
        candidate_id for candidate_id in replayed_ids
        if replayed_ids.count(candidate_id) > 1
    })
    truth_ids = set(expected_ids)
    truth_trials = [
        item for item in matrix["trials"]
        if str(item["candidate_id"]) in truth_ids
    ]
    expected_report = summarize_auto03(truth_trials, expected)
    replayed_report = summarize_auto03(truth_trials, replayed_results)
    expected_metrics = expected_report["metrics"]
    replayed_metrics = replayed_report["metrics"]
    metric_match = expected_metrics == replayed_metrics
    missing_topics = sorted(REQUIRED_TOPICS - {
        topic for topic, count in topic_counts.items() if count > 0
    })
    candidate_match = expected_ids == replayed_ids
    return {
        "schema_version": 1,
        "stage": "AUTO-03",
        "bag_readable": True,
        "required_topic_count": len(REQUIRED_TOPICS),
        "required_topics_present_count": len(REQUIRED_TOPICS) - len(missing_topics),
        "required_topic_coverage": (
            (len(REQUIRED_TOPICS) - len(missing_topics)) / len(REQUIRED_TOPICS)
        ),
        "missing_required_topics": missing_topics,
        "runtime_result_count": len(expected_ids),
        "replayed_result_count": len(replayed_ids),
        "duplicate_candidate_ids": duplicate_ids,
        "candidate_order_and_identity_match": candidate_match,
        "task_timeline_reconstructable": (
            candidate_match
            and not duplicate_ids
            and topic_counts.get("/coverage/state", 0) >= len(expected_ids)
        ),
        "metric_replay_exact_match": metric_match,
        "metric_replay_delta_max": 0.0 if metric_match else None,
        "replayed_metrics": replayed_metrics,
        "topic_message_counts": dict(sorted(topic_counts.items())),
        "replay_audit_pass": (
            not missing_topics
            and candidate_match
            and not duplicate_ids
            and metric_match
            and topic_counts.get("/coverage/state", 0) >= len(expected_ids)
        ),
    }


def read_bag(path: Path) -> tuple[list[dict], dict[str, int]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(type_name) for topic, type_name in topic_types.items()
    }
    results: list[dict] = []
    counts: dict[str, int] = {}
    while reader.has_next():
        topic, data, _timestamp = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic == "/auto03/trial_result":
            message = deserialize_message(data, message_types[topic])
            results.append(json.loads(message.data))
    return results, counts


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--runtime", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    matrix = json.loads(Path(args.matrix).read_text(encoding="utf-8"))
    runtime = json.loads(Path(args.runtime).read_text(encoding="utf-8"))
    replayed_results, topic_counts = read_bag(Path(args.bag))
    report = compare_runtime_payloads(
        matrix=matrix,
        runtime=runtime,
        replayed_results=replayed_results,
        topic_counts=topic_counts,
    )
    Path(args.output).write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["replay_audit_pass"] else 2)


if __name__ == "__main__":
    main()
