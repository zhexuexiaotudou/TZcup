import importlib.util
from array import array
from pathlib import Path
import sys
from types import SimpleNamespace


COLLECTOR_PATH = Path(__file__).with_name("collect_formal_single_episode_cleaning_mission.py")
sys.path.insert(0, str(COLLECTOR_PATH.parents[1] / "starter_ws/src/sanitation_coverage"))
SPEC = importlib.util.spec_from_file_location("single_episode_collector", COLLECTOR_PATH)
assert SPEC is not None and SPEC.loader is not None
COLLECTOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COLLECTOR)


def test_duplicate_recorder_names_cannot_hide_a_second_truth_subscription():
    endpoint = SimpleNamespace(node_name="a12_trusted_gt_recorder", node_namespace="/")
    subscribers = COLLECTOR.truth_subscriber_names([endpoint, endpoint])
    assert subscribers != ["/a12_trusted_gt_recorder"]
    assert len(subscribers) == 2


def _complete_stats() -> dict[str, dict[str, int]]:
    return {
        name: {
            "window_received": 1,
            "written": 1,
            "invalid_stamp": 0,
            "invalid_sample": 0,
        }
        for name in COLLECTOR.REPLAY_METRIC_TOPICS
    }


def _window() -> dict[str, int]:
    return {"operator_start_ros_time_ns": 1, "task_complete_ros_time_ns": 2}


def _pairing(complete: bool = True) -> dict[str, object]:
    return {
        "input_streams_complete": complete,
        "status": "COMPLETE" if complete else "FAILED_MISSING_LOCALIZATION_STREAM",
        "paired_sample_count": 1 if complete else 0,
    }


def _sample(stamp_ns: int, sequence: int, x: float = 0.0, y: float = 0.0) -> dict:
    return {"stamp_ns": stamp_ns, "arrival_sequence": sequence, "x_m": x, "y_m": y}


def test_replay_metric_capture_requires_all_runtime_sources() -> None:
    stats = _complete_stats()
    assert COLLECTOR.replay_metric_capture_complete(stats, 1, _pairing(), _window())
    stats["ground_truth_odom"]["window_received"] = 0
    assert not COLLECTOR.replay_metric_capture_complete(stats, 1, _pairing(), _window())


def test_replay_metric_capture_rejects_missing_brush_or_localization_evidence() -> None:
    stats = _complete_stats()
    assert not COLLECTOR.replay_metric_capture_complete(stats, 0, _pairing(), _window())
    assert not COLLECTOR.replay_metric_capture_complete(stats, 1, _pairing(False), _window())


def test_canonical_pairing_is_stable_for_out_of_order_arrival_and_50ms_boundary() -> None:
    pairing = COLLECTOR.canonical_localization_pairing(
        [_sample(3_000_000_000, 1, 3.0), _sample(1_000_000_000, 2, 1.0)],
        [_sample(1_049_000_000, 3, 1.0), _sample(3_049_000_000, 4, 3.0)],
    )
    assert pairing["paired_sample_count"] == 2
    assert pairing["dropped_estimate_count"] == 0
    assert pairing["offline_order"] == "stable_ascending_ros_stamp_ns_then_arrival_sequence"
    assert COLLECTOR.canonical_localization_pairing(
        [_sample(1_000_000_000, 1)], [_sample(1_050_000_000, 2)]
    )["paired_sample_count"] == 1
    assert COLLECTOR.canonical_localization_pairing(
        [_sample(1_000_000_000, 1)], [_sample(1_051_000_000, 2)]
    )["paired_sample_count"] == 0


def test_brush_transitions_invalid_stamp_and_high_fidelity_geometry_are_fail_closed() -> None:
    assert COLLECTOR.positive_stamp_ns(1)
    assert not COLLECTOR.positive_stamp_ns(0)
    assert COLLECTOR.any_brush_rotating([0.0, -1.0, 0.0]) is True
    assert COLLECTOR.full_width_coverage_active([0.0, 0.0, 0.0]) is False
    assert COLLECTOR._validated_brush_command(array("d", [1.0, -2.0, 3.0])) == [1.0, -2.0, 3.0]
    assert COLLECTOR._validated_brush_command([True, 0.0, 0.0]) is None
    assert COLLECTOR._validated_brush_command(["1.0", 0.0, 0.0]) is None
    assert COLLECTOR._validated_brush_command(array("d", [1.0, 2.0])) is None
    assert COLLECTOR.full_width_coverage_active([0.0, -1.0, 0.0]) is False
    assert COLLECTOR.full_width_coverage_active([8.0, -8.0, 12.0]) is True
    assert COLLECTOR.full_width_coverage_active([1.0, 2.0]) is None
    assert COLLECTOR.any_brush_rotating([float("nan"), 1.0, 0.0]) is None
    geometry = COLLECTOR.high_fidelity_brush_geometry()
    assert geometry["source"]["sha256"]
    assert geometry["centers_base_link_m"] == (
        {"brush": "left", "x_m": 0.385, "y_m": 0.545},
        {"brush": "right", "x_m": 0.385, "y_m": -0.545},
    )


def test_streaming_metrics_accept_long_episode_without_buffer_or_overwrite(tmp_path: Path) -> None:
    stream = COLLECTOR.JsonlMetricStream(tmp_path, "ground_truth_odom")
    fused = COLLECTOR.JsonlMetricStream(tmp_path, "fused_odom")
    for index in range(20_001):
        stream.append({"stamp_ns": index + 1, "arrival_sequence": index, "x_m": 0.0, "y_m": 0.0})
        fused.append({"stamp_ns": index + 1, "arrival_sequence": index, "x_m": 0.0, "y_m": 0.0})
    descriptor, fused_descriptor = stream.close(), fused.close()
    assert descriptor["record_count"] == 20_001
    assert len(descriptor["sha256"]) == 64
    assert Path(descriptor["path"]).read_text(encoding="utf-8").count("\n") == 20_001
    pairing = COLLECTOR.canonical_localization_pairing_from_streams(
        Path(fused_descriptor["path"]), Path(descriptor["path"]), tmp_path
    )
    assert pairing["status"] == "COMPLETE"
    assert pairing["paired_sample_count"] == 20_001
    assert len(pairing["sorts"]["fused_odom"]["chunks"]) > 1
    try:
        COLLECTOR.JsonlMetricStream(tmp_path, "ground_truth_odom")
    except FileExistsError:
        pass
    else:  # pragma: no cover
        raise AssertionError("stream creation overwrote an existing evidence file")


def test_final_stream_filters_header_stamp_to_ros_task_window(tmp_path: Path) -> None:
    raw = COLLECTOR.JsonlMetricStream(tmp_path, "fused_raw")
    for stamp_ns in (9, 10, 19, 20):
        raw.append({"stamp_ns": stamp_ns, "x_m": 0.0, "y_m": 0.0})
    descriptor = raw.close()
    final = COLLECTOR.windowed_jsonl_metric_stream(
        Path(descriptor["path"]), tmp_path, "fused", 10, 20
    )
    assert final["record_count"] == 2
    assert final["excluded_by_header_window"] == 2


def test_streamed_canonical_pairer_sorts_disorder_and_uses_50ms_boundary(tmp_path: Path) -> None:
    fused = COLLECTOR.JsonlMetricStream(tmp_path, "fused")
    truth = COLLECTOR.JsonlMetricStream(tmp_path, "truth")
    for sample in (_sample(3_000_000_000, 1, 3.0), _sample(1_000_000_000, 2, 1.0)):
        fused.append(sample)
    for sample in (_sample(1_049_000_000, 3, 1.0), _sample(3_050_000_000, 4, 3.0)):
        truth.append(sample)
    fused_descriptor, truth_descriptor = fused.close(), truth.close()
    pairing = COLLECTOR.canonical_localization_pairing_from_streams(
        Path(fused_descriptor["path"]), Path(truth_descriptor["path"]), tmp_path
    )
    assert pairing["status"] == "COMPLETE"
    assert pairing["canonical_invocation_count"] == 2
    assert pairing["paired_sample_count"] == 2
    assert pairing["dropped_estimate_count"] == 0
    assert pairing["unpaired_truth_count"] == 0
