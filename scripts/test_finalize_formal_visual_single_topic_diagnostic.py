import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/finalize_formal_visual_single_topic_diagnostic.py"
SPEC = importlib.util.spec_from_file_location("finalize_single_visual", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_summarizes_all_transport_layers(tmp_path: Path) -> None:
    (tmp_path / "single_topic_world_report.json").write_text(
        json.dumps({"passed": True, "remaining_total_sensor_count": 1}),
        encoding="utf-8",
    )
    (tmp_path / "gz_topic_info.txt").write_text(
        "Publisher\nSubscriber\nMessage type: gz.msgs.Image\n", encoding="utf-8"
    )
    (tmp_path / "ros_topic_info.txt").write_text(
        "Publisher count: 1\nSubscription count: 1\n", encoding="utf-8"
    )
    (tmp_path / "gz_sample_metadata.json").write_text(
        json.dumps(
            {
                "passed": True,
                "width": 1600,
                "height": 1000,
                "expected_uncompressed_data_bytes_from_step": 4_800_000,
            }
        ),
        encoding="utf-8",
    )
    (tmp_path / "ros_width.txt").write_text("1600\n", encoding="utf-8")
    (tmp_path / "transport_process_maps.json").write_text(
        json.dumps(
            {
                "passed": True,
                "cross_process_checks": {
                    "same_transport_file": True,
                    "same_protobuf_file": True,
                    "same_zmq_file": True,
                },
            }
        ),
        encoding="utf-8",
    )
    report = MODULE.summarize(tmp_path, 0)
    assert report["passed"] is True
    assert report["first_failed_stage"] is None


def test_identifies_first_missing_layer(tmp_path: Path) -> None:
    report = MODULE.summarize(tmp_path, 87)
    assert report["passed"] is False
    assert report["first_failed_stage"] == "single_camera_world"
