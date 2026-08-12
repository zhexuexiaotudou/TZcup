import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT = Path(__file__).with_name("perception_oprv3_product_performance.py")


def _module():
    pytest.importorskip("torch")
    pytest.importorskip("torchvision")
    spec = importlib.util.spec_from_file_location("oprv3_performance", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_performance_summary_uses_real_queue_counts_and_completion_rate():
    module = _module()
    report = module.summarize_performance(
        submitted=101,
        consumed=100,
        dropped=1,
        completion_times=[index * 0.099 for index in range(100)],
        latencies_ms=[120.0] * 95 + [180.0] * 5,
    )
    assert report["metrics"]["effective_hz"] == pytest.approx(10.1010101)
    assert report["metrics"]["drop_rate"] == pytest.approx(1 / 101)
    assert report["metrics"]["end_to_end_p95_ms"] == pytest.approx(123.0)
    assert report["pass"] is True


def test_performance_summary_fails_closed_on_latency_or_no_execution():
    module = _module()
    slow = module.summarize_performance(
        submitted=10,
        consumed=10,
        dropped=0,
        completion_times=[index * 0.1 for index in range(10)],
        latencies_ms=[250.0] * 10,
    )
    assert slow["gates"]["end_to_end_p95_ms"] is False
    empty = module.summarize_performance(
        submitted=0,
        consumed=0,
        dropped=0,
        completion_times=[],
        latencies_ms=[],
    )
    assert empty["metrics"]["formal_product_pipeline_executed"] is False
    assert empty["pass"] is False


def test_formal_pipeline_requires_every_consumed_frame_to_run_all_stages():
    module = _module()
    assert module.formal_pipeline_complete(
        consumed=90,
        model_counts={"detector": 90, "leaf": 45, "puddle": 45},
        latency_samples=90,
        mission_count=1,
    )
    assert not module.formal_pipeline_complete(
        consumed=90,
        model_counts={"detector": 90, "leaf": 45, "puddle": 44},
        latency_samples=90,
        mission_count=1,
    )


def test_mmdet_inference_preserves_native_boxes_and_class_mapping(monkeypatch):
    module = _module()

    class Instances:
        bboxes = SimpleNamespace(tolist=lambda: [[1.0, 2.0, 11.0, 22.0]])
        scores = SimpleNamespace(tolist=lambda: [0.9])
        labels = SimpleNamespace(tolist=lambda: [1])

        def to(self, device):
            assert device == "cpu"
            return self

    fake_api = SimpleNamespace(
        inference_detector=lambda model, image: SimpleNamespace(
            pred_instances=Instances()
        )
    )
    monkeypatch.setitem(__import__("sys").modules, "mmdet.apis", fake_api)
    rgb = np.zeros((24, 32, 3), dtype=np.uint8)
    report = module.detector_mmdet_inference(object(), {}, rgb)
    assert report == {
        "detections": [
            {
                "class_name": "metal_can",
                "score": 0.9,
                "bbox_xyxy": [1.0, 2.0, 11.0, 22.0],
            }
        ]
    }
