from __future__ import annotations

import numpy as np


def _load_module_without_ros():
    # The CI lane does not install ROS Python packages. Keep deterministic image
    # metric tests local by extracting the pure helper definition at runtime.
    namespace = {"np": np}
    source = open(__file__.replace("test_capture_", "capture_"), encoding="utf-8").read()
    start = source.index("def frame_metrics")
    end = source.index("\n\ndef main", start)
    exec(source[start:end], namespace)
    return namespace["frame_metrics"]


def test_frame_metrics_distinguish_visible_and_black_images() -> None:
    frame_metrics = _load_module_without_ros()
    visible = np.zeros((720, 1280, 3), dtype=np.uint8)
    visible[:, :640] = 180
    black = np.zeros_like(visible)
    assert frame_metrics(visible)["luminance_stddev"] > 8.0
    assert frame_metrics(visible)["near_black_fraction"] < 0.95
    assert frame_metrics(black)["near_black_fraction"] == 1.0
