from __future__ import annotations

import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


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


def test_product_and_service_profiles_have_six_real_gazebo_frames() -> None:
    for directory, profile in (
        ("formal_vehicle_visual_acceptance", "product"),
        ("formal_vehicle_service_visual_acceptance", "service"),
    ):
        root = ROOT / "reports" / "engineering" / directory
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
        assert manifest["status"] == "GAZEBO_OGRE2_SIX_CAMERA_CAPTURE_PASSED"
        assert manifest["bodywork_profile"] == profile
        assert manifest["camera_count"] == 6
        assert set(manifest["frames"]) == {
            "front_left", "rear_right", "top_cleaning", "sensor_tower_detail",
            "front_sensor_detail", "arm_mount_detail",
        }
        for frame in manifest["frames"].values():
            assert (root / frame["path"]).is_file()
            assert frame["width"] == 1600 and frame["height"] == 1000
