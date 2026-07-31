import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np


def _find_repository_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "scripts" / "auto13_real_domain.py").is_file():
            return candidate
    raise RuntimeError("could not locate repository root containing auto13_real_domain.py")


ROOT = _find_repository_root()
SPEC = importlib.util.spec_from_file_location(
    "auto13_real_domain", ROOT / "scripts" / "auto13_real_domain.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_privacy_regions_blur_only_requested_patch():
    image = np.zeros((40, 40, 3), dtype=np.uint8)
    image[10:30, 10:30] = np.indices((20, 20)).sum(0)[:, :, None] % 2 * 255
    filtered = MODULE.apply_privacy_regions(image, [[10, 10, 30, 30]])
    assert np.array_equal(filtered[:8, :8], image[:8, :8])
    assert not np.array_equal(filtered[10:30, 10:30], image[10:30, 10:30])


def test_evaluator_computes_real_domain_metrics(tmp_path):
    mask = np.zeros((12, 12), dtype=np.uint8)
    mask[2:10, 2:10] = 1
    np.save(tmp_path / "leaf.npy", mask, allow_pickle=False)
    np.save(tmp_path / "puddle.npy", mask, allow_pickle=False)
    instances = [
        {"class_id": name, "bbox_xyxy": [1, 1, 9, 9]}
        for name in MODULE.DISCRETE_CLASSES
    ]
    ground_truth = {
        "frames": [
            {
                "frame_id": "frame_000001",
                "instances": instances,
                "area_masks": {
                    "leaf_pile": "leaf.npy",
                    "puddle": "puddle.npy",
                },
                "hard_negative": False,
            },
            {
                "frame_id": "frame_000002",
                "instances": [],
                "hard_negative": True,
            },
        ],
        "map_localization_rmse_m": 0.1,
    }
    predictions = {
        "frames": [
            {
                "frame_id": "frame_000001",
                "instances": [
                    {**item, "confidence": 0.99} for item in instances
                ],
                "area_masks": {
                    "leaf_pile": "leaf.npy",
                    "puddle": "puddle.npy",
                },
            },
            {"frame_id": "frame_000002", "instances": []},
        ],
        "synthetic_reference_macro_f1": 1.0,
    }
    truth_path = tmp_path / "truth.json"
    prediction_path = tmp_path / "predictions.json"
    output = tmp_path / "metrics.json"
    truth_path.write_text(json.dumps(ground_truth), encoding="utf-8")
    prediction_path.write_text(json.dumps(predictions), encoding="utf-8")
    args = type(
        "Args",
        (),
        {
            "ground_truth": str(truth_path),
            "predictions": str(prediction_path),
            "output": str(output),
        },
    )()
    assert MODULE.evaluate(args) == 2
    metrics = json.loads(output.read_text(encoding="utf-8"))
    assert metrics["discrete_macro_f1"] == 1.0
    assert metrics["area_macro_miou"] == 1.0
    assert metrics["negative_specificity"] == 1.0
    assert metrics["real_domain_gate_pass"] is False
