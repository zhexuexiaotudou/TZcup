import importlib.util
import sys
from pathlib import Path

import pytest


pytest.importorskip("numpy")
torch = pytest.importorskip("torch")


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/calibrate_controlled_perception_gpu.py"
SPEC = importlib.util.spec_from_file_location("calibrate_controlled_perception_gpu", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_calibration_samples_are_deterministic_and_have_all_classes():
    first_image, first_labels = MODULE.synthetic_sample(MODULE.VAL_SEED_START)
    second_image, second_labels = MODULE.synthetic_sample(MODULE.VAL_SEED_START)
    assert (first_image == second_image).all()
    assert (first_labels == second_labels).all()
    assert first_image.shape == (MODULE.HEIGHT, MODULE.WIDTH, 3)
    assert set(int(value) for value in torch.unique(torch.from_numpy(first_labels))) == {
        0,
        1,
        2,
        3,
        4,
        5,
    }


def test_prototype_metric_keeps_background_leaf_failure_visible():
    image, labels = MODULE.synthetic_sample(MODULE.VAL_SEED_START)
    images = torch.from_numpy(image.transpose(2, 0, 1)[None]).float().div_(255.0)
    target = torch.from_numpy(labels[None]).long()
    result = MODULE.metrics(MODULE.prototype_logits(images), target)
    assert result["background_leaf_false_positive_pixels"] > 0
    assert 0.0 <= result["background_leaf_false_positive_rate"] <= 1.0
