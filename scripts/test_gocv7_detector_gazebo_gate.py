import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_module(name):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_ga1_threshold_grid_is_bounded_and_holdout_only():
    module = load_module("screen_gocv7_ga1.py")
    assert module.THRESHOLDS[0] == 0.05
    assert module.THRESHOLDS[-1] == 0.45
    source = (ROOT / "scripts/screen_gocv7_ga1.py").read_text(encoding="utf-8")
    assert '"selection_data": "GOCV7_GA1_HOLDOUT_ONLY"' in source
    assert '"existing_24_mission_read_before_selection_freeze": False' in source


def test_ga1_training_never_reads_formal_evaluation_data():
    source = (ROOT / "scripts/train_gocv7_ga1.py").read_text(encoding="utf-8")
    assert '"training_dataset": "GA1_TRAIN_ONLY"' in source
    assert '"selection_dataset": "GA1_HOLDOUT_ONLY"' in source
    assert '"existing_24_mission_used": False' in source
    assert '"G5_V2_read": False' in source


def test_ga1_metrics_count_wrong_class_matches_as_wrong_actionable():
    module = load_module("screen_gocv7_ga1.py")
    payload = {
        "categories": [
            {"id": 1, "name": "plastic_bottle"},
            {"id": 2, "name": "metal_can"},
            {"id": 3, "name": "paper_litter"},
        ],
        "images": [
            {"id": index, "frame_index": index, "negative_only": False}
            for index in range(1, 4)
        ],
        "annotations": [
            {
                "id": index,
                "image_id": index,
                "category_id": 1,
                "bbox": [10, 10, 20, 20],
                "bbox_short_side_px": 20,
                "target_id": "target",
                "mission_id": "mission",
                "actionable": True,
            }
            for index in range(1, 4)
        ],
    }
    frames = {
        index: [
            {
                "class_name": "metal_can",
                "score": 0.9,
                "bbox_xyxy": [10, 10, 30, 30],
            }
        ]
        for index in range(1, 4)
    }
    result = module.metrics(payload, frames, 0.5)
    assert result["eventual_detection_recall"] == 1.0
    assert result["eventual_correct_class_recall"] == 0.0
    assert result["actionable_precision"] == 0.0
    assert result["wrong_actionable_rate"] == 1.0
