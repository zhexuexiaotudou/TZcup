import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/screen_ddrv4_d1.py"
SOURCE = SCRIPT.read_text(encoding="utf-8")
SPEC = importlib.util.spec_from_file_location("screen_ddrv4_d1", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_selection_freezes_before_val_is_opened_and_only_selected_route_sees_val():
    freeze = SOURCE.index("atomic_json(selection_path, selection)")
    val_open = SOURCE.index('load_truth(args.prepared / "val.json"')
    assert freeze < val_open
    assert '"G7_VAL_candidate_count": 1' in SOURCE
    assert '"G7_VAL_evaluation_count": 1' in SOURCE
    assert '"G6_used": False' in SOURCE
    assert '"G5_V2_read": False' in SOURCE


def test_metrics_cover_required_static_gates_and_small_target():
    raw = [
        {
            "truth": [
                {"bbox_xyxy": [0, 0, 10, 10], "label": 1, "small_lt18": True},
                {"bbox_xyxy": [20, 20, 50, 50], "label": 2, "small_lt18": False},
                {"bbox_xyxy": [60, 60, 90, 90], "label": 3, "small_lt18": False},
            ],
            "predictions": [
                {"bbox_xyxy": [0, 0, 10, 10], "score": 0.9, "label": 1},
                {"bbox_xyxy": [20, 20, 50, 50], "score": 0.9, "label": 2},
                {"bbox_xyxy": [60, 60, 90, 90], "score": 0.9, "label": 3},
            ],
        }
    ]
    result = MODULE.apply_gates(MODULE.metrics(raw, 0.5))
    assert result["recall"] == result["precision"] == result["macro_f1"] == 1.0
    assert result["metal_can_recall"] == result["paper_litter_precision"] == 1.0
    assert result["small_lt18"]["recall"] == 1.0
    assert result["all_required_gates_pass"] is True


def test_false_positive_rate_is_per_all_frames_and_fails_gate():
    raw = [
        {
            "truth": [],
            "predictions": [{"bbox_xyxy": [0, 0, 10, 10], "score": 0.9, "label": 1}],
        },
        {"truth": [], "predictions": []},
    ]
    result = MODULE.apply_gates(MODULE.metrics(raw, 0.5))
    assert result["false_positive_per_frame"] == 0.5
    assert result["gates"]["false_positive_per_frame_at_most_0_05"] is False
