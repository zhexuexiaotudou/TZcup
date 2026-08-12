import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).with_name("audit_crv6_online_dev.py")
SPEC = importlib.util.spec_from_file_location("audit_crv6_online_dev", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_threshold_helpers_fail_closed_on_missing_values():
    assert MODULE.at_least(None, 0.95) is False
    assert MODULE.at_most(None, 0.01) is False


def test_crv6_thresholds_are_stricter_than_legacy_area_gate():
    source = SCRIPT.read_text(encoding="utf-8")
    assert 'area_metrics["boundary_f1"], 0.80' in source
    assert 'area_metrics["negative_area_actionable_fp_per_frame"], 0.02' in source
    assert 'map_metrics.get("id_consistency"), 0.97' in source
    assert 'map_metrics.get("track_fragmentation"), 0.03' in source


def test_g5_v2_and_gt_boundaries_are_enforced():
    source = SCRIPT.read_text(encoding="utf-8")
    assert 'G5_V2_SEALED_FINAL_read' in source
    assert 'production_target_ids_or_coordinates_provided' in source
    assert '"G5_V2_read": False' in source


def test_discrete_metrics_exclude_area_targets_and_normalize_projection():
    source = SCRIPT.read_text(encoding="utf-8")
    assert 'item["class_name"] in DISCRETE_CLASSES' in source
    assert 'encounter["class_name"] in DISCRETE_CLASSES' in source
    assert 'projection_successful_correct_detection_count' in source
