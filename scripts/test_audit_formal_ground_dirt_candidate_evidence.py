import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/audit_formal_ground_dirt_candidate_evidence.py"
SPEC = importlib.util.spec_from_file_location("ground_dirt_evidence_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_incomplete_episode_evidence_fails_counterfactual_replay_closed(tmp_path):
    (tmp_path / "best_front_frame.png").write_bytes(b"saved-rgb")
    (tmp_path / "dosod_raw_diagnostic.json").write_text("{}", encoding="utf-8")
    (tmp_path / "perception_acceptance.json").write_text("{}", encoding="utf-8")
    report = MODULE.audit_episode(tmp_path)
    assert report["counterfactual_iou_replay_possible"] is False
    missing = report["saved_evidence"]["missing_product_intermediates"]
    assert "product_rgb_depth_frames.npz" in missing["timestamp_aligned_rgb_depth_camera_info"]
    assert "edgesam_prompt_masks.npz" in missing["per_prompt_edgesam_masks"]
    assert "ground_dirt_per_class_rasters.npz" in missing["per_class_projected_rasters"]


def test_complete_named_intermediate_closure_is_replayable(tmp_path):
    for names in MODULE.EXPECTED_INTERMEDIATES.values():
        for name in names:
            (tmp_path / name).write_bytes(b"evidence")
    report = MODULE.audit_episode(tmp_path)
    assert report["counterfactual_iou_replay_possible"] is True


def test_audit_contract_prohibits_truth_tuning_and_product_changes():
    source = SCRIPT.read_text(encoding="utf-8")
    assert '"truth_used_to_choose_product_parameters": False' in source
    assert '"product_threshold_prompt_or_weight_changed": False' in source
    assert "Do not use resulting IoU to tune candidate parameters" in source
    assert "eligible_as_formal_product_acceptance" in source
