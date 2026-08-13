from pathlib import Path
import sys

import prepare_trcrv10_g10_domain as g10

PACKAGE = Path(__file__).resolve().parents[1] / "starter_ws/src/sanitation_learning"
sys.path.insert(0, str(PACKAGE))

from sanitation_learning import g4_scene


def test_g10_mission_plan_meets_protocol_minimums() -> None:
    assert 6 * g10.SCENES_PER_WORLD["train"] >= 45
    assert 3 * g10.SCENES_PER_WORLD["val"] >= 18
    assert 3 * g10.SCENES_PER_WORLD["test"] >= 18


def test_g10_approach_lanes_cover_far_mid_close_travel() -> None:
    assert g4_scene.G10_TARGET_START_DISTANCE_M == 6.2
    assert g4_scene.G10_TARGET_LATERAL_BY_CLASS_M == {
        "metal_can": -0.57,
        "paper_litter": 0.66,
        "plastic_bottle": -0.57,
    }
    assert 125 * 0.02 >= 2.48


def test_g10_centered_route_balances_target_classes() -> None:
    classes = [g4_scene.g10_target_class(f"world_{index}", scene) for index in range(6) for scene in range(8)]
    counts = {class_id: classes.count(class_id) for class_id in g4_scene.G8_DISCRETE_CLASSES}
    assert max(counts.values()) - min(counts.values()) <= 1


def test_g10_drive_by_lanes_keep_physical_clearance() -> None:
    half_widths = {"metal_can": 0.05, "paper_litter": 0.11, "plastic_bottle": 0.05}
    assert all(
        abs(g4_scene.G10_TARGET_LATERAL_BY_CLASS_M[class_id]) - 0.36 - half_width >= 0.15
        for class_id, half_width in half_widths.items()
    )


def test_g10_identifiability_grid_is_development_only() -> None:
    assert g4_scene.G10_DIAGNOSTIC_DISTANCES_M == (0.85, 0.95, 1.10, 1.30, 1.55, 1.90, 2.40, 3.00)
    source = Path(g4_scene.__file__).read_text(encoding="utf-8")
    assert '"production_runtime_eligible": False' in source
    assert '"GT_crop_allowed_offline_only": True' in source
    assert "--g10-identifiability-diagnostic" in source


def test_g10_capture_orchestrator_denies_sealed_dev_val() -> None:
    source = (Path(__file__).parent / "run_trcrv10_g10_capture.ps1").read_text(encoding="utf-8")
    assert "[ValidateSet('train', 'val')]" in source
    assert "'test'" not in source
    assert "-G10ApproachSequence" in source
    assert "-CaptureFrameCount 125" in source
    assert "-CaptureTimeoutSeconds 1200" in source


def test_capture_wrapper_keeps_product_camera_defaults_and_allows_audited_diagnostic_override() -> None:
    source = (Path(__file__).parent / "run_auto05r_g4_capture_docker.ps1").read_text(encoding="utf-8")
    for contract in (
        '[double]$CameraX = 0.36',
        '[double]$CameraY = 0.0',
        '[double]$CameraZ = 0.66',
        '[double]$CameraPitchRad = 0.872664626',
        'AUTO05R_CAMERA_PROFILE_ID=$CameraProfileId',
    ):
        assert contract in source


def test_identifiability_capture_is_split_isolated_and_balanced() -> None:
    source = (Path(__file__).parent / "run_trcrv10_identifiability_capture.ps1").read_text(encoding="utf-8")
    assert "[ValidateSet('train_diag', 'holdout_diag')]" in source
    assert "-ScenesPerWorld 24" in source
    assert "-CaptureFrameCount 34" in source
    assert "-G10IdentifiabilityDiagnostic" in source
    assert "trcrv10_diag_v1_low_oblique_evaluator_only" in source
    combinations = {
        (scene % 8, g4_scene.g10_target_class("world_fixed", scene))
        for scene in range(24)
    }
    assert len(combinations) == 24


def test_g10_new_mode_is_opt_in_and_gt_forbidden_at_runtime() -> None:
    source = Path(g4_scene.__file__).read_text(encoding="utf-8")
    assert "g10_approach_sequence: bool = False" in source
    assert '"gt_runtime_forbidden": True' in source
    assert "--g10-approach-sequence" in source


def test_g10_sealed_splits_are_explicit() -> None:
    assert g10.SPLIT_MAP == {
        "train": "G10_TRAIN",
        "val": "G10_HOLDOUT",
        "test": "G10_DEV_VAL_SEALED",
    }
    source = Path(g10.__file__).read_text(encoding="utf-8")
    assert '"G10_DEV_VAL_SEALED_read": False' in source
    assert '"VAL_NEW_read": False' in source
    assert '"G5_V2_read": False' in source
