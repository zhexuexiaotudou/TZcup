from pathlib import Path

import pytest

from sanitation_perception.open_vocab import (
    OpenVocabularyContractError,
    Proposal2D,
    check_runtime_readiness,
    fuse_proposals,
    load_open_vocabulary_profile,
)


ROOT = Path(__file__).resolve().parents[1]


def test_s100_profile_is_development_only_and_fail_closed_without_artifacts():
    profile = load_open_vocabulary_profile(ROOT / "config" / "open_vocab_s100_profile.yaml")
    assert profile.platform == "rdk_s100"
    assert profile.development_only
    assert not profile.journey6_evidence_allowed
    readiness = check_runtime_readiness(
        profile,
        artifact_root=None,
        runtime_available=False,
        source_revisions_locked=False,
    )
    assert not readiness.ready
    assert "rdk_s100_runtime" in readiness.missing
    assert "detector_s100_platform_support" in readiness.missing
    assert "detector_artifact_contract" in readiness.missing
    assert "segmenter_artifact_contract" in readiness.missing


def test_dosod_edgesam_and_depth_geometry_fuse_without_truth():
    fused = fuse_proposals(
        (
            Proposal2D("cube-1", "dosod", "trash cube", 0.82, (10, 12, 30, 32)),
            Proposal2D("cube-1", "edgesam", "trash cube", 0.80, (10, 12, 30, 32), mask_area_px=284),
            Proposal2D("cube-1", "rgbd_geometry", "cube", 0.91, (11, 13, 29, 31), depth_xyz_m=(1.2, -0.1, 0.03)),
        )
    )
    assert len(fused) == 1
    assert fused[0].label == "cube"
    assert fused[0].mask_area_px == 284
    assert fused[0].depth_xyz_m == (1.2, -0.1, 0.03)
    assert fused[0].sources == ("dosod", "edgesam", "rgbd_geometry")


@pytest.mark.parametrize("source", ["gazebo_truth", "evaluation_ground_truth", "/ground_truth/cubes"])
def test_truth_sources_are_rejected(source):
    with pytest.raises(OpenVocabularyContractError, match="truth source"):
        fuse_proposals((Proposal2D("x", source, "cube", 1.0, (0, 0, 1, 1)),))
