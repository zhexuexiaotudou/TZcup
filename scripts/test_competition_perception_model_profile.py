from pathlib import Path

import pytest

from competition_perception_fixture import create
from competition_perception_model_profile import (
    CONTROLLED_FIXTURE_PROFILE,
    ModelProfileError,
    resolve_model_profile,
)


ROOT = Path(__file__).resolve().parents[1]


def test_controlled_fixture_uses_stage5a_color_model():
    profile = resolve_model_profile(ROOT, CONTROLLED_FIXTURE_PROFILE)
    assert profile.scope == "synthetic_color_domain_only"
    assert profile.path.name == "stage5a_synthetic_color_prototype.onnx"
    assert profile.sha256 == "f7a10487a2577dc6675abb6784769308a0c21bfb11c0c9eb42af6aa2a5152e73"


def test_controlled_fixture_rejects_stage5b_or_unknown_profiles():
    with pytest.raises(ModelProfileError):
        resolve_model_profile(ROOT, "stage5b_learned_perception_v1")
    with pytest.raises(ModelProfileError):
        resolve_model_profile(ROOT, "missing-profile")


def test_evaluation_plan_binds_model_identity(tmp_path):
    output = tmp_path / "fixture"
    plan = create(ROOT, output)
    assert plan["model_profile"]["id"] == CONTROLLED_FIXTURE_PROFILE
    assert plan["model_profile"]["path"].endswith("stage5a_synthetic_color_prototype.onnx")
    assert len(plan["model_profile"]["sha256"]) == 64
