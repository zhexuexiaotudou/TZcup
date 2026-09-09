from pathlib import Path

import pytest

from audit_j6f2_area_dev import frame_records, validate_development_path


@pytest.mark.parametrize("token", ["G5_V2", "SEALED_FINAL", "DEV_VAL"])
def test_area_replay_rejects_forbidden_sources(token, tmp_path):
    path = tmp_path / token / "train"
    path.mkdir(parents=True)
    with pytest.raises(ValueError, match="forbidden"):
        validate_development_path(path)


@pytest.mark.parametrize("token", ["g5-v2", "sealed final", "dev-val"])
def test_area_replay_rejects_separator_variants_of_forbidden_sources(token, tmp_path):
    path = tmp_path / token / "train"
    path.mkdir(parents=True)
    with pytest.raises(ValueError, match="forbidden"):
        validate_development_path(path)


def test_area_replay_requires_explicit_train_or_development(tmp_path):
    path = tmp_path / "holdout"
    path.mkdir()
    with pytest.raises(ValueError, match="TRAIN/development"):
        validate_development_path(path)


def test_area_replay_accepts_train_path(tmp_path):
    path = tmp_path / "train"
    path.mkdir()
    validate_development_path(Path(path))


def test_area_replay_does_not_treat_constraint_as_train(tmp_path):
    path = tmp_path / "constraint"
    path.mkdir()
    with pytest.raises(ValueError, match="TRAIN/development"):
        validate_development_path(path)


def test_area_replay_rejects_nonpositive_frame_limit(tmp_path):
    path = tmp_path / "smoke_train_world0"
    path.mkdir()
    with pytest.raises(ValueError, match="positive"):
        frame_records(path, 0)
