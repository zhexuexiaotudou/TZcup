import json
from pathlib import Path

import pytest

import crcrv11_baseline as baseline


def test_required_directories_match_protocol():
    assert "forensic" in baseline.REQUIRED_DIRECTORIES
    assert "five_view" in baseline.REQUIRED_DIRECTORIES
    assert "x86_release" in baseline.REQUIRED_DIRECTORIES


def test_write_json_round_trip(tmp_path: Path):
    path = tmp_path / "result.json"
    baseline.write_json(path, {"G10_DEV_VAL_SEALED_read": False})
    assert json.loads(path.read_text()) == {"G10_DEV_VAL_SEALED_read": False}


def test_load_json_rejects_invalid_json(tmp_path: Path):
    path = tmp_path / "bad.json"
    path.write_text("not-json", encoding="utf-8")
    with pytest.raises(json.JSONDecodeError):
        baseline.load_json(path)
