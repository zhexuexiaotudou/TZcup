import json
from pathlib import Path

import pytest

from replay_competition_perception_model import ROOT, load_raw_output_presence


def test_raw_output_presence_is_loaded_exactly(tmp_path):
    summary = tmp_path / "summary.json"
    summary.write_text(
        json.dumps({
            "selected_frames": [
                {"frame_id": "frame_01", "raw_output_present": False},
                {"frame_id": "frame_02", "raw_output_present": True},
            ]
        }),
        encoding="utf-8",
    )
    assert load_raw_output_presence(summary) == {
        "frame_01": False,
        "frame_02": True,
    }


def test_raw_output_presence_rejects_missing_or_non_boolean(tmp_path):
    summary = tmp_path / "summary.json"
    summary.write_text(
        json.dumps({"selected_frames": [{"frame_id": "frame_01"}]}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        load_raw_output_presence(summary)


def test_replay_script_exposes_cuda_provider_without_changing_cpu_default():
    text = (ROOT / "scripts/replay_competition_perception_model.py").read_text(encoding="utf-8")
    assert 'providers = args.provider or ["CPUExecutionProvider"]' in text
    assert '"CUDAExecutionProvider"' in text
