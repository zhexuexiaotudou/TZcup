import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import finalize_auto05r_factorized_capture as module


def _capture(tmp_path: Path, role: str) -> Path:
    data = tmp_path / role
    data.mkdir()
    (data / "factorized_capture_plan.json").write_text(
        json.dumps({"role": role}), encoding="utf-8"
    )
    for index in range(10):
        scene_dir = data / "scenes" / f"scene_{1000 + index:04d}"
        scene_dir.mkdir(parents=True)
        objects = (
            [{"semantic_label": 0, "split_eligibility": ["val"]}]
            if role == "D5"
            else [
                {
                    "semantic_label": 1,
                    "split_eligibility": ["val" if role == "D1" else "train"],
                }
            ]
        )
        (scene_dir / "scene_manifest.json").write_text(
            json.dumps(
                {
                    "split": role,
                    "negative_only": role == "D5",
                    "objects": objects,
                    "factorized_diagnostic": {
                        "role": role,
                        "single_factor_capture": True,
                    },
                }
            ),
            encoding="utf-8",
        )
    return data


def _fake_qa(role: str):
    def finalize(_data, output, **_kwargs):
        output.mkdir(parents=True)
        frames = [
            {"scene_seed": 1000 + index // 10, "frame_index": index % 10, "split": role}
            for index in range(100)
        ]
        (output / "g4_frame_manifest.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in frames), encoding="utf-8"
        )
        (output / "g4_instance_records.jsonl").write_text("", encoding="utf-8")
        return {
            "errors": [],
            "gates": {name: True for name in module.ESSENTIAL_GATES},
        }

    return finalize


@pytest.mark.parametrize("role", ("D1", "D2", "D3", "D4", "D5"))
def test_finalize_factorized_capture(monkeypatch, tmp_path: Path, role: str) -> None:
    data = _capture(tmp_path, role)
    monkeypatch.setattr(module, "finalize_g4_dataset", _fake_qa(role))
    report = module.finalize_factorized_capture(data, tmp_path / "evidence", role)
    assert report["factorized_diagnostic_pass"] is True
    assert report["formal_G4_gate_claimed"] is False
    assert report["frame_count"] == 100
    assert report["positive_scene_count"] == (0 if role == "D5" else 10)
