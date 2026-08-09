import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from build_auto05r_screening_dataset import build_screening_view


def _jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")


def test_build_screening_view_merges_all_roles(tmp_path: Path) -> None:
    base_data = tmp_path / "base_data"
    (base_data / "scenes" / "scene_0000").mkdir(parents=True)
    (base_data / "scenes" / "scene_0000" / "frame.bin").write_bytes(b"base")
    base_evidence = tmp_path / "base_evidence"
    _jsonl(
        base_evidence / "g4_frame_manifest.jsonl",
        [{"scene_seed": 0, "frame_index": 0, "split": "train"}],
    )
    _jsonl(base_evidence / "g4_instance_records.jsonl", [])
    (base_evidence / "g4_dataset_qa.json").write_text("{}", encoding="utf-8")
    diagnostics = {}
    for index, role in enumerate(("D1", "D2", "D3", "D4", "D5"), start=10):
        data = tmp_path / f"{role}_data"
        (data / "scenes" / f"scene_{index:04d}").mkdir(parents=True)
        (data / "scenes" / f"scene_{index:04d}" / "frame.bin").write_bytes(role.encode())
        evidence = tmp_path / f"{role}_evidence"
        rows = [
            {"scene_seed": index, "frame_index": frame, "split": role}
            for frame in range(100)
        ]
        _jsonl(evidence / "raw_g4_qa" / "g4_frame_manifest.jsonl", rows)
        _jsonl(evidence / "raw_g4_qa" / "g4_instance_records.jsonl", [])
        (evidence / "factorized_diagnostic_qa.json").write_text(
            json.dumps(
                {
                    "role": role,
                    "factorized_diagnostic_pass": True,
                    "scene_count": 10,
                    "frame_count": 100,
                }
            ),
            encoding="utf-8",
        )
        diagnostics[role] = (data, evidence)
    report = build_screening_view(
        base_data,
        base_evidence,
        diagnostics,
        tmp_path / "output_data",
        tmp_path / "output_evidence",
    )
    assert report["total_frames"] == 501
    assert all(report["frame_counts_by_role"][role] == 100 for role in diagnostics)
    assert report["G5_SEALED_FINAL_included"] is False
