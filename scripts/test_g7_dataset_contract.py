from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g7_detector_dataset import G7Plan, SPLITS, build_g7_dataset, load_jsonl

from audit_ddrv4_g7 import audit


def smoke_plan():
    return G7Plan(frames_by_split={name: 2 for name in SPLITS}, frames_per_scene=1, full_negative_target=4, formal=False)


def test_g7_smoke_emits_required_reports_and_pixels(tmp_path):
    root = tmp_path / "g7"
    qa = build_g7_dataset(root, smoke_plan())
    assert qa["G7_DATASET_PASS"] is True
    assert qa["gates"]["sealed_data_not_read"] is True
    assert qa["gates"]["g6_data_not_read"] is True
    assert qa["access_audit"] == {"G6_read": False, "G5_read": False, "G5_V2_read": False}
    required = {"G7_DATASET_QA.json", "G7_SPLIT_MANIFEST.json", "G7_ASSET_REGISTRY.json", "G7_WORLD_REGISTRY.json", "G7_DOMAIN_MATRIX.json", "G7_NEGATIVE_TAXONOMY.json", "G7_SMALL_OBJECT_DISTRIBUTION.json"}
    assert required == {path.name for path in (root / "reports").glob("*.json")}
    rows = load_jsonl(root / "G7_FRAME_MANIFEST.jsonl")
    assert len(rows) == 16
    assert all((root / row["rgb_path"]).is_file() for row in rows)


def test_g7_rejects_nonempty_output(tmp_path):
    root = tmp_path / "g7"; root.mkdir(); (root / "foreign.txt").write_text("not g7")
    try:
        build_g7_dataset(root, smoke_plan())
    except FileExistsError:
        pass
    else:
        raise AssertionError("non-empty output must fail closed")


def test_independent_g7_audit_rereads_pixels(tmp_path):
    root = tmp_path / "g7-audit"
    build_g7_dataset(root, smoke_plan())
    report = audit(root)
    assert report["G7_INDEPENDENT_AUDIT_PASS"] is True
    assert report["checked_file_count"] == 16 * 5
    assert report["mismatch_count"] == 0
