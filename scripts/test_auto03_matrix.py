import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "generate_auto03_matrix", ROOT / "scripts" / "generate_auto03_matrix.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_matrix_meets_required_case_counts_and_oracle_boundary():
    manifest = json.loads(
        (ROOT / "artifacts/stage5br3_20260720_review/g2_worlds/g2_world_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    payload = MODULE.generate(manifest)
    trials = payload["trials"]
    targets = [item for item in trials if item["case_type"] in {"reachable", "unreachable_keepout"}]
    assert len({item["world_id"] for item in trials}) == 6
    assert len({(item["world_id"], item["scene_id"]) for item in trials}) == 60
    assert len(targets) == 200
    assert min(
        sum(item["class_id"] == class_id for item in targets)
        for class_id in MODULE.CLASS_ORDER
    ) >= 30
    assert sum(item["case_type"] == "unreachable_keepout" for item in trials) == 30
    assert sum(item["case_type"] == "false_candidate" for item in trials) == 30
    assert sum(item["case_type"] == "stale_dropout" for item in trials) == 20
    assert all(
        set(item["oracle_candidate"]) == set(payload["oracle_policy"]["published_fields"])
        for item in trials
    )
    assert all("observation_pose" not in item["oracle_candidate"] for item in trials)
