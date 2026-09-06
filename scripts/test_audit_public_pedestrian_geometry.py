import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/audit_public_pedestrian_geometry.py"
SPEC = importlib.util.spec_from_file_location("audit_public_pedestrian_geometry", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_public_geometry_audit_is_truth_bounded_and_counts_paths():
    config = MODULE.load_config(MODULE.DEFAULT_CONFIG)
    report = MODULE.audit_public_geometry(
        config,
        split_map_counts=(("train", 1),),
        missions_per_map=2,
    )
    assert report["scope"] == "public_train_val_only"
    assert report["hidden_accessed"] is False
    assert report["episode_count"] == 2
    assert report["pedestrian_path_count"] == 16
    assert report["pedestrian_pair_count"] == 56
    assert report["pedestrian_static_collision_path_count"] == 0
    assert report["pedestrian_cube_collision_path_count"] == 0
    assert report["pedestrian_pair_violation_count"] == 0
    assert report["passed"] is True


def test_public_geometry_audit_rejects_hidden_scope():
    config = MODULE.load_config(MODULE.DEFAULT_CONFIG)
    try:
        MODULE.audit_public_geometry(config, split_map_counts=(("hidden", 1),))
    except ValueError as exc:
        assert "public train/val" in str(exc)
    else:
        raise AssertionError("hidden scope must be rejected")
