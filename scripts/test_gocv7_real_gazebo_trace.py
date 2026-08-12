import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def load_module():
    path = ROOT / "scripts/gocv7_real_gazebo_trace.py"
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mission_roles_are_exact_and_paths_are_preserved():
    module = load_module()
    values = [f"{role}=C:/{role}" for role in sorted(module.REQUIRED_ROLES)]
    parsed = module.parse_mission_specs(values)
    assert {role for role, _path in parsed} == module.REQUIRED_ROLES
    assert {path.as_posix() for _role, path in parsed} == {
        f"C:/{role}" for role in module.REQUIRED_ROLES
    }


def test_mission_roles_fail_closed_on_missing_role():
    module = load_module()
    values = [f"{role}=/{role}" for role in sorted(module.REQUIRED_ROLES)[:-1]]
    with pytest.raises(ValueError, match="mission roles must be exactly"):
        module.parse_mission_specs(values)


def test_detection_signature_exposes_class_score_box_and_actionability():
    module = load_module()
    rows = [{
        "class_name": "metal_can",
        "score": 0.8123456789,
        "bbox_xyxy": [1.25, 2.5, 30.75, 40.0],
        "actionable": True,
    }]
    assert module.detection_signature(rows) == [
        ("metal_can", 0.81234568, (1.25, 2.5, 30.75, 40.0), True)
    ]


def test_root_cause_separates_domain_score_and_runtime_losses():
    module = load_module()
    base = {
        "entered_actionable_window": True,
        "pipelines": {
            "P0_NATIVE": {
                "eventual_correct_class": False,
                "ever_correct_observation": False,
            },
            "P1_ADAPTER": {"eventual_correct_class": False},
            "P2_PRODUCT": {"eventual_correct_class": False},
        },
    }
    assert module.root_cause_for(base) == "IMAGE_DOMAIN_SHIFT"
    base["pipelines"]["P0_NATIVE"]["ever_correct_observation"] = True
    assert module.root_cause_for(base) == "SCORE_CALIBRATION_MISMATCH"
    base["pipelines"]["P0_NATIVE"]["eventual_correct_class"] = True
    assert module.root_cause_for(base) == "CLASS_INDEX_MISMATCH"
