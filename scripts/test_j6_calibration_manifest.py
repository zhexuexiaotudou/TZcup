import hashlib
import json
from pathlib import Path
import sys

import yaml


SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

from j6_calibration_manifest import audit, write_outputs


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_fixture(tmp_path: Path, *, per_role: int = 4):
    data = tmp_path / "calibration_data"
    data.mkdir()
    records = []
    for role in ("detector_frame", "second_pass_roi"):
        role_dir = data / role
        role_dir.mkdir()
        for index in range(per_role):
            image = role_dir / f"{index}.png"
            image.write_bytes(f"{role}-{index}".encode())
            records.append({
                "relative_path": image.relative_to(data).as_posix(),
                "role": role,
                "split": "calibration_train",
                "sha256": digest(image),
                "strata": {
                    "target_class": ["plastic_bottle", "background_or_unknown"][index % 2],
                    "scene": ["road", "curb"][index % 2],
                    "lighting": ["day", "shadow"][index % 2],
                    "distance_bucket": ["near", "far"][index % 2],
                },
            })
    inventory = tmp_path / "records.jsonl"
    inventory.write_text("".join(json.dumps(row) + "\n" for row in records), encoding="utf-8")
    config = {
        "schema_version": 1,
        "source": {
            "source_id": "nonsealed-calibration-train-v1",
            "provenance_uri": "urn:tzcup:development:calibration-train-v1",
            "record_inventory_sha256": digest(inventory),
            "sealed_access_allowed": False,
            "allowed_splits": ["calibration_train"],
        },
        "preprocess": {
            "source_color": "rgb",
            "input_width": 640,
            "input_height": 640,
            "letterbox": {
                "enabled": True,
                "preserve_aspect_ratio": True,
                "placement": "center",
                "pad_value": 114,
                "interpolation": "bilinear",
            },
            "nv12": {
                "layout": "nv12",
                "matrix": "bt601",
                "value_range": "limited",
                "chroma_order": "uv",
                "width_alignment": 2,
                "height_alignment": 2,
            },
        },
        "stratification": {
            "required_dimensions": ["target_class", "scene", "lighting", "distance_bucket"],
            "minimum_distinct_values": {name: 2 for name in ("target_class", "scene", "lighting", "distance_bucket")},
            "minimum_per_value": {name: 2 for name in ("target_class", "scene", "lighting", "distance_bucket")},
        },
    }
    config_path = tmp_path / "source.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    return config_path, inventory, data


def test_calibration_manifest_locks_both_roles_and_distribution(tmp_path):
    config, inventory, data = make_fixture(tmp_path)
    manifest, distribution, sums = audit(
        source_config=config,
        records_path=inventory,
        data_root=data,
        detector_minimum=4,
        second_pass_minimum=4,
    )
    assert manifest["calibration_ready"] is True
    assert manifest["counts"] == {"detector_frame": 4, "second_pass_roi": 4}
    assert distribution["stratification_pass"] is True
    assert distribution["distribution"]["detector_frame"]["target_class"] == {
        "background_or_unknown": 2,
        "plastic_bottle": 2,
    }
    assert len(sums) == 8
    output = tmp_path / "evidence"
    write_outputs(output, manifest, distribution, sums)
    assert (output / "J6_CALIBRATION_MANIFEST.json").is_file()
    assert (output / "J6_CALIBRATION_DISTRIBUTION.json").is_file()
    assert len((output / "J6_CALIBRATION_SHA256SUMS").read_text(encoding="utf-8").splitlines()) == 10


def test_default_production_minimums_remain_1000_per_role(tmp_path):
    config, inventory, data = make_fixture(tmp_path)
    manifest, _, _ = audit(source_config=config, records_path=inventory, data_root=data)
    assert manifest["calibration_ready"] is False
    count_blockers = [row for row in manifest["blockers"] if row["code"] == "calibration_count_below_minimum"]
    assert {row["role"]: row["required"] for row in count_blockers} == {
        "detector_frame": 1000,
        "second_pass_roi": 1000,
    }


def test_preprocess_letterbox_and_nv12_drift_fails_closed(tmp_path):
    config, inventory, data = make_fixture(tmp_path)
    value = yaml.safe_load(config.read_text(encoding="utf-8"))
    value["preprocess"]["letterbox"]["pad_value"] = 0
    value["preprocess"]["nv12"]["matrix"] = "bt709"
    config.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    manifest, _, _ = audit(
        source_config=config,
        records_path=inventory,
        data_root=data,
        detector_minimum=4,
        second_pass_minimum=4,
    )
    codes = {row["code"] for row in manifest["blockers"]}
    assert "letterbox_contract_not_frozen" in codes
    assert "nv12_contract_not_frozen" in codes
