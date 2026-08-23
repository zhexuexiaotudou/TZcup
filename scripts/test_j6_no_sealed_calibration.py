import json
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

from j6_calibration_manifest import audit
from build_journey6_source_bundle import build
from test_j6_calibration_manifest import digest, make_fixture


@pytest.mark.parametrize(
    ("field", "value", "token"),
    (
        ("split", "DEV_VAL", "dev_val"),
        ("relative_path", "G5_V2/frame.png", "g5_v2"),
        ("relative_path", "SEALED_FINAL/frame.png", "sealed_final"),
    ),
)
def test_forbidden_calibration_sources_are_rejected_before_file_access(tmp_path, field, value, token):
    config, inventory, data = make_fixture(tmp_path)
    rows = [json.loads(line) for line in inventory.read_text(encoding="utf-8").splitlines()]
    rows[0][field] = value
    inventory.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
    import yaml
    source = yaml.safe_load(config.read_text(encoding="utf-8"))
    source["source"]["record_inventory_sha256"] = digest(inventory)
    if field == "split":
        source["source"]["allowed_splits"].append(value)
    config.write_text(yaml.safe_dump(source, sort_keys=False), encoding="utf-8")
    manifest, _, _ = audit(
        source_config=config,
        records_path=inventory,
        data_root=data,
        detector_minimum=1,
        second_pass_minimum=1,
    )
    blocked = [row for row in manifest["blockers"] if row["code"] == "forbidden_calibration_record"]
    assert blocked
    assert token in blocked[0]["tokens"]
    assert manifest["calibration_ready"] is False


def test_sealed_access_flag_can_never_be_enabled(tmp_path):
    config, inventory, data = make_fixture(tmp_path)
    import yaml
    source = yaml.safe_load(config.read_text(encoding="utf-8"))
    source["source"]["sealed_access_allowed"] = True
    config.write_text(yaml.safe_dump(source, sort_keys=False), encoding="utf-8")
    manifest, _, _ = audit(source_config=config, records_path=inventory, data_root=data, detector_minimum=4, second_pass_minimum=4)
    assert "sealed_access_must_be_false" in {row["code"] for row in manifest["blockers"]}
    assert manifest["sealed_access_allowed"] is False


def test_source_bundle_rejects_sealed_reference_before_payload_access(tmp_path):
    import yaml
    root = SCRIPTS.parent
    template = yaml.safe_load((root / "deploy" / "journey6" / "source_bundle" / "source_bundle.template.yaml").read_text(encoding="utf-8"))
    for component in template["components"]:
        if component["id"] == "detector_canonical_onnx":
            component["path"] = str(tmp_path / "SEALED_FINAL" / "model.onnx")
    template_path = tmp_path / "template.yaml"
    template_path.write_text(yaml.safe_dump(template, sort_keys=False), encoding="utf-8")
    status = build(template_path, tmp_path / "output", root)
    blocked = [row for row in status["blockers"] if row["code"] == "forbidden_source_component_path"]
    assert blocked and blocked[0]["component"] == "detector_canonical_onnx"
    assert "sealed_final" in blocked[0]["tokens"]


def test_forbidden_inventory_path_is_rejected_before_invalid_content_is_parsed(tmp_path):
    forbidden = tmp_path / "SEALED_FINAL"
    forbidden.mkdir()
    records = forbidden / "records.jsonl"
    records.write_text("this is deliberately not json", encoding="utf-8")
    manifest, distribution, sums = audit(
        source_config=forbidden / "missing-source.yaml",
        records_path=records,
        data_root=forbidden / "data",
    )
    assert manifest["calibration_ready"] is False
    assert {row["field"] for row in manifest["blockers"]} == {"source_config", "records", "data_root"}
    assert distribution["stratification_pass"] is False
    assert sums == []
