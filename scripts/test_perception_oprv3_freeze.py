import argparse
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("oprv3_freeze", ROOT / "scripts/perception_oprv3_freeze.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_json(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def test_freeze_rejects_unpassed_development_gate(tmp_path):
    report = tmp_path / "dev.json"
    write_json(report, {"protocol": "OPRV3-07", "OPRV3_X86_DEV_PASS": False})
    args = argparse.Namespace(dev_report=report)
    with pytest.raises(ValueError, match="did not pass"):
        MODULE.build_freeze(args, "2026-08-11T00:00:00Z")


def test_atomic_writer_creates_required_freeze_files(tmp_path):
    source = "7" * 40
    freeze = {
        "freeze_id": "test-freeze", "evaluated_source_commit": source,
        "freeze_tool_revision": "8" * 40,
        "runtime": {"container_image": "example/image@sha256:" + "9" * 64},
    }
    args = argparse.Namespace(output_dir=tmp_path / "freeze")
    output = MODULE.write_freeze(args, freeze, {"locked": True}, {"status": {"frozen": True}})
    expected = {
        "MODEL_FREEZE_X86.json", "PERCEPTION_X86_FREEZE_MANIFEST.json",
        "PERCEPTION_X86_DEPENDENCY_LOCK.json", "perception_pipeline_x86_frozen.yaml", "SHA256SUMS",
    }
    assert {path.name for path in output.iterdir()} == expected
    manifest = json.loads((output / "PERCEPTION_X86_FREEZE_MANIFEST.json").read_text())
    assert manifest["status"] == "FROZEN_X86"
    assert manifest["G5_SEALED_FINAL_read"] is False
    assert yaml.safe_load((output / "perception_pipeline_x86_frozen.yaml").read_text())["status"]["frozen"] is True
    lines = (output / "SHA256SUMS").read_text().splitlines()
    assert len(lines) == 4


def test_atomic_writer_refuses_existing_output(tmp_path):
    output = tmp_path / "freeze"
    output.mkdir()
    args = argparse.Namespace(output_dir=output)
    with pytest.raises(ValueError, match="output already exists"):
        MODULE.write_freeze(args, {}, {}, {})
