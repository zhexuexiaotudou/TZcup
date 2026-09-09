import json
from pathlib import Path
import sys

import yaml


SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

from build_journey6_source_bundle import REQUIRED_COMPONENTS, build


def test_committed_source_bundle_is_truthfully_blocked_and_reference_only(tmp_path):
    output = tmp_path / "output"
    status = build(
        ROOT / "deploy" / "journey6" / "source_bundle" / "source_bundle.template.yaml",
        output,
        ROOT,
    )
    assert status["source_bundle_ready"] is False
    codes = {row["code"] for row in status["blockers"]}
    assert "model_selection_not_frozen" in codes
    assert "model_license_not_release_clear" in codes
    assert not any(
        row.get("component") == "cpp_postprocess" for row in status["blockers"]
    )
    assert set(path.name for path in output.iterdir()) == {
        "J6_SOURCE_BUNDLE_MANIFEST.json",
        "J6_SOURCE_BUNDLE_SHA256SUMS",
        "J6_SOURCE_BUNDLE_STATUS.json",
    }
    manifest = json.loads(
        (output / "J6_SOURCE_BUNDLE_MANIFEST.json").read_text(encoding="utf-8")
    )
    cpp_component = next(
        item for item in manifest["components"] if item["id"] == "cpp_postprocess"
    )
    assert len(cpp_component["observed_sha256"]) == 64


def test_committed_cpp_postprocess_points_to_native_graph_external_source():
    template = yaml.safe_load(
        (
            ROOT
            / "deploy"
            / "journey6"
            / "source_bundle"
            / "source_bundle.template.yaml"
        ).read_text(encoding="utf-8")
    )
    components = {item["id"]: item for item in template["components"]}
    component = components["cpp_postprocess"]
    assert component["path"] == (
        "deploy/journey6/board_runtime/src/d1_yolov9_postprocess.cpp"
    )
    assert component["copy_policy"] == "reference_only"
    assert (ROOT / component["path"]).is_file()


def _write(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(value, dict):
        path.write_text(json.dumps(value), encoding="utf-8")
    else:
        path.write_bytes(value)
    return path


def test_complete_source_prerequisites_can_be_locked_without_copying_payloads(tmp_path):
    prerequisites = tmp_path / "prerequisites"
    calibration_manifest = _write(prerequisites / "J6_CALIBRATION_MANIFEST.json", {"calibration_ready": True, "sealed_access_allowed": False})
    calibration_distribution = _write(prerequisites / "J6_CALIBRATION_DISTRIBUTION.json", {"stratification_pass": True})
    calibration_sums = prerequisites / "calibration.sums"
    calibration_sums.write_text(
        f"{__import__('hashlib').sha256(calibration_manifest.read_bytes()).hexdigest()}  J6_CALIBRATION_MANIFEST.json\n"
        f"{__import__('hashlib').sha256(calibration_distribution.read_bytes()).hexdigest()}  J6_CALIBRATION_DISTRIBUTION.json\n",
        encoding="utf-8",
    )
    golden_payload = _write(prerequisites / "golden.npz", b"golden-tensor-payload")
    golden_digest = __import__("hashlib").sha256(golden_payload.read_bytes()).hexdigest()
    paths = {
        "detector_canonical_onnx": _write(prerequisites / "detector.onnx", b"detector-source-reference"),
        "classifier_canonical_onnx": _write(prerequisites / "classifier.onnx", b"classifier-source-reference"),
        "area_canonical_onnx": _write(prerequisites / "area.onnx", b"area-source-reference"),
        "model_lock": _write(prerequisites / "model_lock.json", {"selection_frozen": True, "selected": {"detector": "d", "close_range_classifier": "c"}}),
        "model_license_audit": _write(prerequisites / "license.json", {"release_license_pass": True}),
        "calibration_manifest": calibration_manifest,
        "calibration_distribution": calibration_distribution,
        "calibration_sha256sums": calibration_sums,
        "nv12_contract": _write(prerequisites / "nv12.py", b"NV12 = True\n"),
        "python_postprocess": _write(prerequisites / "postprocess.py", b"def decode(): pass\n"),
        "cpp_postprocess": _write(prerequisites / "postprocess.cpp", b"void Decode() {}\n"),
        "golden_tensor_lock": _write(prerequisites / "golden.json", {"golden_tensor_ready": True, "tensors": [{"name": "output", "path": "golden.npz", "sha256": golden_digest}]}),
        "toolchain_lock": _write(prerequisites / "toolchain.json", {"target_family": "journey6", "status": "validated"}),
        "board_runtime_source": _write(prerequisites / "runtime" / "main.cpp", b"int main() {}\n").parent,
        "install_source": _write(prerequisites / "install.sh", b"exit 0\n"),
        "healthcheck_source": _write(prerequisites / "health.sh", b"exit 0\n"),
        "rollback_source": _write(prerequisites / "rollback.sh", b"exit 0\n"),
        "hil_config": _write(prerequisites / "hil.yaml", b"schema_version: 1\n"),
    }
    profiles = prerequisites / "profiles"
    profiles.mkdir()
    for march in ("nash-e", "nash-m", "nash-p"):
        (profiles / f"{march}.yaml").write_text(yaml.safe_dump({"target_march": march, "status": "validated"}), encoding="utf-8")
    paths["nash_profiles"] = profiles
    assert set(paths) == REQUIRED_COMPONENTS
    template = {
        "schema_version": 1,
        "bundle_id": "test-source-bundle",
        "target_family": "journey6",
        "target_sku": "auto",
        "target_march": "auto",
        "source_only": True,
        "components": [
            {"id": component, "path": str(path), "copy_policy": "reference_only", "expected_sha256": None}
            for component, path in sorted(paths.items())
        ],
    }
    template_path = tmp_path / "template.yaml"
    template_path.write_text(yaml.safe_dump(template, sort_keys=False), encoding="utf-8")
    output = tmp_path / "bundle"
    status = build(template_path, output, ROOT)
    assert status["source_bundle_ready"] is True
    assert status["blockers"] == []
    assert not (output / "detector.onnx").exists()
    manifest = json.loads((output / "J6_SOURCE_BUNDLE_MANIFEST.json").read_text(encoding="utf-8"))
    assert all(item["copy_policy"] == "reference_only" and item["observed_sha256"] for item in manifest["components"])


def test_compiled_hbm_can_never_be_a_source_component(tmp_path):
    hbm = _write(tmp_path / "model.hbm", b"compiled")
    template = yaml.safe_load((ROOT / "deploy" / "journey6" / "source_bundle" / "source_bundle.template.yaml").read_text(encoding="utf-8"))
    for item in template["components"]:
        if item["id"] == "detector_canonical_onnx":
            item["path"] = str(hbm)
    template_path = tmp_path / "template.yaml"
    template_path.write_text(yaml.safe_dump(template, sort_keys=False), encoding="utf-8")
    status = build(template_path, tmp_path / "output", ROOT)
    assert "compiled_or_archive_payload_forbidden" in {row["code"] for row in status["blockers"]}
