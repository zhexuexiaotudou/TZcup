from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
from pathlib import Path


HERE = Path(__file__).parent


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


sealer = _load("capture_dosod_official_preprocess")
producer = _load("run_dosod_official_preprocess_capture")


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_runs_a_real_subprocess_and_seals_its_planes(tmp_path, monkeypatch):
    raw = tmp_path / "raw.rgb"; raw.write_bytes(b"r" * 12)
    binary, source = Path(sys.executable), tmp_path / "official_source.py"
    source.write_text("source")
    contract_root = tmp_path / "root"; (contract_root / "config").mkdir(parents=True)
    identity = {"status": "VERIFIED", "binary_path": str(binary.resolve()), "binary_sha256": _sha(binary), "source_path": str(source.resolve()), "source_sha256": _sha(source), "source_revision": "rev", "dpkg_package": "pkg", "dpkg_version": "1", "dpkg_path_role": "official"}
    (contract_root / "config" / "dosod_single_frame_preprocessing_oracle_contract.json").write_text(json.dumps({"official_preprocess_identity": identity}))
    monkeypatch.setattr(sealer, "ROOT", contract_root)
    monkeypatch.setattr(sealer, "_pilot_binding", lambda *_: {"path": str(raw.resolve()), "sha256": _sha(raw), "byte_size": 12, "width": 2, "height": 2, "step": 6, "encoding": "rgb8", "frame_id": "camera", "stamp_ns": 1})
    monkeypatch.setattr(producer, "_dpkg", lambda *_: ("pkg\t1\tofficial\n", 0))
    helper = tmp_path / "official.py"
    helper.write_text("import pathlib,sys\npathlib.Path(sys.argv[1]).write_bytes(b'y'*409600)\npathlib.Path(sys.argv[2]).write_bytes(b'u'*204800)\n")
    receipt = producer.run(pilot_manifest=tmp_path / "pilot.json", pilot_record_index=0, adapter_binary=binary, adapter_source=source, dpkg_package="pkg", dpkg_path_role="official", output=tmp_path / "out", timeout_seconds=10, producer_command=[str(binary.resolve()), str(helper), "{images_y}", "{images_uv}", "{raw_rgb}"])
    assert receipt["status"] == "OFFICIAL_PREPROCESS_CAPTURED"
    assert receipt["official_preprocessor"]["command"][-1] == str(raw.resolve())


def test_refuses_a_command_that_does_not_consume_all_real_inputs(tmp_path, monkeypatch):
    binary = Path(sys.executable)
    source = tmp_path / "source"; source.write_text("source")
    monkeypatch.setattr(sealer, "_pilot_binding", lambda *_: {"path": str(source.resolve()), "width": 2, "height": 2, "step": 6, "encoding": "rgb8"})
    try:
        producer.run(pilot_manifest=tmp_path / "pilot", pilot_record_index=0, adapter_binary=binary, adapter_source=source, dpkg_package="pkg", dpkg_path_role="role", output=tmp_path / "out", timeout_seconds=1, producer_command=[str(binary.resolve()), "{raw_rgb}"])
    except ValueError as error:
        assert "placeholders" in str(error)
    else:
        raise AssertionError("producer command without Y/UV outputs was accepted")
