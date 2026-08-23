from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_script(name: str):
    path = ROOT / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


fetch = load_script("fetch_pretrained_models")
model_audit = load_script("audit_pretrained_model")
license_audit = load_script("audit_model_license")


def registry() -> dict:
    return yaml.safe_load(
        (ROOT / "config" / "pretrained_model_sources.yaml").read_text(
            encoding="utf-8"
        )
    )


def test_registry_pins_exact_candidate_revisions_and_source_hashes() -> None:
    models = registry()["models"]
    assert {
        key: value["source"]["revision"] for key, value in models.items()
    } == {
        "d1_littercam_yolov9c": "861363597e109f9f0840f537f48d890cef5b5461",
        "d2_suhan_yolo11n": "d7a78128455ef607a922f50681187f8b32b2af53",
        "c1_wastewise_yolov8n_cls": "a30c36c6b181ac0d2eb387bbd4f6d4a5b88ee078",
        "c2_ecodetect_efficientnetb0": "9719e6fc9a352d62209529e0e0573fff3bb7dc3d",
    }
    assert models["d2_suhan_yolo11n"]["source"]["expected_sha256"]
    assert models["c1_wastewise_yolov8n_cls"]["source"]["expected_sha256"]
    assert all(len(value["model_card"]["sha256"]) == 64 for value in models.values())
    assert all(value["download_command"] for value in models.values())


@pytest.mark.parametrize(
    "model_id", ["d1_littercam_yolov9c", "c2_ecodetect_efficientnetb0"]
)
def test_registry_source_without_onnx_is_fail_closed(model_id: str) -> None:
    with pytest.raises(fetch.FetchBlocked, match="does not contain"):
        fetch.validate_fetchable(model_id, registry()["models"][model_id])


def test_download_resumes_and_verifies_sha(monkeypatch, tmp_path: Path) -> None:
    payload = b"pinned-model-content" * 100
    expected = hashlib.sha256(payload).hexdigest()
    destination = tmp_path / "model.onnx"
    partial = destination.with_suffix(".onnx.part")
    partial.write_bytes(payload[:57])
    seen_range = []

    class Response:
        status = 206

        def __init__(self) -> None:
            self._position = 57

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def getcode(self):
            return self.status

        def read(self, count: int) -> bytes:
            chunk = payload[self._position : self._position + count]
            self._position += len(chunk)
            return chunk

    def fake_urlopen(request, timeout):
        del timeout
        seen_range.append(request.get_header("Range"))
        return Response()

    monkeypatch.setattr(fetch.urllib.request, "urlopen", fake_urlopen)
    result = fetch.download_with_resume(
        "https://example.invalid/model.onnx",
        destination,
        expected,
        len(payload),
        timeout=1,
        retries=0,
        chunk_size=31,
    )
    assert destination.read_bytes() == payload
    assert result["resumed_from_bytes"] == 57
    assert seen_range == ["bytes=57-"]


def test_short_response_retries_from_new_partial_offset(monkeypatch, tmp_path: Path) -> None:
    payload = b"retryable-download" * 20
    expected = hashlib.sha256(payload).hexdigest()
    destination = tmp_path / "model.onnx"
    offsets: list[int] = []

    class Response:
        def __init__(self, offset: int, stop: int) -> None:
            self.status = 200 if offset == 0 else 206
            self._position = offset
            self._stop = stop

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def getcode(self):
            return self.status

        def read(self, count: int) -> bytes:
            chunk = payload[self._position : min(self._position + count, self._stop)]
            self._position += len(chunk)
            return chunk

    def fake_urlopen(request, timeout):
        del timeout
        header = request.get_header("Range")
        offset = int(header.removeprefix("bytes=").removesuffix("-")) if header else 0
        offsets.append(offset)
        stop = 73 if len(offsets) == 1 else len(payload)
        return Response(offset, stop)

    monkeypatch.setattr(fetch.urllib.request, "urlopen", fake_urlopen)
    fetch.download_with_resume(
        "https://example.invalid/model.onnx",
        destination,
        expected,
        len(payload),
        timeout=1,
        retries=1,
        chunk_size=29,
    )
    assert destination.read_bytes() == payload
    assert offsets == [0, 73]


def test_offline_cache_rejects_wrong_hash(tmp_path: Path) -> None:
    cache = tmp_path / "cache"
    cache.mkdir()
    (cache / "model.onnx").write_bytes(b"wrong")
    destination = tmp_path / "artifact" / "model.onnx"
    assert not fetch.copy_from_offline_cache(
        cache, "model", "model.onnx", destination, "0" * 64, 5
    )
    assert not destination.exists()


def test_model_id_cannot_escape_artifact_root() -> None:
    with pytest.raises(fetch.FetchBlocked, match="unsafe model id"):
        fetch._safe_model_id("../outside")


def test_license_audit_blocks_unresolved_dependencies() -> None:
    report = license_audit.build_audit(registry())
    assert report["release_allowed"] is False
    assert report["all_components_resolved"] is False
    assert all(record["release_allowed"] is False for record in report["models"])


def test_registered_missing_onnx_cannot_be_audited(tmp_path: Path) -> None:
    with pytest.raises(model_audit.AuditBlocked, match="no source ONNX"):
        model_audit.resolve_registered_model(
            ROOT / "config" / "pretrained_model_sources.yaml",
            "d1_littercam_yolov9c",
            tmp_path,
        )


def test_onnx_dependency_block_is_explicit_when_package_missing(monkeypatch) -> None:
    monkeypatch.setitem(sys.modules, "onnx", None)
    with pytest.raises(model_audit.AuditBlocked, match="ONNX_AUDIT_BLOCKED"):
        model_audit.load_onnx()


def test_selection_file_keeps_product_claims_disabled() -> None:
    selection = yaml.safe_load(
        (ROOT / "config" / "pretrained_model_selection.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert selection["selection_frozen"] is False
    assert selection["selected"] == {
        "detector": None,
        "close_range_classifier": None,
    }
    assert selection["competition_claim_allowed"] is False
    assert selection["release_allowed"] is False


def test_license_cli_writes_machine_readable_blocked_report(tmp_path: Path) -> None:
    output = tmp_path / "PRETRAINED_MODEL_LICENSE_AUDIT.json"
    old_argv = sys.argv
    sys.argv = [
        "audit_model_license.py",
        "--registry",
        str(ROOT / "config" / "pretrained_model_sources.yaml"),
        "--output",
        str(output),
    ]
    try:
        assert license_audit.main() == 2
    finally:
        sys.argv = old_argv
    assert json.loads(output.read_text(encoding="utf-8"))["release_allowed"] is False
