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
export_d1 = load_script("export_d1_canonical_onnx")
d1_parity = load_script("d1_pt_onnx_parity")
d1_development = load_script("evaluate_d1_development")


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


def test_d1_pinned_pt_is_fetchable_without_weakening_onnx_boundary() -> None:
    entry = registry()["models"]["d1_littercam_yolov9c"]
    source = fetch.resolve_source_artifact(
        "d1_littercam_yolov9c", entry, "best.pt"
    )
    assert source["revision"] == "861363597e109f9f0840f537f48d890cef5b5461"
    assert source["expected_sha256"] == (
        "1cf60873661811f51cd84fb6aafb403646b67d2add57c4851b0be48ebdff2873"
    )
    with pytest.raises(fetch.FetchBlocked, match="not a pinned"):
        fetch.resolve_source_artifact("d1_littercam_yolov9c", entry, "last.pt")


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


def test_d1_export_policy_has_only_three_ordered_routes(tmp_path: Path) -> None:
    routes = export_d1.route_commands(["docker", "run", "image"])
    assert [route for route, _command, _output in routes] == ["E1", "E2", "E3"]
    assert "--opset 17" in " ".join(routes[0][1])
    assert "--batch-size 1" in " ".join(routes[0][1])
    assert "--dynamic" not in " ".join(routes[0][1])
    assert "onnx_end2end" not in " ".join(routes[0][1])


def test_d1_manifest_template_is_non_claiming_and_static() -> None:
    manifest = yaml.safe_load(
        (ROOT / "config" / "d1_canonical_onnx_manifest_template.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["development_only"] is True
    assert manifest["competition_claim_allowed"] is False
    assert manifest["release_allowed"] is False
    assert manifest["export_contract"]["input_shape"] == [1, 3, 640, 640]
    assert manifest["export_contract"]["opset"] == 17
    assert manifest["export_contract"]["dynamic_axes"] is False
    assert manifest["export_contract"]["embedded_nms"] is False
    assert manifest["artifact"]["canonical_contract_pass"] is False
    assert manifest["output_contract"]["primary_output_index"] == 1
    assert manifest["output_contract"]["auxiliary_output_index"] == 0


def test_d1_dual_detect_primary_output_contract_is_fail_closed() -> None:
    inspection = {"detection_head_from_model_yaml": "DualDDetect"}
    audit = {
        "outputs": [
            {"name": "output0", "shape": [1, 14, 8400]},
            {"name": "1774", "shape": [1, 14, 8400]},
        ]
    }
    contract = export_d1.d1_output_contract(inspection, audit)
    assert contract["primary_output_index"] == 1
    assert contract["primary_output_name"] == "1774"
    with pytest.raises(RuntimeError, match="unexpected DualDDetect"):
        export_d1.d1_output_contract(inspection, {"outputs": audit["outputs"][:1]})


def test_d1_parity_selection_is_train_only_deterministic_and_hashed(
    tmp_path: Path,
) -> None:
    old = Path("F:/Project/TZcup-product-evidence")
    mapped = tmp_path / "mapped"
    images = []
    for image_id in (2, 1, 3):
        path = mapped / "dataset" / "rgb" / f"frame_{image_id}.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"image-{image_id}".encode())
        images.append(
            {
                "id": image_id,
                "file_name": str(old / "dataset" / "rgb" / path.name),
                "source_split": "train" if image_id != 3 else "holdout",
            }
        )
    coco = tmp_path / "data.json"
    coco.write_text(json.dumps({"images": images}), encoding="utf-8")
    selection = d1_parity.select_train_images(coco, old, mapped, limit=2)
    assert [item["image_id"] for item in selection["images"]] == [1, 2]
    assert all(item["source_split"] == "train" for item in selection["images"])
    assert all(len(item["sha256"]) == 64 for item in selection["images"])


def test_d1_parity_selection_rejects_blocked_split_token(tmp_path: Path) -> None:
    old = Path("F:/Project/TZcup-product-evidence")
    mapped = tmp_path / "mapped"
    bad = mapped / "SEALED_FINAL" / "rgb" / "frame.png"
    bad.parent.mkdir(parents=True)
    bad.write_bytes(b"blocked")
    coco = tmp_path / "data.json"
    coco.write_text(
        json.dumps(
            {
                "images": [
                    {
                        "id": 1,
                        "file_name": str(old / "SEALED_FINAL" / "rgb" / "frame.png"),
                        "source_split": "train",
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="blocked split token"):
        d1_parity.select_train_images(coco, old, mapped, limit=1)


def test_d1_development_metrics_cover_class_confusion_and_negative_fp() -> None:
    selection = {
        "annotation_count": 3,
        "images": [
            {
                "image_id": 1,
                "mission_id": "mission-a",
                "annotations": [
                    {
                        "category_id": 1,
                        "bbox_xyxy": [0, 0, 10, 10],
                        "bbox_short_side_px": 10,
                    },
                    {
                        "category_id": 2,
                        "bbox_xyxy": [20, 20, 30, 30],
                        "bbox_short_side_px": 10,
                    },
                ],
            },
            {
                "image_id": 2,
                "mission_id": "mission-b",
                "annotations": [
                    {
                        "category_id": 3,
                        "bbox_xyxy": [0, 0, 8, 8],
                        "bbox_short_side_px": 8,
                    }
                ],
            },
            {"image_id": 3, "mission_id": "mission-c", "annotations": []},
        ],
    }
    inference = {
        "images": [
            {
                "image_id": 1,
                "predictions": [
                    {
                        "target_category_id": 1,
                        "source_class_index": 1,
                        "bbox_xyxy": [0, 0, 10, 10],
                        "confidence": 0.9,
                    },
                    {
                        "target_category_id": 3,
                        "source_class_index": 7,
                        "bbox_xyxy": [20, 20, 30, 30],
                        "confidence": 0.8,
                    },
                ],
            },
            {"image_id": 2, "predictions": []},
            {
                "image_id": 3,
                "predictions": [
                    {
                        "target_category_id": 2,
                        "source_class_index": 2,
                        "bbox_xyxy": [1, 1, 4, 4],
                        "confidence": 0.7,
                    }
                ],
            },
        ]
    }
    metrics = d1_development.evaluate_threshold(selection, inference, 0.5)
    assert metrics["objects_micro"] == {
        "tp": 1,
        "fp": 2,
        "fn": 2,
        "precision": pytest.approx(1 / 3),
        "recall": pytest.approx(1 / 3),
        "f1": pytest.approx(1 / 3),
    }
    assert metrics["proposal_recall_class_agnostic"] == pytest.approx(2 / 3)
    assert metrics["negative_frames_with_false_positive"] == 1
    assert metrics["confusion_gt_rows"]["metal_can"]["paper_litter"] == 1
    assert metrics["small_total"] == 3


def test_d1_development_gate_fails_when_any_target_class_has_zero_tp() -> None:
    selection = {
        "source_coco": "train.json",
        "source_coco_sha256": "a" * 64,
        "selection_rule": "TRAIN only",
        "forbidden_read_flags": {
            "G10_DEV_VAL_SEALED_read": False,
            "VAL_NEW_read": False,
            "G5_V2_read": False,
        },
        "image_count": 1,
        "annotation_count": 1,
        "category_names": d1_development.CATEGORY_NAMES,
        "images": [
            {
                "image_id": 1,
                "mission_id": "m",
                "annotations": [
                    {
                        "category_id": 1,
                        "bbox_xyxy": [0, 0, 10, 10],
                        "bbox_short_side_px": 10,
                    }
                ],
            }
        ],
    }
    inference = {
        "onnx_sha256": "b" * 64,
        "providers": ["CPUExecutionProvider"],
        "preprocessing": {},
        "output_contract": {},
        "nms": {},
        "source_to_target_category": {},
        "images": [
            {
                "image_id": 1,
                "predictions": [
                    {
                        "target_category_id": 1,
                        "source_class_index": 1,
                        "bbox_xyxy": [0, 0, 10, 10],
                        "confidence": 0.9,
                    }
                ],
            }
        ],
    }
    report = d1_development.build_report(selection, inference)
    assert report["development_usable"] is False
    assert "zero_true_positive:metal_can" in report["failure_reasons"]
    assert "zero_true_positive:paper_litter" in report["failure_reasons"]
