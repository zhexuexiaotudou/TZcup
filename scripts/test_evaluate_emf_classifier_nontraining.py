import hashlib
import json

import pytest
from evaluate_emf_classifier_nontraining import (
    EXPECTED_SOURCE_MANIFEST_SHA256,
    MAXIMUM_UNKNOWN_PROBABILITIES,
    TARGET_PROBABILITY_MINIMUMS,
    EvaluationError,
    evaluate_files,
)


def write_smoke(tmp_path, model_id, rows, *, suffix="smoke"):
    if model_id == "c1_wastewise_yolov8n_cls":
        class_order = [
            "battery",
            "biological",
            "cardboard",
            "glass",
            "metal",
            "paper",
            "plastic",
            "trash",
        ]
        model_sha256 = (
            "2b46d491091dbc0ed98a0f1eaee7fe5739c8fd3eb5bd5935396c3b2712e1f7a6"
        )
        artifacts = None
        rows = [
            dict(
                item,
                probabilities={
                    "battery": 0.0,
                    "biological": 0.0,
                    **item["probabilities"],
                },
            )
            for item in rows
        ]
    else:
        class_order = ["cardboard", "glass", "metal", "paper", "plastic", "trash"]
        model_sha256 = None
        artifacts = {
            "config.json": "341bb75a50a7dbd13034e189a02ad7cf54e8a6af28357d66c138a397e0d28c6e",
            "model.safetensors": "a67e2f6a82914be03cfb85218bc4e7683c8e81fe3fc4a5f9bed3abc8e93757c8",
            "preprocessor_config.json": "9b36b57ebaf20f09bf4c22100ccc21877ea6bfe5aead0c00c59f8af8ccefacfc",
        }
    payload = {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "model_id": model_id,
        "model_sha256": model_sha256,
        "artifacts": artifacts,
        "source_manifest_sha256": EXPECTED_SOURCE_MANIFEST_SHA256,
        "class_order": class_order,
        "class_mapping": {
            "metal": "metal_can",
            "paper": "paper_litter",
            "plastic": "plastic_bottle",
        },
        "rows": rows,
    }
    path = tmp_path / f"{suffix}.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def row(record_id, actual, *, metal, paper, plastic, unknown):
    return {
        "record_id": f"G10_TRAIN:{record_id}",
        "source_image_sha256": hashlib.sha256(record_id.encode()).hexdigest(),
        "bbox_xyxy": [1.0, 2.0, 11.0, 12.0],
        "relative_file": f"fixed/{record_id}.png",
        "actual_product_class": actual,
        "probabilities": {
            "cardboard": unknown,
            "glass": 0.0,
            "metal": metal,
            "paper": paper,
            "plastic": plastic,
            "trash": 0.0,
        },
    }


def test_fixed_grid_reports_all_candidates_but_never_selects_or_trains(tmp_path):
    rows = [
        row(
            "background",
            "background",
            metal=0.05,
            paper=0.05,
            plastic=0.05,
            unknown=0.85,
        ),
        row(
            "plastic",
            "plastic_bottle",
            metal=0.05,
            paper=0.05,
            plastic=0.75,
            unknown=0.15,
        ),
        row("metal", "metal_can", metal=0.75, paper=0.05, plastic=0.05, unknown=0.15),
        row(
            "paper", "paper_litter", metal=0.05, paper=0.75, plastic=0.05, unknown=0.15
        ),
    ]
    c1 = write_smoke(tmp_path, "c1_wastewise_yolov8n_cls", rows, suffix="c1")
    c4 = write_smoke(tmp_path, "c4_prithiv_trash_net_siglip2", rows, suffix="c4")

    report = evaluate_files([c1, c4])

    expected = 2 * len(TARGET_PROBABILITY_MINIMUMS) * len(MAXIMUM_UNKNOWN_PROBABILITIES)
    assert len(report["candidates"]) == expected
    assert all(
        not item["selected"] and not item["frozen"] for item in report["candidates"]
    )
    assert report["upper_bound_summary"]["global"]["maximum_train_macro_f1"] == 1.0
    assert report["threshold_selection"] == {
        "selected": False,
        "frozen": False,
        "reason": (
            "A5 threshold selection requires an untouched complete HOLDOUT; the current "
            "HOLDOUT has no plastic_bottle examples."
        ),
    }
    assert report["EMF_NONTRAINING_ADJUSTMENT_COMPLETE"] is False
    assert report["training"] is False
    assert report["training_authorized"] is False


@pytest.mark.parametrize(
    "marker", ["G5", "G5_V2", "VAL_NEW", "DEV_VAL", "SEALED_FINAL"]
)
def test_forbidden_data_markers_are_rejected_anywhere_in_payload(tmp_path, marker):
    rows = [row("one", "background", metal=0.1, paper=0.1, plastic=0.1, unknown=0.7)]
    path = write_smoke(tmp_path, "c1_wastewise_yolov8n_cls", rows)
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["nested_provenance"] = {"source": f"archive/{marker}/rows"}
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(EvaluationError, match="forbidden evaluation data marker"):
        evaluate_files([path])


def test_non_train_rows_and_mismatched_probability_schema_fail_closed(tmp_path):
    rows = [row("one", "background", metal=0.1, paper=0.1, plastic=0.1, unknown=0.7)]
    rows[0]["record_id"] = "G10_HOLDOUT:one"
    path = write_smoke(tmp_path, "c4_prithiv_trash_net_siglip2", rows)
    with pytest.raises(EvaluationError, match="G10_TRAIN"):
        evaluate_files([path])

    rows[0]["record_id"] = "G10_TRAIN:one"
    rows[0]["probabilities"].pop("trash")
    path = write_smoke(tmp_path, "c4_prithiv_trash_net_siglip2", rows, suffix="bad")
    with pytest.raises(EvaluationError, match="class_order"):
        evaluate_files([path])


def test_requires_both_models_and_the_fixed_source_manifest(tmp_path):
    rows = [row("one", "background", metal=0.1, paper=0.1, plastic=0.1, unknown=0.7)]
    c1 = write_smoke(tmp_path, "c1_wastewise_yolov8n_cls", rows, suffix="c1")
    with pytest.raises(EvaluationError, match="exactly one C1 and one C4"):
        evaluate_files([c1])

    c4 = write_smoke(tmp_path, "c4_prithiv_trash_net_siglip2", rows, suffix="c4")
    payload = json.loads(c4.read_text(encoding="utf-8"))
    payload["source_manifest_sha256"] = "a" * 64
    c4.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(EvaluationError, match="fixed crop bank"):
        evaluate_files([c1, c4])


def test_rejects_artifact_schema_and_crop_identity_mismatch(tmp_path):
    rows = [row("one", "background", metal=0.1, paper=0.1, plastic=0.1, unknown=0.7)]
    c1 = write_smoke(tmp_path, "c1_wastewise_yolov8n_cls", rows, suffix="c1")
    c4 = write_smoke(tmp_path, "c4_prithiv_trash_net_siglip2", rows, suffix="c4")

    payload = json.loads(c1.read_text(encoding="utf-8"))
    payload["model_sha256"] = "0" * 64
    c1.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(EvaluationError, match="model SHA-256 contract"):
        evaluate_files([c1, c4])

    c1 = write_smoke(tmp_path, "c1_wastewise_yolov8n_cls", rows, suffix="c1-fixed")
    payload = json.loads(c4.read_text(encoding="utf-8"))
    payload["rows"][0]["source_image_sha256"] = "f" * 64
    c4.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(
        EvaluationError, match="crop identities are not exactly aligned"
    ):
        evaluate_files([c1, c4])
