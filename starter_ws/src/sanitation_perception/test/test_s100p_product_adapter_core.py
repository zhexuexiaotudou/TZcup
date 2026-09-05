"""Unit tests for the ROS-free S100P formal product adapter core."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from sanitation_perception.s100p_product_adapter import _perf_latency_ms
from sanitation_perception.s100p_product_adapter_core import (
    Detection,
    EdgeSamPromptBatch,
    Roi,
    S100PProductAdapterError,
    decode_edgesam_label_features,
    detections_from_ai_like,
    ground_dirt_prompt_batch,
    load_verified_board_artifact_contract,
)


def _roi(x, y, width, height, confidence, class_id=None):
    row = {"rect": {"x_offset": x, "y_offset": y, "width": width, "height": height}, "confidence": confidence}
    if class_id is not None:
        row["type"] = class_id
    return row


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_frozen_board_artifacts(tmp_path: Path) -> tuple[Path, dict[str, Path]]:
    vocabulary_rows = [
        ["small litter cube", "trash cube", "piece of litter"],
        ["fallen leaves", "leaf pile"],
        ["dust patch", "soil patch", "dirty ground"],
        ["puddle", "wet patch", "standing water"],
    ]
    blobs = {
        "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm": b"dosod-hbm",
        "dosod/tzcup_offline_vocabulary.json": (
            json.dumps(vocabulary_rows, indent=2) + "\n"
        ).encode(),
        "edgesam/edgesam_encoder_512.hbm": b"edgesam-encoder-hbm",
        "edgesam/edgesam_decoder_512.hbm": b"edgesam-decoder-hbm",
    }
    paths = {}
    for relative, payload in blobs.items():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        paths[relative] = path
    artifact_rows = {
        "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm": {
            "model_role": "project_four_class_dosod_s100p_detector",
            "source_revision": "c50129b5badf6ed7bb85e692ab493d8bdb58da6a",
        },
        "dosod/tzcup_offline_vocabulary.json": {
            "model_role": "frozen_project_prompt_vocabulary",
            "source_revision": "c50129b5badf6ed7bb85e692ab493d8bdb58da6a",
            "semantic_class_ids": [
                "litter_cube",
                "fallen_leaves",
                "dust_or_soil",
                "puddle",
            ],
            "emitted_labels": [
                "small litter cube",
                "fallen leaves",
                "dust patch",
                "puddle",
            ],
        },
        "edgesam/edgesam_encoder_512.hbm": {
            "model_role": "edgesam_512_s100p_image_encoder",
            "source_revision": "d24d99671f41a9c0003061248bded64a481e9059",
        },
        "edgesam/edgesam_decoder_512.hbm": {
            "model_role": "edgesam_512_s100p_box_prompt_decoder",
            "source_revision": "d24d99671f41a9c0003061248bded64a481e9059",
        },
    }
    for relative, row in artifact_rows.items():
        row["sha256"] = _sha256(paths[relative])
        row["byte_size"] = paths[relative].stat().st_size
    manifest = tmp_path / "artifact_manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "board_runtime_contract": {
                    "platform": "rdk_s100",
                    "board": "RDK S100P",
                    "soc": "Journey 6P",
                    "march": "nash-m",
                },
                "artifacts": artifact_rows,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return manifest, paths


def test_flatten_ai_like_targets_rejects_unknown_or_conflicting_classes_fail_closed():
    with pytest.raises(S100PProductAdapterError, match="unknown frozen"):
        detections_from_ai_like([{"type": "person", "rois": [_roi(1, 2, 3, 4, 0.8)]}])
    with pytest.raises(S100PProductAdapterError, match="disagree"):
        detections_from_ai_like(
            [{"type": "puddle", "rois": [_roi(1, 2, 3, 4, 0.8, "dust_or_soil")]}]
        )


def test_flatten_ai_like_targets_converts_target_or_roi_class_and_validates_geometry():
    detections = detections_from_ai_like(
        [
            {"type": "puddle", "rois": [_roi(1, 2, 3, 4, 0.8)]},
            {"rois": [_roi(5, 6, 7, 8, 0.7, "litter_cube")]},
        ]
    )
    assert [(row.class_id, row.roi.xyxy, row.source_index) for row in detections] == [
        ("puddle", (1.0, 2.0, 4.0, 6.0), 0),
        ("litter_cube", (5.0, 6.0, 12.0, 14.0), 1),
    ]
    with pytest.raises(S100PProductAdapterError, match="positive"):
        detections_from_ai_like([{"type": "puddle", "rois": [_roi(1, 2, 0, 4, 0.8)]}])


def test_verified_board_vocabulary_maps_real_hobot_dosod_emitted_labels(tmp_path):
    manifest, paths = _write_frozen_board_artifacts(tmp_path)
    contract = load_verified_board_artifact_contract(
        artifact_manifest_path=manifest,
        artifact_paths=paths,
    )
    detections = detections_from_ai_like(
        [
            {"type": label, "rois": [_roi(index, 0, 2, 2, 0.9, label)]}
            for index, label in enumerate(
                ("small litter cube", "fallen leaves", "dust patch", "puddle")
            )
        ],
        emitted_label_to_class_id=contract.emitted_label_to_class_id,
    )
    assert [row.class_id for row in detections] == [
        "litter_cube",
        "fallen_leaves",
        "dust_or_soil",
        "puddle",
    ]
    for alias in ("trash cube", "leaf pile", "soil patch", "wet patch"):
        with pytest.raises(S100PProductAdapterError, match="unknown frozen"):
            detections_from_ai_like(
                [{"type": alias, "rois": [_roi(0, 0, 2, 2, 0.9, alias)]}],
                emitted_label_to_class_id=contract.emitted_label_to_class_id,
            )


def test_board_artifact_contract_rejects_target_or_vocabulary_manifest_drift(tmp_path):
    manifest, paths = _write_frozen_board_artifacts(tmp_path)
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    payload["board_runtime_contract"]["march"] = "nash-e"
    manifest.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(S100PProductAdapterError, match="nash-m"):
        load_verified_board_artifact_contract(
            artifact_manifest_path=manifest,
            artifact_paths=paths,
        )

    payload["board_runtime_contract"]["march"] = "nash-m"
    vocabulary_path = paths["dosod/tzcup_offline_vocabulary.json"]
    vocabulary = json.loads(vocabulary_path.read_text(encoding="utf-8"))
    vocabulary[0][0] = "changed emitted label"
    vocabulary_path.write_text(json.dumps(vocabulary), encoding="utf-8")
    vocabulary_row = payload["artifacts"]["dosod/tzcup_offline_vocabulary.json"]
    vocabulary_row["sha256"] = _sha256(vocabulary_path)
    vocabulary_row["byte_size"] = vocabulary_path.stat().st_size
    manifest.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(S100PProductAdapterError, match="emitted labels"):
        load_verified_board_artifact_contract(
            artifact_manifest_path=manifest,
            artifact_paths=paths,
        )


def test_board_artifact_contract_rejects_tampered_model_bytes(tmp_path):
    manifest, paths = _write_frozen_board_artifacts(tmp_path)
    paths["edgesam/edgesam_encoder_512.hbm"].write_bytes(b"tampered")
    with pytest.raises(S100PProductAdapterError, match="hash or byte size"):
        load_verified_board_artifact_contract(
            artifact_manifest_path=manifest,
            artifact_paths=paths,
        )


def test_ground_dirt_prompt_batch_excludes_cube_bounds_per_class_and_large_rois():
    detections = (
        Detection("litter_cube", 0.99, Roi(0, 0, 1, 1), 0),
        Detection("puddle", 0.2, Roi(0, 0, 10, 10), 1),
        Detection("puddle", 0.9, Roi(10, 0, 10, 10), 2),
        Detection("puddle", 0.8, Roi(20, 0, 10, 10), 3),
        Detection("puddle", 0.7, Roi(30, 0, 10, 10), 4),
        Detection("puddle", 0.95, Roi(0, 0, 80, 80), 5),
        Detection("fallen_leaves", 0.6, Roi(0, 20, 10, 10), 6),
    )
    batch = ground_dirt_prompt_batch(detections, stamp_ns=123, image_width=100, image_height=100)
    assert [row.source_index for row in batch.prompts] == [6, 2, 3, 4]
    assert all(row.class_id != "litter_cube" for row in batch.prompts)


def test_decode_edgesam_labels_requires_exact_stamp_dimensions_roi_order_and_labels():
    prompts = (
        Detection("puddle", 0.8, Roi(1, 2, 3, 2), 10),
        Detection("dust_or_soil", 0.7, Roi(5, 2, 2, 2), 11),
    )
    batch = EdgeSamPromptBatch(1000, 4, 2, prompts)
    rois = [_roi(1, 2, 3, 2, 0.8), _roi(5, 2, 2, 2, 0.7)]
    decoded = decode_edgesam_label_features(
        batch,
        output_stamp_ns=1000,
        feature_values=[0, 1, 2, 1, 0, 2, 2, 0],
        capture_width=4,
        capture_height=2,
        expected_capture_width=4,
        expected_capture_height=2,
        output_prompt_rois=rois,
        output_prompt_class_ids=["puddle", "dust_or_soil"],
    )
    assert decoded.masks[0] == (False, True, False, True, False, False, False, False)
    assert decoded.masks[1] == (False, False, True, False, False, True, True, False)

    bad_cases = (
        {"output_stamp_ns": 1001},
        {"capture_width": 2},
        {"capture_width": 8, "capture_height": 4},
        {"output_prompt_rois": list(reversed(rois))},
        {"output_prompt_class_ids": ["dust_or_soil", "puddle"]},
        {"feature_values": []},
        {"feature_values": [0, 1, 3, 1, 0, 2, 2, 0]},
        {"feature_values": [0, 1.5, 2, 1, 0, 2, 2, 0]},
    )
    defaults = {
        "output_stamp_ns": 1000,
        "feature_values": [0, 1, 2, 1, 0, 2, 2, 0],
        "capture_width": 4,
        "capture_height": 2,
        "expected_capture_width": 4,
        "expected_capture_height": 2,
        "output_prompt_rois": rois,
        "output_prompt_class_ids": ["puddle", "dust_or_soil"],
    }
    for updates in bad_cases:
        with pytest.raises(S100PProductAdapterError):
            decode_edgesam_label_features(batch, **(defaults | updates))


def test_decode_edgesam_rejects_empty_prompt_batch_and_short_capture():
    empty = EdgeSamPromptBatch(0, 2, 2, ())
    with pytest.raises(S100PProductAdapterError, match="no expected prompts"):
        decode_edgesam_label_features(
            empty,
            output_stamp_ns=0,
            feature_values=[0, 0, 0, 0],
            capture_width=2,
            capture_height=2,
            expected_capture_width=2,
            expected_capture_height=2,
            output_prompt_rois=[],
            output_prompt_class_ids=[],
        )


def test_decode_accepts_real_s100p_network_mask_shape_and_segment_bbox_adjustment():
    prompt = Detection("puddle", 0.8, Roi(1020, 300, 432, 600), 0)
    batch = EdgeSamPromptBatch(55, 1920, 1080, (prompt,))
    values = [0.0] * (512 * 288)
    values[123] = 1.0
    decoded = decode_edgesam_label_features(
        batch,
        output_stamp_ns=55,
        feature_values=values,
        capture_width=512,
        capture_height=288,
        expected_capture_width=512,
        expected_capture_height=288,
        output_prompt_rois=[_roi(1020, 300, 427, 596, 0.8, "puddle")],
        output_prompt_class_ids=["puddle"],
    )
    assert decoded.image_width == 512
    assert decoded.image_height == 288
    assert decoded.masks[0][123]


def test_decode_rejects_weak_roi_overlap_even_with_correct_class_order():
    prompt = Detection("puddle", 0.8, Roi(100, 100, 100, 100), 0)
    batch = EdgeSamPromptBatch(9, 1920, 1080, (prompt,))
    with pytest.raises(S100PProductAdapterError, match="geometry"):
        decode_edgesam_label_features(
            batch,
            output_stamp_ns=9,
            feature_values=[0.0] * (512 * 288),
            capture_width=512,
            capture_height=288,
            expected_capture_width=512,
            expected_capture_height=288,
            output_prompt_rois=[_roi(150, 100, 100, 100, 0.8, "puddle")],
            output_prompt_class_ids=["puddle"],
        )


def test_perf_latency_accepts_only_positive_predict_infer_metric():
    message = SimpleNamespace(
        perfs=[
            SimpleNamespace(type="dosod_preprocess", time_ms_duration=2.0),
            SimpleNamespace(type="dosod_predict_infer", time_ms_duration=11.5),
            SimpleNamespace(type="dosod_postprocess", time_ms_duration=4.0),
        ]
    )
    assert _perf_latency_ms(message) == 11.5
    message.perfs = [SimpleNamespace(type="dosod_preprocess", time_ms_duration=2.0)]
    assert _perf_latency_ms(message) is None
