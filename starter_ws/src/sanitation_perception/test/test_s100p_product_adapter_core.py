"""Unit tests for the ROS-free S100P formal product adapter core."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from sanitation_perception.s100p_product_adapter import _perf_latency_ms
from sanitation_perception.s100p_product_adapter_core import (
    Detection,
    EdgeSamPromptBatch,
    ExactStampRgbdCache,
    PendingDosodCache,
    PendingEdgeSamCache,
    Roi,
    S100P_POSTPROCESS_THRESHOLDS,
    S100PProductAdapterError,
    decode_edgesam_label_features,
    detections_from_ai_like,
    ground_dirt_prompt_batch,
    filter_s100p_product_detections,
    load_verified_board_artifact_contract,
    requires_edgesam_handoff,
    validate_exact_rgbd_projection_binding,
    validate_exact_tf_binding,
    validate_dosod_source_rois,
    validate_public_map_binding,
)
import sanitation_perception.s100p_product_adapter_core as core
import sanitation_perception.s100p_development_artifact_contract as development
from sanitation_perception.s100p_development_artifact_contract import (
    load_verified_development_artifact_contract,
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


def _write_development_artifacts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> tuple[Path, dict[str, Path]]:
    vocabulary_rows = [["small litter cube"], ["fallen leaves"], ["dust patch"], ["puddle"]]
    blobs = {
        "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm": b"A2 development HBM",
        "dosod/tzcup_offline_vocabulary.json": (json.dumps(vocabulary_rows) + "\n").encode(),
        "edgesam/edgesam_encoder_512.hbm": b"development encoder",
        "edgesam/edgesam_decoder_512.hbm": b"development decoder",
    }
    paths: dict[str, Path] = {}
    for relative, contents in blobs.items():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(contents)
        paths[relative] = path
    disas = tmp_path / "hbrt4-disas.json"
    disas.write_text('{"capture":"hbrt4-disas"}', encoding="utf-8")
    hbm_sha = _sha256(paths["dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"])
    abi_gate = tmp_path / "abi_gate.json"
    abi_gate.write_text(json.dumps({
        "status": "NON_FORMAL_ABI_GATE_PASSED",
        "classification": "NON_FORMAL_ABI_DEVELOPMENT",
        "pass": True,
        "hbm": {"sha256": hbm_sha, "byte_size": paths["dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"].stat().st_size},
        "tool": {"argv": ["hbrt4-disas", "--json"], "returncode": 0, "path": "/tool/hbrt4-disas", "sha256": "a" * 64, "version": "fixture"},
        "disas": {"sha256": _sha256(disas)},
        "outputs": [
            {"name": "scores", "dtype": "TYPE_TAG_SI16", "dims": [1, 8400, 4], "strides_bytes": [67200, 8, 2]},
            {"name": "boxes", "dtype": "TYPE_TAG_SI16", "dims": [1, 8400, 4], "strides_bytes": [67200, 8, 2]},
        ],
    }), encoding="utf-8")
    monkeypatch.setattr(development, "DEVELOPMENT_DOSOD_HBM_SHA256", hbm_sha)
    monkeypatch.setattr(development, "DEVELOPMENT_DOSOD_HBM_BYTE_SIZE", paths["dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"].stat().st_size)
    monkeypatch.setattr(development, "DEVELOPMENT_VOCABULARY_SHA256", _sha256(paths["dosod/tzcup_offline_vocabulary.json"]))
    monkeypatch.setattr(development, "DEVELOPMENT_EDGESAM_ENCODER_SHA256", _sha256(paths["edgesam/edgesam_encoder_512.hbm"]))
    monkeypatch.setattr(development, "DEVELOPMENT_EDGESAM_ENCODER_BYTE_SIZE", paths["edgesam/edgesam_encoder_512.hbm"].stat().st_size)
    monkeypatch.setattr(development, "DEVELOPMENT_EDGESAM_DECODER_SHA256", _sha256(paths["edgesam/edgesam_decoder_512.hbm"]))
    monkeypatch.setattr(development, "DEVELOPMENT_EDGESAM_DECODER_BYTE_SIZE", paths["edgesam/edgesam_decoder_512.hbm"].stat().st_size)
    monkeypatch.setattr(development, "DEVELOPMENT_ABI_GATE_SHA256", _sha256(abi_gate))
    monkeypatch.setattr(development, "DEVELOPMENT_DISAS_SHA256", _sha256(disas))
    artifacts = {
        relative: {
            "path": str(path.resolve()), "sha256": _sha256(path), "byte_size": path.stat().st_size,
            "model_role": development.DEVELOPMENT_DOSOD_ROLE if relative.startswith("dosod/dosod") else core.BOARD_ARTIFACT_SPECS[relative][0],
            "source_revision": development.DEVELOPMENT_DOSOD_SOURCE_REVISION if relative.startswith("dosod/dosod") else core.BOARD_ARTIFACT_SPECS[relative][1],
        }
        for relative, path in paths.items()
    }
    manifest = tmp_path / "development_artifact_manifest.json"
    manifest.write_text(json.dumps({
        "schema_version": 1, "manifest_id": development.DEVELOPMENT_MANIFEST_ID,
        "status": development.DEVELOPMENT_STATUS, "classification": development.DEVELOPMENT_CLASSIFICATION,
        "formal": False, "board_acceptance": False, "claim_scope": development.DEVELOPMENT_CLAIM_SCOPE,
        "artifacts": artifacts,
        "custom_vocabulary": {"artifact": "dosod/tzcup_offline_vocabulary.json", "kind": "CUSTOM4", "semantic_class_ids": list(core.FROZEN_CLASS_ORDER), "emitted_labels": [row[0] for row in vocabulary_rows]},
        "abi_gate": {"path": str(abi_gate.resolve()), "sha256": _sha256(abi_gate), "byte_size": abi_gate.stat().st_size},
        "disas": {"path": str(disas.resolve()), "sha256": _sha256(disas), "byte_size": disas.stat().st_size},
    }), encoding="utf-8")
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


def test_s100p_postprocess_uses_the_frozen_per_class_thresholds():
    assert dict(S100P_POSTPROCESS_THRESHOLDS) == {
        "litter_cube": 0.005,
        "fallen_leaves": 0.0025,
        "dust_or_soil": 0.002,
        "puddle": 0.003,
    }
    detections = tuple(
        Detection(class_id, threshold, Roi(0, 0, 1, 1), index)
        for index, (class_id, threshold) in enumerate(S100P_POSTPROCESS_THRESHOLDS.items())
    )
    below = Detection("puddle", 0.0029, Roi(0, 0, 1, 1), 9)
    assert filter_s100p_product_detections((*detections, below)) == detections
    with pytest.raises(S100PProductAdapterError, match="frozen S100P domain"):
        filter_s100p_product_detections((Detection("unknown", 0.9, Roi(0, 0, 1, 1), 10),))


def test_projection_inputs_require_one_exact_rgb_depth_camerainfo_frame():
    valid = {
        "rgb_stamp_ns": 42,
        "rgb_frame_id": "front_camera_optical",
        "rgb_width": 4,
        "rgb_height": 2,
        "depth_stamp_ns": 42,
        "depth_frame_id": "front_camera_optical",
        "depth_width": 4,
        "depth_height": 2,
        "depth_encoding": "16UC1",
        "camera_info_stamp_ns": 42,
        "camera_info_frame_id": "front_camera_optical",
        "camera_info_width": 4,
        "camera_info_height": 2,
        "camera_k": (1.0, 0.0, 2.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0),
    }
    assert validate_exact_rgbd_projection_binding(**valid) is None
    for update in (
        {"depth_stamp_ns": 43},
        {"depth_frame_id": "other_optical"},
        {"depth_encoding": "rgb8"},
        {"camera_info_stamp_ns": 43},
        {"camera_info_frame_id": "other_optical"},
        {"camera_info_width": 5},
        {"camera_k": (0.0,) * 9},
        {"camera_k": (1.0,) * 8},
    ):
        with pytest.raises(S100PProductAdapterError):
            validate_exact_rgbd_projection_binding(**(valid | update))


def test_dosod_rois_are_already_bound_to_the_original_848x480_source_frame():
    source = Detection("puddle", 0.8, Roi(0.0, 0.0, 84.8, 84.8), 0)
    assert validate_dosod_source_rois((source,)) == (source,)
    with pytest.raises(S100PProductAdapterError, match="848x480"):
        validate_dosod_source_rois((Detection("puddle", 0.8, Roi(800, 0, 64, 64), 0),))


def test_exact_stamp_rgbd_cache_preserves_n_until_dosod_n_then_consumes_it_once():
    cache = ExactStampRgbdCache(3)
    for stamp in (100, 200, 300):
        cache.put_rgb(stamp, f"rgb-{stamp}")
        cache.put_depth(stamp, f"depth-{stamp}")
        cache.put_camera_info(stamp, f"info-{stamp}")
    assert cache.consume(100) == ("rgb-100", "depth-100", "info-100")
    with pytest.raises(S100PProductAdapterError, match="no exact"):
        cache.consume(100)


def test_pending_edgesam_handoff_is_consumed_on_terminal_attempt_and_replay_fails():
    pending = PendingEdgeSamCache(2)
    pending.put(100, "frame-100")
    assert pending.consume(100) == "frame-100"
    with pytest.raises(S100PProductAdapterError, match="no pending"):
        pending.consume(100)
    pending.put(200, "frame-200")
    assert pending.consume(200) == "frame-200"  # terminal failure also consumes


def test_empty_ground_prompt_batch_is_terminal_and_never_requires_edgesam_pending_state():
    batch = EdgeSamPromptBatch(stamp_ns=100, image_width=848, image_height=480, prompts=())
    assert not requires_edgesam_handoff(batch)


def test_pending_dosod_waits_for_exact_late_depth_and_info_then_rejects_replay_expiry_and_eviction():
    sources = ExactStampRgbdCache(3)
    pending = PendingDosodCache(limit=1, max_age_ns=50)
    sources.put_rgb(100, "rgb-n")
    pending.put(100, "dosod-n", now_ns=1_000)
    assert pending.take_if_ready(100, sources) is None
    sources.put_depth(100, "depth-n")
    sources.put_camera_info(100, "info-n")
    assert pending.take_if_ready(100, sources) == (
        "dosod-n", ("rgb-n", "depth-n", "info-n")
    )
    assert pending.take_if_ready(100, sources) is None
    pending.put(200, "dosod-200", now_ns=1_000)
    assert pending.put(300, "dosod-300", now_ns=1_001) == (200,)
    assert pending.expire(1_052) == (300,)


def test_exact_join_frontier_blocks_history_eviction_replay_but_allows_staged_out_of_order_stamp():
    sources = ExactStampRgbdCache(2)
    pending = PendingDosodCache(limit=2, max_age_ns=100)
    for stamp in (1000, 900):
        sources.put_rgb(stamp, f"rgb-{stamp}")
        sources.put_depth(stamp, f"depth-{stamp}")
        sources.put_camera_info(stamp, f"info-{stamp}")
        pending.put(stamp, f"dosod-{stamp}", now_ns=10)
    assert pending.take_if_ready(1000, sources) == (
        "dosod-1000", ("rgb-1000", "depth-1000", "info-1000")
    )
    for put in (sources.put_rgb, sources.put_depth, sources.put_camera_info):
        with pytest.raises(S100PProductAdapterError, match="consumed frontier"):
            put(1000, "late-replay")
    with pytest.raises(S100PProductAdapterError, match="duplicate DOSOD"):
        pending.put(1000, "dosod-replay", now_ns=11)
    assert pending.take_if_ready(900, sources) == (
        "dosod-900", ("rgb-900", "depth-900", "info-900")
    )


def test_consumed_frontier_rejects_replay_after_bounded_history_evicts_it():
    sources = ExactStampRgbdCache(2)
    pending = PendingDosodCache(limit=2, max_age_ns=100)
    for stamp in (1000, 2000, 3000):
        sources.put_rgb(stamp, f"rgb-{stamp}")
        sources.put_depth(stamp, f"depth-{stamp}")
        sources.put_camera_info(stamp, f"info-{stamp}")
        pending.put(stamp, f"dosod-{stamp}", now_ns=stamp)
        assert pending.take_if_ready(stamp, sources) == (
            f"dosod-{stamp}", (f"rgb-{stamp}", f"depth-{stamp}", f"info-{stamp}")
        )
    for put in (sources.put_rgb, sources.put_depth, sources.put_camera_info):
        with pytest.raises(S100PProductAdapterError, match="consumed frontier"):
            put(1000, "replayed-source")
    with pytest.raises(S100PProductAdapterError, match="duplicate DOSOD"):
        pending.put(1000, "replayed-dosod", now_ns=3_001)


def test_public_map_and_exact_tf_bindings_are_strict_and_static_tf_is_explicit():
    valid = dict(
        frame_id="map", expected_frame_id="map", width=2, height=3,
        resolution=0.05, origin_values=(0, 0, 0, 0, 0, 0, 1), data_length=6,
    )
    assert validate_public_map_binding(**valid) is None
    for change in ({"frame_id": "odom"}, {"data_length": 5}, {"resolution": 0.0}, {"origin_values": (0, 0, 0, 0, 0, 0, float("nan"))}):
        with pytest.raises(S100PProductAdapterError):
            validate_public_map_binding(**(valid | change))
    assert validate_exact_tf_binding(image_stamp_ns=2_000_000_000, transform_stamp_ns=0, max_age_s=0.75) == "static"
    assert validate_exact_tf_binding(image_stamp_ns=2_000_000_000, transform_stamp_ns=1_250_000_000, max_age_s=0.75) == "dynamic"
    for transform_stamp in (2_000_000_001, 1_249_999_999):
        with pytest.raises(S100PProductAdapterError):
            validate_exact_tf_binding(image_stamp_ns=2_000_000_000, transform_stamp_ns=transform_stamp, max_age_s=0.75)
    with pytest.raises(S100PProductAdapterError, match="0.75"):
        validate_exact_tf_binding(image_stamp_ns=2_000_000_000, transform_stamp_ns=2_000_000_000, max_age_s=0.751)


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


def test_development_artifact_mode_loads_only_a_trust_anchored_manifest(tmp_path, monkeypatch):
    manifest, paths = _write_development_artifacts(tmp_path, monkeypatch)
    contract = load_verified_development_artifact_contract(artifact_manifest_path=manifest, artifact_paths=paths)
    assert contract.model_hashes["dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"] == _sha256(
        paths["dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"]
    )
    assert dict(contract.emitted_label_to_class_id) == {
        "small litter cube": "litter_cube", "fallen leaves": "fallen_leaves",
        "dust patch": "dust_or_soil", "puddle": "puddle",
    }


def test_development_mode_keeps_the_version_controlled_a2_trust_anchors():
    assert development.DEVELOPMENT_DOSOD_SOURCE_REVISION == "55b5525e0df3dff35e9ba75fa601601dff057b81"
    assert development.DEVELOPMENT_DOSOD_HBM_BYTE_SIZE == 13055760
    assert development.DEVELOPMENT_DOSOD_HBM_SHA256 == "5a17357c8dd52769a1ef0829a12ed03f1253139315f7b272441013aa50979f9b"
    assert development.DEVELOPMENT_VOCABULARY_SHA256 == "c5b10ba0e26ee28cdbf5192775e7d2ddb3f5852e515f59a074b38b7ed69d7ffd"
    assert development.DEVELOPMENT_ABI_GATE_SHA256 == "7cc1877c42f3de88fdae53c949460a2d30812141a4b086c945ae67b1be03ca4b"
    assert development.DEVELOPMENT_DISAS_SHA256 == "85d6048c1fb68ce53f238258df1aa83f8656fe9362d2e9942706267c5874f2b0"
    assert core.BOARD_ARTIFACT_SPECS["edgesam/edgesam_encoder_512.hbm"][1] == "d24d99671f41a9c0003061248bded64a481e9059"


def test_formal_and_development_artifact_modes_explicitly_reject_each_other(tmp_path, monkeypatch):
    development_manifest, development_paths = _write_development_artifacts(tmp_path, monkeypatch)
    with pytest.raises(S100PProductAdapterError, match="formal S100P nash-m contract"):
        load_verified_board_artifact_contract(
            artifact_manifest_path=development_manifest, artifact_paths=development_paths
        )
    formal_manifest, formal_paths = _write_frozen_board_artifacts(tmp_path / "formal")
    with pytest.raises(S100PProductAdapterError, match="classification"):
        load_verified_development_artifact_contract(
            artifact_manifest_path=formal_manifest, artifact_paths=formal_paths
        )


@pytest.mark.parametrize(
    "mutation, match",
    [
        (lambda payload: payload["artifacts"]["dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"].update(model_role="formal_role"), "provenance"),
        (lambda payload: payload["artifacts"]["dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"].update(source_revision="c50129b5"), "provenance"),
        (lambda payload: payload.update(status="PREPARED_NOT_DEPLOYED"), "classification"),
        (lambda payload: payload["custom_vocabulary"].update(kind="FORMAL"), "custom vocabulary"),
    ],
)
def test_development_artifact_manifest_rejects_role_revision_status_and_vocabulary_drift(tmp_path, monkeypatch, mutation, match):
    manifest, paths = _write_development_artifacts(tmp_path, monkeypatch)
    payload = json.loads(manifest.read_text(encoding="utf-8")); mutation(payload)
    manifest.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(S100PProductAdapterError, match=match):
        load_verified_development_artifact_contract(artifact_manifest_path=manifest, artifact_paths=paths)


def test_development_artifact_manifest_rejects_tamper_and_abi_receipt_drift(tmp_path, monkeypatch):
    manifest, paths = _write_development_artifacts(tmp_path, monkeypatch)
    paths["dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"].write_bytes(b"tampered")
    with pytest.raises(S100PProductAdapterError, match="hash or byte size"):
        load_verified_development_artifact_contract(artifact_manifest_path=manifest, artifact_paths=paths)

    manifest, paths = _write_development_artifacts(tmp_path / "abi", monkeypatch)
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    abi_gate = Path(payload["abi_gate"]["path"])
    receipt = json.loads(abi_gate.read_text(encoding="utf-8")); receipt["outputs"][1]["strides_bytes"] = [67200, 16, 2]
    abi_gate.write_text(json.dumps(receipt), encoding="utf-8")
    payload["abi_gate"]["sha256"] = _sha256(abi_gate)
    payload["abi_gate"]["byte_size"] = abi_gate.stat().st_size
    manifest.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setattr(development, "DEVELOPMENT_ABI_GATE_SHA256", _sha256(abi_gate))
    with pytest.raises(S100PProductAdapterError, match="output ABI"):
        load_verified_development_artifact_contract(artifact_manifest_path=manifest, artifact_paths=paths)


def test_development_artifact_manifest_rejects_symlinked_payload(tmp_path, monkeypatch):
    manifest, paths = _write_development_artifacts(tmp_path, monkeypatch)
    target = paths["edgesam/edgesam_encoder_512.hbm"]
    link = target.with_name("encoder-link.hbm")
    try:
        os.symlink(target, link)
    except OSError as exc:
        pytest.skip(f"symlink creation is unavailable: {exc}")
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    row = payload["artifacts"]["edgesam/edgesam_encoder_512.hbm"]
    row["path"] = str(link.absolute())
    manifest.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(S100PProductAdapterError, match="no symlink components"):
        load_verified_development_artifact_contract(artifact_manifest_path=manifest, artifact_paths=(paths | {"edgesam/edgesam_encoder_512.hbm": link}))


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


def test_pending_dosod_expiry_timer_uses_steady_clock_not_ros_sim_time():
    adapter = Path(__file__).parents[1] / "sanitation_perception" / "s100p_product_adapter.py"
    source = adapter.read_text(encoding="utf-8")
    assert "from rclpy.clock import Clock, ClockType" in source
    assert "clock=Clock(clock_type=ClockType.STEADY_TIME)" in source
