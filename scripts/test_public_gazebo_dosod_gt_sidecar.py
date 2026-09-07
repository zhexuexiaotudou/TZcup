from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("sidecar", HERE / "public_gazebo_dosod_gt_sidecar.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(MODULE)


def _identity(**changes):
    value = {
        "generation_nonce": "1" * 32,
        "episode_manifest_sha256": "a" * 64,
        "rgb_source_sha256": "b" * 64,
        "rgb_stamp_ns": 3,
        "semantic_stamp_ns": 3,
        "instance_stamp_ns": 3,
    }
    value.update(changes)
    return value


def _labels():
    semantic = np.zeros((4, 5, 3), dtype=np.uint8)
    # SegmentationCamera semantic labels occupy the final byte; the first two
    # bytes are deliberately non-repeated to reject the obsolete RGB decoder.
    semantic[1:3, 2:5, 0] = 17
    semantic[1:3, 2:5, 1] = 23
    semantic[1:3, 2:5, 2] = 2
    instance = np.zeros((4, 5, 3), dtype=np.uint8)
    instance[1:3, 2:5, 0] = 7
    instance[1:3, 2:5, 2] = 2
    return semantic, instance


def test_sidecar_binds_exact_stamp_nonce_rgb_hash_and_public_boxes(tmp_path: Path):
    semantic, instance = _labels()
    value = MODULE.build_sidecar(identity=_identity(), semantic_rgb=semantic, instance_rgb=instance)
    assert value["boxes"] == [{"instance_id": 7, "class_id": "fallen_leaves", "label": 2, "xyxy": [2, 1, 5, 3]}]
    record = MODULE.write_sidecar(tmp_path / "sidecar.json", value)
    assert record["byte_size"] > 0 and len(record["sha256"]) == 64


@pytest.mark.parametrize(
    "identity, semantic, instance, reason",
    [
        (_identity(instance_stamp_ns=4), *_labels(), "stamp_not_exact"),
        (_identity(), np.dstack([np.zeros((4, 5), dtype=np.uint8), np.ones((4, 5), dtype=np.uint8), np.full((4, 5), 9, dtype=np.uint8)]), _labels()[1], "label_unknown"),
        (_identity(), _labels()[0], np.dstack([np.zeros((4, 5), dtype=np.uint8), np.ones((4, 5), dtype=np.uint8), np.zeros((4, 5), dtype=np.uint8)]), "semantic_mismatch"),
    ],
)
def test_sidecar_rejects_stamp_and_label_drift(identity, semantic, instance, reason):
    with pytest.raises(MODULE.SidecarRejected, match=reason):
        MODULE.build_sidecar(identity=identity, semantic_rgb=semantic, instance_rgb=instance)


def test_sidecar_uses_uint16_instance_count_and_label_count_pair_key():
    semantic, instance = _labels()
    # Same count under two labels must form independent objects; count 300
    # proves the original 24-bit-ID decoder cannot accidentally pass.
    semantic[0, 0, 2] = 1
    instance[0, 0, 0], instance[0, 0, 1], instance[0, 0, 2] = 44, 1, 1
    value = MODULE.build_sidecar(identity=_identity(), semantic_rgb=semantic, instance_rgb=instance)
    assert [(box["label"], box["instance_id"]) for box in value["boxes"]] == [(1, 300), (2, 7)]
