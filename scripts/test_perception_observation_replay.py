"""Synthetic boundary tests, not recognition or real-runtime evidence."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
from replay_product_observation_capture import replay_capture
from sanitation_perception.product_intermediate_capture import ProductIntermediateCapture
from sanitation_perception.product_projection import CameraIntrinsics, PublicGrid, project_rgbd_observation


def capture(root, *, invalid_depth=False, cube=False):
    depth = np.full((8, 8), np.nan if invalid_depth else 2, dtype=np.float32)
    transform = np.eye(4)
    transform[2, 3] = -2
    diagnostics = {}
    project_rgbd_observation(depth, CameraIntrinsics(4, 4, 3.5, 3.5), transform,
        PublicGrid(20, 20, .25, -2.5, -2.5), boxes_xyxy=np.empty((0, 4)),
        class_ids=[], masks=[], confidences=[], sample_stride=1, diagnostics_out=diagnostics)
    ProductIntermediateCapture(root).capture_frame(sensor="front", rgb_stamp_s=1., depth_stamp_s=.98,
        rgb=np.zeros((8, 8, 3), dtype=np.uint8), depth=depth,
        camera_info={"frame_id": "camera_optical", "stamp_s": 1., "width": 8, "height": 8,
            "k": [4., 0., 3.5, 0., 4., 3.5, 0., 0., 1.], "d": []},
        map_from_camera=transform, detections=([{"detection_index": 0, "class_id": "litter_cube", "confidence": .8, "xyxy": [2., 2., 4., 4.]}] if cube else []), prompt_decisions=[], prompt_detection_indices=np.array([], dtype=int),
        prompt_masks=[], prompt_qualities=[], projection_diagnostics=diagnostics,
        map_occupancy=np.zeros((20, 20), dtype=np.int8),
        map_metadata={"frame_id": "map", "stamp_s": .9, "width": 20, "height": 20,
            "resolution": .25, "origin_x": -2.5, "origin_y": -2.5})
    return root / "frames/frame-0000"


def rewrite_metadata(frame, **updates):
    path = frame / "metadata.json"
    meta = json.loads(path.read_text())
    meta.update(updates)
    path.write_text(json.dumps(meta), encoding="utf-8")
    manifest_path = frame / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["files"]["metadata.json"] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")


def test_capture_roundtrip_is_projection_only(tmp_path):
    capture(tmp_path)
    report = replay_capture(tmp_path)
    assert report["status"] == "replayed"
    assert report["frames"][0]["changed_raster_cells"] == 0
    assert report["frames"][0]["observed_cells"] > 0
    assert not report["runtime_verified"]
    assert not report["recognition_accuracy_verified"]
    assert not report["capture_authenticity_verified"]
    assert not report["model_inference_executed"]


def test_missing_capture_is_not_ready(tmp_path):
    assert replay_capture(tmp_path)["reason"] == "no_product_capture_frames"


def test_corrupt_payload_is_not_ready(tmp_path):
    frame = capture(tmp_path)
    with (frame / "arrays.npz").open("ab") as stream:
        stream.write(b"corrupt")
    report = replay_capture(tmp_path)
    assert report["status"] == "not_ready"
    assert "hash mismatch" in report["frames"][0]["reason"]


@pytest.mark.parametrize("updates", [
    {"depth_stamp_s": 0.}, {"rgb_stamp_s": float("nan")},
    {"map_content_sha256": "../outside"}, {"map_manifest_sha256": "0" * 64},
    {"camera_info": {"width": 2, "height": 2}},
])
def test_bad_metadata_is_rejected(tmp_path, updates):
    frame = capture(tmp_path)
    rewrite_metadata(frame, **updates)
    assert replay_capture(tmp_path)["status"] == "not_ready"


def test_all_invalid_depth_is_not_ready(tmp_path):
    capture(tmp_path, invalid_depth=True)
    report = replay_capture(tmp_path)
    assert report["status"] == "not_ready"
    assert report["frames"][0]["reason"] == "no_usable_projected_observation"


def test_corrupt_map_is_not_ready(tmp_path):
    capture(tmp_path)
    path = next((tmp_path / "maps").glob("*/arrays.npz"))
    path.write_bytes(b"not-an-array")
    assert replay_capture(tmp_path)["status"] == "not_ready"



def test_recorded_cube_detection_reaches_map_projection(tmp_path):
    capture(tmp_path, cube=True)
    report = replay_capture(tmp_path)
    assert report["status"] == "replayed"
    targets = report["frames"][0]["projected_targets"]
    assert len(targets) == 1
    assert targets[0]["detection_index"] == 0
    assert targets[0]["xyz"][2] == pytest.approx(0.)


def test_recorded_prompt_misalignment_fails_closed(tmp_path):
    frame = capture(tmp_path)
    rewrite_metadata(frame, detections=[{"detection_index": 0, "class_id": "puddle", "confidence": .8, "xyxy": [2., 2., 4., 4.]}])
    report = replay_capture(tmp_path)
    assert report["status"] == "not_ready"
    assert "prompt selection differs" in report["frames"][0]["reason"]
