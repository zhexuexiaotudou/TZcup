import json
from pathlib import Path

import numpy as np
import pytest

from sanitation_perception.dosod_ros_adapter import DosodOnnxDetector
from sanitation_perception.edgesam_ros_adapter import EdgeSamOnnxSegmenter
from sanitation_perception.product_projection import (
    CameraIntrinsics,
    PublicGrid,
    project_rgbd_observation,
)
from sanitation_perception.pc_open_vocab_adapter import (
    _ground_dirt_prompt_indices,
    _projection_masks,
    serialize_wrist_grasp_recheck,
)


PACKAGE = Path(__file__).resolve().parents[1]


class _Io:
    def __init__(self, name):
        self.name = name


class _DosodSession:
    def get_inputs(self):
        return [_Io("images")]

    def get_outputs(self):
        return [_Io("scores"), _Io("boxes")]

    def run(self, output_names, feed):
        assert output_names == ["scores", "boxes"]
        assert feed["images"].shape == (1, 3, 640, 640)
        scores = np.zeros((1, 3, 4), dtype=np.float32)
        scores[0, 0, 0] = 0.90
        scores[0, 1, 0] = 0.80
        scores[0, 2, 3] = 0.75
        boxes = np.asarray([[[100, 100, 300, 300], [110, 110, 290, 290], [320, 320, 500, 500]]], dtype=np.float32)
        return scores, boxes


class _CoordinateRoundTripSession:
    def get_inputs(self):
        return [_Io("images")]

    def get_outputs(self):
        return [_Io("scores"), _Io("boxes")]

    def run(self, output_names, feed):
        assert output_names == ["scores", "boxes"]
        assert feed["images"].shape == (1, 3, 640, 640)
        scores = np.zeros((1, 1, 4), dtype=np.float32)
        scores[0, 0, 0] = 0.90
        # Original 848x480 box (106, 32, 742, 448), after symmetric square
        # padding by 184 px and resize by 640/848.
        scale = 640.0 / 848.0
        boxes = np.asarray(
            [[[106.0 * scale, (32.0 + 184.0) * scale,
               742.0 * scale, (448.0 + 184.0) * scale]]],
            dtype=np.float32,
        )
        return scores, boxes


class _EncoderSession:
    def get_inputs(self):
        return [_Io("image")]

    def run(self, output_names, feed):
        assert feed["image"].shape == (1, 3, 1024, 1024)
        return [np.zeros((1, 256, 64, 64), dtype=np.float32)]


class _DecoderSession:
    def get_inputs(self):
        return [_Io("image_embeddings"), _Io("point_coords"), _Io("point_labels")]

    def run(self, output_names, feed):
        assert feed["point_labels"].tolist() == [[2.0, 3.0]]
        scores = np.asarray([[0.1, 0.9, 0.2, 0.3]], dtype=np.float32)
        masks = np.zeros((1, 4, 256, 256), dtype=np.float32)
        masks[0, 1, 40:180, 30:200] = 1.0
        return scores, masks


def test_dosod_adapter_runs_fixed_class_nms_and_restores_image_coordinates():
    detector = DosodOnnxDetector(session=_DosodSession(), score_threshold=0.5)
    results = detector.infer(np.zeros((480, 640, 3), dtype=np.uint8))
    assert [result.class_id for result in results] == ["litter_cube", "puddle"]
    assert results[0].confidence == np.float32(0.9)
    assert all(np.isfinite(result.xyxy).all() for result in results)


def test_dosod_adapter_exactly_round_trips_square_padding_coordinates():
    detector = DosodOnnxDetector(
        session=_CoordinateRoundTripSession(), score_threshold=0.5
    )
    results = detector.infer(np.zeros((480, 848, 3), dtype=np.uint8))
    assert len(results) == 1
    assert results[0].class_id == "litter_cube"
    assert results[0].xyxy == pytest.approx((106.0, 32.0, 742.0, 448.0), abs=1e-4)


def test_dosod_adapter_supports_measured_class_specific_thresholds():
    detector = DosodOnnxDetector(
        session=_DosodSession(),
        score_threshold=0.85,
        class_score_thresholds={"puddle": 0.70},
    )
    results = detector.infer(np.zeros((480, 640, 3), dtype=np.uint8))
    assert [result.class_id for result in results] == ["litter_cube", "puddle"]


def test_edgesam_adapter_uses_box_prompts_and_best_mask():
    segmenter = EdgeSamOnnxSegmenter(
        encoder_session=_EncoderSession(), decoder_session=_DecoderSession()
    )
    masks, qualities = segmenter.segment_boxes(
        np.zeros((480, 640, 3), dtype=np.uint8), np.asarray([[10, 20, 200, 300]])
    )
    assert len(masks) == 1 and masks[0].shape == (480, 640)
    assert masks[0].any()
    assert qualities[0] == np.float32(0.9)


def test_public_map_projection_emits_trinary_dirt_and_litter_target():
    depth = np.full((8, 8), 2.0, dtype=np.float32)
    transform = np.eye(4)
    transform[2, 3] = -2.0
    masks = [np.ones((8, 8), dtype=bool), np.ones((8, 8), dtype=bool)]
    raster, targets = project_rgbd_observation(
        depth,
        CameraIntrinsics(4.0, 4.0, 3.5, 3.5),
        transform,
        PublicGrid(20, 20, 0.25, -2.5, -2.5),
        boxes_xyxy=np.asarray([[1, 1, 6, 6], [0, 0, 8, 8]], dtype=np.float32),
        class_ids=["litter_cube", "puddle"],
        masks=masks,
        confidences=[0.9, 0.8],
        sample_stride=1,
    )
    assert set(np.unique(raster)) <= {0, 1, 204}
    assert 204 in raster
    assert len(targets) == 1
    assert targets[0].detection_index == 0
    assert np.isfinite(targets[0].xyz).all()


def test_edgesam_is_reserved_for_ground_dirt_and_cube_uses_box_mask():
    class_ids = ["litter_cube", "puddle", "dust_or_soil"]
    indices = _ground_dirt_prompt_indices(class_ids)
    assert indices.tolist() == [1, 2]
    boxes = np.asarray([[1, 2, 5, 7], [0, 0, 3, 3], [4, 4, 8, 8]], dtype=np.float32)
    dirt_masks = [np.eye(10, dtype=bool), np.fliplr(np.eye(10, dtype=bool))]
    masks, qualities = _projection_masks(
        (10, 10), boxes, class_ids, dirt_masks, [0.8, 0.7]
    )
    assert masks[0][2:7, 1:5].all()
    assert int(masks[0].sum()) == 20
    assert np.array_equal(masks[1], dirt_masks[0])
    assert np.array_equal(masks[2], dirt_masks[1])
    assert qualities == [1.0, 0.8, 0.7]


def test_edgesam_prompt_count_is_bounded_to_three_highest_score_boxes_per_dirt_class():
    # Input order is the detector's descending-confidence order.
    class_ids = [
        "puddle",
        "litter_cube",
        "puddle",
        "fallen_leaves",
        "dust_or_soil",
        "fallen_leaves",
    ]
    assert _ground_dirt_prompt_indices(class_ids).tolist() == [0, 2, 3, 4, 5]


def test_edgesam_prompt_selection_rejects_near_full_frame_false_box():
    class_ids = ["fallen_leaves", "fallen_leaves", "puddle"]
    boxes = np.asarray(
        [[0, 0, 100, 100], [10, 10, 30, 30], [40, 40, 70, 70]],
        dtype=np.float32,
    )
    assert _ground_dirt_prompt_indices(class_ids, boxes, (100, 100)).tolist() == [1, 2]


def test_ros_product_adapter_lists_every_formal_camera_and_no_evaluator_subscription():
    source = (PACKAGE / "sanitation_perception" / "pc_open_vocab_adapter.py").read_text(
        encoding="utf-8"
    )
    for topic in (
        "/sensors/front_rgbd/depth/image_rect_raw/image",
        "/sensors/wrist_rgbd/depth/image_rect_raw/image",
        "/sensors/rear_left_fisheye/image_raw",
        "/sensors/rear_right_fisheye/image_raw",
        "/map",
    ):
        assert topic in source
    assert "create_subscription" in source
    assert '"/ground_truth/' not in source
    assert '"/evaluator/' not in source
    assert "/perception/ground_dirt/masks" in source
    assert "/perception/garbage/targets" in source
    assert "TargetTracker" in source
    assert "self.target_tracker.update" in source
    assert "target.uuid = tracked.uuid" in source
    assert "uuid.uuid5" not in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert 'declare_parameter("tf_max_age_s", 0.75)' in source
    assert 'declare_parameter("score_threshold", 0.005)' in source
    assert 'declare_parameter("fallen_leaves_score_threshold", 0.0025)' in source
    assert 'declare_parameter("dust_or_soil_score_threshold", 0.002)' in source
    assert '"stale_map_tf_rejected"' in source
    assert '"map", image_message.header.frame_id, Time()' in source
    assert "boxes[dirt_indices]" in source
    assert "Publish DOSOD immediately" in source
    assert '"dosod_product_ok"' in source
    assert "self.create_timer(" in source
    assert '"alive"' in source
    assert "diagnostic_qos" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "MutuallyExclusiveCallbackGroup" in source
    assert "callback_group=self.diagnostic_callback_group" in source
    assert "self.inference_callback_group = MutuallyExclusiveCallbackGroup()" in source
    assert "self.cache_callback_group = MutuallyExclusiveCallbackGroup()" in source
    assert source.count("callback_group=self.inference_callback_group") == 2
    assert source.count("callback_group=self.cache_callback_group") == 3
    assert "MultiThreadedExecutor(num_threads=3)" in source
    assert '"/perception/wrist/grasp_recheck"' in source
    rgbd_start = source.index("def _on_rgbd")
    assert source.index("self.detection_publisher.publish(product)", rgbd_start) < source.index(
        "self.segmenter.segment_boxes", rgbd_start
    )


def test_wrist_recheck_forwards_truth_free_3d_geometry_with_unknown_material():
    payload = json.loads(
        serialize_wrist_grasp_recheck(
            target_id="track-1",
            frame_id="map",
            pose=(1.0, -0.2, 0.015, 0.0, 0.0, 0.0, 1.0),
            size_m=(0.03, 0.03, 0.03),
            confidence=0.92,
        )
    )
    assert payload["schema_version"] == 2
    assert payload["pose"]["z_m"] == 0.015
    assert payload["material"] == "unknown"
    assert payload["truth_used"] is False


def test_wrist_recheck_rejects_low_confidence_or_invalid_geometry():
    with pytest.raises(ValueError, match="confidence"):
        serialize_wrist_grasp_recheck(
            target_id="track-2",
            frame_id="map",
            pose=(1.0, -0.2, 0.015, 0.0, 0.0, 0.0, 1.0),
            size_m=(0.03, 0.03, 0.03),
            confidence=0.49,
        )
