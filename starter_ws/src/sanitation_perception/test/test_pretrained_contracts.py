import numpy as np

from sanitation_perception.pretrained_contracts import (
    decode_material_classifier,
    decode_yolo_detect,
    fuse_detector_classifier,
)


def test_yolo_decode_supports_transposed_layout_and_graph_external_nms():
    output = np.zeros((1, 6, 3), dtype=np.float32)
    output[0, :, 0] = [100, 100, 20, 20, 0.9, 0.1]
    output[0, :, 1] = [101, 101, 20, 20, 0.8, 0.2]
    output[0, :, 2] = [300, 300, 30, 30, 0.1, 0.95]
    detections = decode_yolo_detect(
        output,
        class_order=["plastic_bottle", "drinks_can"],
        class_mapping={"plastic_bottle": "plastic_bottle", "drinks_can": "metal_can"},
        score_threshold=0.5,
        nms_iou_threshold=0.5,
        input_size=(640, 640),
    )
    assert [item.product_class for item in detections] == ["metal_can", "plastic_bottle"]


def test_yolov5_objectness_is_explicit_in_the_contract():
    output = np.array([[[100, 100, 20, 20, 0.5, 0.9]]], dtype=np.float32)
    assert decode_yolo_detect(
        output,
        class_order=["bottle"],
        class_mapping={"bottle": "plastic_bottle"},
        score_threshold=0.5,
        nms_iou_threshold=0.5,
        input_size=(640, 640),
        has_objectness=True,
    ) == []
    assert len(
        decode_yolo_detect(
            output,
            class_order=["bottle"],
            class_mapping={"bottle": "plastic_bottle"},
            score_threshold=0.4,
            nms_iou_threshold=0.5,
            input_size=(640, 640),
            has_objectness=True,
        )
    ) == 1


def test_classifier_unknown_rejection_and_fusion_never_confirms():
    classification = decode_material_classifier(
        np.array([[0.0, 5.0, 0.0]], dtype=np.float32),
        class_order=["trash", "plastic", "glass"],
        class_mapping={"plastic": "plastic_bottle"},
        minimum_confidence=0.8,
    )
    detection = decode_yolo_detect(
        np.array([[[100, 100, 20, 20, 0.99]]], dtype=np.float32),
        class_order=["plastic_bottle"],
        class_mapping={"plastic_bottle": "plastic_bottle"},
        score_threshold=0.5,
        nms_iou_threshold=0.5,
        input_size=(640, 640),
    )[0]
    fused = fuse_detector_classifier(
        detection,
        classification,
        stable_track=True,
        valid_depth=True,
        reobserve_count=0,
    )
    assert fused["status"] == "READY_FOR_ACTION_VERIFIER"
    assert fused["confirmed"] is False
    assert fused["clean_now"] is False


def test_fusion_requires_depth_and_caps_reobservation():
    detection = decode_yolo_detect(
        np.array([[[100, 100, 20, 20, 0.99]]], dtype=np.float32),
        class_order=["paper_waste"],
        class_mapping={"paper_waste": "paper_litter"},
        score_threshold=0.5,
        nms_iou_threshold=0.5,
        input_size=(640, 640),
    )[0]
    classification = {"accepted": True, "product_evidence": "paper_litter"}
    assert fuse_detector_classifier(
        detection, classification, stable_track=True, valid_depth=False, reobserve_count=1
    )["status"] == "OBSERVE_AGAIN"
    assert fuse_detector_classifier(
        detection, classification, stable_track=True, valid_depth=False, reobserve_count=2
    )["status"] == "DEFER"
