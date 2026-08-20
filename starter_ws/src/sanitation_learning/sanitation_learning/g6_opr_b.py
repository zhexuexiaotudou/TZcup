"""OPR-B two-stage P2/small-anchor detector contract."""

from __future__ import annotations


ANCHOR_SIZES = (
    (8, 12, 16),
    (16, 24, 32),
    (32, 48, 64),
    (64, 96, 128),
    (128, 192, 256),
)
ASPECT_RATIOS = ((0.5, 1.0, 2.0),) * 5


def build_opr_b(*, weights_required: bool = True):
    """Build official Faster R-CNN v2 with explicit P2 small anchors."""
    import torchvision
    from torchvision.models.detection.anchor_utils import AnchorGenerator
    from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
    from torchvision.models.detection.rpn import RPNHead

    weights = (
        torchvision.models.detection.FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
        if weights_required
        else None
    )
    model = torchvision.models.detection.fasterrcnn_resnet50_fpn_v2(
        weights=weights,
        weights_backbone=None,
        min_size=480,
        max_size=640,
        box_score_thresh=0.01,
        box_detections_per_img=100,
        rpn_pre_nms_top_n_train=2000,
        rpn_post_nms_top_n_train=1000,
        rpn_pre_nms_top_n_test=1000,
        rpn_post_nms_top_n_test=300,
    )
    anchors = AnchorGenerator(sizes=ANCHOR_SIZES, aspect_ratios=ASPECT_RATIOS)
    model.rpn.anchor_generator = anchors
    model.rpn.head = RPNHead(
        model.backbone.out_channels,
        anchors.num_anchors_per_location()[0],
        conv_depth=2,
    )
    features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(features, 4)
    model.model_id = "opr_b_fasterrcnn_r50_fpn_v2_p2_small_anchors_v1"
    model.architecture_role = "OPR-B_two_stage_not_frozen"
    model.opr_b_provenance = {
        "source": "torchvision",
        "torchvision_version": torchvision.__version__,
        "weights": str(weights),
    }
    return model


__all__ = ["ANCHOR_SIZES", "ASPECT_RATIOS", "build_opr_b"]
