from __future__ import annotations

import sys
from pathlib import Path

import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning import g4_models  # noqa: E402


def test_input_shape_constants_match_spec() -> None:
    assert tuple(g4_models.DISCOVERY_INPUT_SHAPE) == (1, 3, 480, 640)
    assert tuple(g4_models.CLASSIFIER_INPUT_SHAPE) == (1, 3, 192, 192)
    assert tuple(g4_models.SEGMENTER_INPUT_SHAPE) == (1, 10, 384, 512)
    assert tuple(g4_models.DISCOVERY_STRIDES) == (4, 8)
    assert tuple(g4_models.CLASSIFIER_CLASSES) == (
        "background",
        "plastic_bottle",
        "metal_can",
        "paper_litter",
    )
    assert tuple(g4_models.SEGMENTER_TASKS) == ("leaf", "puddle")
    assert g4_models.DEFAULT_ONNX_OPSET == 17


def test_onnx_contract_functions_exist() -> None:
    assert callable(g4_models.export_fixed_onnx)
    assert callable(g4_models.operator_inventory)
    assert callable(g4_models.torch_onnx_parity)
    assert callable(g4_models.decode_discovery_flat)
    assert callable(g4_models.build_g4_models)


def test_build_models_accepts_from_scratch_control_flag() -> None:
    pytest.importorskip("torch")
    pytest.importorskip("torchvision")
    models = g4_models.build_g4_models(from_scratch_control=True)
    assert set(models) == {"discovery", "classifier", "leaf", "puddle"}


def test_discovery_detector_output_shapes() -> None:
    torch = pytest.importorskip("torch")
    model = g4_models.build_g4_models()["discovery"]
    outputs = model(torch.zeros(1, 3, 480, 640))
    assert outputs["objectness_logits"].shape == (1, 1, 120, 160)
    assert outputs["offset"].shape == (1, 2, 120, 160)
    assert outputs["bbox_size"].shape == (1, 2, 120, 160)


def test_legacy_discovery_control_is_explicit_and_exclusive() -> None:
    pytest.importorskip("torch")
    model = g4_models.build_g4_model("discovery", legacy_fpn_control=True)
    assert model.architecture_role == "legacy_small_fpn_control"
    with pytest.raises(ValueError):
        g4_models.build_g4_model(
            "discovery",
            legacy_fpn_control=True,
            from_scratch_control=True,
        )
    with pytest.raises(ValueError):
        g4_models.build_g4_model("classifier", legacy_fpn_control=True)


def test_classifier_output_shape() -> None:
    torch = pytest.importorskip("torch")
    model = g4_models.build_g4_models()["classifier"]
    logits = model(torch.zeros(1, 3, 192, 192))
    assert logits.shape == (1, len(g4_models.CLASSIFIER_CLASSES))


def test_segmenter_output_shapes() -> None:
    torch = pytest.importorskip("torch")
    models = g4_models.build_g4_models()
    for task in ("leaf", "puddle"):
        models[task].eval()
        outputs = models[task](torch.zeros(1, 10, 384, 512))
    assert outputs["logits"].shape == (1, 1, 384, 512)
    assert outputs["boundary_logits"].shape == (1, 1, 384, 512)


def test_segmenters_can_share_encoder_with_independent_decoders() -> None:
    pytest.importorskip("torch")
    models = g4_models.build_g4_models(shared_encoder=True)
    assert models["leaf"].encoder is models["puddle"].encoder
    assert models["leaf"].decoder is not models["puddle"].decoder
    assert (
        models["leaf"].boundary_head is not models["puddle"].boundary_head
    )


def test_area_candidate_preserves_rgb_stem_and_separates_geometry_branch() -> None:
    pytest.importorskip("torch")
    pytest.importorskip("torchvision")
    model = g4_models.build_g4_model("leaf", from_scratch_control=True)
    assert model.encoder.rgb_stem[0].in_channels == 3
    assert model.encoder.geometry_stem[0].in_channels == 7
    assert model.boundary_head.conv[0].in_channels > 1


def test_model_summary_cards_not_trained() -> None:
    pytest.importorskip("torch")
    cards = g4_models.model_summary()
    assert set(cards) == {"discovery", "classifier", "leaf", "puddle"}
    expected_inputs = {
        "discovery": [1, 3, 480, 640],
        "classifier": [1, 3, 192, 192],
            "leaf": [1, 10, 384, 512],
            "puddle": [1, 10, 384, 512],
    }
    for task, card in cards.items():
        assert card["state"] == "not_trained"
        assert card["parameter_count"] > 0
        assert card["inputs"][0]["shape"] == expected_inputs[task]
        assert card["inputs"][0]["dtype"] == "float32"
        assert card["outputs"][0]["dtype"] == "float32"
    assert cards["classifier"]["outputs"][0]["shape"] == [1, 4]
    assert cards["discovery"]["outputs"][0]["shape"] == [1, 1, 120, 160]
    assert cards["leaf"]["outputs"][0]["shape"] == [1, 1, 384, 512]


def test_onnx_export_inventory_and_parity(tmp_path) -> None:
    torch = pytest.importorskip("torch")
    pytest.importorskip("onnx")
    onnxruntime = pytest.importorskip("onnxruntime")
    model = g4_models.build_g4_models()["discovery"]
    model.eval()
    onnx_path = tmp_path / "discovery.onnx"
    g4_models.export_fixed_onnx(model, None, onnx_path, opset=17)
    assert onnx_path.is_file()
    inventory = g4_models.operator_inventory(onnx_path)
    assert "Conv" in inventory
    assert not any(
        op_type.startswith("custom")
        for op_type in inventory
    )
    session = onnxruntime.InferenceSession(
        str(onnx_path), providers=["CPUExecutionProvider"]
    )
    inputs = torch.randn(1, 3, 480, 640)
    parity = g4_models.torch_onnx_parity(model, session, inputs)
    assert parity["max_absolute_error"] < 1e-3
    assert parity["decoded_agreement"] is True


def test_classifier_onnx_parity_argmax_agreement(tmp_path) -> None:
    torch = pytest.importorskip("torch")
    pytest.importorskip("onnx")
    onnxruntime = pytest.importorskip("onnxruntime")
    model = g4_models.build_g4_models()["classifier"]
    model.eval()
    onnx_path = tmp_path / "classifier.onnx"
    g4_models.export_fixed_onnx(model, None, onnx_path, opset=17)
    session = onnxruntime.InferenceSession(
        str(onnx_path), providers=["CPUExecutionProvider"]
    )
    inputs = torch.randn(1, 3, 192, 192)
    parity = g4_models.torch_onnx_parity(model, session, inputs)
    assert parity["max_absolute_error"] < 1e-3
    assert parity["argmax_agreement"] == 1.0
    assert parity["top1_agreement"] == 1.0
    assert parity["max_probability_error"] < 1e-3


def test_segmenter_onnx_parity_mask_agreement(tmp_path) -> None:
    torch = pytest.importorskip("torch")
    pytest.importorskip("onnx")
    onnxruntime = pytest.importorskip("onnxruntime")
    model = g4_models.build_g4_models()["leaf"]
    model.eval()
    onnx_path = tmp_path / "leaf.onnx"
    g4_models.export_fixed_onnx(model, None, onnx_path, opset=17)
    session = onnxruntime.InferenceSession(
        str(onnx_path), providers=["CPUExecutionProvider"]
    )
    inputs = torch.randn(1, 10, 384, 512)
    parity = g4_models.torch_onnx_parity(model, session, inputs)
    assert parity["max_absolute_error"] < 1e-3
    assert "binary_mask_iou" in parity
    assert "boundary_mask_agreement" in parity
