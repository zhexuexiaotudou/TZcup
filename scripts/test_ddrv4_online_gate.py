from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "scripts/perception_oprv3_moving_benchmark.py").read_text(encoding="utf-8")


def test_ddrv4_online_detector_is_hash_bound_to_holdout_selection():
    assert "def load_mmdet_detector(" in SOURCE
    assert 'selection.get("selection_data") != "G7_IN_DOMAIN_HOLDOUT_ONLY"' in SOURCE
    assert 'selection.get("G7_VAL_read_before_selection_freeze") is not False' in SOURCE
    assert 'expected != sha256(checkpoint)' in SOURCE
    assert '"G5_V2_SEALED_FINAL_read": False' in SOURCE
    load_body = SOURCE.split("def load_mmdet_detector", 1)[1].split(
        "def detector_frame_map_mmdet", 1
    )[0]
    assert load_body.index("patch_mmdet_cuda_nms()") < load_body.index(
        "from mmdet.apis import init_detector"
    )


def test_online_product_inference_uses_rgb_without_gt_injection():
    body = SOURCE.split("def detector_frame_map_mmdet", 1)[1].split("def area_frame_map_onnx", 1)[0]
    assert "read_rgb(row)" in body
    assert "inference_detector(model, images)" in body
    assert "truth" not in body
    assert "target" not in body


def test_g6_area_leaf_key_is_mapped_without_retuning():
    assert 'gate_key = "leaf" if class_name == "leaf_pile" else class_name' in SOURCE
    assert "selected.get(class_name) or selected.get(gate_key)" in SOURCE
    assert 'float(record["threshold"])' in SOURCE
    assert 'str(record["morphology"])' in SOURCE
