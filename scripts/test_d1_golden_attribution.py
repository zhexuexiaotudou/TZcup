import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "d1_golden", ROOT / "scripts" / "d1_golden_attribution.py"
)
golden = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(golden)


def test_letterbox_roundtrip_is_exact_for_project_box():
    box = [13.5, 27.25, 91.75, 113.0]
    decoded, error = golden.letterbox_roundtrip(box, 1280, 720)
    assert error <= 1e-9
    assert decoded == box


def test_golden_selection_covers_all_classes_and_prefers_large_targets():
    images = []
    for image_id in range(1, 13):
        category_id = 1 + (image_id - 1) % 3
        images.append(
            {
                "image_id": image_id,
                "annotations": [
                    {"category_id": category_id, "bbox_short_side_px": float(image_id)}
                ],
            }
        )
    selected = golden.select_frames(images)
    assert len(selected) == 10
    assert {item["annotations"][0]["category_id"] for item in selected} == {1, 2, 3}
    assert max(item["image_id"] for item in selected) == 12
