from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_product_packages_do_not_import_reference_models():
    forbidden = ("groundingdino", "grounded_sam", "sam2", "yolo_world", "reference_vision")
    product_roots = (
        ROOT / "starter_ws/src/sanitation_perception/sanitation_perception",
        ROOT / "starter_ws/src/sanitation_spot_cleaning/sanitation_spot_cleaning",
    )
    violations = []
    for product_root in product_roots:
        for path in product_root.glob("*.py"):
            text = path.read_text(encoding="utf-8").lower()
            if any(f"import {name}" in text or f"from {name}" in text for name in forbidden):
                violations.append(str(path.relative_to(ROOT)))
    assert violations == []


def test_reference_dependencies_are_absent_from_product_setup_and_package_xml():
    paths = (
        ROOT / "starter_ws/src/sanitation_perception/setup.py",
        ROOT / "starter_ws/src/sanitation_perception/package.xml",
        ROOT / "docker/Dockerfile.perception-product",
    )
    combined = "\n".join(path.read_text(encoding="utf-8").lower() for path in paths)
    assert "groundingdino" not in combined
    assert "yolo-world" not in combined
    assert "facebookresearch/sam2" not in combined
