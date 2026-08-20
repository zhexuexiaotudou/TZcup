from pathlib import Path
import tempfile

import trcrv10_assets as assets


def test_bottle_has_transparency_neck_and_cap_without_scale_change() -> None:
    text = assets.model_sdf("bottle_1", "plastic_bottle", "cylinder", (.035, .17), "pet_plastic_glossy", "a.png")
    assert "<transparency>" in text
    assert "bottle_neck" in text
    assert "bottle_cap" in text
    assert "<radius>0.0350</radius>" in text


def test_can_has_metal_response_and_rims() -> None:
    text = assets.model_sdf("can_1", "metal_can", "cylinder", (.04, .10), "aluminum_can_glossy", "a.png")
    assert "can_top_rim" in text
    assert "can_bottom_rim" in text
    assert "<metalness>0.850</metalness>" in text


def test_paper_has_irregular_edge_and_creases() -> None:
    text = assets.model_sdf("paper_1", "paper_litter", "box", (.20, .13, .008), "paper_matte", "a.png")
    assert "<polyline>" in text
    assert text.count("<point>") == 8
    assert "paper_crease_a" in text
    assert "paper_crease_b" in text


def test_crumpled_paper_keeps_volume_and_irregular_surface() -> None:
    text = assets.model_sdf("paper_wad", "paper_litter", "ellipsoid", (.07, .07, .045), "paper_matte", "a.png")
    assert "paper_crumpled_core" in text
    assert "<ellipsoid>" in text
    assert "paper_irregular_sheet" in text


def test_generated_target_sdfs_pass_structural_audit() -> None:
    cases = (
        ("plastic_bottle", "cylinder", (.035, .17), "pet_plastic_glossy"),
        ("metal_can", "cylinder", (.04, .10), "aluminum_can_glossy"),
        ("paper_litter", "box", (.20, .13, .008), "paper_matte"),
    )
    with tempfile.TemporaryDirectory() as root:
        for class_id, kind, values, family in cases:
            path = Path(root) / f"{class_id}.sdf"
            path.write_text(assets.model_sdf(class_id, class_id, kind, values, family, "a.png"), encoding="utf-8")
            assert assets.audit_sdf(path, class_id)["pass"]


def test_domain_contract_forbids_visual_class_markers() -> None:
    source = Path(assets.__file__).read_text(encoding="utf-8")
    for forbidden in ("no fixed class color", "no class text", "no QR code", "no category marker", "no physical scale inflation"):
        assert forbidden in source
