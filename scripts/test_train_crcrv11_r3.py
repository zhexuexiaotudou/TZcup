from pathlib import Path


def test_r3_source_enforces_shared_backbone_and_complementary_evidence():
    source = Path(__file__).with_name("train_crcrv11_r3.py").read_text(encoding="utf-8")
    assert 'evidence.get("R3_COMPLEMENTARY_EVIDENCE") is not True' in source
    assert "self.features, self.avgpool = base.features, base.avgpool" in source
    assert "torch.cat((self.encode(tight), self.encode(context)), dim=1)" in source
    assert "dual_view_convnext_tiny" in source


def test_r3_source_keeps_sealed_sets_unread():
    source = Path(__file__).with_name("train_crcrv11_r3.py").read_text(encoding="utf-8")
    assert '"G10_DEV_VAL_SEALED_read": False' in source
    assert '"VAL_NEW_read": False' in source
    assert '"G5_V2_read": False' in source
