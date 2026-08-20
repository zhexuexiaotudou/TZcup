from pathlib import Path

import infer_trcrv10_proposal as inference


def test_grounding_model_uses_fixed_closed_set_prompt() -> None:
    assert inference.CLASS_PROMPT == ("plastic_bottle", "metal_can", "paper_litter")


def test_inference_does_not_tune_or_read_sealed_data() -> None:
    source = Path(inference.__file__).read_text(encoding="utf-8")
    assert '"threshold_tuning_performed": False' in source
    assert '"G10_DEV_VAL_SEALED_read": False' in source
    assert '"VAL_NEW_read": False' in source
    assert '"G5_V2_read": False' in source


def test_class_output_is_explicitly_non_authoritative() -> None:
    source = Path(inference.__file__).read_text(encoding="utf-8")
    assert "source_class_label" in source
    assert "max_class_score_as_litter_objectness" in source
