import prepare_trcrv10_classifier_crops as crops


def test_required_positive_views_are_complete() -> None:
    assert crops.POSITIVE_VIEWS == ("gt", "detector_jitter", "proposal", "partial", "context")


def test_geometry_variants_are_distinct() -> None:
    box = [10, 20, 30, 50]
    assert crops.jitter(box) != box
    assert crops.partial(box)[2] < box[2]
    expanded = crops.expand(box, 1.6)
    assert expanded[0] < box[0] and expanded[3] > box[3]


def test_four_class_contract_includes_background() -> None:
    source = open(crops.__file__, encoding="utf-8").read()
    assert "background_or_unknown" in source
    assert "classifier crops accept G10 TRAIN only" in source
    assert '"HOLDOUT_read": False' in source
