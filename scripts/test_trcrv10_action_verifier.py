from trcrv10_action_verifier import VerifierConfig, verify


def valid(**updates):
    row = {
        "tight_class": "metal_can", "context_class": "metal_can",
        "tight_probability": .99, "context_probability": .98,
        "depth_valid_fraction": .95, "map_covariance_m2": .01,
        "persistence_frames": 4, "bbox_short_side_px": 80,
        "physical_impossibility": False,
    }
    row.update(updates)
    return row


def test_valid_observation_accepts() -> None:
    assert verify(valid(), VerifierConfig())["decision"] == "ACCEPT"


def test_all_protocol_hard_vetoes_block_accept() -> None:
    cases = (
        {"tight_class": "background_or_unknown"},
        {"context_class": "paper_litter"},
        {"tight_probability": .80},
        {"depth_valid_fraction": .1},
        {"map_covariance_m2": .5},
        {"persistence_frames": 1},
        {"bbox_short_side_px": 20},
        {"physical_impossibility": True},
    )
    assert all(verify(valid(**case), VerifierConfig())["decision"] != "ACCEPT" for case in cases)


def test_low_height_alone_does_not_veto_paper_or_crushed_can() -> None:
    for class_id in ("paper_litter", "metal_can"):
        row = valid(tight_class=class_id, context_class=class_id, physical_height_m=0.0)
        assert verify(row, VerifierConfig())["decision"] == "ACCEPT"
