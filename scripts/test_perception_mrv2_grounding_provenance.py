#!/usr/bin/env python3

from perception_mrv2_grounding_provenance import (
    OFFICIAL_CHECKPOINT_BYTES,
    OFFICIAL_CHECKPOINT_NAME,
    OFFICIAL_CHECKPOINT_URL,
)


def test_official_grounding_dino_release_contract_is_pinned():
    assert OFFICIAL_CHECKPOINT_NAME == "groundingdino_swint_ogc.pth"
    assert OFFICIAL_CHECKPOINT_BYTES == 693_997_677
    assert OFFICIAL_CHECKPOINT_URL.startswith(
        "https://github.com/IDEA-Research/GroundingDINO/releases/download/"
    )
