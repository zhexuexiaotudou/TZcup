from pathlib import Path
import sys

import numpy as np
import pytest


torch = pytest.importorskip("torch")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g6_area_recovery import (  # noqa: E402
    AREA_SIZE,
    AreaTemporalFilter,
    G6BoundaryAwareAreaNet,
    g6_area_loss,
    negative_area_mask,
    physical_component_filter,
    preprocess_g6_area,
)


def test_preprocess_and_boundary_aware_model_contract() -> None:
    rgb = np.full((480, 640, 3), (104, 108, 110), dtype=np.uint8)
    depth = np.full((480, 640), 4500, dtype=np.uint16)
    features = preprocess_g6_area(rgb, depth)
    assert features.shape == (10, AREA_SIZE[1], AREA_SIZE[0])
    assert np.isfinite(features).all()
    model = G6BoundaryAwareAreaNet(base_channels=4).eval()
    with torch.no_grad():
        output = model(torch.from_numpy(features[None]))
    assert output["semantic_logits"].shape == (1, 2, AREA_SIZE[1], AREA_SIZE[0])
    assert output["boundary_logits"].shape == (1, 2, AREA_SIZE[1], AREA_SIZE[0])
    assert model.semantic_head is not model.boundary_head


def test_negative_taxonomy_and_loss_contract() -> None:
    rgb = np.zeros((24, 32, 3), dtype=np.uint8)
    rgb[4:12, 8:20] = (55, 82, 94)
    mask = negative_area_mask(rgb, "wet_asphalt_not_puddle")
    assert int(mask.sum()) == 96
    output = {
        "semantic_logits": torch.zeros((1, 2, 8, 8)),
        "boundary_logits": torch.zeros((1, 2, 8, 8)),
    }
    losses = g6_area_loss(
        output,
        torch.zeros((1, 2, 8, 8)),
        torch.zeros((1, 2, 8, 8)),
        torch.ones((1, 1, 8, 8)),
    )
    assert torch.isfinite(losses["total"])
    assert losses["negative_penalty"] > 0


def test_physical_area_and_registered_temporal_filters() -> None:
    mask = np.zeros((32, 32), dtype=np.uint8)
    mask[2:4, 2:4] = 1
    mask[10:24, 10:24] = 1
    depth = np.full(mask.shape, 3.0, dtype=np.float32)
    filtered = physical_component_filter(
        mask,
        depth,
        fx=300.0,
        fy=300.0,
        minimum_area_m2=0.005,
    )
    assert not filtered[2:4, 2:4].any()
    assert filtered[10:24, 10:24].all()

    temporal = AreaTemporalFilter(window=3, minimum_hits=2)
    assert not temporal.update(filtered).any()
    assert np.array_equal(temporal.update(filtered), filtered)
    temporal.reset()
    assert not temporal.update(filtered).any()
