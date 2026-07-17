from sanitation_ground_truth.visibility import DiscObject, visible_targets


def test_fully_occluded_target_is_not_published():
    objects = [
        DiscObject("opaque_obstacle", 1.0, 0.0, 0.4, is_target=False),
        DiscObject("hidden_target", 2.0, 0.0, 0.2, is_target=True),
        DiscObject("visible_target", 2.0, 1.5, 0.2, is_target=True),
    ]
    visible = visible_targets((0.0, 0.0), objects)
    assert "hidden_target" not in visible
    assert visible["visible_target"]["visibility"] > 0.9
