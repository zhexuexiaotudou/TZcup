from real_rgbd_capture import validate_placement_protocol


def test_independent_placement_protocol_accepts_auditable_measurement():
    report = validate_placement_protocol(
        {"schema_version": 1, "coordinate_frame": "map", "placements": [
            {"frame_id": "f1", "object_id": "bottle-1",
             "class_id": "plastic_bottle", "position_map_m": [1.0, 2.0, 0.0],
             "measurement_method": "fiducial", "uncertainty_m": 0.01,
             "independent_of_perception": True}
        ]}
    )
    assert report["independent_placement_gate_pass"] is True


def test_self_generated_or_imprecise_truth_is_rejected():
    report = validate_placement_protocol(
        {"schema_version": 1, "coordinate_frame": "map", "placements": [
            {"frame_id": "f1", "object_id": "paper-1",
             "class_id": "paper_litter", "position_map_m": [1.0, 2.0, 0.0],
             "measurement_method": "model_prediction", "uncertainty_m": 0.2,
             "independent_of_perception": False}
        ]}
    )
    assert report["independent_placement_gate_pass"] is False
    assert len(report["errors"]) == 3
