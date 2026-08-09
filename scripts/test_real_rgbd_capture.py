import json

import numpy as np

from real_rgbd_capture import (
    apply_privacy_regions,
    build_parser,
    create_placement_command,
    validate_placement_protocol,
)


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


def test_privacy_filter_changes_only_declared_region():
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    image[16:48:2, 16:48] = 255
    filtered = apply_privacy_regions(image, [[16, 16, 48, 48]])
    assert np.array_equal(filtered[:16], image[:16])
    assert not np.array_equal(filtered[16:48, 16:48], image[16:48, 16:48])


def test_create_placement_records_independent_truth(tmp_path):
    target = tmp_path / "placements.json"
    args = build_parser().parse_args(
        [
            "create-placement", "--output", str(target), "--frame-id", "f1",
            "--object-id", "can-1", "--class-id", "metal_can", "--x", "1",
            "--y", "2", "--z", "0", "--measurement-method", "fiducial",
            "--uncertainty-m", "0.01",
        ]
    )
    assert create_placement_command(args) == 0
    payload = json.loads(target.read_text(encoding="utf-8"))
    assert payload["placements"][0]["independent_of_perception"] is True
