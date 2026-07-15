#!/usr/bin/env python3
import json
import sys
from pathlib import Path

import yaml


def main():
    root = Path(sys.argv[1])
    radius_cases = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in sorted((root / "radius_grid").glob("*.json"))
    ]
    separation_cases = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in sorted((root / "separation_grid").glob("*.json"))
    ]
    if len(radius_cases) < 5 or len(separation_cases) < 5:
        raise RuntimeError("parameter grid is incomplete")
    selected_radius = min(radius_cases, key=lambda case: case["body_distance_error_pct"])
    selected_separation = min(
        separation_cases,
        key=lambda case: case["mean_body_yaw_error_deg"]
        + 0.5 * case["left_right_asymmetry_deg"],
    )
    selected = {
        "physical_wheel_radius": 0.14,
        "physical_track_width": 0.80,
        "drive_wheel_radius": selected_radius["drive_wheel_radius"],
        "drive_wheel_separation": selected_separation["drive_wheel_separation"],
        "wheel_mu_longitudinal": 1.0,
        "wheel_mu_lateral": 1.0,
        "slip_compliance_longitudinal": 0.0,
        "slip_compliance_lateral": 0.0,
        "enable_wheel_slip": False,
    }
    report = {
        "schema_version": 1,
        "fit_order": ["drive_wheel_radius", "drive_wheel_separation"],
        "baseline": {"drive_wheel_radius": 0.14, "drive_wheel_separation": 0.80},
        "radius_grid": radius_cases,
        "separation_grid": separation_cases,
        "selection_objective": {
            "radius": "minimum actual 5 m body distance error",
            "separation": "minimum mean +/-360 body yaw error plus 0.5 * left-right asymmetry",
        },
        "selected": selected,
        "selected_radius_case": selected_radius,
        "selected_separation_case": selected_separation,
        "grid_complete": len(radius_cases) >= 5 and len(separation_cases) >= 5,
    }
    (root / "wheel_parameter_fit.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (root / "selected_vehicle_dynamics.yaml").write_text(
        yaml.safe_dump({"vehicle_dynamics": selected}, sort_keys=False),
        encoding="utf-8",
    )
    (root / "friction_slip_scan.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "executed": False,
                "eligible_after_selected_drive_validation": True,
                "reason": "Per Stage4S ordering, friction/WheelSlip is scanned only if body command tracking remains invalid after drive radius/separation fitting.",
                "grid": [],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(selected["drive_wheel_radius"], selected["drive_wheel_separation"])


if __name__ == "__main__":
    main()
