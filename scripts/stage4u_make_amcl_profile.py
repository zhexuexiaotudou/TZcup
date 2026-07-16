#!/usr/bin/env python3
"""Create a traceable AMCL sensitivity profile without editing the baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import yaml


PROFILES = {
    "baseline_360x10": {
        "max_beams": 180,
        "laser_z_hit": 0.30,
        "laser_z_rand": 0.70,
        "laser_sigma_hit": 0.30,
        "update_min_a": 0.02,
        "update_min_d": 0.02,
    },
    "precision_720x20": {
        "max_beams": 360,
        "laser_z_hit": 0.85,
        "laser_z_rand": 0.15,
        "laser_sigma_hit": 0.08,
        "update_min_a": 0.01,
        "update_min_d": 0.01,
    },
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument("--profile", required=True, choices=sorted(PROFILES))
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    document = yaml.safe_load(args.base.read_text(encoding="utf-8"))
    overrides = PROFILES[args.profile]
    document["amcl"]["ros__parameters"].update(overrides)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    report = {
        "schema_version": 1,
        "profile": args.profile,
        "base": str(args.base),
        "base_sha256": hashlib.sha256(args.base.read_bytes()).hexdigest(),
        "overrides": overrides,
        "lidar": {
            "samples": 720 if args.profile == "precision_720x20" else 360,
            "update_rate_hz": 20 if args.profile == "precision_720x20" else 10,
        },
        "sensitivity_only": True,
        "default_profile_changed": False,
    }
    args.report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
