from pathlib import Path

import pytest
import yaml

from sanitation_perception.product_pipeline_node import (
    SUPPORTED_RUNTIME_CONTRACT,
    stamp_nanoseconds,
    validate_product_runtime_contract,
)


class Stamp:
    sec = 12
    nanosec = 345


class Header:
    stamp = Stamp()


class Message:
    header = Header()


def test_rgb_stamp_is_converted_exactly_for_timestamped_tf():
    assert stamp_nanoseconds(Message()) == 12_000_000_345


def test_product_contract_rejects_cpu_or_missing_iobinding():
    runtime = {
        "postprocess_contract": SUPPORTED_RUNTIME_CONTRACT,
        "required_provider": "CUDAExecutionProvider",
        "io_binding_required": True,
        "cpu_fallback_forbidden": True,
        "maximum_candidates": 16,
        "minimum_valid_depth_ratio": 0.05,
        "minimum_area_region_pixels": 20,
        "minimum_rgb_stddev": 2.0,
        "maximum_dark_or_saturated_fraction": 0.98,
    }
    validate_product_runtime_contract({"runtime": runtime})
    for key, value in (
        ("required_provider", "CPUExecutionProvider"),
        ("io_binding_required", False),
        ("cpu_fallback_forbidden", False),
        ("maximum_candidates", 0),
        ("minimum_valid_depth_ratio", 0.0),
        ("minimum_area_region_pixels", 2),
        ("minimum_rgb_stddev", 0.0),
        ("maximum_dark_or_saturated_fraction", 1.0),
    ):
        invalid = {"runtime": {**runtime, key: value}}
        with pytest.raises(RuntimeError):
            validate_product_runtime_contract(invalid)


def test_repository_placeholder_cannot_activate_product_runtime():
    path = Path(__file__).parents[1] / "config" / "perception_pipeline_manifest.yaml"
    pipeline = yaml.safe_load(path.read_text(encoding="utf-8"))
    with pytest.raises(RuntimeError, match="postprocess contract"):
        validate_product_runtime_contract(pipeline)
