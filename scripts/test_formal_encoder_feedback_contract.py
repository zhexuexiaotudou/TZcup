from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

import pytest
import yaml

from validate_formal_encoder_feedback_contract import (
    DEFAULT_CONTRACT,
    EncoderFeedbackContractError,
    validate,
)


ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "starter_ws/src/sanitation_vehicle_description/scripts/formal_encoder_quantization.py"


def _core_module():
    spec = importlib.util.spec_from_file_location("formal_encoder_quantization_test", CORE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_receiver_and_encoder_static_contract_is_complete() -> None:
    result = validate()
    assert result == {
        "status": "FORMAL_RECEIVER_AND_ENCODER_HARDWARE_STATIC_VALID",
        "a300_encoder_link_count": 4,
        "pololu_encoder_link_count": 3,
        "encoder_topic_contract_count": 4,
        "a300_hardware_resolution_pending": True,
    }


def test_incremental_encoder_quantization_is_symmetric_and_count_exact() -> None:
    core = _core_module()
    step = 2.0 * math.pi / 4480
    assert core.encoder_count(0.49 * step, 4480) == 0
    assert core.encoder_count(0.50 * step, 4480) == 1
    assert core.encoder_count(-0.50 * step, 4480) == -1
    assert core.encoder_count(2.0 * math.pi, 4480) == 4480


def test_group_velocity_comes_from_integer_count_difference() -> None:
    core = _core_module()
    quantizer = core.EncoderGroupQuantizer(("wheel",), 4096)
    first = quantizer.sample(1_000_000_000, {"wheel": 0.0})
    second = quantizer.sample(1_100_000_000, {"wheel": 10.0 * 2.0 * math.pi / 4096})
    assert first.velocity_rad_s == (0.0,)
    assert second.counts == (10,)
    assert second.position_rad[0] == pytest.approx(10.0 * 2.0 * math.pi / 4096)
    assert second.velocity_rad_s[0] == pytest.approx(100.0 * 2.0 * math.pi / 4096)


def test_non_monotonic_clock_resets_velocity_baseline() -> None:
    core = _core_module()
    quantizer = core.EncoderGroupQuantizer(("brush",), 4480)
    quantizer.sample(2_000_000_000, {"brush": 1.0})
    reset = quantizer.sample(1_000_000_000, {"brush": 2.0})
    assert reset.velocity_rad_s == (0.0,)


def test_a300_simulation_resolution_cannot_be_presented_as_vendor_data(tmp_path: Path) -> None:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    contract["a300_wheel_encoders"]["simulation_quantizer"]["source_class"] = (
        "clearpath_official"
    )
    mutated = tmp_path / "encoder.yaml"
    mutated.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    with pytest.raises(EncoderFeedbackContractError, match="Clearpath specification"):
        validate(contract_path=mutated)
