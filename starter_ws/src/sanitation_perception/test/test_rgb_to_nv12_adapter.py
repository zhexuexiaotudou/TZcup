"""Tests for the formal RGB/BGR to NV12 image boundary."""

import numpy as np
import pytest

from sanitation_perception.rgb_to_nv12_adapter import (
    S100P_SOURCE_HEIGHT,
    S100P_SOURCE_WIDTH,
    Nv12ConversionError,
    SuccessDiagnosticThrottle,
    SourceStampSelector,
    image_bytes_to_nv12,
    s100p_nv12_pair,
)


def test_bgr_and_rgb_convert_to_tightly_packed_nv12():
    bgr = np.zeros((2, 4, 3), dtype=np.uint8)
    bgr[:, :, 2] = 255
    bgr_nv12 = image_bytes_to_nv12(
        bgr.tobytes(), width=4, height=2, encoding="bgr8", step=12
    )
    rgb_nv12 = image_bytes_to_nv12(
        bgr[:, :, ::-1].tobytes(), width=4, height=2, encoding="rgb8", step=12
    )
    assert len(bgr_nv12) == 12
    assert bgr_nv12 == rgb_nv12


def test_nv12_passthrough_is_exact_and_fail_closed_on_bad_input():
    source = bytes(range(12))
    assert image_bytes_to_nv12(
        source, width=4, height=2, encoding="nv12", step=4
    ) == source
    with pytest.raises(Nv12ConversionError, match="even"):
        image_bytes_to_nv12(source, width=3, height=2, encoding="nv12", step=3)
    with pytest.raises(Nv12ConversionError, match="payload"):
        image_bytes_to_nv12(source, width=4, height=2, encoding="nv12", step=5)
    with pytest.raises(Nv12ConversionError, match="unsupported"):
        image_bytes_to_nv12(bytes(24), width=4, height=2, encoding="mono8", step=4)
    with pytest.raises(Nv12ConversionError, match="payload"):
        image_bytes_to_nv12(bytes(10), width=4, height=2, encoding="bgr8", step=12)


def test_s100p_pair_publishes_identical_original_packed_nv12_to_both_bpu_topics():
    source = np.zeros((S100P_SOURCE_HEIGHT, S100P_SOURCE_WIDTH, 3), dtype=np.uint8)
    source[:, :, 2] = 255
    dosod, edgesam = s100p_nv12_pair(
        source.tobytes(), width=S100P_SOURCE_WIDTH, height=S100P_SOURCE_HEIGHT,
        encoding="bgr8", step=S100P_SOURCE_WIDTH * 3,
    )
    assert len(dosod) == S100P_SOURCE_WIDTH * S100P_SOURCE_HEIGHT * 3 // 2
    assert len(edgesam) == S100P_SOURCE_WIDTH * S100P_SOURCE_HEIGHT * 3 // 2
    assert dosod == edgesam
    with pytest.raises(Nv12ConversionError, match="848x480"):
        s100p_nv12_pair(bytes(12), width=4, height=2, encoding="bgr8", step=12)


def test_success_diagnostics_use_monotonic_rate_limit_but_errors_do_not():
    throttle = SuccessDiagnosticThrottle()
    assert throttle.allow(0, 1_000_000_000)
    assert not throttle.allow(0, 1_999_999_999)
    assert throttle.allow(2, 1_999_999_999)
    assert throttle.allow(0, 2_000_000_000)


def test_stamp_selector_is_source_time_deterministic_two_hz_and_rejects_replay():
    selector = SourceStampSelector()
    assert selector.select(1_000_000_000) == (True, "selected")
    selector.commit_selected(1_000_000_000)
    assert selector.select(1_100_000_000) == (False, "rate_limited")
    assert selector.select(1_500_000_000) == (True, "selected")
    selector.commit_selected(1_500_000_000)
    assert selector.select(1_500_000_000) == (False, "duplicate_or_out_of_order")
    assert selector.select(1_400_000_000) == (False, "duplicate_or_out_of_order")
