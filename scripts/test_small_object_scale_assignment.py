#!/usr/bin/env python3

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g4_direct_fcos import direct_input_size


def test_mrv2_resolutions_are_explicit_and_stride_aligned():
    assert direct_input_size(input_size=(640, 480)) == (640, 480)
    assert direct_input_size(input_size=(960, 720)) == (960, 720)
    assert direct_input_size(input_size=(1280, 960)) == (1280, 960)
    with pytest.raises(ValueError):
        direct_input_size(input_size=(950, 720))
