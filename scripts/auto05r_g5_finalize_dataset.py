#!/usr/bin/env python3
"""Strict host CLI for sealing and auditing the G5 final dataset."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g5_dataset import main_finalize  # noqa: E402


if __name__ == "__main__":
    main_finalize()
