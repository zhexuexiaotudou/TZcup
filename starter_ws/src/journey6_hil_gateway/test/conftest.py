from __future__ import annotations

import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
for path in (PACKAGE_ROOT, SOURCE_ROOT / "sanitation_perception"):
    rendered = str(path)
    if rendered not in sys.path:
        sys.path.insert(0, rendered)
