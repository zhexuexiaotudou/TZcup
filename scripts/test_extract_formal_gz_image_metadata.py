import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/extract_formal_gz_image_metadata.py"
SPEC = importlib.util.spec_from_file_location("extract_gz_image_metadata", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_extracts_bounded_rgb_image_metadata() -> None:
    result = MODULE.extract(
        'width: 1600\nheight: 1000\npixel_format_type: RGB_INT8\n'
        'step: 4800\ndata: "\\001\\002"\n'
    )
    assert result["status"] == "FORMAL_GZ_IMAGE_SAMPLE_RECEIVED"
    assert result["expected_uncompressed_data_bytes_from_step"] == 4_800_000


@pytest.mark.parametrize("missing", ("width", "height", "step"))
def test_rejects_missing_dimensions(missing: str) -> None:
    fields = {
        "width": "width: 1600\n",
        "height": "height: 1000\n",
        "step": "step: 4800\n",
    }
    text = "".join(value for key, value in fields.items() if key != missing)
    text += 'pixel_format_type: RGB_INT8\ndata: "x"\n'
    with pytest.raises(ValueError, match=missing):
        MODULE.extract(text)
