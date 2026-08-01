from pathlib import Path

from PIL import Image, ImageDraw

from gazebo_viewport_probe import analyze_viewport


def test_all_black_viewport_is_rejected(tmp_path: Path):
    image_path = tmp_path / "black.png"
    Image.new("RGB", (1000, 800), "black").save(image_path)

    result = analyze_viewport(image_path)

    assert result["render_visible"] is False
    assert result["near_black_ratio"] == 1.0


def test_scene_content_in_left_viewport_is_accepted(tmp_path: Path):
    image_path = tmp_path / "scene.png"
    image = Image.new("RGB", (1000, 800), "#d9ecf5")
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 360, 560, 800), fill="#575d61")
    draw.rectangle((250, 360, 310, 800), fill="#f3ca32")
    draw.ellipse((60, 220, 180, 420), fill="#247a38")
    image.save(image_path)

    result = analyze_viewport(image_path)

    assert result["render_visible"] is True
    assert result["near_black_ratio"] < 0.1
