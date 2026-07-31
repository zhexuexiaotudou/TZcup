from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_representative_frame_is_taken_from_the_completed_end_of_video():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert 'ffmpeg -nostdin -y -sseof -5 -i "${OUTPUT_DIR}/visual_demo.mp4"' in launcher
    assert '-frames:v 1 -update 1 "${OUTPUT_DIR}/visual_demo_frame.png"' in launcher
    assert 'ffmpeg -nostdin -y -ss 5 -i "${OUTPUT_DIR}/visual_demo.mp4"' not in launcher
