import json

from auto10_speech import (
    constrained_recover,
    generate_manifest,
    normalized_text,
)


def test_manifest_covers_required_matrix(tmp_path):
    generate_manifest(tmp_path, 500)
    manifest = json.loads(
        (tmp_path / "speech_manifest.json").read_text(encoding="utf-8")
    )
    cases = manifest["cases"]
    assert len(cases) == 500
    assert len({item["voice"] for item in cases}) == 3
    assert len({item["speech_rate"] for item in cases}) == 3
    assert len({item["noise_level"] for item in cases}) == 4
    assert len({item["reverb_profile"] for item in cases}) >= 2
    assert {item["language"] for item in cases} == {"zh", "en"}


def test_constrained_recovery_uses_transcript_only():
    command, score = constrained_recover("emergency stop")
    assert command == "emergency stop"
    assert score == 1
    assert normalized_text("紧急，停止！") == "紧急停止"
