from pathlib import Path

from sanitation_hmi.snapshot_io import write_text_snapshot


def test_snapshot_write_retries_transient_share_lock(tmp_path, monkeypatch):
    target = tmp_path / "telemetry.json"
    original_replace = Path.replace
    calls = {"count": 0}

    def flaky_replace(path, destination):
        calls["count"] += 1
        if calls["count"] < 3:
            raise PermissionError("transient Windows share lock")
        return original_replace(path, destination)

    monkeypatch.setattr(Path, "replace", flaky_replace)
    assert write_text_snapshot(target, "fresh\n", retry_delay_sec=0.0)
    assert target.read_text(encoding="utf-8") == "fresh\n"
    assert calls["count"] == 3


def test_snapshot_write_fails_closed_without_raising(tmp_path, monkeypatch):
    target = tmp_path / "telemetry.json"

    def locked_replace(_path, _destination):
        raise PermissionError("persistent Windows share lock")

    monkeypatch.setattr(Path, "replace", locked_replace)
    assert not write_text_snapshot(
        target, "fresh\n", max_attempts=2, retry_delay_sec=0.0
    )
    assert not target.exists()
    assert target.with_suffix(".json.tmp").exists()
