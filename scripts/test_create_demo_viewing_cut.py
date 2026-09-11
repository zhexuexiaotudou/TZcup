import csv
import hashlib
import json
from pathlib import Path

import pytest

from create_demo_viewing_cut import REQUIRED_EVENTS, build, make_segments


def write_fixture(root: Path, *, missing: str | None = None) -> tuple[Path, Path, Path]:
    run = root / "run"
    run.mkdir()
    for name in ("raw.json", "observation.json", "formal.json", "demo.mp4", "events.json"):
        (run / name).write_bytes(b"fixture")
    bag = run / "bag"; bag.mkdir()
    (bag / "metadata.yaml").write_bytes(b"metadata")
    (bag / "data.mcap").write_bytes(b"mcap")
    def descriptor(name: str) -> dict:
        path = run / name
        return {"path": str(path), "size_bytes": path.stat().st_size, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    mcap_files = []
    for path in sorted(bag.iterdir()):
        mcap_files.append({"relative_path": path.name, "size_bytes": path.stat().st_size, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    manifest = {
        "schema_version": "tzcup.competition_demo_artifact_manifest.v1",
        "run_root": str(run),
        "run_identity": {"session_id": "s", "runtime_id": "r", "episode_id": "e", "session_start_epoch_ns": 1_000_000_000},
        "recording_window": {"operator_start_epoch_ns": 1_000_000_000, "mission_complete_epoch_ns": 401_000_000_000},
        "timeline_status": "complete",
        "missing_required_events": [],
        "delivery_status": "READY_FOR_REVIEW",
        "artifacts": {"raw_collection": descriptor("raw.json"), "video_observation": descriptor("observation.json"), "formal_session_current": descriptor("formal.json"), "video": descriptor("demo.mp4"), "mcap": {"path": str(bag), "files": mcap_files, "tree_sha256": hashlib.sha256("\n".join(f"{item['relative_path']} {item['sha256']}" for item in mcap_files).encode()).hexdigest()}, "event_reports": [descriptor("events.json")]},
    }
    manifest_path = root / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    timeline = root / "timeline.csv"
    with timeline.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["event", "status", "epoch_ns", "source", "reason"])
        writer.writeheader()
        for index, event in enumerate(REQUIRED_EVENTS):
            writer.writerow({"event": event, "status": "missing" if event == missing else "observed", "epoch_ns": 26_000_000_000 + index * 60_000_000_000, "source": "fixture", "reason": event})
    checksums = root / "checksums.sha256"
    checksums.write_text("\n".join(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}" for path in (manifest_path, timeline)) + "\n", encoding="utf-8")
    return manifest_path, timeline, checksums


def test_valid_contract_writes_source_bound_edl(tmp_path, monkeypatch):
    manifest, timeline, checksums = write_fixture(tmp_path)
    monkeypatch.setattr("create_demo_viewing_cut.video_duration", lambda *_: 400.0)
    receipt = build(manifest, timeline, checksums, tmp_path / "out", "ffprobe", None, False)
    assert receipt["status"] == "READY_FOR_REVIEW"
    edl = json.loads((tmp_path / "out" / "viewing_cut.edl.json").read_text(encoding="utf-8"))
    assert [row["event"] for row in edl["segments"]] == list(REQUIRED_EVENTS)
    assert edl["duration_sec"] == 300.0


def test_missing_event_fails_closed_without_edl(tmp_path, monkeypatch):
    manifest, timeline, checksums = write_fixture(tmp_path, missing="grasp_drop")
    monkeypatch.setattr("create_demo_viewing_cut.video_duration", lambda *_: 400.0)
    receipt = build(manifest, timeline, checksums, tmp_path / "out", "ffprobe", None, False)
    assert receipt["status"] == "BLOCKED_MISSING_REQUIRED_EVIDENCE"
    assert "grasp_drop" in receipt["missing"][0]
    assert not (tmp_path / "out" / "viewing_cut.edl.json").exists()


def test_checksum_drift_fails_closed(tmp_path, monkeypatch):
    manifest, timeline, checksums = write_fixture(tmp_path)
    timeline.write_text(timeline.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    monkeypatch.setattr("create_demo_viewing_cut.video_duration", lambda *_: 400.0)
    receipt = build(manifest, timeline, checksums, tmp_path / "out", "ffprobe", None, False)
    assert receipt["status"] == "BLOCKED_MISSING_REQUIRED_EVIDENCE"
    assert "checksums" in receipt["missing"][0]


def test_mcap_inventory_drift_fails_closed(tmp_path, monkeypatch):
    manifest, timeline, checksums = write_fixture(tmp_path)
    (tmp_path / "run" / "bag" / "unexpected.mcap").write_bytes(b"different")
    monkeypatch.setattr("create_demo_viewing_cut.video_duration", lambda *_: 400.0)
    receipt = build(manifest, timeline, checksums, tmp_path / "out", "ffprobe", None, False)
    assert receipt["status"] == "BLOCKED_MISSING_REQUIRED_EVIDENCE"
    assert "mcap" in receipt["missing"][0]


def test_incomplete_upstream_delivery_fails_closed(tmp_path, monkeypatch):
    manifest, timeline, checksums = write_fixture(tmp_path)
    data = json.loads(manifest.read_text(encoding="utf-8"))
    data["delivery_status"] = "BLOCKED_MISSING_REQUIRED_EVIDENCE"
    manifest.write_text(json.dumps(data), encoding="utf-8")
    checksums.write_text("\n".join(
        f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}"
        for path in (manifest, timeline)
    ) + "\n", encoding="utf-8")
    monkeypatch.setattr("create_demo_viewing_cut.video_duration", lambda *_: 400.0)
    receipt = build(manifest, timeline, checksums, tmp_path / "out", "ffprobe", None, False)
    assert receipt["status"] == "BLOCKED_MISSING_REQUIRED_EVIDENCE"
    assert "delivery_status" in receipt["missing"][0]


def test_overlapping_events_cannot_be_edited_into_a_false_duration():
    rows = {event: {"status": "observed", "epoch_ns": "100000000000", "source": "x", "reason": "x"} for event in REQUIRED_EVENTS}
    with pytest.raises(ValueError, match="overlap"):
        make_segments(rows, 1_000_000_000, 400.0)
