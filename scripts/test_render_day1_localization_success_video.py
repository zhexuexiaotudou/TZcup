import importlib.util
import json
from pathlib import Path
import sys


SPEC = importlib.util.spec_from_file_location(
    "localization_video",
    Path(__file__).with_name("render_day1_localization_success_video.py"),
)
video = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = video
SPEC.loader.exec_module(video)


def test_route_state_only_uses_completed_goals():
    events = [
        {"sim_s": 10.0, "kind": "goal_sent", "data": {"status": 4}},
        {"sim_s": 20.0, "kind": "goal_result", "data": {"status": 4}},
        {"sim_s": 30.0, "kind": "goal_sent", "data": {"status": 4}},
        {"sim_s": 50.0, "kind": "goal_result", "data": {"status": 4}},
    ]
    assert video.route_state(events, 5.0) == "Tracking active"
    assert video.route_state(events, 15.0) == "Goal 1 / 2 in progress"
    assert video.route_state(events, 25.0) == "Goal 1 complete"
    assert video.route_state(events, 40.0) == "Goal 2 / 2 in progress"
    assert video.route_state(events, 60.0) == "2 / 2 goals completed"


def test_verify_inputs_binds_mcap_and_receipt(tmp_path):
    focus = tmp_path / "focus.json"
    focus.write_text(
        json.dumps(
            {
                "files": {"mcap": video.DEFAULT_MCAP_SHA256},
            }
        ),
        encoding="utf-8",
    )
    candidate_csv = tmp_path / "candidate.csv"
    candidate_csv.write_text("value\n", encoding="utf-8")
    receipt = tmp_path / "receipt.json"
    receipt.write_text(
        json.dumps(
            {
                "status": "OFFLINE_CANDIDATE_PASS",
                "metrics": {
                    "candidate": {
                        "rmse_m": video.EXPECTED_CANDIDATE_METRICS["rmse_m"]
                    }
                },
                "evidence": {
                    "candidate_csv_sha256": video.sha256_file(candidate_csv)
                },
            }
        ),
        encoding="utf-8",
    )
    hashes = video.verify_inputs(
        mcap=None,
        expected_mcap_sha256=video.DEFAULT_MCAP_SHA256,
        focus_result=focus,
        candidate_csv=candidate_csv,
        receipt=receipt,
    )
    assert hashes["candidate_csv"] == video.sha256_file(candidate_csv)


def test_video_copy_is_success_only():
    root = Path(__file__).resolve().parents[1]
    text = "\n".join(
        [
            (
            root
            / "artifacts/day1_localization_video_20260914/README.md"
            ).read_text(),
            (
            root
            / "artifacts/day1_localization_video_20260914/narration_5min.md"
            ).read_text(),
        ]
    ).lower()
    assert "fail" not in text
    assert "ambiguous" not in text
