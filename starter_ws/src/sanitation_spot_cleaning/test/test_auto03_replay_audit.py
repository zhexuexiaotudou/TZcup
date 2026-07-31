import importlib.util
from pathlib import Path


def _find_script() -> Path:
    for candidate in Path(__file__).resolve().parents:
        script = candidate / "scripts" / "auto03_replay_audit.py"
        if script.is_file():
            return script
    raise RuntimeError("could not locate repository root containing auto03_replay_audit.py")


SCRIPT = _find_script()
SPEC = importlib.util.spec_from_file_location("auto03_replay_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_replay_audit_requires_exact_trial_sequence_metrics_and_topics():
    truth = {
        "candidate_id": "candidate",
        "world_id": "world",
        "scene_id": "scene",
        "case_type": "unreachable_keepout",
        "class_id": "plastic_bottle",
    }
    runtime = {
        "candidate_id": "candidate",
        "coverage_boundary_pause_safe": True,
        "preflight_path_success": False,
        "navigation_attempted": False,
        "navigate_success": False,
        "coverage_resumed": True,
        "cleaning_commanded": False,
        "terminal_state": "UNREACHABLE",
        "terminal_reason": "compute_path_failed",
    }
    counts = {topic: 1 for topic in MODULE.REQUIRED_TOPICS}
    report = MODULE.compare_runtime_payloads(
        matrix={"trials": [truth]},
        runtime={"trials": [runtime]},
        replayed_results=[runtime],
        topic_counts=counts,
    )
    assert report["replay_audit_pass"]
    assert report["metric_replay_delta_max"] == 0.0


def test_replay_audit_fails_when_required_topic_is_missing():
    runtime = {"candidate_id": "candidate"}
    report = MODULE.compare_runtime_payloads(
        matrix={"trials": [{
            "candidate_id": "candidate",
            "world_id": "world",
            "scene_id": "scene",
            "case_type": "stale_dropout",
            "class_id": "plastic_bottle",
        }]},
        runtime={"trials": [runtime]},
        replayed_results=[runtime],
        topic_counts={"/auto03/trial_result": 1, "/coverage/state": 1},
    )
    assert not report["replay_audit_pass"]
    assert report["missing_required_topics"]
