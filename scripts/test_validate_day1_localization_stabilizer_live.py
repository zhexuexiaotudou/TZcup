import importlib.util
import json
from pathlib import Path
import sys


SPEC = importlib.util.spec_from_file_location(
    "live_validator",
    Path(__file__).with_name("validate_day1_localization_stabilizer_live.py"),
)
validator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = validator
SPEC.loader.exec_module(validator)


def _write_json(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")


def _fixture(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_json(
        run / "effective_parameters.json",
        {
            "schema_version": 1,
            "all_expected": True,
            "nodes": {
                "/map_odom_stabilizer": {
                    "tau_sec": {"actual": 1.5},
                    "max_filter_dt_sec": {"actual": 0.1},
                    "max_gap_sec": {"actual": 0.5},
                    "input_tf_topic": {
                        "actual": "/localization/raw_map_odom"
                    },
                }
            },
        },
    )
    _write_json(
        run / "tf_authority.json",
        {
            "graph_nodes": ["/global_ekf", "/map_odom_stabilizer"],
            "endpoint_registry": {
                "gid": {"node": "/map_odom_stabilizer"}
            },
            "tf_edges": {
                "map->odom": {
                    "message_count": 500,
                    "messages_by_gid": {"gid": 500},
                }
            },
            "topics": {
                "/localization/raw_map_odom": {
                    "message_count": 500,
                    "publishers": [{"node": "/global_ekf"}],
                    "subscriptions": [{"node": "/map_odom_stabilizer"}],
                }
            },
        },
    )
    _write_json(
        run / "localization_focus.json",
        {
            "status": "PASS",
            "accuracy": {
                "samples": 2711,
                "rmse_m": 0.029,
                "p95_m": 0.041,
                "max_m": 0.049,
            },
        },
    )
    status = {
        "status": "READY",
        "blocked_reason": None,
        "uses_ground_truth": False,
        "uses_future": False,
        "accepted_updates": 500,
        "published_tf_messages": 500,
        "rejected_updates": 0,
        "tau_sec": 1.5,
    }
    (run / "map_odom_stabilizer.status.yaml").write_text(
        "data: '" + json.dumps(status) + "'\n---\n", encoding="utf-8"
    )
    (run / "map_odom_stabilizer.node.txt").write_text(
        "Publishers:\n  /tf\nSubscribers:\n"
        "  /localization/raw_map_odom\n"
        "  /localization/map_odom_stabilizer/status\n",
        encoding="utf-8",
    )
    _write_json(
        run / "resource_release.json",
        {
            "status": "RELEASED",
            "gazebo_process_remaining": False,
            "ros_runtime_process_remaining": False,
            "formal_lock_available": True,
        },
    )
    (run / "driver.rc").write_text("0\n", encoding="utf-8")
    return run


def test_live_receipt_accepts_complete_online_only_evidence(tmp_path):
    receipt = validator.validate(_fixture(tmp_path))
    assert receipt["status"] == "LIVE_CANDIDATE_PASS"
    assert all(gate["passed"] for gate in receipt["gates"].values())


def test_live_receipt_rejects_duplicate_tf_owner(tmp_path):
    run = _fixture(tmp_path)
    authority = json.loads((run / "tf_authority.json").read_text())
    authority["endpoint_registry"]["other"] = {"node": "/global_ekf"}
    authority["tf_edges"]["map->odom"]["messages_by_gid"]["other"] = 10
    _write_json(run / "tf_authority.json", authority)
    receipt = validator.validate(run)
    assert receipt["status"] == "FAIL"
    assert receipt["gates"]["authority"]["passed"] is False


def test_live_receipt_rejects_unreleased_resource(tmp_path):
    run = _fixture(tmp_path)
    resource = json.loads((run / "resource_release.json").read_text())
    resource["gazebo_process_remaining"] = True
    _write_json(run / "resource_release.json", resource)
    receipt = validator.validate(run)
    assert receipt["status"] == "FAIL"
    assert receipt["gates"]["resource_release"]["passed"] is False
