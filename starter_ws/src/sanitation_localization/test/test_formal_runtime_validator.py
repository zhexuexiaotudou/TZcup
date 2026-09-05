"""Regression tests for formal localization runtime acceptance."""

from copy import deepcopy
import json

from sanitation_localization.formal_runtime_validator import (
    main,
    validate_runtime_report,
)


GIDS = {
    "local": "01",
    "global": "02",
    "slam": "03",
    "amcl": "04",
    "navsat": "05",
    "wheel": "06",
    "imu": "07",
    "gnss": "08",
}


def _endpoint(gid: str, node: str) -> dict:
    return {"gid": gid, "node": node, "topic_type": "test"}


def _topic(
    count: int,
    publishers=(),
    subscriptions=(),
    gid: str | None = None,
):
    return {
        "message_count": count,
        "messages_by_gid": {gid: count} if gid else {},
        "publishers": list(publishers),
        "subscriptions": list(subscriptions),
    }


def _base(mode: str) -> dict:
    endpoints = {
        GIDS["local"]: _endpoint(GIDS["local"], "/local_ekf"),
        GIDS["global"]: _endpoint(GIDS["global"], "/global_ekf"),
        GIDS["slam"]: _endpoint(GIDS["slam"], "/slam_toolbox"),
        GIDS["amcl"]: _endpoint(GIDS["amcl"], "/amcl"),
        GIDS["navsat"]: _endpoint(GIDS["navsat"], "/navsat_transform"),
        GIDS["wheel"]: _endpoint(GIDS["wheel"], "/wheel_driver"),
        GIDS["imu"]: _endpoint(GIDS["imu"], "/imu_driver"),
        GIDS["gnss"]: _endpoint(GIDS["gnss"], "/gnss_driver"),
    }
    local_sub = [_endpoint("11", "/local_ekf")]
    report = {
        "schema_version": 1,
        "mode": mode,
        "collector_contract": {
            "world_truth_used": False,
            "subscribed_topics": [
                "/tf",
                "/tf_static",
                "/odom",
                "/odom/unfiltered",
                "/imu/data",
                "/amcl_pose",
                "/gnss/fix",
                "/odometry/gps",
                "/localization/fused_odom",
            ],
        },
        "graph_nodes": ["/local_ekf"],
        "endpoint_registry": endpoints,
        "topics": {
            "/odom": _topic(
                10,
                [_endpoint(GIDS["local"], "/local_ekf")],
                [],
                GIDS["local"],
            ),
            "/odom/unfiltered": _topic(10, [], local_sub, GIDS["wheel"]),
            "/imu/data": _topic(10, [], local_sub, GIDS["imu"]),
        },
        "tf_edges": {},
    }
    return report


def _valid_mapping() -> dict:
    report = _base("mapping")
    report["graph_nodes"].append("/slam_toolbox")
    report["tf_edges"]["map->odom"] = {
        "message_count": 10,
        "messages_by_gid": {GIDS["slam"]: 10},
    }
    return report


def _valid_cleaning() -> dict:
    report = _base("cleaning")
    report["graph_nodes"].extend(
        ["/global_ekf", "/amcl", "/navsat_transform"]
    )
    report["tf_edges"]["map->odom"] = {
        "message_count": 10,
        "messages_by_gid": {GIDS["global"]: 10},
    }
    global_sub = [_endpoint("12", "/global_ekf")]
    report["topics"]["/odom"]["subscriptions"] = global_sub
    report["topics"]["/localization/fused_odom"] = _topic(
        10,
        [_endpoint(GIDS["global"], "/global_ekf")],
        [],
        GIDS["global"],
    )
    report["topics"]["/amcl_pose"] = _topic(
        10,
        [_endpoint(GIDS["amcl"], "/amcl")],
        global_sub,
        GIDS["amcl"],
    )
    report["topics"]["/odometry/gps"] = _topic(
        10,
        [_endpoint(GIDS["navsat"], "/navsat_transform")],
        global_sub,
        GIDS["navsat"],
    )
    report["topics"]["/gnss/fix"] = _topic(
        10,
        [_endpoint(GIDS["gnss"], "/gnss_driver")],
        [_endpoint("13", "/navsat_transform")],
        GIDS["gnss"],
    )
    return report


def _failed_ids(result: dict) -> set[str]:
    return {item["id"] for item in result["checks"] if not item["passed"]}


def test_valid_mapping_report_passes():
    """Accept a mapping graph with one SLAM map-to-odom authority."""
    result = validate_runtime_report(_valid_mapping())
    assert result["status"] == "PASS"


def test_mapping_rejects_second_tf_authority_and_running_global_filter():
    """Reject simultaneous mapping and cleaning global authorities."""
    report = _valid_mapping()
    report["graph_nodes"].append("/global_ekf")
    report["tf_edges"]["map->odom"]["messages_by_gid"][GIDS["global"]] = 4
    report["tf_edges"]["map->odom"]["message_count"] = 14
    result = validate_runtime_report(report)
    assert result["status"] == "BLOCKED"
    assert {
        "map_to_odom_unique_authority",
        "mapping_global_ekf_absent",
    } <= _failed_ids(result)


def test_rejects_duplicate_odom_publisher_even_when_it_is_silent():
    """Reject an extra odom publisher even if no messages were observed."""
    report = _valid_mapping()
    report["topics"]["/odom"]["publishers"].append(
        _endpoint("99", "/wheel_driver")
    )
    result = validate_runtime_report(report)
    assert "local_ekf_unique_odom_publisher" in _failed_ids(result)


def test_valid_cleaning_report_passes():
    """Accept complete AMCL and GNSS cleaning-mode fusion evidence."""
    result = validate_runtime_report(_valid_cleaning())
    assert result["status"] == "PASS"


def test_cleaning_rejects_slam_and_missing_amcl_and_gnss_activity():
    """Reject cleaning with SLAM ownership or inactive global inputs."""
    report = _valid_cleaning()
    report["graph_nodes"].append("/slam_toolbox")
    report["topics"]["/amcl_pose"]["message_count"] = 0
    report["topics"]["/gnss/fix"]["message_count"] = 0
    result = validate_runtime_report(report)
    assert result["status"] == "BLOCKED"
    assert {
        "cleaning_slam_toolbox_absent",
        "global_ekf_receives_amcl",
        "navsat_transform_uses_live_gnss",
        "amcl_pose_source_active",
    } <= _failed_ids(result)


def test_rejects_unknown_tf_authority_gid():
    """Reject transform messages whose publisher cannot be attributed."""
    report = _valid_cleaning()
    report["tf_edges"]["map->odom"]["messages_by_gid"] = {"deadbeef": 10}
    result = validate_runtime_report(report)
    assert "map_to_odom_unique_authority" in _failed_ids(result)


def test_rejects_any_declared_truth_subscription():
    """Reject any collector contract that admits simulation truth."""
    report = deepcopy(_valid_mapping())
    report["collector_contract"]["subscribed_topics"].append(
        "/world/campus/model/info"
    )
    result = validate_runtime_report(report)
    assert "no_world_truth_input" in _failed_ids(result)


def test_cli_writes_durable_pass_record(tmp_path):
    """Exercise the installed-validator path and its durable JSON contract."""
    input_path = tmp_path / "runtime.json"
    output_path = tmp_path / "acceptance.json"
    input_path.write_text(json.dumps(_valid_cleaning()), encoding="utf-8")

    result = main(
        ["--input", str(input_path), "--output", str(output_path)]
    )

    assert result == 0
    acceptance = json.loads(output_path.read_text(encoding="utf-8"))
    assert acceptance["status"] == "PASS"
    assert acceptance["summary"] == {
        "passed_checks": 14,
        "total_checks": 14,
    }
