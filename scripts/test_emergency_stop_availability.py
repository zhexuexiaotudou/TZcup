import json

from emergency_stop_availability import dashboard_observed


def test_dashboard_observed_requires_emergency_topic(tmp_path):
    telemetry = tmp_path / "telemetry.json"
    assert not dashboard_observed(telemetry)
    telemetry.write_text(json.dumps({"topics_seen": ["/cmd_vel"]}), encoding="utf-8")
    assert not dashboard_observed(telemetry)
    telemetry.write_text(
        json.dumps({"topics_seen": ["/cmd_vel", "/emergency_stop"]}),
        encoding="utf-8",
    )
    assert dashboard_observed(telemetry)
