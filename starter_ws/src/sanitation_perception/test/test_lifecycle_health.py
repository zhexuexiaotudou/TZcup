from sanitation_perception.lifecycle_health import ProductHealth, WatchdogConfig


def config():
    return WatchdogConfig(500.0, 200.0, 3, 2, 2)


def active_health():
    health = ProductHealth(config())
    health.transition("INACTIVE", "configured")
    health.transition("ACTIVE", "activated")
    health.record_frame(0.0)
    return health


def test_camera_tf_latency_and_session_faults_fail_closed():
    health = active_health()
    assert health.perception_spot_clean_allowed is True
    assert health.evaluate(0.6) == "DEGRADED"
    assert health.perception_spot_clean_allowed is False
    health.record_frame(0.6)
    assert health.evaluate(0.6) == "ACTIVE"
    health.record_tf_error(); health.record_tf_error()
    assert health.evaluate(0.6) == "DEGRADED"
    health.record_tf_success(); health.record_frame(0.7)
    assert health.evaluate(0.7) == "ACTIVE"
    for value in (201.0, 250.0, 220.0):
        health.record_inference(0.7, value)
    assert health.evaluate(0.7) == "DEGRADED"
    health.record_session_error(); health.record_session_error()
    assert health.evaluate(0.7) == "ERROR"


def test_oom_enters_error_and_snapshot_blocks_spot_clean():
    health = active_health()
    health.record_session_error(oom=True)
    snapshot = health.snapshot(0.1)
    assert snapshot["state"] == "ERROR"
    assert snapshot["reason"] == "inference_oom"
    assert snapshot["perception_spot_clean_allowed"] is False
