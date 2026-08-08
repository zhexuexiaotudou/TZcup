from sanitation_perception.performance_monitor import (
    PerformanceConfig, PerformanceMonitor, SoakAudit,
)


CONFIG = PerformanceConfig(150.0, 200.0, 10.0, 0.01, 7200.0, 0.05)


def test_product_performance_gate_uses_p95_fps_and_drop_rate(monkeypatch):
    monitor = PerformanceMonitor(CONFIG)
    monitor.started_s = 100.0
    monkeypatch.setattr(
        "sanitation_perception.performance_monitor.current_gpu_memory_bytes",
        lambda: 123,
    )
    for _ in range(100):
        monitor.record_submission()
        monitor.record_frame(
            {"inference_pipeline": 120.0, "end_to_end": 160.0},
            candidate_count=2,
        )
    report = monitor.snapshot(now_s=109.0)
    assert report["performance_gate_pass"] is True
    assert report["effective_hz"] > 10.0
    assert report["gpu_memory_bytes"] == 123


def test_drop_or_slow_tail_fails_performance_gate():
    monitor = PerformanceMonitor(CONFIG)
    monitor.started_s = 0.0
    for index in range(100):
        monitor.record_submission(dropped=int(index < 2))
        monitor.record_frame(
            {"inference_pipeline": 250.0 if index >= 94 else 100.0,
             "end_to_end": 260.0 if index >= 94 else 150.0}
        )
    report = monitor.snapshot(now_s=9.0)
    assert report["gates"]["inference_pipeline_p95"] is False
    assert report["gates"]["drop_rate"] is False


def test_soak_requires_duration_memory_and_fault_free_run():
    good = SoakAudit(CONFIG, started_s=0.0, initial_rss_bytes=1000)
    assert good.finish(ended_s=7200.0, final_rss_bytes=1050)["soak_gate_pass"]
    bad = SoakAudit(CONFIG, started_s=0.0, initial_rss_bytes=1000, crash_count=1)
    assert not bad.finish(ended_s=7199.0, final_rss_bytes=1100)["soak_gate_pass"]
