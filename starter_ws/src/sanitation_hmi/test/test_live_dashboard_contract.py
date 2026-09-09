from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_live_dashboard_subscribes_to_latched_occupancy_grid_with_bounded_state():
    source = (ROOT / "sanitation_hmi/live_server.py").read_text(encoding="utf-8")
    state = (ROOT / "sanitation_hmi/live_state.py").read_text(encoding="utf-8")

    assert "OccupancyGrid," in source
    assert '"/map",' in source
    assert "self._on_map," in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "OCCUPANCY_GRID_MAX_AXIS = 192" in state
    assert "def update_occupancy_grid(" in state


def test_final_demo_renders_downsampled_grid_and_has_local_view_without_losing_overview():
    demo = (ROOT / "web/demo.html").read_text(encoding="utf-8")

    assert "SLAM 占据栅格（下采样）" in demo
    assert "drawOccupancyGrid" in demo
    assert "局部实时视图" in demo
    assert 'mapView === "local"' in demo
    assert 'mapView === "overview"' in demo
    # Keep the first live image immediate while bounding repeated full-state
    # JSON serialization to one update per second.
    assert 'fetch("/api/v1/telemetry", {cache: "no-store"})' in demo
    assert "setInterval(refresh, 1000);\n    refresh();" in demo
