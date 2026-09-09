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


def test_final_dashboard_binds_only_live_camera_and_product_perception_inputs():
    source = (ROOT / "sanitation_hmi/live_server.py").read_text(encoding="utf-8")
    state = (ROOT / "sanitation_hmi/live_state.py").read_text(encoding="utf-8")
    demo = (ROOT / "web/demo.html").read_text(encoding="utf-8")
    package_xml = (ROOT / "package.xml").read_text(encoding="utf-8")

    assert '"/sensors/front_rgbd/depth/image_rect_raw/image"' in source
    assert '"/perception/garbage/targets"' in source
    assert '"/perception/open_vocab/diagnostics"' in source
    assert '"/api/v1/images/front_camera"' in source
    assert "update_front_camera" in source
    assert "update_perception_targets" in source
    assert "update_perception_diagnostics" in source
    assert "<exec_depend>diagnostic_msgs</exec_depend>" in package_xml
    assert "LIVE_INPUT_STALE_SECONDS = 5.0" in state
    assert "evaluation_sample_brush_enabled" in state
    assert '"/formal_mapping/lifecycle_status"' in source
    assert '"/formal_mapping/map_ready"' in source
    assert '"/formal_mapping/explorer_status"' in source
    assert '"/formal_saved_map_coverage/state"' in source
    assert "update_mapping_lifecycle" in source
    assert "update_saved_map_coverage" in source
    assert "lifecycle_qos = QoSProfile(" in source
    assert "durability=DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert 'id="front-camera-image"' in demo
    assert 'id="perception-targets"' in demo
    assert 'id="perception-diagnostics"' in demo
    assert "/api/v1/images/front_camera?age=" in demo


def test_live_dashboard_does_not_present_default_values_or_evaluation_as_live_vehicle_state():
    state = (ROOT / "sanitation_hmi/live_state.py").read_text(encoding="utf-8")
    demo = (ROOT / "web/demo.html").read_text(encoding="utf-8")

    assert "self._linear_speed: float | None = None" in state
    assert "self._brush_enabled: bool | None = None" in state
    assert "self._emergency_stop: bool | None = None" in state
    assert 'return data?.vehicle?.estimated_pose_map || data?.vehicle?.odometry_preview_pose_odom;' in demo
    assert 'id="runtime-chain-status"' in demo
    assert "HTTP 可达 · ROS 核心" in demo
    assert "sourceStatusLabel(brushInput.status)" in demo
    assert "sourceStatusLabel(safetyInput.status)" in demo


def test_mapping_statuses_keep_retained_or_observed_age_and_expose_map_pose_progress():
    state = (ROOT / "sanitation_hmi/live_state.py").read_text(encoding="utf-8")
    demo = (ROOT / "web/demo.html").read_text(encoding="utf-8")

    assert '"freshness_mode": "retained"' in state
    assert '"freshness_mode": "observed"' in state
    assert '"map_pose"' in state
    assert '"known_cell_delta"' in state
    assert 'id="map-observation"' in demo
    assert 'id="map-pose-observation"' in demo
    assert "覆盖路径（建图阶段不适用）" in demo


def test_live_dashboard_shutdown_is_idempotent_after_sigint():
    source = (ROOT / "sanitation_hmi/live_server.py").read_text(encoding="utf-8")

    assert "ExternalShutdownException" in source
    assert "except (KeyboardInterrupt, ExternalShutdownException):" in source
    assert "if rclpy.ok():" in source
    assert "            rclpy.shutdown()" in source
