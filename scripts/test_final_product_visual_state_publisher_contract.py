from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUBLISHER = (ROOT / "scripts/publish_final_product_visual_state.py").read_text(encoding="utf-8")


def test_publisher_is_file_driven_and_only_emits_the_final_demo_topic() -> None:
    assert 'parser.add_argument("--state-file", type=Path, required=True)' in PUBLISHER
    assert 'create_publisher(String, "/final_demo/state", qos)' in PUBLISHER
    assert 'self.create_timer(period_sec, self._publish)' in PUBLISHER
    assert 'ros2 launch' not in PUBLISHER


def test_publisher_fails_closed_on_nonfinal_product_state() -> None:
    assert 'payload.get("field_dimensions_m") != [200, 100]' in PUBLISHER
    assert 'payload.get("vehicle") != "A300"' in PUBLISHER
    assert 'payload.get("formal_product_acceptance") is not False' in PUBLISHER
    assert 'raise ValueError("visual state must not claim formal product acceptance")' in PUBLISHER
    assert 'args.period_sec != 1.0' in PUBLISHER


def test_publisher_shutdown_is_idempotent_after_external_shutdown() -> None:
    assert "ExternalShutdownException" in PUBLISHER
    assert "except (KeyboardInterrupt, ExternalShutdownException):" in PUBLISHER
    assert "if rclpy.ok():" in PUBLISHER
    assert "            rclpy.shutdown()" in PUBLISHER
