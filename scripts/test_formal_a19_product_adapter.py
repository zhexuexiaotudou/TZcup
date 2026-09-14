from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))

import formal_a19_product_adapter as adapter
from sanitation_perception.a19_fault_hooks import A19ProductFaultError, ProductFaultHooks


def message(*, sec: int = 2, nanosec: int = 900_000_000, frame: str = "camera", width: int = 8, data: bytes = b"\x01\x02"):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
            frame_id=frame,
        ),
        width=width,
        data=data,
    )


def product_argv() -> list[str]:
    return [
        "ros2", "launch", "sanitation_product_demo_integration",
        "product_demo.launch.py", "gui:=false",
        "world:=/tmp/world.sdf", "episode_manifest:=/tmp/episode_manifest.json",
        "pedestrian_schedule:=/tmp/pedestrians.json", "start_pedestrians:=true",
        "saved_map_artifact_dir:=/tmp/saved_map",
        "perception_artifact_root:=/tmp/perception",
        "policy_checkpoint:=/tmp/q_policy.json",
        "maximum_task_distance_m:=1.0", "episode_seed:=1",
        "operation_speed_profile:=dry_cleaning_competition_candidate",
        "max_linear_velocity:=0.45",
        *[f"{name}:={topic}" for name, topic in adapter.PRODUCT_TOPIC_OVERRIDES.items()],
    ]


def test_current_formal_contract_exposes_only_live_product_fault_consumers() -> None:
    contract = json.loads(
        (ROOT / "config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json").read_text(encoding="utf-8")
    )
    supported, unsupported = adapter.contract_capabilities(contract)
    assert supported == [
        "rgb_freeze", "depth_freeze", "timestamp_skew",
        "camera_info_mismatch", "tf_unavailable", "invalid_depth",
        "proposal_flood", "proposal_dropout", "classifier_exception",
        "classifier_timeout", "action_verifier_failure", "reobserve_timeout",
        "cuda_provider_failure", "model_hash_mismatch", "corrupt_model",
        "sustained_slow_inference",
        "nav2_path_unavailable", "dynamic_obstacle_blocks_observation",
    ]
    assert unsupported == [row["fault"] for row in contract["fault_schedule"] if row["fault"] not in adapter.SUPPORTED_FAULTS]
    assert unsupported == []


def test_product_argv_requires_real_product_launch_and_every_proxy_binding(monkeypatch) -> None:
    monkeypatch.setattr(adapter.shutil, "which", lambda name: sys.executable if name == "ros2" else None)
    argv = product_argv()
    assert adapter.parse_product_argv(json.dumps(argv)) == argv
    with pytest.raises(adapter.AdapterError, match="A19 proxy output"):
        adapter.parse_product_argv(json.dumps(argv[:-1]))
    with pytest.raises(adapter.AdapterError, match="canonical product_demo"):
        adapter.parse_product_argv(json.dumps(["ros2", "launch", "other", "other.launch.py", "gui:=false"]))
    incomplete = [item for item in argv if not item.startswith("episode_seed:=")]
    with pytest.raises(adapter.AdapterError, match="episode_seed"):
        adapter.parse_product_argv(json.dumps(incomplete))


@pytest.mark.parametrize("channel", ["front_rgb", "wrist_rgb", "rear_left_rgb", "rear_right_rgb"])
def test_rgb_freeze_drops_real_proxy_ingress(channel: str) -> None:
    forwarded, readback = adapter.transform_sensor_message(channel, message(), "rgb_freeze", {"duration_s": 10})
    assert forwarded is None
    assert readback == {"action": "dropped", "channel": channel}


def test_depth_freeze_controller_requires_observed_ingress_before_ack() -> None:
    controller = adapter.SensorFaultController()
    controller.begin("depth_freeze", {"duration_s": 10})
    assert controller.readback() is None
    assert controller.apply("front_depth", message()) is None
    readback = controller.readback()
    assert readback is not None and readback["observed"] is True
    assert readback["dropped_channels"] == ["front_depth"]
    assert controller.clear()["cleared"] is True
    assert controller.recovery_readback() is None
    assert controller.apply("front_depth", message()) is not None
    assert controller.recovery_readback()["forwarded_channels"] == ["front_depth"]


def test_timestamp_camera_tf_and_invalid_depth_mutate_forwarded_messages() -> None:
    shifted, shifted_readback = adapter.transform_sensor_message(
        "front_rgb", message(), "timestamp_skew", {"skew_ms": 250, "duration_s": 10}
    )
    assert (shifted.header.stamp.sec, shifted.header.stamp.nanosec) == (3, 150_000_000)
    assert shifted_readback["action"] == "timestamp_shifted"

    mismatched, mismatch_readback = adapter.transform_sensor_message(
        "front_info", message(width=640), "camera_info_mismatch", {"width_delta_px": 8, "duration_s": 10}
    )
    assert mismatched.width == 648
    assert mismatch_readback["before"] == 640

    unavailable, tf_readback = adapter.transform_sensor_message(
        "front_rgb", message(frame="camera_color_optical_frame"), "tf_unavailable",
        {"frame": "camera_color_optical_frame", "duration_s": 10},
    )
    assert unavailable.header.frame_id == "formal_a19_missing_camera_color_optical_frame"
    assert tf_readback["action"] == "frame_rewritten_to_unavailable"

    invalid, invalid_readback = adapter.transform_sensor_message(
        "front_depth", message(data=b"\x01\x02\x03"), "invalid_depth",
        {"invalid_fraction": 1.0, "duration_s": 10},
    )
    assert invalid.data == b"\0\0\0"
    assert invalid_readback["after_nonzero_bytes"] == 0


def test_live_inner_pipeline_faults_require_strict_parameters() -> None:
    adapter.validate_fault_parameters("classifier_exception", {"exception_count": 1})
    adapter.validate_fault_parameters("proposal_flood", {"proposals_per_frame": 2048, "duration_s": 10})
    with pytest.raises(adapter.AdapterError, match="drop probability"):
        adapter.validate_fault_parameters("proposal_dropout", {"drop_probability": 0.5, "duration_s": 10})
    with pytest.raises(adapter.AdapterError, match="only an observed 1.0"):
        adapter.validate_fault_parameters("invalid_depth", {"invalid_fraction": 0.5, "duration_s": 10})
    adapter.validate_fault_parameters("nav2_path_unavailable", {"duration_s": 10})
    adapter.validate_fault_parameters("dynamic_obstacle_blocks_observation", {"duration_s": 15, "minimum_block_distance_m": 0.5})


@pytest.mark.parametrize("profile", ["transport_stress", "wet_surface", "degraded_drive"])
def test_frozen_non_nominal_profiles_mutate_real_proxy_ingress(profile: str, monkeypatch: pytest.MonkeyPatch) -> None:
    profiles = adapter.load_profile_settings(ROOT / adapter.PROFILE_CONFIG)
    monkeypatch.setattr(adapter.time, "sleep", lambda _: None)
    controller = adapter.SensorFaultController(); controller.set_profile(profile, profiles[profile])
    for _ in range(max(200, int(1.0 / profiles[profile]["sensor_dropout_probability"]))):
        controller.apply("front_rgb", message())
    readback = controller.profile_readback()
    assert readback is not None and readback["profile"] == profile
    assert readback["sensor_latency_ms"] == profiles[profile]["sensor_latency_ms"]
    assert readback["dropped_messages"] >= 1


def test_native_physical_profile_readback_requires_command_to_output_binding() -> None:
    profiles = adapter.load_profile_settings(ROOT / adapter.PROFILE_CONFIG)
    settings = profiles["wet_surface"]
    command = [2.0, 3.0, 2.0, 3.0]
    unscaled_torque = [4.0, 5.0, 4.0, 5.0]
    payload = {
        "profile": {
            "wheel_slip_ratio": settings["wheel_slip_ratio"],
            "actuator_gain": settings["actuator_gain"],
        },
        "commanded_wheel_speed_rad_s": command,
        "effective_wheel_speed_rad_s": [value / (1.0 - settings["wheel_slip_ratio"]) for value in command],
        "unscaled_wheel_torque_nm": unscaled_torque,
        "applied_wheel_torque_nm": [value * settings["actuator_gain"] for value in unscaled_torque],
        "measured_wheel_speed_rad_s": [1.0, 1.5, 1.0, 1.5],
    }
    readback = adapter.physical_profile_readback_from_status(payload, settings)
    assert readback["wheel_slip_ratio"] == 0.15
    assert readback["actuator_gain"] == 0.85
    payload["applied_wheel_torque_nm"][0] = 4.0
    with pytest.raises(adapter.AdapterError, match="actuator output"):
        adapter.physical_profile_readback_from_status(payload, settings)
    payload["applied_wheel_torque_nm"] = [value * settings["actuator_gain"] for value in unscaled_torque]
    payload["commanded_wheel_speed_rad_s"] = [0.0] * 4
    payload["effective_wheel_speed_rad_s"] = [0.0] * 4
    payload["unscaled_wheel_torque_nm"] = [0.0] * 4
    payload["applied_wheel_torque_nm"] = [0.0] * 4
    with pytest.raises(adapter.AdapterError, match="no live wheel command"):
        adapter.physical_profile_readback_from_status(payload, settings)
    contract = json.loads((ROOT / "config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json").read_text(encoding="utf-8"))
    blocked = adapter.capability_blockers(contract, profiles, ["CPUExecutionProvider"])
    assert blocked["unsupported_faults"] == ["cuda_provider_failure"]
    assert blocked["unsupported_profiles"] == {}


def test_adapter_forbids_profile_restart_and_operator_fault_spoofing() -> None:
    source = (ROOT / "scripts/formal_a19_product_adapter.py").read_text(encoding="utf-8")
    profile_branch = source[source.index('elif kind == "set_profile":'):source.index('elif kind == "inject_fault":')]
    fault_branch = source[source.index('elif kind == "inject_fault":'):source.index('elif kind == "shutdown":')]
    assert "stop_product()" not in profile_branch
    assert "command_drive_profile" in profile_branch
    assert "physical_profile_readback" in profile_branch
    assert "command_operator(" not in fault_branch
    assert "dynamic_blocker_recovery_readback" in fault_branch
    assert "read_named_model_pose" in source


def test_provider_and_model_hooks_use_real_files_but_never_mutate_them(tmp_path: Path) -> None:
    dosod, edgesam = tmp_path / "dosod.onnx", tmp_path / "edgesam.onnx"
    dosod.write_bytes(b"dosod-model")
    edgesam.write_bytes(b"edgesam-model")
    calls: list[Path] = []
    cpu_only = ProductFaultHooks(
        {"dosod": dosod, "edgesam": edgesam},
        provider_probe=lambda provider: {"available_providers": ["CPUExecutionProvider"], "selected_provider": None, "requested": provider},
        model_probe=lambda path: {"loader_accepted_shadow": True},
    )
    with pytest.raises(A19ProductFaultError, match="UNSUPPORTED cuda_provider_failure"):
        cpu_only.begin("cuda_provider_failure", {"provider": "CUDAExecutionProvider", "duration_s": 10})
    def model_probe(path: Path):
        calls.append(path)
        if path != dosod:
            raise RuntimeError("invalid protobuf")
        return {"loader_accepted_shadow": True}
    hooks = ProductFaultHooks(
        {"dosod": dosod, "edgesam": edgesam},
        provider_probe=lambda provider: {"available_providers": [provider, "CPUExecutionProvider"], "selected_provider": provider, "session_providers": [provider]},
        model_probe=model_probe,
    )
    original = {path: path.read_bytes() for path in (dosod, edgesam)}
    provider = hooks.begin("cuda_provider_failure", {"provider": "CUDAExecutionProvider", "duration_s": 10})
    assert provider["selected_provider"] == "CUDAExecutionProvider"
    with pytest.raises(RuntimeError, match="cuda_provider_failure"):
        hooks.before_inference()
    assert hooks.trigger_readback()["inference_path_triggered"] is True
    assert hooks.clear()["trigger_was_observed"] is True
    mismatch = hooks.begin("model_hash_mismatch", {"model": "dosod", "mismatch_count": 1})
    assert mismatch["actual_sha256"] != mismatch["expected_sha256"]
    hooks.clear()
    corrupt = hooks.begin("corrupt_model", {"model": "edgesam", "corrupt_bytes": 4})
    assert corrupt["shadow_only"] is True and corrupt["loader_error"] == "invalid protobuf" and calls and not calls[-1].exists()
    hooks.clear()
    hooks.begin("sustained_slow_inference", {"latency_ms": 1, "duration_s": 10})
    assert hooks.before_inference()["observed_delay_ms"] >= 1
    hooks.clear()
    assert {path: path.read_bytes() for path in (dosod, edgesam)} == original


def test_product_launch_threads_all_proxy_topics_into_the_pc_adapter() -> None:
    product = (ROOT / "starter_ws/src/sanitation_product_demo_integration/launch/product_demo.launch.py").read_text(encoding="utf-8")
    perception_launch = (ROOT / "starter_ws/src/sanitation_perception/launch/formal_pc_open_vocab.launch.py").read_text(encoding="utf-8")
    perception = (ROOT / "starter_ws/src/sanitation_perception/sanitation_perception/pc_open_vocab_adapter.py").read_text(encoding="utf-8")
    for name in {
        "front_rgb_topic", "front_depth_topic", "front_camera_info_topic",
        "wrist_rgb_topic", "wrist_depth_topic", "wrist_camera_info_topic",
        "rear_left_rgb_topic", "rear_right_rgb_topic",
    }:
        assert f'"{name}"' in perception_launch
        assert f'"{name}"' in perception
        assert f'"perception_{name}"' in product or "f\"perception_{name}\"" in product
    source = (ROOT / "scripts/formal_a19_product_adapter.py").read_text(encoding="utf-8")
    assert "process_group_rss_bytes" in source
    assert '"/odom/unfiltered"' in source
    assert '"capability_blocked"' in source
    assert '"unsupported_faults"' in source
    assert '"cuda_provider_failure"' in source
    assert '"/planner_server/change_state"' in source
    assert '"/compute_path_to_pose"' in source
    assert '"/world/{world}/set_pose"' in source
    assert '"/formal_a19/perception_fault"' in perception
    assert "ProductFaultHooks" in perception
    assert '"CameraInfo and RGB dimensions differ"' in perception
    assert '"depth image has no finite positive samples"' in perception
    assert adapter.DRIVETRAIN_PROFILE_TOPIC in source
    assert adapter.DRIVETRAIN_STATUS_TOPIC in source
