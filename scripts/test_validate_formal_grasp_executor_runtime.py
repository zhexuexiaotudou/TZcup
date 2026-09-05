from pathlib import Path


SOURCE = (
    Path(__file__).with_name("validate_formal_grasp_executor_runtime.py")
).read_text(encoding="utf-8")
RUNNER = Path(__file__).with_name("run_formal_grasp_executor_runtime.sh").read_text(
    encoding="utf-8"
)
GRASP_LAUNCH = Path(
    "starter_ws/src/sanitation_manipulation/launch/formal_cube_pick_place.launch.py"
).read_text(encoding="utf-8")


def test_runtime_probe_has_no_simulator_truth_or_direct_actuator_authority():
    assert "SetEntityPose" not in SOURCE
    assert "Entity.MODEL" not in SOURCE
    assert '"model_name"' not in SOURCE
    assert "/arm_controller" not in SOURCE
    assert "/gripper_controller" not in SOURCE
    assert "/storage_controller" not in SOURCE
    assert "/manipulation/grasp/attach" not in SOURCE
    assert "/manipulation/grasp/detach" not in SOURCE
    assert 'Bool, "/safety/actuators_enabled", self._safety_permit' in SOURCE
    assert 'create_publisher(Bool, "/safety/actuators_enabled"' not in SOURCE
    assert '"/odom"' not in SOURCE
    assert "/formal_vehicle/simulation/command/emergency_stop" in SOURCE
    assert "/formal_vehicle/simulation/command/main_power" in SOURCE
    assert '"/cmd_vel_gate"' in SOURCE


def test_runtime_probe_accepts_only_verified_in_bin_result():
    assert 'node.result.get("verified_in_bin") is True' in SOURCE
    assert '"truth_used": False' in SOURCE
    assert '"schema_version": 2' in SOURCE
    assert '"size_m": [0.030, 0.030, 0.030]' in SOURCE
    assert '"material": "unknown"' in SOURCE
    assert '"frame_id": "base_footprint"' in SOURCE
    assert '"/perception/wrist/grasp_recheck"' in SOURCE
    assert "FORMAL_PRODUCT_GRASP_AND_DRY_BIN_ACCEPTANCE_PASSED" in SOURCE


def test_runtime_probe_requires_and_records_fresh_final_runtime_identity():
    assert 'parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)' in SOURCE
    assert 'parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)' in SOURCE
    assert 'parser.add_argument("--runtime-binding", required=True, type=Path)' in SOURCE
    assert "runtime_binding = load_binding(binding_path)" in SOURCE
    assert "formal grasp runtime requires the canonical vehicle snapshot" in SOURCE
    assert "runtime binding is not fresh for the active acceptance session" in SOURCE
    assert "sanitation_manipulation resolves outside the bound frozen overlay" in SOURCE
    assert '"runtime_identity": runtime_identity' in SOURCE
    assert '"runtime_gate_binding_sha256"' in SOURCE
    assert '"runtime_closure"' in SOURCE
    assert '"active_overlay_matches_runtime_binding": True' in SOURCE
    assert "refusing to overwrite retained grasp runtime report" in SOURCE
    assert 'parser.add_argument("--preembedded-report", required=True, type=Path)' in SOURCE
    assert 'parser.add_argument("--preembedded-world", required=True, type=Path)' in SOURCE
    assert 'parser.add_argument("--preembedded-vehicle-urdf", required=True, type=Path)' in SOURCE
    assert 'parser.add_argument("--preembedded-cube-urdf", required=True, type=Path)' in SOURCE
    assert 'parser.add_argument("--preembedded-source-world", required=True, type=Path)' in SOURCE
    assert "validate_preembedded_grasp_world" in SOURCE
    assert '"preembedded_grasp_world_binding": preembedded_grasp' in SOURCE


def test_runner_starts_physics_bridge_and_product_executor_as_separate_surfaces():
    assert "formal_cube_pick_place.launch.py" in RUNNER
    assert "FORMAL_MANIPULATION_CUBE_NAME" in RUNNER
    assert "formal_physical_grasp.launch.py" in RUNNER
    assert 'executable="manipulation_sim_bridge"' in GRASP_LAUNCH
    assert "dry_bin/observed_status_json" not in RUNNER
    assert "bridge_pid" not in RUNNER
    assert "validate_formal_grasp_executor_runtime.py" in RUNNER
    assert "FORMAL_GRASP_EXECUTOR_RUNTIME_BINDING" in RUNNER
    assert '${output}.runtime_binding.json' in RUNNER
    assert 'for retained in "${output}" "${runtime_binding}" "${launch_log}"' in RUNNER
    assert '[[ -e "${retained}" || -L "${retained}" ]]' in RUNNER
    assert "formal_runtime_gate_binding.py" in RUNNER
    assert "generate_formal_vehicle_snapshot.py" in RUNNER
    assert 'formal_source_bound_verify_overlay "${runtime_ws}"' in RUNNER
    assert '--snapshot "${snapshot}" --session "${session}" --runtime-binding "${runtime_binding}"' in RUNNER
    assert RUNNER.index('for retained in "${output}" "${runtime_binding}" "${launch_log}"') < RUNNER.index(
        "source /opt/ros/jazzy/setup.bash"
    )
    assert RUNNER.index("formal_runtime_install_traps cleanup") < RUNNER.index(
        "ros2 launch sanitation_manipulation formal_cube_pick_place.launch.py"
    )
    assert "whole_vehicle_safety_manager" in GRASP_LAUNCH


def test_grasp_runner_preembeds_both_contact_models_and_archives_them():
    assert "prepare_formal_preembedded_sensor_world.py" in RUNNER
    assert '--preembedded-vehicle-urdf "${preembedded_vehicle_urdf}"' in RUNNER
    assert '--preembedded-cube-urdf "${preembedded_cube_urdf}"' in RUNNER
    assert '--preembedded-source-world "${source_world}"' in RUNNER
    assert '--additional-urdf "${preembedded_cube_urdf}"' in RUNNER
    assert 'world:="${preembedded_world}"' in RUNNER
    assert "spawn_vehicle:=false spawn_single_cube:=false" in RUNNER
    assert '"${preembedded_world}" "${preembedded_report}"' in RUNNER
    assert '"${preembedded_vehicle_urdf}" "${preembedded_cube_urdf}"' in RUNNER
    assert '--preembedded-report "${preembedded_report}" --preembedded-world "${preembedded_world}"' in RUNNER
    launch = Path(
        "starter_ws/src/sanitation_manipulation/launch/formal_cube_pick_place.launch.py"
    ).read_text(encoding="utf-8")
    for argument in ("world", "vehicle_model", "cube_model", "spawn_vehicle"):
        assert f'DeclareLaunchArgument("{argument}"' in launch
    assert "condition=IfCondition(spawn_vehicle)" in launch
