from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "starter_ws/src/sanitation_safety"


def test_final_contract_requires_the_strict_interlock_evidence_chain():
    contract = yaml.safe_load(
        (ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml")
        .read_text(encoding="utf-8")
    )
    required = contract["evidence_gates"]["whole_vehicle_interlock"][
        "required_values"
    ]
    expected = {
        "checks.managed_command_topics_have_single_gateway_writer",
        "checks.safety_input_topics_have_single_attributed_writer",
        "checks.status_json_samples_continuous_and_source_attributed",
        "checks.initial_inhibit_attributed_to_safety_relay_disabled",
        "checks.all_held_controller_goals_moved_before_collision",
        "checks.all_held_controller_goals_canceled_on_collision",
        "checks.held_joints_stable_after_collision",
        "checks.all_held_controller_goals_moved_before_rear_collision",
        "checks.all_held_controller_goals_canceled_on_rear_collision",
        "checks.held_joints_stable_after_rear_collision",
        "checks.all_held_controller_goals_moved_before_relay_false",
        "checks.all_held_controller_goals_canceled_on_relay_false",
        "checks.held_joints_stable_after_relay_false",
        "checks.all_held_controller_goals_moved_before_heartbeat_timeout",
        "checks.all_held_controller_goals_canceled_on_heartbeat_timeout",
        "checks.held_joints_stable_after_heartbeat_timeout",
        "checks.all_held_controller_goals_moved_before_relock",
        "checks.all_held_controller_goals_canceled_on_relock",
        "checks.held_joint_drift_within_threshold",
        "checks.final_physical_estop_attributed_to_manual_estop",
    }
    assert expected <= set(required)
    assert all(required[field] is True for field in expected)
    canceled = {
        "/arm_controller/follow_joint_trajectory": 5,
        "/cleaning_controller/follow_joint_trajectory": 5,
        "/gripper_controller/follow_joint_trajectory": 5,
        "/storage_controller/follow_joint_trajectory": 5,
    }
    assert required["emergency_stop_position_goal_statuses"] == canceled

    gate = contract["evidence_gates"]["whole_vehicle_interlock"]
    mapping_keys = gate["required_mapping_keys"]
    assert mapping_keys["hard_interlock_evidence"] == [
        "front_bumper_contact",
        "rear_bumper_contact",
        "safety_relay_false",
        "heartbeat_timeout",
    ]
    actions = list(canceled)
    joints = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        "cleaning_lift_joint",
        "robotiq_85_left_knuckle_joint",
        "dry_deposit_gate_joint",
    }
    assert mapping_keys["emergency_stop_position_goal_motion_from_start"] == actions
    assert set(mapping_keys["inhibited_joint_hold_evidence"]) == joints
    for phase in mapping_keys["hard_interlock_evidence"]:
        assert mapping_keys[
            f"hard_interlock_evidence.{phase}.position_goal_motion_from_start"
        ] == actions
        assert set(
            mapping_keys[
                f"hard_interlock_evidence.{phase}.inhibited_joint_hold_evidence"
            ]
        ) == joints
    phase_values = gate["required_mapping_item_values"]["hard_interlock_evidence"]
    assert phase_values == {
        "all_position_controller_goals_moved_before_trigger": True,
        "all_position_controller_goals_canceled": True,
        "all_held_joints_stable_after_cancel": True,
        "position_goal_result_statuses": canceled,
    }
    object_mappings = gate["required_mapping_item_values"]
    assert object_mappings["emergency_stop_position_goal_motion_from_start"] == {}
    assert object_mappings["inhibited_joint_hold_evidence"] == {}
    for phase in mapping_keys["hard_interlock_evidence"]:
        assert object_mappings[
            f"hard_interlock_evidence.{phase}.position_goal_motion_from_start"
        ] == {}
        assert object_mappings[
            f"hard_interlock_evidence.{phase}.inhibited_joint_hold_evidence"
        ] == {}


def test_package_declares_runtime_dependencies_for_enforced_controller_lifecycle():
    package_xml = (PACKAGE / "package.xml").read_text(encoding="utf-8")
    for dependency in (
        "action_msgs",
        "builtin_interfaces",
        "controller_manager_msgs",
        "sensor_msgs",
        "trajectory_msgs",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_xml


def test_integration_contract_preserves_actions_and_isolates_velocity_topics():
    contract = (PACKAGE / "README.md").read_text(encoding="utf-8")
    for safety_input in ("/safety/command/brush", "/safety/command/pump"):
        assert safety_input in contract
    assert "/base_controller/cmd_vel" in contract
    for action in (
        "/cleaning_controller/follow_joint_trajectory",
        "/arm_controller/follow_joint_trajectory",
        "/gripper_controller/follow_joint_trajectory",
    ):
        assert action in contract
    assert "only publisher" in contract
    assert "validate_whole_vehicle_actuator_interlock.py" in contract


def test_runtime_acceptance_checks_both_lock_edges_and_enabled_forwarding():
    source = (ROOT / "scripts/validate_whole_vehicle_actuator_interlock.py").read_text(
        encoding="utf-8"
    )
    assert '"/base_controller/cmd_vel"' in source
    for check in (
        "locked_base_zero",
        "locked_brush_zero",
        "locked_pump_zero",
        "enabled_brush_forwarded",
        "enabled_pump_forwarded",
        "enabled_base_forwarded",
        "managed_command_topics_have_single_gateway_writer",
        "standard_trajectory_actions_available_when_enabled",
        "collision_zeros_base_brush_and_pump",
        "collision_velocity_controllers_inactive",
        "active_arm_goal_canceled_on_collision",
        "active_cleaning_goal_canceled_on_collision",
        "active_gripper_goal_canceled_on_collision",
        "active_storage_goal_canceled_on_collision",
        "all_held_controller_goals_moved_before_collision",
        "all_held_controller_goals_canceled_on_collision",
        "held_joints_stable_after_collision",
        "rear_collision_zeros_base_brush_and_pump",
        "rear_collision_velocity_controllers_inactive",
        "active_arm_goal_canceled_on_rear_collision",
        "active_cleaning_goal_canceled_on_rear_collision",
        "active_gripper_goal_canceled_on_rear_collision",
        "active_storage_goal_canceled_on_rear_collision",
        "all_held_controller_goals_moved_before_rear_collision",
        "all_held_controller_goals_canceled_on_rear_collision",
        "held_joints_stable_after_rear_collision",
        "relay_false_zeros_base_brush_and_pump",
        "relay_false_velocity_controllers_inactive",
        "active_arm_goal_canceled_on_relay_false",
        "active_cleaning_goal_canceled_on_relay_false",
        "active_gripper_goal_canceled_on_relay_false",
        "active_storage_goal_canceled_on_relay_false",
        "all_held_controller_goals_moved_before_relay_false",
        "all_held_controller_goals_canceled_on_relay_false",
        "held_joints_stable_after_relay_false",
        "heartbeat_timeout_zeros_base_brush_and_pump",
        "heartbeat_timeout_velocity_controllers_inactive",
        "active_arm_goal_canceled_on_heartbeat_timeout",
        "active_cleaning_goal_canceled_on_heartbeat_timeout",
        "active_gripper_goal_canceled_on_heartbeat_timeout",
        "active_storage_goal_canceled_on_heartbeat_timeout",
        "all_held_controller_goals_moved_before_heartbeat_timeout",
        "all_held_controller_goals_canceled_on_heartbeat_timeout",
        "held_joints_stable_after_heartbeat_timeout",
        "active_arm_goal_canceled_on_relock",
        "active_cleaning_goal_canceled_on_relock",
        "active_gripper_goal_canceled_on_relock",
        "active_storage_goal_canceled_on_relock",
        "all_held_controller_goals_moved_before_relock",
        "all_held_controller_goals_canceled_on_relock",
        "held_joint_drift_within_threshold",
        "relock_brush_zero",
        "relock_pump_zero",
        "relock_base_zero",
    ):
        assert check in source


def test_runtime_acceptance_drives_and_records_each_hard_interlock_source():
    source = (ROOT / "scripts/validate_whole_vehicle_actuator_interlock.py").read_text(
        encoding="utf-8"
    )
    assert "Contacts(contacts=[Contact()] if rear_collision else [])" in source
    assert "Bool(data=relay_enabled)" in source
    assert "if send_heartbeat:" in source
    assert 'label="rear bumper contact"' in source
    assert "rear_collision=True" in source
    assert 'label="safety relay false"' in source
    assert "relay_enabled=False" in source
    assert 'label="heartbeat timeout"' in source
    assert "send_heartbeat=False" in source
    for evidence in (
        '"front_bumper_contact": front_collision_evidence',
        '"rear_bumper_contact": rear_collision_evidence',
        '"safety_relay_false": relay_disabled_evidence',
        '"heartbeat_timeout": heartbeat_timeout_evidence',
    ):
        assert evidence in source


def test_runtime_acceptance_attributes_physical_inputs_and_status_reasons():
    source = (ROOT / "scripts/validate_whole_vehicle_actuator_interlock.py").read_text(
        encoding="utf-8"
    )
    assert 'Bool, "/emergency_stop", 10' not in source
    assert '"/formal_vehicle/simulation/command/emergency_stop"' in source
    assert '"/formal_vehicle/power/bms_fault"' in source
    assert '"/formal_vehicle/power/traction_permitted"' in source
    assert "self.bms_fault.publish(Bool(data=False))" in source
    assert "self.traction_permitted.publish(Bool(data=True))" in source
    assert 'String, "/safety/status_json", self._on_status' in source
    assert "def assert_single_safety_input_writers" in source
    assert "def _status_reason_evidence" in source
    assert "MAXIMUM_STATUS_SAMPLE_GAP_SEC" in source
    assert 'target_reason="safety_relay_disabled"' in source
    assert 'target_reason="front_bumper_contact"' in source
    assert 'target_reason="rear_bumper_contact"' in source
    assert 'target_reason="heartbeat_timeout"' in source
    assert 'target_reason="manual_estop"' in source
    assert 'ALLOWED_ADDITIONAL_STATUS_REASONS = {"manipulator_base_inhibit"}' in source
    assert "for stamp, permitted in node.permit_samples" in source
    assert '"actuator_permit_sample_count": len(permit_samples)' in source
    assert (
        "locked_start = node.drive_phase(\n"
        "            estop=False,\n"
        "            relay_enabled=False,"
    ) in source
    assert "settle_sec=1.0" in source
    assert "MINIMUM_PHASE_DURATION_SEC = 1.25" in source
    assert "args.phase_duration < MINIMUM_PHASE_DURATION_SEC" in source
    assert "status_json_samples_continuous_and_source_attributed" in source
    assert '"safety_input_writer_evidence": safety_input_writer_evidence' in source
    assert '"published_healthy_safety_inputs": {' in source


def test_status_and_permit_streams_cover_the_complete_settled_window():
    source = (ROOT / "scripts/validate_whole_vehicle_actuator_interlock.py").read_text(
        encoding="utf-8"
    )
    for marker in (
        "def _periodic_window_evidence(",
        '"first_arrival_gap_sec"',
        '"final_arrival_gap_sec"',
        '"status_window_continuity"',
        '"permit_window_continuity"',
        "MAXIMUM_PERMIT_SAMPLE_GAP_SEC",
        "window_end = time.monotonic()",
    ):
        assert marker in source
    assert "maximum_timer_gap_sec is invalid or exceeds" in source
    assert "consumed_generation > unsafe_generation" in source
    assert "consumed_unsafe_generations[-1] != unsafe_generations[-1]" in source


def test_blocking_discovery_and_controller_queries_preserve_safety_inputs():
    source = (ROOT / "scripts/validate_whole_vehicle_actuator_interlock.py").read_text(
        encoding="utf-8"
    )
    assert "def wait_for_action_server_healthy(" in source
    assert "def wait_for_service_with_inputs(" in source
    assert "timeout_sec=min(0.10, max(0.0, remaining))" in source
    assert "self.publish_inputs(" in source
    assert "self.wait_future(future, timeout_sec=timeout_sec, **input_arguments)" in source
    assert "collision=collision" in source
    assert "rear_collision=rear_collision" in source
    assert "relay_enabled=relay_enabled" in source
    assert "send_heartbeat=send_heartbeat" in source


def test_writer_attribution_compares_fully_qualified_node_names():
    source = (ROOT / "scripts/validate_whole_vehicle_actuator_interlock.py").read_text(
        encoding="utf-8"
    )
    assert "def _endpoint_fqn(namespace: str, name: str)" in source
    assert "self.get_fully_qualified_name()" in source
    assert 'expected_node = "/whole_vehicle_safety_manager"' in source
    assert '"/emergency_stop": "/formal_auxiliary_bridge"' in source
    assert "writers != [expected_node]" in source


def test_emergency_stop_cancels_live_goals_on_every_held_position_controller():
    source = (ROOT / "scripts/validate_whole_vehicle_actuator_interlock.py").read_text(
        encoding="utf-8"
    )
    for action in (
        "/cleaning_controller/follow_joint_trajectory",
        "/arm_controller/follow_joint_trajectory",
        "/gripper_controller/follow_joint_trajectory",
        "/storage_controller/follow_joint_trajectory",
    ):
        assert action in source
    assert "start_all_live_position_goals" in source
    assert "wait_for_all_live_position_goal_motion" in source
    assert "wait_for_position_goal_cancellations" in source
    assert "send_futures[action_name] = client.send_goal_async(goal)" in source
    assert "initial_positions = {" in source
    assert "position_goal_statuses" in source
    assert "GoalStatus.STATUS_CANCELED" in source
    assert '"emergency_stop_position_goal_motion_from_start"' in source
    assert '"emergency_stop_position_goal_statuses"' in source


def test_cleaning_lift_cancel_uses_measured_state_deceleration():
    controllers = yaml.safe_load(
        (
            ROOT
            / "starter_ws/src/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml"
        ).read_text(encoding="utf-8")
    )
    constraints = controllers["cleaning_controller"]["ros__parameters"][
        "constraints"
    ]
    assert constraints["decelerate_on_cancel"] is True
    assert constraints["cleaning_lift_joint"]["max_deceleration_on_cancel"] == 0.05

    cleaning_xacro = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro"
    ).read_text(encoding="utf-8")
    assert '<limit lower="0.0" upper="0.10002" effort="300.0" velocity="0.0048"/>' in cleaning_xacro
    assert '<dynamics damping="80.0" friction="80.0"/>' in cleaning_xacro
    assert "5.95 kg (58.35 N under gravity)" in cleaning_xacro
    assert "solver bound 20 um beyond the product's 100 mm travel" in cleaning_xacro
    assert "ros2_control position interface remains capped at" in cleaning_xacro
    control_xacro = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/control_interfaces.xacro"
    ).read_text(encoding="utf-8")
    assert (
        '<xacro:hf_position_joint name="cleaning_lift_joint" lower="0.0" '
        'upper="0.100" velocity="0.0048" effort="300.0" initial_position="0.0"/>'
    ) in control_xacro

    core = (
        PACKAGE / "sanitation_safety/whole_vehicle_safety_core.py"
    ).read_text(encoding="utf-8")
    manager = (
        PACKAGE / "sanitation_safety/whole_vehicle_safety_manager.py"
    ).read_text(encoding="utf-8")
    assert 'SAFETY_NATIVE_CANCEL_HOLD_CONTROLLERS = ("cleaning_controller",)' in core
    assert manager.count("if controller in SAFETY_NATIVE_CANCEL_HOLD_CONTROLLERS:") >= 3


def test_each_hard_interlock_proves_all_four_goals_moved_canceled_and_held():
    source = (ROOT / "scripts/validate_whole_vehicle_actuator_interlock.py").read_text(
        encoding="utf-8"
    )
    phase = source[source.index("def _run_hard_interlock_phase(") : source.index("def main()")]
    for call in (
        "start_all_live_position_goals",
        "wait_for_all_live_position_goal_motion",
        "wait_for_position_goal_cancellations",
        "_joint_hold_evidence",
    ):
        assert call in phase
    for evidence in (
        '"position_goal_motion_from_start"',
        '"pre_trigger_joint_positions"',
        '"position_goal_result_statuses"',
        '"inhibited_joint_hold_evidence"',
        '"all_position_controller_goals_moved_before_trigger"',
        '"all_position_controller_goals_canceled"',
        '"all_held_joints_stable_after_cancel"',
    ):
        assert evidence in phase
    assert "_inhibit_transition_joint_reference" in phase
    assert "if send_heartbeat:" in phase
    assert "reference_positions=hold_reference_joints" in phase
    assert '"hold_reference_joint_positions"' in phase
    assert '"hold_reference_evidence"' in phase
    assert '"mode": "last_joint_sample_before_actuator_permit_revocation"' in source
    assert '"max_abs_from_trigger"' in source
    assert "reference_positions=pre_estop_joints" in source
    for joint, threshold in (
        ("shoulder_pan_joint", "0.005"),
        ("cleaning_lift_joint", "0.00025"),
        ("robotiq_85_left_knuckle_joint", "0.01"),
        ("dry_deposit_gate_joint", "0.02"),
    ):
        assert f'{{"{joint}": {threshold}}}' in source


def test_formal_launch_orders_safe_controller_loading_before_gateway_start():
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    package_xml = (
        ROOT / "starter_ws/src/sanitation_vehicle_description/package.xml"
    ).read_text(encoding="utf-8")
    assert '"enable_safety_manager",' in launch
    assert 'default_value="true"' in launch
    assert '"brush_controller", "recovery_controller"' in launch
    assert '"--inactive"' in launch
    assert "OnProcessExit(" in launch
    assert "target_action=safe_active_controller_spawner" in launch
    assert "target_action=safe_velocity_controller_loader" in launch
    assert 'executable="whole_vehicle_safety_manager"' in launch
    assert 'executable="service_drain_safety_manager"' in launch
    assert "on_exit=[safety_manager, service_drain_safety_manager]" in launch
    assert "<exec_depend>sanitation_safety</exec_depend>" in package_xml


def test_service_drain_is_installed_and_has_fixed_zero_inhibit_target():
    setup = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    core = (
        PACKAGE / "sanitation_safety/whole_vehicle_safety_core.py"
    ).read_text(encoding="utf-8")
    manager = (
        PACKAGE / "sanitation_safety/service_drain_manager.py"
    ).read_text(encoding="utf-8")
    assert "service_drain_safety_manager = " in setup
    assert '"service_controller": {"wastewater_drain_valve_joint": 0.0}' in core
    assert '"/safety/actuators_enabled"' in manager
    assert '"/service_controller/joint_trajectory"' in manager
    assert 'CAP_JOINT = "wastewater_drain_service_cap_joint"' in manager
    assert '"/formal_vehicle/service/raw/drain_hose_contact"' in manager
    assert "bool(message.contacts)" in manager
    assert 'self._core.update(\n                "cap_open"' in manager


def test_deployed_arm_inhibits_base_without_cutting_manipulator_power():
    core = (
        PACKAGE / "sanitation_safety/whole_vehicle_safety_core.py"
    ).read_text(encoding="utf-8")
    manager = (
        PACKAGE / "sanitation_safety/whole_vehicle_safety_manager.py"
    ).read_text(encoding="utf-8")
    assert "MANIPULATOR_BASE_INHIBIT" in core
    assert "and not self.base_motion_inhibited" in core
    assert '"/manipulation/base_motion_inhibited"' in manager
    assert "ARM_STOWED_POSITIONS" in manager
    assert "not self._arm_is_stowed()" in manager


def test_runtime_runner_uses_the_fail_closed_formal_launch_path():
    source = (
        ROOT / "scripts/run_whole_vehicle_actuator_interlock_runtime.sh"
    ).read_text(encoding="utf-8")
    assert "enable_safety_manager:=true" in source
    assert "start_simulation_safety_inputs:=false" in source
    assert "start_power_system_simulators:=false" in source
    assert "whole_vehicle_safety_manager" in source
    assert "validate_whole_vehicle_actuator_interlock.py" in source
    assert "--output" in source
    assert "formal_runtime_gate_binding.py" in source
    assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in source
    assert "FORMAL_ACCEPTANCE_SESSION" in source
    assert "ros2 topic echo /safety/actuators_enabled" in source
    assert "ros2 topic echo /joint_states" in source
    assert "timeout 20s ros2 node list --no-daemon --spin-time 3.0" in source
    assert "timeout 20s ros2 service list -t --no-daemon --spin-time 3.0" in source
    assert source.count("--once --no-daemon --spin-time 3.0 --timeout 4") == 2
    assert "def bounded_ros_probe" not in source
    assert "bounded_ros_probe()" in source
    assert "result != 0 && result != 124" in source
    assert "grep -Eq '^data: (true|false)$'" in source
    assert "grep -qx 'name:'" in source
    assert "ros2 service type /controller_manager/list_controllers" not in source
    assert "enable_safety_manager:=false" not in source
    assert "simulation_initial_estop_active:=false" in source
    assert '"${runtime_binding}" "${launch_log}"' in source
    assert 'kill -0 "${launch_pid}"' in source
    assert 'ps -o stat= -p "${launch_pid}"' in source
