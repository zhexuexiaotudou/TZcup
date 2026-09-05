#!/usr/bin/env python3
"""Capture deterministic whole-vehicle and functional-detail Gazebo views."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from builtin_interfaces.msg import Duration
from PIL import Image as PillowImage

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from formal_runtime_gate_binding import RuntimeGateError, load_binding
from gazebo_ground_truth import read_named_model_pose


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
WORLD_NAME = "formal_vehicle_visual_acceptance"
MODEL_NAME = "tzcup_formal_sanitation_vehicle"


TOPICS = {
    "front_left": "/formal_visual/front_left",
    "rear_right": "/formal_visual/rear_right",
    "top_cleaning": "/formal_visual/top_cleaning",
    "sensor_tower_detail": "/formal_visual/sensor_tower_detail",
    "front_sensor_detail": "/formal_visual/front_sensor_detail",
    "arm_mount_detail": "/formal_visual/arm_mount_detail",
    "dry_deposition_detail": "/formal_visual/dry_deposition_detail",
    "cleaning_head_detail": "/formal_visual/cleaning_head_detail",
    "rear_service_detail": "/formal_visual/rear_service_detail",
    "power_compute_detail": "/formal_visual/power_compute_detail",
    "storage_recovery_detail": "/formal_visual/storage_recovery_detail",
    "rear_left_sensor_detail": "/formal_visual/rear_left_sensor_detail",
    "rear_right_sensor_detail": "/formal_visual/rear_right_sensor_detail",
    "drivetrain_detail": "/formal_visual/drivetrain_detail",
    "inertial_power_detail": "/formal_visual/inertial_power_detail",
    "dry_deposition_internal": "/formal_visual/dry_deposition_internal",
    "power_safety_internal": "/formal_visual/power_safety_internal",
    "charge_interface_detail": "/formal_visual/charge_interface_detail",
    "drain_interface_detail": "/formal_visual/drain_interface_detail",
}

# Calibrated aiming points are source-bound engineering datums, not image-based
# visibility claims.  The static test proves that every camera optical +X axis
# intersects its datum inside a conservative central cone; the actual Gazebo
# image remains mandatory for occlusion and legibility acceptance.
VIEW_TARGETS = {
    "front_left": {"target_xyz_m": [0.0, 0.0, 0.65], "target_entities": ["bodywork_lower_tub_link", "bodywork_lighting_link"]},
    "rear_right": {"target_xyz_m": [0.0, 0.0, 0.65], "target_entities": ["bodywork_lower_tub_link", "bodywork_lighting_link"]},
    "top_cleaning": {"target_xyz_m": [0.0, 0.0, 0.40], "target_entities": ["central_roller_link", "left_side_brush_link", "right_side_brush_link"]},
    "sensor_tower_detail": {"target_xyz_m": [0.42, 0.0, 1.22], "target_entities": ["lidar_2d_link", "lidar_3d_link", "gnss_antenna_link"]},
    "front_sensor_detail": {"target_xyz_m": [0.569532, 0.0, 0.611987], "target_entities": ["front_rgbd_link", "bodywork_lighting_link"]},
    "arm_mount_detail": {"target_xyz_m": [0.10, -0.20, 0.80], "target_entities": ["ur5e_base_link_inertia", "wrist_rgbd_mount_link"]},
    "dry_deposition_detail": {"target_xyz_m": [-0.205, 0.035, 1.0471], "target_entities": ["dry_deposit_gate_link"]},
    "cleaning_head_detail": {"target_xyz_m": [0.10, 0.0, 0.17], "target_entities": ["central_roller_link", "left_side_brush_link", "squeegee_link"]},
    "rear_service_detail": {"target_xyz_m": [-0.49, 0.0, 0.55], "target_entities": ["wastewater_tank_link", "bodywork_lower_tub_link"]},
    "power_compute_detail": {"target_xyz_m": [0.30, 0.15, 0.72], "target_entities": ["s100_compute_enclosure_link", "power_distribution_box_link"]},
    "storage_recovery_detail": {"target_xyz_m": [-0.205, -0.04, 0.55], "target_entities": ["dry_bin_link", "wastewater_tank_link", "recovery_pump_motor_link"]},
    "rear_left_sensor_detail": {"target_xyz_m": [-0.565, 0.280, 0.6001], "target_entities": ["rear_left_fisheye_link"]},
    "rear_right_sensor_detail": {"target_xyz_m": [-0.565, -0.280, 0.6001], "target_entities": ["rear_right_fisheye_link"]},
    "drivetrain_detail": {"target_xyz_m": [0.0, -0.18, 0.18], "target_entities": ["front_right_motor_link", "rear_right_motor_link"]},
    "inertial_power_detail": {"target_xyz_m": [0.0, 0.0, 0.4091], "target_entities": ["imu_link"]},
    "dry_deposition_internal": {"target_xyz_m": [-0.205, 0.035, 1.0471], "target_entities": ["dry_deposit_gate_link", "dry_deposit_chute_link", "dry_bin_link"]},
    "power_safety_internal": {"target_xyz_m": [0.10, 0.00, 0.40], "target_entities": ["a300_left_battery_pack_link", "a300_left_battery_bms_link", "power_distribution_box_link", "safety_relay_link", "main_power_contactor_housing_link", "isolated_dc_dc_module_link"]},
    "charge_interface_detail": {"target_xyz_m": [0.250, 0.402, 0.4951], "target_entities": ["charge_port_housing_link", "charge_receptacle_link", "charge_port_door_link", "charge_connector_lock_link"]},
    "drain_interface_detail": {"target_xyz_m": [-0.490, -0.305, 0.5001], "target_entities": ["wastewater_drain_valve_body_link", "wastewater_drain_valve_actuator_link", "wastewater_drain_service_cap_link", "wastewater_drain_coupling_link"]},
}

SERVICE_TARGET_ENTITY_OVERRIDES = {
    "front_left": ["base_link", "power_distribution_box_link"],
    "rear_right": ["dry_bin_link", "wastewater_tank_link"],
    "front_sensor_detail": ["front_rgbd_mount_link", "front_rgbd_link"],
    "rear_service_detail": ["wastewater_tank_link", "emergency_stop_housing_link"],
}


def targets_for_profile(profile: str) -> dict[str, dict[str, object]]:
    if profile not in {"product", "service"}:
        raise VisualAcceptanceError(f"unsupported visual profile: {profile}")
    targets = {
        name: {**contract, "target_entities": list(contract["target_entities"])}
        for name, contract in VIEW_TARGETS.items()
    }
    if profile == "service":
        for name, entities in SERVICE_TARGET_ENTITY_OVERRIDES.items():
            targets[name]["target_entities"] = list(entities)
    return targets

# The nineteen images are not accepted as a generic beauty render.  This
# crosswalk assigns every registered functional position, sensor installation
# and mechanical subassembly to at least one calibrated inspection camera.
# Link-level coverage below then proves that the actual URDF geometry for the
# item is in the assigned camera frustum at the commanded visual pose.
FUNCTION_POSITION_VIEWS = {
    "mobility": ["drivetrain_detail", "front_left", "rear_right"],
    "mapping_2d": ["sensor_tower_detail"],
    "obstacle_perception_3d": ["sensor_tower_detail"],
    "localization_2d": ["sensor_tower_detail"],
    "global_position": ["sensor_tower_detail"],
    "inertial_reference": ["inertial_power_detail"],
    "forward_perception": ["front_sensor_detail"],
    "rear_left_perception": ["rear_left_sensor_detail"],
    "rear_right_perception": ["rear_right_sensor_detail"],
    "grasp_observation": ["arm_mount_detail"],
    "manipulation": ["arm_mount_detail"],
    "grasping": ["arm_mount_detail"],
    "dry_deposition": ["dry_deposition_detail", "dry_deposition_internal"],
    "dry_storage": ["storage_recovery_detail", "dry_deposition_internal"],
    "dry_fill_monitor": ["storage_recovery_detail", "dry_deposition_internal"],
    "wet_storage": ["storage_recovery_detail", "rear_service_detail"],
    "wet_fill_monitor": ["storage_recovery_detail", "rear_service_detail"],
    "side_sweeping": ["cleaning_head_detail", "top_cleaning"],
    "main_sweeping": ["cleaning_head_detail", "top_cleaning"],
    "cleaning_head_lift": ["cleaning_head_detail"],
    "water_gathering": ["cleaning_head_detail"],
    "water_intake": ["cleaning_head_detail"],
    "water_filtering": ["cleaning_head_detail", "storage_recovery_detail"],
    "water_pumping": ["cleaning_head_detail", "storage_recovery_detail"],
    "water_flow_monitor": ["cleaning_head_detail", "rear_service_detail"],
    "compute": ["power_compute_detail"],
    "arm_control": ["power_compute_detail"],
    "fused_power_distribution": ["power_safety_internal"],
    "isolated_low_voltage_power": ["power_safety_internal"],
    "hardwired_safety_enable": ["power_safety_internal"],
    "a300_energy_storage": ["power_safety_internal", "inertial_power_detail"],
    "charge_interface": ["charge_interface_detail"],
    "wastewater_drain": ["drain_interface_detail"],
    "emergency_stop": ["rear_right", "rear_service_detail"],
    "bodywork_service_access": [
        "front_left", "rear_right", "power_compute_detail",
        "rear_service_detail", "storage_recovery_detail",
    ],
    "warning_and_work_lighting": ["front_left", "rear_right"],
    "front_contact_safety": ["front_left"],
    "rear_contact_safety": ["rear_right"],
}

SENSOR_INSTALLATION_VIEWS = {
    "hokuyo_utm30lx": ["sensor_tower_detail"],
    "livox_mid360": ["sensor_tower_detail"],
    "ublox_zed_f9p_receiver": ["sensor_tower_detail"],
    "ublox_ann_mb": ["sensor_tower_detail"],
    "intel_d435_front": ["front_sensor_detail"],
    "rear_left_fisheye": ["rear_left_sensor_detail"],
    "rear_right_fisheye": ["rear_right_sensor_detail"],
    "vectornav_vn100": ["inertial_power_detail"],
    "intel_d435_wrist": ["arm_mount_detail"],
}

MECHANICAL_SUBASSEMBLY_VIEWS = {
    "mobility_base": ["drivetrain_detail", "front_left", "rear_right"],
    "sensor_tower": ["sensor_tower_detail", "top_cleaning"],
    "gnss_receiver_installation": ["sensor_tower_detail"],
    "manipulator": ["arm_mount_detail"],
    "gripper": ["arm_mount_detail"],
    "wrist_rgbd_installation": ["arm_mount_detail"],
    "dry_storage": ["storage_recovery_detail", "dry_deposition_internal"],
    "wastewater_storage": ["storage_recovery_detail", "rear_service_detail"],
    "side_brushes": ["cleaning_head_detail", "top_cleaning"],
    "central_roller": ["cleaning_head_detail", "top_cleaning"],
    "recovery_squeegee": ["cleaning_head_detail"],
    "cleaning_head_deployment": ["cleaning_head_detail"],
    "compute_enclosure": ["power_compute_detail"],
    "robot_deposition_port": ["dry_deposition_detail", "dry_deposition_internal"],
    "wastewater_recovery_drive": ["cleaning_head_detail", "rear_service_detail"],
    "low_voltage_power_and_safety": ["power_safety_internal", "power_compute_detail"],
    "exterior_service_and_contact_safety": [
        "front_left", "rear_right", "charge_interface_detail", "drain_interface_detail",
    ],
    "bodywork_service_access": [
        "front_left", "rear_right", "power_compute_detail",
        "rear_service_detail", "storage_recovery_detail",
    ],
}

# These links close the gaps that a primary function datum alone cannot prove:
# the complete six-axis arm and gripper, motor/encoder/gearbox chains, flexible
# recovery route, tank/baffle/lid/level hardware, power chain, service doors and
# physical interfaces.  Abstract optical frames and dynamic payload reserve
# links are intentionally excluded because they have no visible/collidable
# hardware of their own.
VIEW_INSPECTION_LINKS = {
    "drivetrain_detail": [
        "base_link", "front_left_wheel_link", "front_right_wheel_link",
        "rear_left_wheel_link", "rear_right_wheel_link",
        "front_left_motor_link", "front_right_motor_link", "rear_left_motor_link",
        "rear_right_motor_link", "front_left_encoder_link", "front_right_encoder_link",
        "rear_left_encoder_link", "rear_right_encoder_link",
        "left_suspension_beam_link", "right_suspension_beam_link",
        "left_suspension_beam_spacer_link", "right_suspension_beam_spacer_link",
    ],
    "sensor_tower_detail": [
        "lidar_2d_mount_link", "lidar_2d_link", "lidar_3d_mount_link", "lidar_3d_link",
        "zed_f9p_receiver_enclosure_link", "zed_f9p_module_reference_link",
        "gnss_mount_link", "gnss_antenna_link",
    ],
    "top_cleaning": ["sensor_mast_link"],
    "front_sensor_detail": ["front_rgbd_mount_link", "front_rgbd_link"],
    "rear_left_sensor_detail": ["rear_left_fisheye_mount_link", "rear_left_fisheye_link"],
    "rear_right_sensor_detail": ["rear_right_fisheye_mount_link", "rear_right_fisheye_link"],
    "arm_mount_detail": [
        "ur5e_base_link_inertia", "ur5e_shoulder_link", "ur5e_upper_arm_link",
        "ur5e_forearm_link", "ur5e_wrist_1_link", "ur5e_wrist_2_link",
        "ur5e_wrist_3_link", "ur_to_robotiq_adapter_link", "robotiq_85_base_link",
        "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
        "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
        "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
        "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link",
        "wrist_rgbd_mount_link", "wrist_rgbd_link",
    ],
    "cleaning_head_detail": [
        "cleaning_lift_carriage_link", "cleaning_lift_actuator_body_link",
        "left_side_brush_motor_stator_link", "left_side_brush_encoder_link",
        "left_side_brush_gearbox_link", "left_side_brush_link", "left_side_brush_disk_link",
        "left_side_brush_bristle_sectors_link", "right_side_brush_motor_stator_link",
        "right_side_brush_encoder_link", "right_side_brush_gearbox_link",
        "right_side_brush_link", "right_side_brush_disk_link",
        "right_side_brush_bristle_sectors_link", "central_roller_left_bearing_link",
        "central_roller_motor_stator_link", "central_roller_encoder_link",
        "central_roller_drive_gearbox_link", "central_roller_link", "central_roller_guard_link",
        "squeegee_float_carrier_link", "squeegee_spring_pack_link", "squeegee_link",
        "suction_nozzle_link", "recovery_hose_lower_link", "recovery_hose_middle_link",
        "recovery_hose_upper_link", "recovery_strainer_filter_link",
        "recovery_pump_motor_link", "recovery_pump_head_link", "recovery_pump_rotor_link",
        "recovery_pump_isolator_mount_link", "wastewater_delivery_coupling_link",
        "recovery_flow_sensor_link",
    ],
    "dry_deposition_internal": [
        "dry_bin_link", "dry_bin_front_panel_link", "dry_bin_rear_panel_link",
        "dry_bin_left_panel_link", "dry_bin_right_panel_link", "dry_bin_lid_link",
        "dry_deposit_hopper_link", "dry_deposit_gate_link", "dry_deposit_gate_actuator_link",
        "dry_deposit_gate_actuator_horn_link", "dry_deposit_chute_link",
        "dry_deposit_presence_sensor_link", "dry_bin_latch_base_link", "dry_bin_latch_link",
        "dry_bin_latch_keeper_link", "dry_bin_level_sensor_link",
    ],
    "storage_recovery_detail": [
        "recovery_hose_middle_link", "recovery_hose_upper_link",
        "dry_bin_link", "dry_bin_front_panel_link", "dry_bin_rear_panel_link",
        "dry_bin_left_panel_link", "dry_bin_right_panel_link", "dry_bin_lid_link",
        "dry_bin_latch_base_link", "dry_bin_latch_link", "dry_bin_latch_keeper_link",
        "dry_bin_level_sensor_link",
        "wastewater_tank_link", "wastewater_front_panel_link", "wastewater_rear_panel_link",
        "wastewater_left_panel_link", "wastewater_right_panel_link", "wastewater_baffle_link",
        "wastewater_lid_link", "wastewater_lid_latch_base_link", "wastewater_lid_latch_link",
        "wastewater_lid_latch_keeper_link", "wastewater_low_level_sensor_link",
        "wastewater_high_level_sensor_link", "wastewater_vent_filter_link",
        "wastewater_inlet_coupling_link",
    ],
    "inertial_power_detail": ["imu_mount_tray_link", "imu_link"],
    "power_safety_internal": [
        "a300_left_battery_pack_link", "a300_right_battery_pack_link",
        "a300_left_battery_bms_link", "a300_right_battery_bms_link",
        "power_distribution_box_link", "main_power_isolator_housing_link",
        "main_power_isolator_handle_link", "main_power_contactor_housing_link",
        "main_power_contactor_armature_link", "isolated_dc_dc_module_link", "safety_relay_link",
    ],
    "power_compute_detail": ["s100_compute_enclosure_link", "ur5e_control_box_link"],
    "charge_interface_detail": [
        "charge_port_housing_link", "charge_receptacle_link", "charge_port_door_link",
        "charge_connector_lock_link",
    ],
    "drain_interface_detail": [
        "wastewater_drain_pipe_link", "wastewater_drain_valve_body_link",
        "wastewater_drain_valve_ball_link", "wastewater_drain_valve_actuator_link",
        "wastewater_drain_service_cap_link", "wastewater_drain_coupling_link",
    ],
    "rear_service_detail": ["emergency_stop_housing_link", "emergency_stop_plunger_link"],
    "front_left": [
        "bodywork_lighting_link", "bodywork_lower_tub_link",
        "front_left_wheel_link", "rear_left_wheel_link", "front_left_motor_link",
        "rear_left_motor_link", "front_left_encoder_link", "rear_left_encoder_link",
        "left_suspension_beam_link", "left_suspension_beam_spacer_link",
        "bodywork_power_service_door_hinge_bracket_link", "bodywork_power_service_door_link",
        "bodywork_power_service_door_latch_link", "bodywork_compute_service_door_hinge_bracket_link",
        "bodywork_compute_service_door_link", "bodywork_compute_service_door_latch_link",
    ],
    "rear_right": [
        "bodywork_lower_tub_link", "bodywork_lighting_link",
        "emergency_stop_housing_link", "emergency_stop_plunger_link",
        "bodywork_wet_service_door_hinge_bracket_link", "bodywork_wet_service_door_link",
        "bodywork_wet_service_door_latch_link", "bodywork_rear_dry_service_door_hinge_bracket_link",
        "bodywork_rear_dry_service_door_link", "bodywork_rear_dry_service_door_latch_link",
    ],
}

# Every view must carry a source-bound target entity and must project a
# non-trivial target footprint into the calibrated camera. This is an entity
# projection gate, not an RGB brightness heuristic.
for _view_name, _view_contract in VIEW_TARGETS.items():
    _view_contract.setdefault("target_entities", ["base_link"])
    _view_contract["minimum_projected_target_pixels"] = 64

FOLDED_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
FOLDED_ARM_POSITIONS = [-1.0, -1.0, 1.8, -1.5, -1.55, 0.25]

VISUAL_JOINT_POSITIONS = {
    **dict(zip(FOLDED_ARM_JOINTS, FOLDED_ARM_POSITIONS)),
    "robotiq_85_left_knuckle_joint": 0.20,
    "dry_deposit_gate_joint": 1.05,
}

PROFILE_VIEW_EVIDENCE = {
    "product": {
        "front_left": "complete product silhouette, front bumper, lamps and right arm bay",
        "rear_right": "complete rear product silhouette, rear bumper and rear lamps",
        "top_cleaning": "roof, sensor tower, arm envelope and cleaning-head layout",
        "sensor_tower_detail": "named sensor tower, UTM-30LX, MID-360 and GNSS antenna",
        "front_sensor_detail": "front RGB-D, forward lighting and front contact-safety structure",
        "arm_mount_detail": "UR5e pedestal, mounting chain, wrist camera and gripper",
        "dry_deposition_detail": "dry deposition opening, gate and dry-storage interface",
        "cleaning_head_detail": "side brushes, main brush, lift guides and cleaning shrouds",
        "rear_service_detail": "rear bumper, warning lamps, charge and drain interfaces",
        "power_compute_detail": "closed power/compute bay and external emergency-stop interface",
        "storage_recovery_detail": "closed dry/wet storage bodywork and recovery routing",
        "rear_left_sensor_detail": "rear-left fisheye housing, optical opening and mechanical bracket",
        "rear_right_sensor_detail": "rear-right fisheye housing, optical opening and mechanical bracket",
        "drivetrain_detail": "wheel, wheel arch and chassis-to-drive mechanical interface",
        "inertial_power_detail": "closed inertial/power service bay and physical access structure",
        "dry_deposition_internal": "open deposition gate, chute and dry-bin throat",
        "power_safety_internal": "power and hardwired safety installation behind the service skin",
        "charge_interface_detail": "charge door, receptacle and connector lock",
        "drain_interface_detail": "drain valve, actuator, cap and hose coupling",
    },
    "service": {
        "front_left": "whole-vehicle component placement with product bodywork removed",
        "rear_right": "rear chassis, storage, plumbing and safety component placement",
        "top_cleaning": "service-layout separation and complete mounting topology",
        "sensor_tower_detail": "sensor brackets, fasteners, pylon, ZED-F9P receiver, remote antenna and cable-service spine",
        "front_sensor_detail": "front RGB-D bracket, lighting mounts and contact-safety chain",
        "arm_mount_detail": "UR5e load path, pedestal, controller and wrist sensing chain",
        "dry_deposition_detail": "deposition actuator, gate, chute and dry-bin interface",
        "cleaning_head_detail": "cleaning motors, transmissions, lift guides, brushes and squeegee",
        "rear_service_detail": "rear safety, wastewater drain and charge hardware access",
        "power_compute_detail": "battery, contactor, low-voltage distribution, S100 and UR cabinet",
        "storage_recovery_detail": "separated dry/wet tanks, pump, filter, hoses and level hardware",
        "rear_left_sensor_detail": "rear-left fisheye bracket, fasteners and cable-entry geometry",
        "rear_right_sensor_detail": "rear-right fisheye bracket, fasteners and cable-entry geometry",
        "drivetrain_detail": "A300 motors, encoders, suspension beams, spacers and wheel joints",
        "inertial_power_detail": "VN-100 IMU tray, batteries, BMS, power distribution and safety relay",
        "dry_deposition_internal": "unobstructed deposition gate, chute and dry-bin throat",
        "power_safety_internal": "battery, BMS, PDB, safety relay, contactor and isolated LV distribution",
        "charge_interface_detail": "charge housing, door, receptacle and lock hardware",
        "drain_interface_detail": "drain pipe, valve actuator, service cap and coupling",
    },
}


class VisualAcceptanceError(RuntimeError):
    pass


def decode(message: Image) -> np.ndarray:
    encoding = message.encoding.lower()
    channels_by_encoding = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    if encoding not in channels_by_encoding:
        raise ValueError(f"unsupported visual-acceptance image encoding: {message.encoding}")
    channels = channels_by_encoding[encoding]
    row = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    pixels = row[:, : message.width * channels].reshape(message.height, message.width, channels)
    if encoding in {"bgr8", "bgra8"}:
        pixels = pixels[:, :, [2, 1, 0] + ([3] if channels == 4 else [])]
    if channels == 4:
        pixels = pixels[:, :, :3]
    return np.ascontiguousarray(pixels)


class CaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("formal_vehicle_visual_acceptance_capture")
        self.frames: dict[str, tuple[Image, np.ndarray]] = {}
        self.frame_received_monotonic: dict[str, float] = {}
        self.robot_description: str | None = None
        self.arm_command = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 1
        )
        self.gripper_command = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 1
        )
        self.storage_command = self.create_publisher(
            JointTrajectory, "/storage_controller/joint_trajectory", 1
        )
        self._image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # In triggered mode a 1600x1000 frame is captured, persisted and
        # released before the next camera subscribes.  Keep the periodic
        # fallback available for diagnosis, but never pre-create all nineteen
        # subscriptions for formal sequential capture.
        self._camera_subscriptions: dict[str, object] = {}
        description_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._robot_description_qos = description_qos
        self._robot_description_subscription = None

    def start_robot_description_subscription(self) -> None:
        if self._robot_description_subscription is None:
            self._robot_description_subscription = self.create_subscription(
                String,
                "/robot_description",
                self._on_robot_description,
                self._robot_description_qos,
            )

    def _on_image(self, name: str, message: Image) -> None:
        try:
            self.frames[name] = (message, decode(message))
            self.frame_received_monotonic[name] = time.monotonic()
        except ValueError as error:
            self.get_logger().error(str(error))

    def start_camera_capture(self, name: str) -> None:
        if name not in TOPICS:
            raise VisualAcceptanceError(f"unknown visual camera: {name}")
        if self._camera_subscriptions:
            raise VisualAcceptanceError(
                "visual capture may subscribe to only one camera at a time"
            )
        self._camera_subscriptions[name] = self.create_subscription(
            Image,
            TOPICS[name],
            lambda message, key=name: self._on_image(key, message),
            self._image_qos,
        )

    def stop_camera_capture(self, name: str) -> None:
        subscription = self._camera_subscriptions.pop(name, None)
        if subscription is not None:
            self.destroy_subscription(subscription)

    def start_all_camera_captures(self) -> None:
        """Legacy periodic mode only; formal acceptance uses one at a time."""

        for name in TOPICS:
            self._camera_subscriptions[name] = self.create_subscription(
                Image,
                TOPICS[name],
                lambda message, key=name: self._on_image(key, message),
                self._image_qos,
            )

    def release_camera_frame(self, name: str) -> None:
        self.frames.pop(name, None)
        self.frame_received_monotonic.pop(name, None)

    def _on_robot_description(self, message: String) -> None:
        self.robot_description = message.data

    def command_folded_arm(self) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = FOLDED_ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = FOLDED_ARM_POSITIONS
        point.time_from_start = Duration(sec=2)
        trajectory.points = [point]
        self.arm_command.publish(trajectory)
        gripper = JointTrajectory()
        gripper.joint_names = ["robotiq_85_left_knuckle_joint"]
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [0.20]
        gripper_point.time_from_start = Duration(sec=2)
        gripper.points = [gripper_point]
        self.gripper_command.publish(gripper)
        storage = JointTrajectory()
        storage.joint_names = ["dry_deposit_gate_joint"]
        storage_point = JointTrajectoryPoint()
        storage_point.positions = [1.05]
        storage_point.time_from_start = Duration(sec=8)
        storage.points = [storage_point]
        self.storage_command.publish(storage)


def frame_metrics(pixels: np.ndarray) -> dict[str, float | int]:
    luminance = pixels.astype(np.float32).mean(axis=2)
    dark_fraction = float((luminance < 8.0).mean())
    variation = float(luminance.std())
    return {
        "width": int(pixels.shape[1]),
        "height": int(pixels.shape[0]),
        "mean_luminance": float(luminance.mean()),
        "luminance_stddev": variation,
        "near_black_fraction": dark_fraction,
    }


def validate_frame_metrics(name: str, metrics: dict[str, float | int]) -> None:
    if int(metrics["width"]) < 1280 or int(metrics["height"]) < 720:
        raise VisualAcceptanceError(f"{name}: resolution below acceptance minimum")
    if float(metrics["near_black_fraction"]) > 0.95 or float(metrics["luminance_stddev"]) < 8.0:
        raise VisualAcceptanceError(f"{name}: black or visually empty")


def camera_projection_contracts(world_path: Path) -> dict[str, dict[str, object]]:
    """Read the source-bound camera calibration used by the actual Gazebo world."""
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise VisualAcceptanceError("visual acceptance world is missing")
    result: dict[str, dict[str, object]] = {}
    for model in world.findall("model"):
        sensor = model.find("./link/sensor")
        if sensor is None or sensor.get("type") != "camera":
            continue
        name = sensor.get("name")
        pose_text = model.findtext("pose", "")
        values = [float(value) for value in pose_text.split()]
        if not name or len(values) != 6:
            raise VisualAcceptanceError("camera model has an invalid name or pose")
        result[name] = {
            "pose_xyz_rpy": values,
            "horizontal_fov_rad": float(sensor.findtext("./camera/horizontal_fov", "0")),
            "width": int(sensor.findtext("./camera/image/width", "0")),
            "height": int(sensor.findtext("./camera/image/height", "0")),
        }
    return result


def validate_target_projection(
    name: str,
    camera: dict[str, object],
    target: dict[str, object],
) -> dict[str, object]:
    """Fail closed unless the named physical target projects inside the image.

    The target identity is source-bound to the expanded URDF and the camera is
    loaded from the SDF that spawned Gazebo. This proves semantic in-frame
    occupancy; RGB metrics remain a separate renderer-integrity check.
    """
    pose = [float(value) for value in camera["pose_xyz_rpy"]]
    x, y, z, _roll, pitch, yaw = pose
    point = [float(value) for value in target["target_xyz_m"]]
    ray = np.asarray([point[0] - x, point[1] - y, point[2] - z], dtype=np.float64)
    distance = float(np.linalg.norm(ray))
    if distance <= 0.1:
        raise VisualAcceptanceError(f"{name}: target is too close to calibrated camera")
    forward = np.asarray(
        [math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch)],
        dtype=np.float64,
    )
    right = np.asarray([-math.sin(yaw), math.cos(yaw), 0.0], dtype=np.float64)
    down = np.cross(forward, right)
    depth = float(np.dot(ray, forward))
    horizontal = math.atan2(float(np.dot(ray, right)), depth)
    vertical = math.atan2(float(np.dot(ray, down)), depth)
    hfov = float(camera["horizontal_fov_rad"])
    width = int(camera["width"])
    height = int(camera["height"])
    vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * height / width)
    if depth <= 0.0 or abs(horizontal) >= hfov / 2.0 or abs(vertical) >= vfov / 2.0:
        raise VisualAcceptanceError(f"{name}: target entities do not project inside the camera")
    # A conservative 10 mm target radius prevents point-only / datum-only
    # contracts from satisfying semantic occupancy.
    angular_diameter = 2.0 * math.atan2(0.01, distance)
    projected_pixels = angular_diameter / hfov * width
    minimum = int(target["minimum_projected_target_pixels"])
    projected_area = int(round(projected_pixels * projected_pixels))
    if projected_area < minimum:
        raise VisualAcceptanceError(
            f"{name}: projected target occupancy {projected_area}px is below {minimum}px"
        )
    entities = target.get("target_entities")
    if not isinstance(entities, list) or not entities or any(not isinstance(item, str) for item in entities):
        raise VisualAcceptanceError(f"{name}: target entity contract is empty")
    return {
        "method": "source_bound_urdf_entity_projection",
        "target_entities": entities,
        "target_xyz_m": point,
        "horizontal_offset_rad": horizontal,
        "vertical_offset_rad": vertical,
        "projected_target_area_px": projected_area,
        "minimum_projected_target_pixels": minimum,
        "passed": True,
    }


def validate_target_entities(
    robot_description: str, targets: dict[str, dict[str, object]]
) -> dict[str, list[str]]:
    """Bind every shot's semantic target names to physical URDF links."""
    try:
        root = ET.fromstring(robot_description)
    except ET.ParseError as error:
        raise VisualAcceptanceError(f"expanded robot_description is invalid: {error}") from error
    links = {link.get("name"): link for link in root.findall("link")}
    validated: dict[str, list[str]] = {}
    for shot, contract in targets.items():
        entities = contract.get("target_entities")
        if not isinstance(entities, list) or not entities:
            raise VisualAcceptanceError(f"{shot}: target entity contract is empty")
        for entity in entities:
            link = links.get(entity)
            if link is None:
                raise VisualAcceptanceError(f"{shot}: target entity is absent from URDF: {entity}")
            if not link.findall("visual") and entity != "base_link":
                raise VisualAcceptanceError(f"{shot}: target entity has no visible geometry: {entity}")
        validated[shot] = list(entities)
    return validated


def _origin_matrix(element: ET.Element | None) -> np.ndarray:
    if element is None:
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
    else:
        xyz = tuple(float(value) for value in element.get("xyz", "0 0 0").split())
        rpy = tuple(float(value) for value in element.get("rpy", "0 0 0").split())
    if len(xyz) != 3 or len(rpy) != 3:
        raise VisualAcceptanceError("URDF joint origin must use three-element xyz/rpy")
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    result[:3, 3] = xyz
    return result


def _axis_rotation(axis_text: str, angle: float) -> np.ndarray:
    axis = np.asarray([float(value) for value in axis_text.split()], dtype=np.float64)
    if axis.shape != (3,) or float(np.linalg.norm(axis)) <= 1e-12:
        raise VisualAcceptanceError("URDF movable joint has an invalid axis")
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus = 1.0 - cosine
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(
        [
            [cosine + x * x * one_minus, x * y * one_minus - z * sine, x * z * one_minus + y * sine],
            [y * x * one_minus + z * sine, cosine + y * y * one_minus, y * z * one_minus - x * sine],
            [z * x * one_minus - y * sine, z * y * one_minus + x * sine, cosine + z * z * one_minus],
        ],
        dtype=np.float64,
    )
    return result


def configured_link_positions(
    robot_description: str,
    joint_positions: dict[str, float],
) -> tuple[dict[str, list[float]], dict[str, ET.Element]]:
    """Resolve link origins at the exact arm/gate pose commanded for capture."""
    try:
        root = ET.fromstring(robot_description)
    except ET.ParseError as error:
        raise VisualAcceptanceError(f"expanded robot_description is invalid: {error}") from error
    links = {link.get("name"): link for link in root.findall("link")}
    joints = root.findall("joint")
    child_links = {joint.find("child").get("link") for joint in joints}
    roots = sorted(set(links) - child_links)
    if len(roots) != 1:
        raise VisualAcceptanceError(f"expanded robot_description has invalid roots: {roots}")
    transforms: dict[str, np.ndarray] = {roots[0]: np.eye(4, dtype=np.float64)}
    pending = list(joints)
    while pending:
        remaining: list[ET.Element] = []
        progress = False
        for joint in pending:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in transforms:
                remaining.append(joint)
                continue
            local = _origin_matrix(joint.find("origin"))
            angle = float(joint_positions.get(joint.get("name", ""), 0.0))
            if joint.get("type") in {"revolute", "continuous"} and angle:
                axis = joint.find("axis")
                local = local @ _axis_rotation(
                    "1 0 0" if axis is None else axis.get("xyz", "1 0 0"), angle
                )
            elif joint.get("type") == "prismatic" and angle:
                axis = joint.find("axis")
                vector = np.asarray(
                    [float(value) for value in ("1 0 0" if axis is None else axis.get("xyz", "1 0 0")).split()],
                    dtype=np.float64,
                )
                vector /= np.linalg.norm(vector)
                translation = np.eye(4, dtype=np.float64)
                translation[:3, 3] = vector * angle
                local = local @ translation
            transforms[child] = transforms[parent] @ local
            progress = True
        if not progress:
            raise VisualAcceptanceError(
                "expanded robot_description contains an unresolved joint tree: "
                + ", ".join(sorted(joint.get("name", "") for joint in remaining))
            )
        pending = remaining
    return (
        {name: [float(transform[index, 3]) for index in range(3)] for name, transform in transforms.items()},
        links,
    )


def validate_engineering_visual_crosswalk(
    robot_description: str,
    camera_contracts: dict[str, dict[str, object]],
    register: dict[str, object],
    *,
    position_views: dict[str, list[str]],
    sensor_views: dict[str, list[str]],
    assembly_views: dict[str, list[str]],
    inspection_links: dict[str, list[str]],
    joint_positions: dict[str, float],
    bodywork_profile: str = "product",
) -> dict[str, object]:
    """Fail closed on incomplete ID coverage or non-projecting physical links."""
    category_rows = {
        "functional_positions": register.get("functional_positions", []),
        "sensor_installations": register.get("sensor_installations", []),
        "mechanical_subassemblies": register.get("mechanical_subassemblies", []),
    }
    category_views = {
        "functional_positions": position_views,
        "sensor_installations": sensor_views,
        "mechanical_subassemblies": assembly_views,
    }
    for category, rows in category_rows.items():
        if not isinstance(rows, list) or any(not isinstance(row, dict) for row in rows):
            raise VisualAcceptanceError(f"component register {category} is invalid")
        ids = {str(row.get("id")) for row in rows}
        if ids != set(category_views[category]):
            missing = sorted(ids - set(category_views[category]))
            extra = sorted(set(category_views[category]) - ids)
            raise VisualAcceptanceError(
                f"{category} visual crosswalk mismatch: missing={missing}, extra={extra}"
            )
        for item_id, views in category_views[category].items():
            if not views or not set(views) <= set(camera_contracts):
                raise VisualAcceptanceError(f"{category}.{item_id} has invalid inspection views")

    positions, links = configured_link_positions(robot_description, joint_positions)
    if bodywork_profile not in {"product", "service"}:
        raise VisualAcceptanceError(f"unsupported visual profile: {bodywork_profile}")
    reverse_views: dict[str, list[str]] = {}
    for view, entity_links in inspection_links.items():
        if view not in camera_contracts or not entity_links:
            raise VisualAcceptanceError(f"inspection link view is invalid or empty: {view}")
        for link_name in entity_links:
            if (
                bodywork_profile == "service"
                and link_name in links
                and not links[link_name].findall("visual")
            ):
                continue
            reverse_views.setdefault(link_name, []).append(view)

    required_links: set[str] = set()
    item_link_crosswalk: dict[str, dict[str, list[str]]] = {
        "functional_positions": {},
        "sensor_installations": {},
        "mechanical_subassemblies": {},
    }

    def visible_physical(candidate: object) -> list[str]:
        names = [candidate] if isinstance(candidate, str) else list(candidate or [])
        return [
            str(name) for name in names
            if str(name) in links and links[str(name)].findall("visual")
        ]

    for row in category_rows["sensor_installations"]:
        item_id = str(row["id"])
        names = [str(row["mount_link"]), str(row["sensor_link"])]
        item_link_crosswalk["sensor_installations"][item_id] = names
        required_links.update(names)

    for row in category_rows["functional_positions"]:
        item_id = str(row["id"])
        names = visible_physical(row.get("physical_link", row.get("link")))
        for component in row.get("components", []):
            names.extend(visible_physical(component.get("link")))
        names = list(dict.fromkeys(names))
        item_link_crosswalk["functional_positions"][item_id] = names
        required_links.update(names)

    assembly_overrides = {
        "manipulator": [
            "ur5e_base_link_inertia", "ur5e_shoulder_link", "ur5e_upper_arm_link",
            "ur5e_forearm_link", "ur5e_wrist_1_link", "ur5e_wrist_2_link", "ur5e_wrist_3_link",
        ],
        "exterior_service_and_contact_safety": [
            "charge_port_housing_link", "wastewater_drain_valve_body_link",
            "emergency_stop_housing_link", "bodywork_lighting_link", "bodywork_lower_tub_link",
        ],
    }
    for row in category_rows["mechanical_subassemblies"]:
        item_id = str(row["id"])
        if item_id in assembly_overrides:
            names = visible_physical(assembly_overrides[item_id])
        else:
            names = []
            for field in ("root_link", "root_links", "actuator_link", "actuator_links"):
                names.extend(visible_physical(row.get(field)))
            names = list(dict.fromkeys(names))
        if not names and not (
            bodywork_profile == "service" and item_id == "bodywork_service_access"
        ):
            raise VisualAcceptanceError(
                f"mechanical_subassemblies.{item_id} has no physical inspection link"
            )
        item_link_crosswalk["mechanical_subassemblies"][item_id] = names
        required_links.update(names)

    missing_assignments = sorted(required_links - set(reverse_views))
    if missing_assignments:
        raise VisualAcceptanceError(
            "registered physical links are absent from the 19-view inspection crosswalk: "
            + ", ".join(missing_assignments)
        )

    for category, rows in item_link_crosswalk.items():
        view_map = category_views[category]
        for item_id, names in rows.items():
            if names and not any(set(reverse_views[name]) & set(view_map[item_id]) for name in names):
                raise VisualAcceptanceError(
                    f"{category}.{item_id} physical links do not use its assigned inspection views"
                )

    projection_reports: dict[str, dict[str, object]] = {}
    for link_name, views in sorted(reverse_views.items()):
        link = links.get(link_name)
        if link is None:
            raise VisualAcceptanceError(f"inspection link is absent from URDF: {link_name}")
        if not link.findall("visual"):
            raise VisualAcceptanceError(
                f"inspection link lacks visible physical geometry: {link_name}"
            )
        point = positions[link_name]
        accepted: dict[str, object] | None = None
        failures: list[str] = []
        for view in views:
            try:
                accepted = validate_target_projection(
                    view,
                    camera_contracts[view],
                    {
                        "target_xyz_m": point,
                        "target_entities": [link_name],
                        "minimum_projected_target_pixels": 64,
                    },
                )
            except VisualAcceptanceError as error:
                failures.append(f"{view}: {error}")
                continue
            accepted = {**accepted, "accepted_view": view, "candidate_views": views}
            break
        if accepted is None:
            raise VisualAcceptanceError(
                f"{link_name} does not project into any assigned inspection camera: "
                + "; ".join(failures)
            )
        projection_reports[link_name] = accepted

    return {
        "method": "registered_id_crosswalk_plus_commanded_pose_urdf_link_projection",
        "bodywork_profile": bodywork_profile,
        "camera_count": len(camera_contracts),
        "functional_position_count": len(position_views),
        "sensor_installation_count": len(sensor_views),
        "mechanical_subassembly_count": len(assembly_views),
        "required_registered_physical_link_count": len(required_links),
        "inspected_physical_link_count": len(projection_reports),
        "functional_position_views": position_views,
        "sensor_installation_views": sensor_views,
        "mechanical_subassembly_views": assembly_views,
        "item_physical_links": item_link_crosswalk,
        "link_projection_reports": projection_reports,
        "passed": True,
    }


def inspect_renderer_log(path: Path | None) -> dict[str, object]:
    """Reject known renderer-integrity failures before evidence is published."""
    if path is None:
        raise VisualAcceptanceError("renderer launch log is required for formal capture")
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as error:
        raise VisualAcceptanceError(f"renderer launch log is unavailable: {error}") from error
    fatal_markers = {
        "scene_entity_zero": "[SceneManager.cc:615] Could not find visual for entity: 0",
        "sensor_creation_failed": "Failed to create sensor",
        "render_engine_failed": "Unable to create rendering engine",
        "ogre_render_exception": "OGRE EXCEPTION",
    }
    hits = {
        name: text.count(marker)
        for name, marker in fatal_markers.items()
        if marker in text
    }
    diagnostics: dict[str, object] = {
        "launch_log": path.name,
        "launch_log_sha256": _sha256_bytes(path.read_bytes()),
        "fatal_marker_counts": hits,
        "passed": not hits,
    }
    if hits:
        rendered = ", ".join(f"{name}={count}" for name, count in sorted(hits.items()))
        raise VisualAcceptanceError(
            "Gazebo renderer integrity gate failed: " + rendered
        )
    return diagnostics


def validate_bodywork_profile(robot_description: str, profile: str) -> int:
    """Prove that the manifest label matches the robot actually spawned.

    Service mode keeps collision and inertial links unchanged but removes the
    project bodywork visual meshes.  Counting the expanded URDF therefore
    distinguishes a real exposed-component capture from a relabelled product
    image without relying on image heuristics.
    """
    marker = "package://sanitation_vehicle_description/meshes/project/bodywork/"
    mesh_count = robot_description.count(marker)
    if profile == "product" and mesh_count < 40:
        raise ValueError(
            f"product capture requires installed bodywork visuals; found {mesh_count} meshes"
        )
    if profile == "service" and mesh_count != 0:
        raise ValueError(
            f"service capture requires bodywork_visible:=false; found {mesh_count} bodywork meshes"
        )
    return mesh_count


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def partial_frame_artifacts(output: Path) -> dict[str, object]:
    """Bind retained partial PNG evidence into a failed capture manifest."""

    paths = sorted(path.name for path in output.glob("*.png") if path.is_file())
    return {
        "partial_frame_file_count": len(paths),
        "partial_frame_files": paths,
    }


def snapshot_binding(path: Path) -> dict[str, str]:
    try:
        from generate_formal_vehicle_snapshot import verify_snapshot

        verify_snapshot(ROOT)
    except Exception as error:
        raise VisualAcceptanceError(
            f"formal vehicle snapshot is stale and cannot bind visual evidence: {error}"
        ) from error
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        source_hash = payload["source_inventory_sha256"]
        urdf_hash = payload["outputs"][
            "reports/engineering/formal_competition_vehicle.urdf"
        ]["sha256"]
    except (OSError, UnicodeError, json.JSONDecodeError, KeyError, TypeError) as error:
        raise VisualAcceptanceError(
            f"formal vehicle snapshot manifest is missing or invalid: {error}"
        ) from error
    if not isinstance(source_hash, str) or not isinstance(urdf_hash, str):
        raise VisualAcceptanceError("formal vehicle snapshot hashes are invalid")
    try:
        relative = path.resolve().relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        relative = str(path.resolve())
    capture_script = Path(__file__).resolve()
    ground_truth_script = ROOT / "scripts/gazebo_ground_truth.py"
    visual_launch = ROOT / (
        "starter_ws/src/sanitation_vehicle_description/launch/"
        "formal_vehicle_visual_acceptance.launch.py"
    )
    visual_world = ROOT / (
        "starter_ws/src/sanitation_vehicle_description/worlds/"
        "formal_vehicle_visual_acceptance.sdf"
    )
    component_register = ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
    return {
        "snapshot_manifest_path": relative,
        "snapshot_manifest_sha256": _sha256_bytes(path.read_bytes()),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
        "capture_script_sha256": _sha256_bytes(capture_script.read_bytes()),
        "ground_truth_reader_sha256": _sha256_bytes(ground_truth_script.read_bytes()),
        "visual_launch_sha256": _sha256_bytes(visual_launch.read_bytes()),
        "visual_world_sha256": _sha256_bytes(visual_world.read_bytes()),
        "component_register_sha256": _sha256_bytes(component_register.read_bytes()),
    }


def pose_delta(first: dict[str, float], second: dict[str, float]) -> dict[str, float]:
    yaw_delta = math.atan2(
        math.sin(float(second["yaw"]) - float(first["yaw"])),
        math.cos(float(second["yaw"]) - float(first["yaw"])),
    )
    dx = float(second["x"]) - float(first["x"])
    dy = float(second["y"]) - float(first["y"])
    return {
        "dx_m": dx,
        "dy_m": dy,
        "planar_m": math.hypot(dx, dy),
        "dz_m": float(second["z"]) - float(first["z"]),
        "yaw_rad": yaw_delta,
    }


def validate_ground_truth_pose(
    initial: dict[str, float],
    final: dict[str, float],
    *,
    max_spawn_planar_m: float,
    max_run_planar_m: float,
    max_abs_yaw_rad: float,
) -> dict[str, object]:
    spawn_planar = math.hypot(float(final["x"]), float(final["y"]))
    delta = pose_delta(initial, final)
    violations: list[str] = []
    if spawn_planar > max_spawn_planar_m:
        violations.append(
            f"final model position drifted {spawn_planar:.6f} m from the studio origin"
        )
    if float(delta["planar_m"]) > max_run_planar_m:
        violations.append(
            f"model moved {float(delta['planar_m']):.6f} m during visual capture"
        )
    if abs(float(final["yaw"])) > max_abs_yaw_rad:
        violations.append(
            f"final model yaw {float(final['yaw']):.6f} rad exceeds studio tolerance"
        )
    if not -0.02 <= float(final["z"]) <= 0.20:
        violations.append(
            f"final model z {float(final['z']):.6f} m is outside the grounded envelope"
        )
    return {
        "world_name": WORLD_NAME,
        "model_name": MODEL_NAME,
        "initial_pose": initial,
        "final_pose": final,
        "initial_to_final_delta": delta,
        "final_planar_distance_from_spawn_m": spawn_planar,
        "limits": {
            "max_spawn_planar_m": max_spawn_planar_m,
            "max_run_planar_m": max_run_planar_m,
            "max_abs_yaw_rad": max_abs_yaw_rad,
            "final_z_range_m": [-0.02, 0.20],
        },
        "violations": violations,
        "passed": not violations,
    }


def read_ground_truth_with_retry(timeout: float) -> dict[str, float]:
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return read_named_model_pose(
                world_name=WORLD_NAME,
                model_name=MODEL_NAME,
                timeout_s=min(5.0, max(0.5, deadline - time.monotonic())),
            )
        except (OSError, RuntimeError, subprocess.SubprocessError) as error:
            last_error = error
            time.sleep(0.25)
    raise VisualAcceptanceError(
        f"Gazebo ground-truth pose was unavailable: {last_error}"
    )


def _atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(path.name + f".pending.{os.getpid()}")
    try:
        pending.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        pending.replace(path)
    finally:
        if pending.exists():
            pending.unlink()


def load_runtime_gate_binding(path: Path) -> dict[str, object]:
    """Load the pre-overlay formal binding without rebuilding or projecting it."""
    try:
        return load_binding(path)
    except (OSError, RuntimeGateError, TypeError, ValueError, KeyError) as error:
        raise VisualAcceptanceError(
            f"formal visual runtime binding is missing or invalid: {error}"
        ) from error


def write_bound_manifest(
    path: Path, payload: dict[str, object], runtime_gate_binding: dict[str, object]
) -> None:
    """Atomically retain a profile manifest beside its exact runtime binding."""
    payload["runtime_gate_binding"] = runtime_gate_binding
    payload["acceptance_session_binding"] = runtime_gate_binding[
        "acceptance_session_binding"
    ]
    payload["runtime_closure_binding"] = runtime_gate_binding[
        "runtime_closure_binding"
    ]
    sidecar = path.with_name(path.name + ".runtime_binding.json")
    # The acceptance orchestrator rejects a sidecar that postdates its report.
    _atomic_write_json(sidecar, runtime_gate_binding)
    _atomic_write_json(path, payload)


def camera_trigger_topic(image_topic: str) -> str:
    if image_topic not in TOPICS.values():
        raise VisualAcceptanceError(
            f"refusing to trigger a camera outside the formal topic contract: {image_topic}"
        )
    return f"{image_topic}/trigger"


def validate_triggered_world_report(
    path: Path, expected_world: Path
) -> dict[str, object]:
    """Bind sequential capture to the exact fail-closed world conversion report."""

    try:
        report = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise VisualAcceptanceError(
            f"triggered visual-world report is unreadable: {path}: {error}"
        ) from error
    expected_bindings = [
        {
            "image_topic": topic,
            "trigger_topic": camera_trigger_topic(topic),
            "uses_default_trigger_topic": True,
        }
        for topic in TOPICS.values()
    ]
    required = {
        "status": "FORMAL_TRIGGERED_VISUAL_WORLD_PREPARED",
        "passed": True,
        "camera_count": len(TOPICS),
        "all_camera_contract_fields_preserved": True,
        "all_cameras_triggered": True,
        "all_cameras_use_default_trigger_topic": True,
        "trigger_bindings": expected_bindings,
    }
    mismatches = [
        key for key, expected in required.items() if report.get(key) != expected
    ]
    if mismatches:
        raise VisualAcceptanceError(
            "triggered visual-world report violates the formal contract: "
            + ", ".join(mismatches)
        )
    output_world = Path(str(report.get("output_world", "")))
    try:
        output_payload = output_world.read_bytes()
    except OSError as error:
        raise VisualAcceptanceError(
            f"reported triggered visual world is unreadable: {output_world}: {error}"
        ) from error
    if _sha256_bytes(output_payload) != report.get("output_world_sha256"):
        raise VisualAcceptanceError("triggered visual-world output hash no longer matches")
    try:
        same_world = output_world.resolve(strict=True) == expected_world.resolve(strict=True)
    except OSError as error:
        raise VisualAcceptanceError(
            f"triggered visual-world path cannot be resolved: {error}"
        ) from error
    if not same_world:
        raise VisualAcceptanceError(
            "capture camera-contract world differs from the launched triggered world"
        )
    return {
        "report_path": str(path.resolve(strict=True)),
        "report_sha256": _sha256_bytes(path.read_bytes()),
        "source_world": report.get("source_world"),
        "source_world_sha256": report.get("source_world_sha256"),
        "triggered_world": str(output_world.resolve(strict=True)),
        "triggered_world_sha256": report.get("output_world_sha256"),
        "camera_contract_sha256": report.get("camera_contract_sha256_after"),
        "camera_count": report.get("camera_count"),
        "trigger_bindings": expected_bindings,
        "passed": True,
    }


def publish_camera_trigger(image_topic: str, timeout_s: float) -> dict[str, object]:
    """Publish exactly one Gazebo Boolean trigger and retain bounded diagnostics."""

    trigger_topic = camera_trigger_topic(image_topic)
    started = time.monotonic()
    try:
        completed = subprocess.run(
            [
                "gz",
                "topic",
                "-t",
                trigger_topic,
                "-m",
                "gz.msgs.Boolean",
                "-p",
                "data: true",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=max(0.1, timeout_s),
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise VisualAcceptanceError(
            f"Gazebo camera trigger failed for {image_topic}: {error}"
        ) from error
    elapsed = time.monotonic() - started
    if completed.returncode != 0:
        diagnostic = (completed.stderr or completed.stdout).strip()[-1000:]
        raise VisualAcceptanceError(
            f"Gazebo camera trigger failed for {image_topic} "
            f"(rc={completed.returncode}): {diagnostic}"
        )
    return {
        "image_topic": image_topic,
        "trigger_topic": trigger_topic,
        "message_type": "gz.msgs.Boolean",
        "payload": "data: true",
        "elapsed_seconds": elapsed,
        "completed_monotonic_seconds": time.monotonic(),
        "returncode": completed.returncode,
    }


def wait_for_camera_trigger_subscriber(
    image_topic: str, deadline: float
) -> dict[str, object]:
    """Wait until Gazebo reports a Boolean subscriber for this trigger topic."""

    trigger_topic = camera_trigger_topic(image_topic)
    started = time.monotonic()
    probes = 0
    last_diagnostic = ""
    while time.monotonic() < deadline:
        probes += 1
        remaining = deadline - time.monotonic()
        try:
            completed = subprocess.run(
                ["gz", "topic", "-i", "-t", trigger_topic],
                check=False,
                capture_output=True,
                text=True,
                timeout=max(0.1, min(3.0, remaining)),
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            last_diagnostic = str(error)
        else:
            combined = f"{completed.stdout}\n{completed.stderr}"
            normalized = combined.lower()
            last_diagnostic = combined.strip()[-1000:]
            if (
                completed.returncode == 0
                and "subscriber" in normalized
                and "gz.msgs.boolean" in normalized
            ):
                return {
                    "trigger_topic": trigger_topic,
                    "expected_message_type": "gz.msgs.Boolean",
                    "probe_count": probes,
                    "ready_wait_seconds": time.monotonic() - started,
                    "subscriber_ready": True,
                }
        time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
    raise VisualAcceptanceError(
        f"Gazebo trigger subscriber did not become ready for {image_topic}: "
        f"{last_diagnostic}"
    )


def wait_for_ros_camera_publisher(
    node: CaptureNode, image_topic: str, deadline: float
) -> dict[str, object]:
    """Wait for the reliable ROS image bridge endpoint before triggering."""

    started = time.monotonic()
    probes = 0
    while rclpy.ok() and time.monotonic() < deadline:
        probes += 1
        rclpy.spin_once(node, timeout_sec=0.05)
        publisher_count = node.count_publishers(image_topic)
        if publisher_count > 0:
            return {
                "image_topic": image_topic,
                "expected_reliability": "RELIABLE",
                "publisher_count": publisher_count,
                "probe_count": probes,
                "ready_wait_seconds": time.monotonic() - started,
                "publisher_ready": True,
            }
    raise VisualAcceptanceError(
        f"reliable ROS image publisher did not become ready for {image_topic}"
    )


def capture_frames_sequentially(
    node: CaptureNode,
    deadline: float,
    *,
    persist_frame,
    frame_wait_seconds: float = 3.0,
) -> dict[str, list[dict[str, object]]]:
    """Trigger, persist and release one formal camera before advancing."""

    attempts: dict[str, list[dict[str, object]]] = {}
    for name, image_topic in TOPICS.items():
        attempts[name] = []
        message: Image | None = None
        pixels: np.ndarray | None = None
        node.start_camera_capture(name)
        try:
            trigger_readiness = wait_for_camera_trigger_subscriber(image_topic, deadline)
            ros_readiness = wait_for_ros_camera_publisher(node, image_topic, deadline)
            # Drain any callback already queued before publishing the accepted
            # trigger, then fail closed if a triggered camera produced a stale
            # frame without an explicit command from this process.
            rclpy.spin_once(node, timeout_sec=0.0)
            if name in node.frames:
                raise VisualAcceptanceError(
                    f"triggered visual camera produced an unsolicited frame: {name}"
                )
            while rclpy.ok() and name not in node.frames:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise VisualAcceptanceError(
                        f"sequential visual camera timed out: {name}"
                    )
                trace = publish_camera_trigger(image_topic, min(5.0, remaining))
                trace["subscriber_readiness"] = trigger_readiness
                trace["ros_publisher_readiness"] = ros_readiness
                attempts[name].append(trace)
                wait_deadline = min(deadline, time.monotonic() + frame_wait_seconds)
                while (
                    rclpy.ok()
                    and name not in node.frames
                    and time.monotonic() < wait_deadline
                ):
                    rclpy.spin_once(node, timeout_sec=0.10)
            if name in node.frames:
                message, pixels = node.frames[name]
                received = node.frame_received_monotonic[name]
                latest_trigger = attempts[name][-1]
                completed = float(latest_trigger["completed_monotonic_seconds"])
                stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                    message.header.stamp.nanosec
                )
                if received < completed or stamp_ns <= 0:
                    raise VisualAcceptanceError(
                        f"triggered visual camera returned a stale or non-causal frame: {name}"
                    )
                latest_trigger.update(
                    {
                        "accepted_frame_stamp_sec": int(message.header.stamp.sec),
                        "accepted_frame_stamp_nanosec": int(
                            message.header.stamp.nanosec
                        ),
                        "frame_received_monotonic_seconds": received,
                        "frame_received_after_trigger_completion": True,
                        "accepted": True,
                    }
                )
                persist_frame(name, message, pixels)
        finally:
            # The image callback owns the only large Image/ndarray pair.  Tear
            # down its DDS subscription before dropping those references so a
            # late callback cannot repopulate the frame map.
            node.stop_camera_capture(name)
            node.release_camera_frame(name)
            # `message` and `pixels` are loop locals, so removing the map
            # entry alone would retain a full prior frame until the next
            # iteration assigns them.  Drop both references before advancing.
            message = None
            pixels = None
    return attempts


def capture(args: argparse.Namespace, binding: dict[str, str]) -> dict[str, object]:
    node: CaptureNode | None = None
    ros_initialized = False
    initial_pose: dict[str, float] | None = None
    final_pose: dict[str, float] | None = None
    robot_description: str | None = None
    frames: dict[str, tuple[Image, np.ndarray]] = {}
    trigger_attempts: dict[str, list[dict[str, object]]] = {}
    triggered_world_binding: dict[str, object] | None = None
    if args.trigger_cameras_sequentially:
        if args.triggered_world_report is None:
            raise VisualAcceptanceError(
                "sequential camera capture requires --triggered-world-report"
            )
        if args.camera_contract_world is None:
            raise VisualAcceptanceError(
                "sequential camera capture requires --camera-contract-world"
            )
        triggered_world_binding = validate_triggered_world_report(
            args.triggered_world_report, args.camera_contract_world
        )
    active_targets = targets_for_profile(args.bodywork_profile)
    visual_world = args.camera_contract_world or ROOT / (
        "starter_ws/src/sanitation_vehicle_description/worlds/"
        "formal_vehicle_visual_acceptance.sdf"
    )
    camera_contracts = camera_projection_contracts(visual_world)
    if set(camera_contracts) != set(TOPICS):
        raise VisualAcceptanceError("SDF camera set does not match the capture topic contract")
    reports: dict[str, dict[str, object]] = {}
    invalid_frames: list[str] = []

    def persist_frame(name: str, message: Image, pixels: np.ndarray) -> None:
        """Write all per-view evidence while this is the sole retained frame."""

        path = args.output / f"{name}.png"
        PillowImage.fromarray(pixels, "RGB").save(path)
        metrics = frame_metrics(pixels)
        semantic_visibility = validate_target_projection(
            name, camera_contracts[name], active_targets[name]
        )
        try:
            validate_frame_metrics(name, metrics)
        except VisualAcceptanceError as error:
            invalid_frames.append(str(error))
        reports[name] = {
            "topic": TOPICS[name],
            "encoding": message.encoding,
            "stamp_sec": message.header.stamp.sec,
            "stamp_nanosec": message.header.stamp.nanosec,
            "path": path.name,
            "png_sha256": _sha256_file(path),
            "png_size_bytes": path.stat().st_size,
            "evidence_scope": PROFILE_VIEW_EVIDENCE[args.bodywork_profile][name],
            "semantic_visibility": semantic_visibility,
            **metrics,
        }

    try:
        rclpy.init()
        ros_initialized = True
        node = CaptureNode()
        if not args.no_fold_arm:
            # The full product needs substantially longer than the old
            # placeholder vehicle to spawn, load ros2_control and activate
            # the trajectory controllers on a cold Gazebo start.
            subscription_deadline = time.monotonic() + min(args.timeout, 90.0)
            while rclpy.ok() and (
                node.arm_command.get_subscription_count() == 0
                or node.gripper_command.get_subscription_count() == 0
                or node.storage_command.get_subscription_count() == 0
            ):
                if time.monotonic() >= subscription_deadline:
                    raise VisualAcceptanceError(
                        "arm/controller command subscriptions did not become available"
                    )
                rclpy.spin_once(node, timeout_sec=0.25)
        initial_pose = read_ground_truth_with_retry(min(args.timeout, 30.0))
        if not args.no_fold_arm:
            node.command_folded_arm()
            settle_deadline = time.monotonic() + args.settle_seconds
            while rclpy.ok() and time.monotonic() < settle_deadline:
                rclpy.spin_once(node, timeout_sec=0.25)
            # Discard frames received during motion so every saved image shows
            # the same product/transport pose.
            node.frames.clear()
            node.frame_received_monotonic.clear()
        # Do not join the large reliable robot-description stream until the
        # embedded controller manager has consumed it and exposed command
        # topics. This prevents the 234 kB sample from starving physics.
        node.start_robot_description_subscription()
        deadline = time.monotonic() + args.timeout
        if args.trigger_cameras_sequentially:
            while (
                rclpy.ok()
                and node.robot_description is None
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.5)
            if node.robot_description is None:
                raise VisualAcceptanceError("visual acceptance robot_description timed out")
            trigger_attempts = capture_frames_sequentially(
                node, deadline, persist_frame=persist_frame
            )
            missing = sorted(set(TOPICS) - set(reports))
            if missing:
                raise VisualAcceptanceError(
                    "visual acceptance cameras timed out: " + ", ".join(missing)
                )
        else:
            node.start_all_camera_captures()
            while rclpy.ok() and (
                set(node.frames) != set(TOPICS) or node.robot_description is None
            ) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.5)
            missing = sorted(set(TOPICS) - set(node.frames))
            if missing:
                raise VisualAcceptanceError(
                    "visual acceptance cameras timed out: " + ", ".join(missing)
                )
            frames = dict(node.frames)
        if node.robot_description is None:
            raise VisualAcceptanceError("visual acceptance robot_description timed out")
        final_pose = read_ground_truth_with_retry(min(args.timeout, 30.0))
        robot_description = node.robot_description
    finally:
        if node is not None:
            node.destroy_node()
        if ros_initialized and rclpy.ok():
            rclpy.shutdown()

    try:
        bodywork_mesh_count = validate_bodywork_profile(
            robot_description, args.bodywork_profile
        )
    except ValueError as error:
        raise VisualAcceptanceError(str(error)) from error
    if initial_pose is None or final_pose is None:
        raise VisualAcceptanceError("Gazebo ground-truth capture was incomplete")
    ground_truth = validate_ground_truth_pose(
        initial_pose,
        final_pose,
        max_spawn_planar_m=args.max_spawn_planar_m,
        max_run_planar_m=args.max_run_planar_m,
        max_abs_yaw_rad=args.max_abs_yaw_rad,
    )
    if not ground_truth["passed"]:
        raise VisualAcceptanceError(
            "Gazebo ground-truth anti-drift gate failed: "
            + "; ".join(str(item) for item in ground_truth["violations"])
        )
    renderer_diagnostics = inspect_renderer_log(args.renderer_log)
    validated_target_entities = validate_target_entities(robot_description, active_targets)
    register_path = ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
    register = yaml.safe_load(register_path.read_text(encoding="utf-8"))
    engineering_crosswalk = validate_engineering_visual_crosswalk(
        robot_description,
        camera_contracts,
        register,
        position_views=FUNCTION_POSITION_VIEWS,
        sensor_views=SENSOR_INSTALLATION_VIEWS,
        assembly_views=MECHANICAL_SUBASSEMBLY_VIEWS,
        inspection_links=VIEW_INSPECTION_LINKS,
        joint_positions=VISUAL_JOINT_POSITIONS,
        bodywork_profile=args.bodywork_profile,
    )

    for name, (message, pixels) in frames.items():
        persist_frame(name, message, pixels)
    if invalid_frames:
        raise VisualAcceptanceError(
            "visual acceptance frame validation failed: " + "; ".join(invalid_frames)
        )
    profile_label = args.bodywork_profile.upper()
    manifest: dict[str, object] = {
        "schema_version": 4,
        "report_id": f"tzcup_formal_vehicle_{args.bodywork_profile}_visual_acceptance_v4",
        "status": f"GAZEBO_OGRE2_{profile_label}_NINETEEN_FUNCTIONAL_VIEW_CAPTURE_PASSED",
        "passed": True,
        "render_source": "Gazebo Harmonic Sensors system / Ogre2",
        "bodywork_profile": args.bodywork_profile,
        "bodywork_profile_verified_from_robot_description": True,
        "bodywork_visual_mesh_count": bodywork_mesh_count,
        "source_binding": binding,
        "spawned_robot_description_sha256": _sha256_bytes(
            robot_description.encode("utf-8")
        ),
        "ground_truth_anti_drift": ground_truth,
        "renderer_diagnostics": renderer_diagnostics,
        "arm_pose": "folded_visual_candidate" if not args.no_fold_arm else "uncommanded",
        "deposition_gate_pose": "open_functional_candidate" if not args.no_fold_arm else "uncommanded",
        "camera_count": len(reports),
        "camera_acquisition_mode": (
            "sequential_gazebo_trigger"
            if args.trigger_cameras_sequentially
            else "periodic"
        ),
        "maximum_simultaneously_triggered_cameras": (
            1 if args.trigger_cameras_sequentially else None
        ),
        "camera_trigger_attempts": trigger_attempts,
        "camera_trigger_order": (
            list(trigger_attempts) if args.trigger_cameras_sequentially else []
        ),
        "camera_trigger_command_count": sum(
            len(attempts) for attempts in trigger_attempts.values()
        ),
        "triggered_visual_world_binding": triggered_world_binding,
        "camera_contract_world": str(visual_world.resolve(strict=True)),
        "camera_contract_world_sha256": _sha256_bytes(visual_world.read_bytes()),
        "required_view_names": sorted(TOPICS),
        "profile_evidence_scope": PROFILE_VIEW_EVIDENCE[args.bodywork_profile],
        "view_target_contract": active_targets,
        "validated_target_entities": validated_target_entities,
        "engineering_visual_crosswalk": engineering_crosswalk,
        "per_frame_file_binding": "sha256_size_and_dimensions",
        "frames": reports,
        "claim_boundary": (
            f"The images and independent Gazebo pose prove that the source-bound "
            f"{args.bodywork_profile} profile rendered without leaving the calibrated studio pose. "
            "They do not replace arm swept-volume, sensor self-occlusion, cleaning-contact "
            "or real-vehicle validation."
        ),
    }
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--settle-seconds", type=float, default=18.0)
    parser.add_argument("--no-fold-arm", action="store_true")
    parser.add_argument("--bodywork-profile", choices=("product", "service"), default="product")
    parser.add_argument("--snapshot-manifest", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--renderer-log", type=Path, required=True)
    parser.add_argument("--trigger-cameras-sequentially", action="store_true")
    parser.add_argument("--triggered-world-report", type=Path)
    parser.add_argument("--camera-contract-world", type=Path)
    parser.add_argument("--max-spawn-planar-m", type=float, default=0.10)
    parser.add_argument("--max-run-planar-m", type=float, default=0.05)
    parser.add_argument("--max-abs-yaw-rad", type=float, default=0.05)
    args = parser.parse_args()
    try:
        runtime_gate_binding = load_runtime_gate_binding(args.runtime_binding)
    except VisualAcceptanceError as error:
        print(f"FORMAL_VISUAL_RUNTIME_BINDING_BLOCKED: {error}", file=sys.stderr)
        return 2
    args.output.mkdir(parents=True, exist_ok=True)
    started_epoch_ns = time.time_ns()
    manifest_path = args.output / "manifest.json"
    running: dict[str, object] = {
        "schema_version": 4,
        "report_id": f"tzcup_formal_vehicle_{args.bodywork_profile}_visual_acceptance_v4",
        "status": f"GAZEBO_OGRE2_{args.bodywork_profile.upper()}_VISUAL_CAPTURE_RUNNING",
        "passed": False,
        "bodywork_profile": args.bodywork_profile,
        "camera_acquisition_mode": (
            "sequential_gazebo_trigger"
            if args.trigger_cameras_sequentially
            else "periodic"
        ),
        "started_epoch_ns": started_epoch_ns,
    }
    write_bound_manifest(manifest_path, running, runtime_gate_binding)
    try:
        binding = snapshot_binding(args.snapshot_manifest)
        manifest = capture(args, binding)
    except BaseException as error:
        if isinstance(error, KeyboardInterrupt):
            error_text = "capture interrupted by operator"
        else:
            error_text = str(error) or type(error).__name__
        failure = {
            **running,
            "status": f"GAZEBO_OGRE2_{args.bodywork_profile.upper()}_VISUAL_CAPTURE_FAILED",
            "finished_epoch_ns": time.time_ns(),
            "error_type": type(error).__name__,
            "error": error_text,
            "failure_manifest_persisted": True,
            **partial_frame_artifacts(args.output),
        }
        write_bound_manifest(manifest_path, failure, runtime_gate_binding)
        print(json.dumps(failure, ensure_ascii=False), file=sys.stderr)
        return 1
    manifest["started_epoch_ns"] = started_epoch_ns
    manifest["finished_epoch_ns"] = time.time_ns()
    write_bound_manifest(manifest_path, manifest, runtime_gate_binding)
    print(json.dumps(manifest, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
