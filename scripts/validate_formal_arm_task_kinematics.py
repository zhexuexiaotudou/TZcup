#!/usr/bin/env python3
"""Verify multi-branch, non-singular IK for the formal arm task anchors."""

from pathlib import Path
import hashlib
import json
import runpy

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = ROOT / "reports/engineering/formal_competition_vehicle.urdf"
REPORT_PATH = ROOT / "reports/engineering/formal_arm_task_kinematics_report.json"
s = runpy.run_path(str(ROOT / "scripts/scan_formal_vehicle_inertia_and_swept_volume.py"))
model = s["Model"](URDF_PATH)
names = s["ARM_JOINTS"]
arm_links = model.descendants("ur5e_base_link_inertia")
lo = np.array([model.joints[n].lower for n in names])
hi = np.array([model.joints[n].upper for n in names])

chain = []
link = "tool0"
while link != "base_footprint":
    joint = model.parent_joint[link]
    chain.append(joint)
    link = joint.parent
chain.reverse()


def fk(q):
    positions = dict(zip(names, q))
    transform = np.eye(4)
    for joint in chain:
        position = positions.get(joint.name, 0.0)
        motion = np.eye(4)
        if joint.kind in {"revolute", "continuous"}:
            motion = s["_axis_rotation"](joint.axis, position)
        elif joint.kind == "prismatic":
            motion = s["_translation"](joint.axis, position)
        transform = transform @ joint.origin @ motion
    return transform


def wrap(q):
    return (np.asarray(q) + np.pi) % (2 * np.pi) - np.pi


def residual(q, target):
    current = fk(q)
    position = current[:3, 3] - target[:3, 3]
    rotation = Rotation.from_matrix(target[:3, :3].T @ current[:3, :3]).as_rotvec()
    return np.r_[position, rotation]


def solve(target, seed_count=64):
    seeds = [
        np.asarray(s["TRANSPORT_POSE"]),
        *(np.asarray(q) for q in s["TASK_ANCHORS"].values()),
    ]
    for i in range(1, seed_count + 1):
        seeds.append(np.array([a + s["_halton"](i, b) * (z - a) for b, a, z in zip(s["HALTON_BASES"], lo, hi)]))
    solutions = []
    for seed in seeds:
        fit = least_squares(
            residual,
            np.clip(seed, lo, hi),
            args=(target,),
            bounds=(lo, hi),
            max_nfev=2500,
            xtol=1e-11,
            ftol=1e-11,
            gtol=1e-11,
        )
        if np.linalg.norm(residual(fit.x, target)) > 2e-5:
            continue
        q = wrap(fit.x)
        if not any(np.linalg.norm(wrap(q - other)) < 1e-3 for other in solutions):
            solutions.append(q)
    return solutions


def jacobian(q, epsilon=1e-6):
    columns = []
    for index in range(6):
        plus, minus = np.array(q), np.array(q)
        plus[index] += epsilon
        minus[index] -= epsilon
        tp, tm = fk(plus), fk(minus)
        dp = (tp[:3, 3] - tm[:3, 3]) / (2 * epsilon)
        dr = Rotation.from_matrix(tm[:3, :3].T @ tp[:3, :3]).as_rotvec() / (2 * epsilon)
        columns.append(np.r_[dp, dr])
    return np.column_stack(columns)


def collision_clear(q, label):
    samples = []
    for gripper in (0.0, 0.4, 0.8):
        positions = dict(zip(names, q))
        positions[s["GRIPPER_MASTER"]] = gripper
        positions.update(s["TASK_AUXILIARY_POSITIONS"].get(label, {}))
        samples.append((label, positions))
    audit = s["_collision_audit"](model, samples, arm_links)
    return audit["required_anchor_blocking_counts"][label] == 0


result = {"urdf_sha256": hashlib.sha256(URDF_PATH.read_bytes()).hexdigest(), "anchors": {}}
for label in ("pregrasp", "pick", "deposit"):
    selected = np.asarray(s["TASK_ANCHORS"][label])
    target = fk(selected)
    solutions = solve(target)
    branch_rows = []
    for q in solutions:
        singular_values = np.linalg.svd(jacobian(q), compute_uv=False)
        branch_rows.append({
            "joint_positions_rad": np.round(q, 9).tolist(),
            "minimum_jacobian_singular_value": float(singular_values[-1]),
            "jacobian_condition_number": float(singular_values[0] / singular_values[-1]),
            "collision_clear_all_gripper_states": collision_clear(q, label),
        })
    selected_singular_values = np.linalg.svd(jacobian(selected), compute_uv=False)
    result["anchors"][label] = {
        "selected_joint_positions_rad": selected.tolist(),
        "tool0_xyz_m": np.round(target[:3, 3], 9).tolist(),
        "selected_minimum_jacobian_singular_value": float(selected_singular_values[-1]),
        "selected_jacobian_condition_number": float(selected_singular_values[0] / selected_singular_values[-1]),
        "selected_collision_clear_all_gripper_states": collision_clear(selected, label),
        "unique_ik_branch_count": len(solutions),
        "collision_clear_ik_branch_count": sum(row["collision_clear_all_gripper_states"] for row in branch_rows),
        "branches": branch_rows,
    }

REPORT_PATH.write_text(
    json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
print(json.dumps({label: {
    "xyz": row["tool0_xyz_m"],
    "smin": row["selected_minimum_jacobian_singular_value"],
    "condition": row["selected_jacobian_condition_number"],
    "branches": row["unique_ik_branch_count"],
    "clear_branches": row["collision_clear_ik_branch_count"],
    "selected_clear": row["selected_collision_clear_all_gripper_states"],
} for label, row in result["anchors"].items()}, indent=2))
