#!/usr/bin/env python3
"""Assemble the Stage4S review boundary without inventing downstream evidence."""

import hashlib
import json
import shutil
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "artifacts" / "stage4s_20260715_review"
GT = ROOT / "artifacts" / "stage4s_gt_20260715_051547"
FIT = ROOT / "artifacts" / "stage4s_fit_20260715_062931"
FRICTION = ROOT / "artifacts" / "stage4s_friction_20260715_072313"
MOTION = ROOT / "artifacts" / "stage4s_motion_20260715_070843"


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(name, payload):
    path = OUT / name
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def skipped(stage, reason, required_precondition):
    return {
        "schema_version": 1,
        "stage": stage,
        "executed": False,
        "status": "blocked_by_first_failed_layer",
        "first_failed_layer": "layer_1_body_command_tracking",
        "required_precondition": required_precondition,
        "reason": reason,
        "success": False,
    }


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    copies = {
        GT / "ground_truth_identity_report.json": "ground_truth_identity_report.json",
        GT / "ground_truth_transform_inventory.json": "ground_truth_transform_inventory.json",
        FIT / "wheel_parameter_fit.json": "wheel_parameter_fit.json",
        FRICTION / "friction_slip_scan.json": "friction_slip_scan.json",
        FRICTION / "selected_vehicle_dynamics.yaml": "selected_vehicle_dynamics.yaml",
        MOTION / "motion_calibration_report.json": "motion_calibration_report.json",
        MOTION / "fault_isolation_report.json": "fault_isolation_report.json",
        MOTION / "motion_calibration_trajectory.png": "motion_calibration_trajectory.png",
        MOTION / "rosbag_info.txt": "rosbag_info.txt",
    }
    for source, name in copies.items():
        shutil.copy2(source, OUT / name)

    gt = read_json(GT / "ground_truth_identity_report.json")
    motion = read_json(MOTION / "motion_calibration_report.json")
    friction = read_json(FRICTION / "friction_slip_scan.json")
    segments = {item["segment"]: item for item in motion["segments"]}
    high = segments["turn_positive_360_0p60"]
    summary = {
        "schema_version": 1,
        "baseline_commit": "413b6ebfb16d40e00a820c1dcf8cb5c87c90e566",
        "branch": "agent/stage4s-motion-calibration",
        "pull_request": "https://github.com/zhexuexiaotudou/TZcup/pull/6",
        "ci": {
            "status": "pass",
            "check": "fast-validation",
            "url": "https://github.com/zhexuexiaotudou/TZcup/actions/runs/29398931666/job/87298758826",
        },
        "ground_truth_identity_pass": bool(gt.get("success")),
        "selected_vehicle_dynamics": yaml.safe_load(
            (FRICTION / "selected_vehicle_dynamics.yaml").read_text(encoding="utf-8")
        )["vehicle_dynamics"],
        "motion_experiment_completed": bool(motion["experiment_completed"]),
        "realistic_motion_calibration_pass": False,
        "first_failed_layer": "layer_1_body_command_tracking",
        "blocking_metric": {
            "segment": "turn_positive_360_0p60",
            "body_yaw_error_deg": high["body_yaw_error_deg"],
            "threshold_deg": motion["thresholds"]["body_turn_tracking_error_deg"],
        },
        "raw_odom_preliminary_gate_pass": motion["fault_isolation"]["raw_odom_valid"],
        "imu_preliminary_gate_pass": motion["fault_isolation"]["imu_valid"],
        "friction_slip_grid_executed": friction["executed"],
        "friction_slip_grid_complete": friction["grid_complete"],
        "ekf_ablation_executed": False,
        "oracle_realistic_executed": False,
        "map_geometry_executed": False,
        "localization_trials_executed": False,
        "coverage_regression_executed": False,
        "READY_FOR_GPT_REVIEW_STAGE4S": False,
        "READY_FOR_STAGE5A": False,
        "stage5_or_j6_started": False,
    }
    write_json("stage4s_summary.json", summary)
    write_json(
        "ekf_ablation_report.json",
        skipped(
            "Stage4S-5 EKF ablation",
            "Stage4S forbids tuning a later layer while body command tracking is the first failed layer.",
            "realistic motion calibration Layers 0-3 pass",
        ),
    )
    (OUT / "selected_ekf.yaml").write_text(
        yaml.safe_dump(
            {
                "selected": False,
                "reason": "not evaluated because Layer 1 body command tracking failed",
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    write_json(
        "oracle_realistic_report.json",
        skipped(
            "Stage4S-6 oracle and realistic lanes",
            "Both lanes are downstream of the motion-calibration gate.",
            "realistic motion calibration pass",
        ),
    )
    write_json(
        "map_geometry_report.json",
        skipped(
            "Stage4S-7 map geometry",
            "The prompt permits rebuilding the map only after calibration passes; no overlay was generated.",
            "realistic motion calibration pass",
        ),
    )
    write_json(
        "realistic_localization_report.json",
        skipped(
            "Stage4S-8 realistic localization",
            "No new calibrated map or selected EKF exists, so 10-seed AMCL evidence would be invalid.",
            "map geometry gate pass",
        ),
    )
    write_json(
        "oracle_localization_report.json",
        skipped(
            "Stage4S-8 oracle localization",
            "Oracle lane was not opened after the earlier calibration gate failed.",
            "oracle/realistic lane gate reached",
        ),
    )
    write_json(
        "coverage_regression_report.json",
        skipped(
            "Stage4S-9 coverage and safety regression",
            "The prompt forbids full Coverage before realistic localization passes.",
            "realistic AMCL XY RMSE <= 0.05 m",
        ),
    )
    write_json(
        "efficiency_report.json",
        {
            "schema_version": 1,
            "executed": False,
            "status": "unchanged_boundary",
            "theoretical_capacity_m2_per_h": 1053.0,
            "inputs": {"brush_width_m": 0.65, "speed_m_per_s": 0.45},
            "competition_target_m2_per_h": 3500.0,
            "competition_efficiency_pass": False,
            "reason": "Stage4S did not alter the independent mechanical efficiency boundary.",
        },
    )

    evidence = list(copies.keys()) + [
        FIT / "selected_vehicle_dynamics.yaml",
        GT / "ground_truth_identity_bag" / "ground_truth_identity_bag_0.mcap",
        MOTION / "motion_calibration_bag" / "motion_calibration_bag_0.mcap",
    ]
    evidence.extend(
        path for path in OUT.iterdir() if path.is_file() and path.name != "manifest.sha256"
    )
    unique = sorted(set(evidence), key=lambda path: str(path).lower())
    manifest = []
    for path in unique:
        rel = path.relative_to(ROOT).as_posix()
        manifest.append(f"{sha256(path)}  {rel}")
    (OUT / "manifest.sha256").write_text("\n".join(manifest) + "\n", encoding="utf-8")

    review = f"""# GPT_REVIEW_STAGE4S

## 结论

`READY_FOR_GPT_REVIEW_STAGE4S=false`  
`READY_FOR_STAGE5A=false`

Stage4S 在首个失败层 `layer_1_body_command_tracking` 停止。专用 Gazebo 真值身份自证通过；轮半径/轮距完成粗细网格拟合；无障碍、仿真时钟一致的 13 段开环实验完整结束。唯一的 Layer 1 阻断项是高速原地正转整圈：车体 yaw 误差 `{high['body_yaw_error_deg']:.4f}°`，门槛 `≤18°`。降低横向摩擦或启用 WheelSlip 均恶化结果，因此未伪选失败配置。

## 基线、分支、PR 与 CI

- 基线：`413b6ebfb16d40e00a820c1dcf8cb5c87c90e566`
- 分支：`agent/stage4s-motion-calibration`
- PR：`https://github.com/zhexuexiaotudou/TZcup/pull/6`
- CI：`fast-validation` 已通过；`https://github.com/zhexuexiaotudou/TZcup/actions/runs/29398931666/job/87298758826`

## Ground truth 身份

- 使用模型级 `OdometryPublisher` 输出 `/ground_truth/model_odom_raw`，不再依赖匿名 `Pose_V.transforms[0]`。
- 适配器严格校验 `frame_id=world`、`child_frame_id=sanitation_vehicle/base_footprint`，错误时 fail-closed。
- 出生点、静止 20 s、前进 1 m、正负 90° 和 world→map_gt 变换均通过。

## 运动标定与参数拟合

- 选择：`drive_wheel_radius=0.14 m`，`drive_wheel_separation=1.22 m`。
- 5 m 车体直线误差：0.30%–0.91%；raw odom 相对真值误差均低于 1%。
- 低速正反整圈车体误差：1.31° / 1.52%；raw odom 与 IMU 初步门槛通过。
- 四个圆弧半径均在 15% 门槛内。
- 高速 +360° 车体误差 `{high['body_yaw_error_deg']:.4f}°`，Layer 1 失败。
- 完整摩擦/WheelSlip 网格保存在 `friction_slip_scan.json`；默认接触为网格最优但仍不通过。

## EKF、Oracle/realistic、地图、定位与 Coverage

由于 Layer 1 是首个失败层，Stage4S-5 至 Stage4S-9 均按提示词强制顺序未执行。对应 JSON 明确记录 `executed=false` 和前置条件；没有生成伪造的 map overlay、10-seed 定位或 Coverage 成功证据。

## Filters、安全、动态障碍、急停与效率

本轮未进入 Coverage/安全回归。Stage4R 证据保持历史状态，不提升为 Stage4S 通过。独立效率边界仍为 `0.65×0.45×3600=1053 m²/h`，未达到 3500 m²/h。

## Gazebo 实帧、rosbag 与完整性

- 真实 Gazebo 轨迹图：`artifacts/stage4s_20260715_review/motion_calibration_trajectory.png`。
- 真值身份 MCAP 与 13 段运动 MCAP 均保留，`rosbag_info.txt` 可审查话题与时长。
- `manifest.sha256` 覆盖关键 JSON、YAML、PNG 与两个 MCAP。

## 风险

- P0：高速原地转向误差超过 Layer 1 门槛，禁止进入 Stage5A。
- P1：EKF 消融、新地图与 10-seed AMCL 尚未执行，因为其前置门未通过。
- P2：WheelSlip 插件候选使高速转向更差；后续应研究速度相关运动模型或控制瞬态，而非继续盲扫静态摩擦。
"""
    (ROOT / "GPT_REVIEW_STAGE4S.md").write_text(review, encoding="utf-8")
    print(OUT)


if __name__ == "__main__":
    main()
