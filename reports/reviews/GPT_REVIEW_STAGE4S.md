# GPT_REVIEW_STAGE4S

## 结论

`READY_FOR_GPT_REVIEW_STAGE4S=false`  
`READY_FOR_STAGE5A=false`

Stage4S 在首个失败层 `layer_1_body_command_tracking` 停止。专用 Gazebo 真值身份自证通过；轮半径/轮距完成粗细网格拟合；无障碍、仿真时钟一致的 13 段开环实验完整结束。唯一的 Layer 1 阻断项是高速原地正转整圈：车体 yaw 误差 `19.1825°`，门槛 `≤18°`。降低横向摩擦或启用 WheelSlip 均恶化结果，因此未伪选失败配置。

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
- 高速 +360° 车体误差 `19.1825°`，Layer 1 失败。
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
