# AUTO-01 几何、相机与障碍安全闭环

## 结论

AUTO-01 选择 `G2-C3 / auto01_g2_v5_retracted` 作为后续自主仿真的唯一 opt-in 导航配置。它把验证相机机械回收到冻结的 Stage4W 导航 footprint 内，并用 base-frame 点云自滤波消除车辆自身回波。默认 `production` 相机和 footprint 不变。

## 数据链

```text
V5_retracted Gazebo RGB-D
  /verification_camera/depth/color/points
    -> pointcloud_self_filter
       transform: camera frame -> base_footprint
       reject: known vehicle AABB only
    -> /verification_camera/depth/color/points/navigation
       + /scan
    -> Nav2 Collision Monitor
    -> /cmd_vel_gate
    -> velocity_gate
    -> /cmd_vel
```

自滤波包围盒为：

```text
min = [-0.60, -0.43, -0.20] m
max = [ 0.72,  0.43,  0.75] m
```

点云以四倍步长采样，输出帧固定为 `base_footprint`。碰撞监视器仍使用完整 Stage4W footprint，并融合未掩膜 LiDAR 与自滤波 RGB-D 点云。该设计让低于 LiDAR 平面的障碍仍可触发保护，同时避免空场把车体自身当成障碍。

## 候选淘汰记录

| 候选 | 结论 | 首个失败层 |
|---|---|---|
| G1-C1 | 拒绝 | 联合外包络持续收到 LiDAR 自回波 |
| G1-C2 | 拒绝 | 原始 scan 不具备高度语义 |
| G1-C3 | 拒绝 | 两次完整覆盖 RMSE 均超过 0.05 m |
| G2-C1 | 拒绝 | 水平相机不保护低障碍 |
| G2-C2 | 拒绝 | 向下点云包含车体，空场 transit 停滞 |
| G2-C3 | 通过 | 自滤波后全部机器门通过 |

## 复现

```powershell
py -3 scripts/ci_fast.py
py -3 scripts/auto01_geometry_audit.py `
  --profile starter_ws/src/sanitation_navigation/config/auto01_g2_v5_retracted.yaml

powershell -NoProfile -ExecutionPolicy Bypass `
  -File scripts/run_auto01_geometry_docker.ps1 `
  -OutputName autonomous_auto01_recheck `
  -FootprintProfile auto01_g2_v5_retracted `
  -CameraProfile V5_retracted `
  -AttemptId AUTO-01-G2-C3
```

正式障碍矩阵另加：

```powershell
-SkipBuild -SkipCold -SkipFormal -RunG2Obstacle -ObstacleTrials 30
```

紧凑、可提交证据见 `artifacts/autonomous_auto01_20260729_evidence/`。原始 MCAP、完整日志与失败运行只在本地保留，不写入普通 Git 历史。

## 声明边界

本阶段结论仅覆盖 Docker headless ROS 2 Jazzy / Gazebo Harmonic 仿真。它不是两名真人的双盲审计，不是物理车辆验证，也不是 Horizon J6 工具链或板端运行证明；历史人工标志继续为 false。
