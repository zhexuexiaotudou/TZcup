# 环境兼容性

## 已验证的本机路径

- Windows 11 + WSL2 Ubuntu 24.04。
- ROS 2 Jazzy、Gazebo Harmonic / Gazebo Sim 8.11.0、Nav2、SLAM Toolbox。
- WSLg 可使用 D3D12/NVIDIA 图形加速；Gazebo 与 RViz 均可启动。
- Docker Desktop 可用于可重复构建、单元测试和 headless 检查。

上述结论只证明工具链和基础运行环境可用，不等同于 A–P 产品验收。当前实时 Ackermann 运行状态和精确指标见 `docs/current-status.md`。

## 固定依赖

第三方仓库与版本以 `repos/locked_revisions.json` 为准。不得在正式证据运行中临时跟随上游 `main`，也不得用未记录的本地包替换锁定依赖。

## 图形运行

```bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
glxinfo -B
```

`glxinfo` 应报告硬件加速。图形显示成功只用于可视化验收；传感器、TF、导航、定位、覆盖和安全仍须读取 ROS 运行证据。

## 支持边界

- 正式基线：Ackermann 驱动、Ackermann 覆盖规划。
- `skid_steer_legacy` 与历史优化覆盖档位仅用于回归和资产展示。
- 真实车辆、真实域数据、Horizon J6 板端和生产部署不由本机 WSLg 可用性自动证明。
