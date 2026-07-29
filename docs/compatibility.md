# 环境兼容性结论

## 2026-07-29 本机 Ubuntu 24.04 WSLg 验收

- 新建 WSL2 发行版 `TZcup-Ubuntu-24.04`，rootfs 位于本地 `F:\WSL\TZcup-Ubuntu-24.04\ext4.vhdx`，不依赖 NAS。
- 系统为 Ubuntu 24.04.4 LTS，WSL 2.7.3.0、WSLg 1.0.73；ROS 2 Jazzy Desktop、`ros_gz`、Gazebo Sim 8.11.0、Nav2、SLAM Toolbox 与 Fields2Cover 已安装。
- Windows 本地回环代理通过 WSL mirrored networking 复用；systemd、WSLg、`/dev/dxg` 与 NVIDIA GPU 可见性均已验证。
- `GALLIUM_DRIVER=d3d12`、`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 时，`glxinfo -B` 报告 `D3D12 (NVIDIA GeForce RTX 4080 Laptop GPU)`、OpenGL 4.6、`Accelerated: yes`。
- 在干净克隆 `main@11ee369590f543d78eab66b7e790ba27c82cc0d5` 上导入锁定的 Linorobot2 与 OpenNav Coverage 后，全工作空间构建通过；最终测试汇总为 `449 tests / 0 errors / 0 failures / 49 skipped`。
- Gazebo GUI 实际显示清扫车三维场景、道路和障碍物；Nav2 默认 RViz 布局实际显示地图、RobotModel、TF、LaserScan 与 Navigation2 面板。运行中 smoke check 为 11/11 必需 topic，`missing_topics=[]`、`success=true`。

复核证据见 `artifacts/wslg_gui_20260729_evidence/`。RViz 启动时曾出现一次 GLSL sampler link warning，随后地图和视图正常建立；rosdep 对上游 `ament_python` 与 `python-trimesh-pip` 的元数据仍有 warning，但不影响本次 build/test。该验收只补足本机 WSLg 图形与基础运行链，不外推到真实车辆、真实域、J6 或正式人工门。

## 2026-07-14 当前主机

- 宿主：Windows 11 64-bit（build 26200）。
- GPU：NVIDIA GeForce RTX 4080 Laptop GPU，驱动 595.79，显存 12,282 MiB。
- 本机 PATH 未发现 `ros2`、`colcon`、`gz` 或 `vcs`。
- WSL 当前没有 Ubuntu 24.04 发行版；已有发行版均不是本项目目标运行环境。
- Docker Desktop 29.5.2 可启动 Linux 容器，可用于 Ubuntu 24.04/Jazzy 的构建、单元测试和 headless 检查。当前验证镜像使用中科大 USTC ROS 2 镜像下载官方签名二进制包；镜像地址可通过 Docker build argument 覆盖。

## 2026-07-14 历史结论

不在 Windows 原生环境混装 ROS/Gazebo，也不降级到 Humble/Fortress。当前采用两层推进：

1. Docker 中固定 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic，并通过 NVIDIA GPU passthrough 完成可重复构建、Ogre2 headless 渲染、传感器、TF 和车辆动力学验收；
2. GUI 交互和截图证据在当时仍需 Ubuntu 24.04 原生或 Ubuntu 24.04 WSLg 环境复核；该缺口已由 2026-07-29 本机验收补齐。

Docker 的 headless GPU 成功已经覆盖真实 Gazebo 物理、传感器、ROS 话题、hybrid 定位、Nav2 和完整 Coverage 闭环，但不能替代 GUI 交互与截图验收。当时提出的新装 Ubuntu 24.04 WSLg 路径已于 2026-07-29 落地并完成图形复核。

Stage4W 已在同一 headless GPU 通道完成 hybrid 定位 10-seed、静态完整 Coverage 5-seed、动态障碍 20 次交互、过滤器、30 次急停和 MCAP 回放。每次静态任务都执行统一几何生成的 17/17 组件；当时的 GUI 缺口不阻塞这些计算与运行证据，现已由 2026-07-29 WSLg 复核补足基础视觉验收。

Stage5A 继续使用该 Docker/headless GPU 通道完成 14 项 ROS 测试、20-scene synthetic 数据、held-out ONNX、30-seed task-state E2E 和真实 Gazebo RGB-D/2D/3D/map 感知录包。该兼容性结论不外推到真实数据精度、J6 工具链/实板或原生 GUI。

Stage5B 至 Stage5BR6W 使用独立 `tzcup/sanitation-jazzy:stage5b` 镜像，在 Stage5A 基础上固定 PyTorch 2.5.1+cu124、ONNX 1.17.0 和 ONNX Runtime 1.20.1，RTX 4080 Laptop GPU 可用于训练。Stage5BR3 已在该 headless GPU 通道完成六个不同世界的真实车辆 RGB-D/semantic/instance 同步契约、80 scene/800 frame 原生采集与 QA、四档分辨率扫描和三次 split-model screening；Stage5BR5 随后完成 V1/V2/V4 六世界相机消融，Stage5BR6-A 又通过实际 V4 精确同步链采集 70 张 label=0 hard-negative。Stage5BR6W 在同一通道实际启动 V4/candidate-footprint Stage4W seed 0，并在 `no_reachable_clean_route` 处 fail-closed；这证明运行环境可执行该 profile，但不构成工程 Oracle 通过。正式阻断仍包括两名独立真人 response 未返回和 Horizon J6 工具链；原生 Ubuntu/WSLg GUI 不再是本机独立缺口。

AUTO-01 继续复用该镜像和 NVIDIA headless 通道，实际构建并运行 opt-in G2-C3：3/3 冷启动、seed0 17/17 完整 Coverage、MCAP 回放以及低/高障碍各 30 次均通过。该结果证明 `V5_retracted`、点云自滤波和 Nav2 安全链在当前容器环境可重复运行；GUI 已于后续 WSLg 基础复核中补足，但仍不外推为真实车辆、真实域或 J6 验收。

AUTO-02 在同一镜像和专用 overlay 中进一步完成 5/5 冷启动、五个静态 seed、20/20 动态交互、keepout/限速、30/30 急停和六个 MCAP 回放门，并冻结 `autonomous_navigation_profile_v1`。受影响的 navigation、coverage、tasks 三包重新构建后共 `72` 项测试无失败；AUTO-02 本身仍是 headless 机器仿真结论，2026-07-29 的 WSLg 复核只补足图形运行，不补足真实车辆、真实域或 J6 证据。

## 第三方锁定

精确版本见 `repos/locked_revisions.json`。2026-07-14 远端核查发现：

- Linorobot2 `jazzy` 存在并锁定；
- OpenNav Coverage `jazzy-v2` 存在并锁定；
- `v1.2.1-devel` 不存在，不再作为自动回退；
- `main` 存在，但未选用。
