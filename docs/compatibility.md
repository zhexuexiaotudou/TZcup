# 环境兼容性结论

## 2026-07-14 当前主机

- 宿主：Windows 11 64-bit（build 26200）。
- GPU：NVIDIA GeForce RTX 4080 Laptop GPU，驱动 595.79，显存 12,282 MiB。
- 本机 PATH 未发现 `ros2`、`colcon`、`gz` 或 `vcs`。
- WSL 当前没有 Ubuntu 24.04 发行版；已有发行版均不是本项目目标运行环境。
- Docker Desktop 29.5.2 可启动 Linux 容器，可用于 Ubuntu 24.04/Jazzy 的构建、单元测试和 headless 检查。当前验证镜像使用中科大 USTC ROS 2 镜像下载官方签名二进制包；镜像地址可通过 Docker build argument 覆盖。

## 结论

不在 Windows 原生环境混装 ROS/Gazebo，也不降级到 Humble/Fortress。当前采用两层推进：

1. Docker 中固定 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic，并通过 NVIDIA GPU passthrough 完成可重复构建、Ogre2 headless 渲染、传感器、TF 和车辆动力学验收；
2. GUI 交互和截图证据仍需在 Ubuntu 24.04 原生或 Ubuntu 24.04 WSLg 环境复核。

Docker 的 headless GPU 成功已经覆盖真实 Gazebo 物理、传感器、ROS 话题、hybrid 定位、Nav2 和完整 Coverage 闭环，但不能替代 GUI 交互与截图验收。若要在本机完成 Stage 2–4W 的完整图形验收，需要新装 Ubuntu 24.04 WSLg，预计额外占用约 15–30 GB，并需要下载 ROS 2、Gazebo 与第三方依赖。

Stage4W 已在同一 headless GPU 通道完成 hybrid 定位 10-seed、静态完整 Coverage 5-seed、动态障碍 20 次交互、过滤器、30 次急停和 MCAP 回放。每次静态任务都执行统一几何生成的 17/17 组件；GUI 缺口不阻塞这些计算与运行证据，但仍是人工视觉验收项。

Stage5A 继续使用该 Docker/headless GPU 通道完成 14 项 ROS 测试、20-scene synthetic 数据、held-out ONNX、30-seed task-state E2E 和真实 Gazebo RGB-D/2D/3D/map 感知录包。该兼容性结论不外推到真实数据精度、J6 工具链/实板或原生 GUI。

Stage5B 至 Stage5BR6 使用独立 `tzcup/sanitation-jazzy:stage5b` 镜像，在 Stage5A 基础上固定 PyTorch 2.5.1+cu124、ONNX 1.17.0 和 ONNX Runtime 1.20.1，RTX 4080 Laptop GPU 可用于训练。Stage5BR3 已在该 headless GPU 通道完成六个不同世界的真实车辆 RGB-D/semantic/instance 同步契约、80 scene/800 frame 原生采集与 QA、四档分辨率扫描和三次 split-model screening；Stage5BR5 随后完成 V1/V2/V4 六世界相机消融，Stage5BR6-A 又通过实际 V4 精确同步链采集 70 张 label=0 hard-negative。当前阻断为两名独立真人 response 尚未返回，因此 V4、policy v2、candidate footprint、Oracle 与训练均保持 fail-closed。Horizon J6 工具链未发现，故转换、量化、实板 FPS 与运行门均 fail-closed；原生 Ubuntu/WSLg GUI 仍是独立人工视觉验收缺口。

## 第三方锁定

精确版本见 `repos/locked_revisions.json`。2026-07-14 远端核查发现：

- Linorobot2 `jazzy` 存在并锁定；
- OpenNav Coverage `jazzy-v2` 存在并锁定；
- `v1.2.1-devel` 不存在，不再作为自动回退；
- `main` 存在，但未选用。
