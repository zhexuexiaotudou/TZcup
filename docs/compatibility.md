# 环境兼容性结论

## 2026-07-14 当前主机

- 宿主：Windows 11 64-bit（build 26200）。
- GPU：NVIDIA GeForce RTX 4080 Laptop GPU，驱动 595.79，显存 12,282 MiB。
- 本机 PATH 未发现 `ros2`、`colcon`、`gz` 或 `vcs`。
- WSL 当前没有 Ubuntu 24.04 发行版；已有发行版均不是本项目目标运行环境。
- Docker Desktop 29.5.2 可启动 Linux 容器，可用于 Ubuntu 24.04/Jazzy 的构建、单元测试和 headless 检查。当前验证镜像使用中科大 USTC ROS 2 镜像下载官方签名二进制包；镜像地址可通过 Docker build argument 覆盖。

## 结论

不在 Windows 原生环境混装 ROS/Gazebo，也不降级到 Humble/Fortress。当前采用两层推进：

1. Docker 中固定 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic，完成可重复构建、静态检查和可行的 headless 测试；
2. Stage 2 的 GUI、传感器、TF、车辆动力学和截图证据，必须在 Ubuntu 24.04 原生或 Ubuntu 24.04 WSLg 环境复核。

Docker 的 headless 成功不能替代真实 GUI 和车辆动力学验收。若要在本机完成 Stage 2–4 的完整图形验收，需要新装 Ubuntu 24.04 WSLg，预计额外占用约 15–30 GB，并需要下载 ROS 2、Gazebo 与第三方依赖。

## 第三方锁定

精确版本见 `repos/locked_revisions.json`。2026-07-14 远端核查发现：

- Linorobot2 `jazzy` 存在并锁定；
- OpenNav Coverage `jazzy-v2` 存在并锁定；
- `v1.2.1-devel` 不存在，不再作为自动回退；
- `main` 存在，但未选用。
