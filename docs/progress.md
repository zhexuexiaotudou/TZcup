# 项目推进记录

## Stage 0：预检与基线锁定

状态：已通过（容器 headless 预检门）。

已完成：

- 将用户提供的完整推进包扁平导入仓库根目录；原工作区为空，没有覆盖既有成果。
- 校验 `MANIFEST.json` 的 35 个条目均存在且字节数一致。
- 初始化 Git，并以独立基线提交保存原始推进包。
- 完成 Windows 宿主、GPU、磁盘、WSL、Docker 与本机 ROS 工具 inventory。
- 实时核查并锁定 Linorobot2、OpenNav Coverage 和 Fields2Cover 上游版本。
- 修复预检脚本，使其输出结构化检查、阻塞项、告警、命令路径与原始探针结果。
- 修复第三方导入策略：使用精确 commit、拒绝覆盖 dirty checkout、校验最终 SHA。
- 构建 `tzcup/sanitation-jazzy:stage0` 验证镜像并执行预检；脚本返回 0。
- 验证 Ubuntu 24.04.4、ROS 2 Jazzy、Gazebo Sim 8.11.0、`ros_gz`、colcon、rosdep、vcs 全部可用。
- 验证 `ros-jazzy-fields2cover` 2.0.0 二进制包可安装。

当前边界：

- Windows 宿主不满足直接运行 ROS 2 Jazzy/Gazebo Harmonic 的要求。
- Docker 可作为 Ubuntu 24.04/Jazzy headless 构建通道；GUI 与动力学证据仍需 Ubuntu 24.04 原生或 WSLg。

证据：

- `artifacts/preflight.json`
- `artifacts/stage0_20260714_*/preflight.json`
- `artifacts/stage0_20260714_*/preflight_run.log`
- `artifacts/stage0_20260714_*/host_inventory.json`

复现命令：

```powershell
docker desktop start
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_docker_preflight.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\collect_stage0_evidence.ps1
```

下一步：进入 Stage 1，在同一 Jazzy 镜像中导入精确锁定的第三方仓库并完成两次构建与测试。
