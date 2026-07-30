# 操作员指南

## 环境

推荐 Windows 11 + Docker Desktop + NVIDIA GPU，或 Ubuntu 24.04、
ROS 2 Jazzy、Gazebo Harmonic。首次使用先阅读根目录 `README.md`、
`README_FIRST.md` 和 `docs/compatibility.md`。

## 一键入口

```powershell
# 快速 CI、状态不变量和秘密扫描
powershell -ExecutionPolicy Bypass -File scripts/run_auto16_release.ps1 -Mode Validate

# Docker 内执行基础构建门
powershell -ExecutionPolicy Bypass -File scripts/run_auto16_release.ps1 -Mode Build

# 在已安装 ROS 2/Gazebo 的 TZcup-Ubuntu-24.04 WSL 中启动基线
powershell -ExecutionPolicy Bypass -File scripts/run_auto16_release.ps1 -Mode Simulation

# 查看正式竞赛矩阵；依赖未通过时以非零退出并给出阻断层
powershell -ExecutionPolicy Bypass -File scripts/run_auto16_release.ps1 -Mode Matrix
```

启动后先检查 `/clock`、TF、里程计、激光雷达、RGB-D、Nav2 lifecycle 和安全
节点，再下发任务。急停、安全区和命令超时不得关闭。训练 GT 默认必须关闭。

## 当前运行边界

AUTO-15 综合矩阵未通过，因此 `Matrix` 模式会按设计 fail closed。真实域与
J6 实机也没有通过。操作员不得把 AUTO-02、09、10、11、12 的独立组件证据
描述成综合比赛成绩。

## 故障排除

- Docker 镜像缺失：先运行 `scripts/run_docker_preflight.ps1`；
- ROS 依赖缺失：按 `README_FIRST.md` 导入 `repos/simulation.repos` 后运行
  `rosdep install`；
- Gazebo 无画面：检查 WSLg、D3D12/OpenGL 和 `DISPLAY`；
- Nav2 未激活：查看 lifecycle、TF 树、map/odom/base_link 和参数服务；
- Matrix 阻断：读取 `FINAL_BLOCKER_REGISTER.json`，禁止手工改写状态。
