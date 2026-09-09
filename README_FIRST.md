# TZcup 安装、启动与验收入口

## 1. 推荐环境

- Ubuntu 24.04（原生或 WSL2/WSLg）
- ROS 2 Jazzy Desktop
- Gazebo Harmonic
- Python 3.12
- 可选 NVIDIA GPU
- Windows 无头阶段门可使用 Docker Desktop

本机约定的 WSL distribution 为 `TZcup-Ubuntu-24.04`。运行前先确认：

```powershell
wsl.exe -d TZcup-Ubuntu-24.04 -- bash -lc "source /opt/ros/jazzy/setup.bash && ros2 --help >/dev/null && gz sim --versions"
```

## 2. 首次构建

在 Ubuntu/WSL 中执行：

```bash
source /opt/ros/jazzy/setup.bash
export SANITATION_WS="$HOME/sanitation_ws"
mkdir -p "$SANITATION_WS/src"
rsync -a starter_ws/src/ "$SANITATION_WS/src/"

bash scripts/bootstrap_jazzy.sh
bash scripts/import_upstream.sh
bash scripts/build_ws.sh
```

## 3. 产品默认演示

从 Windows PowerShell 的仓库根目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_visual_demo.ps1
```

默认 profile 是：

```text
DriveModel=ackermann
CoverageProfile=ackermann
MapSize=small
```

它使用物理前轮转向、后轮牵引、Ackermann odometry/EKF、Hybrid-A* / Reeds-Shepp 与无原地旋转的 Coverage connector。可视演示只证明当前链路可运行，不产生产品完成声明。

只打开 Gazebo 原生控制面板：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_gazebo_cleaning_demo.ps1
```

显式历史回归才允许选择 skid-steer：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_visual_demo.ps1 `
  -DriveModel skid_steer_legacy -CoverageProfile optimized -MapSize small
```

不得把该 legacy 结果作为 V1 产品验收证据。

## 4. 基础 ROS 启动

在 Ubuntu/WSL 中：

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/sanitation_ws/install/setup.bash"
bash scripts/run_baseline.sh
```

另开终端检查 topic、TF 与 lifecycle：

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/sanitation_ws/install/setup.bash"
ros2 topic list
ros2 node list --no-daemon
```

## 5. 代码与合同检查

Windows：

```powershell
py -3 scripts/ci_fast.py
py -3 scripts/product_acceptance.py validate-contract
```

Linux/CI：

```bash
python scripts/ci_fast.py
python scripts/product_acceptance.py validate-contract
```

快速 CI 不替代 ROS build、Gazebo、30 seeds、性能、soak、fault、replay 或 sealed final。

## 6. 正式验收工作区

正式证据必须放在 Git 外的独立目录，并绑定源码、模型、配置、数据、容器、依赖、seed、命令、退出码和每个证据文件的 SHA-256。

生成 manifest 骨架：

```powershell
py -3 scripts/product_acceptance.py template `
  --output C:\tzcup-evidence\acceptance_evidence_manifest.json
```

完成所有 A–P 运行并准备 release 后执行：

```powershell
py -3 scripts/product_acceptance.py evaluate `
  --evidence-manifest C:\tzcup-evidence\acceptance_evidence_manifest.json `
  --evidence-root C:\tzcup-evidence `
  --output-dir C:\tzcup-evidence\final
```

裁决器生成：

```text
FINAL_ACCEPTANCE_STATUS.json
FINAL_ACCEPTANCE_MATRIX.json
FINAL_EVIDENCE_INDEX.md
```

正式输出默认禁止覆盖；如果一次性证据已消费或阈值、模型、代码发生变化，应创建新的开发/冻结版本，不能覆盖原 final 后重考。

## 7. 重要边界

- Production Target List 与 DynamicTrashMap 必须空启动。
- GT 只用于 post-run evaluator。
- Ackermann product run 禁止 Spin、RotateInPlace、point turn 和 zero-speed yaw。
- `actuator success` 不等于 `CLEANED`；清扫后必须重新进入真实 camera FOV 验证。
- `SIMULATION_PRODUCT_COMPLETE` 不等于目标平台或真实场地通过。
- 原始数据、bag、缓存、SDK、checkpoint 和逐帧日志不提交 Git。

门槛与当前缺口分别见 [STAGE_GATES.md](STAGE_GATES.md) 和 [docs/current-status.md](docs/current-status.md)。

## 8. Journey 6 PC 先行检查

在板卡或官方 SDK 到货前，只执行发现、合同和 bundle dry-run，不猜 SKU 或
`march`：

```powershell
py -3 scripts/j6_discover_sdk.py --output C:\tzcup-j6\J6_SDK_INVENTORY.json
py -3 scripts/j6_pc_status.py --output-dir C:\tzcup-j6\status
```

缺少官方 J6 SDK、通过固定开发集门槛的预训练模型证据，或缺少/未通过 30 分钟正式 HIL 证据时，命令以非零退出并
写入 blocker JSON，这是预期的失效关闭结果。完整流程见
[OpenExplorer 工作流](docs/journey6-openexplorer-workflow.md)、
[板端部署](docs/journey6-board-deployment.md) 与
[板卡到货手册](docs/journey6-board-arrival-runbook.md)。
