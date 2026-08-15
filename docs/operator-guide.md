# 操作员指南

本指南只描述当前可执行路径。产品准入以仓库根目录的固定 A–P 合同为唯一口径；任何单项演示、历史 AUTO 阶段记录或人工改写状态，都不能替代正式验收。

## 1. 环境与预检

推荐 Windows 11 + WSL2 Ubuntu 24.04，ROS 2 Jazzy 与 Gazebo Harmonic。首次运行先阅读 `README_FIRST.md` 和 `docs/compatibility.md`。

```powershell
py -3 scripts/ci_fast.py
py -3 scripts/product_acceptance.py validate-contract
```

两条命令必须以零退出码结束。合同校验会检查验收标准原文的 SHA-256，禁止静默修改阈值。

## 2. 启动默认 Ackermann 仿真

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_visual_demo.ps1
```

默认车型、覆盖规划和地图档位分别为 `ackermann`、`ackermann` 和 `small`。训练/控制不得读取 Gazebo ground truth。启动后依次检查 `/clock`、TF、里程计、激光雷达、RGB-D、Nav2 lifecycle 和安全节点，再派发任务。急停、安全区和命令超时保护不得关闭。

只需要 Gazebo 原生任务控制界面时，可运行 `scripts\run_gazebo_cleaning_demo.ps1`；它使用同一 Ackermann 默认链，不改变验收口径。

需要保留证据时，将输出目录放在仓库外的专用目录，并保存完整命令、退出码、原始日志、JSON 报告和文件哈希。

## 3. 生成并评估 A–P 证据

```powershell
py -3 scripts/product_acceptance.py template --output F:\Project\TZcup-product-evidence\formal-run
py -3 scripts/product_acceptance.py evaluate --evidence-root F:\Project\TZcup-product-evidence\formal-run
```

`template` 只生成待填写的证据骨架，不代表通过。`evaluate` 默认拒绝覆盖已有最终结果；只有 16 个门、131 项检查、14 个全局否决项和最终发布物全部满足时，顶层状态才可能为 `true`。

最终输出为：

- `FINAL_ACCEPTANCE_STATUS.json`
- `FINAL_ACCEPTANCE_MATRIX.json`
- `FINAL_EVIDENCE_INDEX.md`
- 恰好一个匹配合同命名规则的发布 ZIP

## 4. 当前运行边界

当前真实 Ackermann 基线已经完成一次完整小场景任务，但定位 P95、重复覆盖率和全流程效率未达到 V1 硬门槛。因此产品准入状态为失败；不得把“仿真能跑完”描述成“产品验收通过”。详见 `docs/current-status.md`。

## 5. 故障排查

- Docker 镜像缺失：运行 `scripts/run_docker_preflight.ps1`。
- ROS 依赖缺失：按 `README_FIRST.md` 导入 `repos/simulation.repos`，再运行 `rosdep install`。
- Gazebo 无画面：检查 WSLg、D3D12/OpenGL 与 `DISPLAY`。
- Nav2 未激活：检查 lifecycle、TF 树、`map/odom/base_link` 和参数服务。
- 验收失败：读取 `FINAL_ACCEPTANCE_MATRIX.json` 的首个失败检查及原始证据，修复实现或重新采样；禁止调低合同阈值。
