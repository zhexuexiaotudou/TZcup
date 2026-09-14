# 仅仿真作品执行入口

本项目的仅仿真比赛路线使用 `config/competition_sim_only_v1.json`。它将独立仿真闭环、S100P 算法运行验证和提交证据包分开验收。该路线不改变实体硬件门、产品 A–P 门或历史失败报告。

先在板端生成一个不可覆盖的实时探针：

```powershell
py -3 scripts/collect_competition_sim_only_board_probe.py `
  --adb F:\Project\TZcup\.workspace\tools\android-platform-tools\platform-tools\adb.exe `
  --serial <connected-serial> `
  --output F:\Project\TZcup\.workspace\evidence\competition-sim-only-<UTC>\board_probe.json
```

把同一个干净提交传到云端后，在云端仓库内运行：

```bash
python3 scripts/collect_competition_sim_only_remote_probe.py \
  --repo "$PWD" \
  --output /root/autodl-tmp/tzcup-competition-sim-only-<UTC>/remote_probe.json
```

用受保护的 SSH/SFTP 辅助脚本取回 `remote_probe.json`，在探针生成后 15 分钟内绑定源码和两端身份：

```powershell
py -3 scripts/prepare_competition_sim_only_evidence.py `
  --repo $PWD `
  --board-probe F:\Project\TZcup\.workspace\evidence\competition-sim-only-<UTC>\board_probe.json `
  --remote-probe F:\Project\TZcup\.workspace\evidence\competition-sim-only-<UTC>\remote_probe.json `
  --output F:\Project\TZcup\.workspace\evidence\competition-sim-only-<UTC>\t0
```

工具要求源码干净、云端 HEAD/tree 与本地完全一致、S100P 型号/aarch64/BPU/overlay 身份完整。探针只证明当时的机器和源码身份；GPU 渲染仍由 `eglinfo -B` 与实际 Gazebo 运行证明。

完成 T0 后，`run_competition_sim_only_cleaning.sh` 只串行收集四类独立分项：fresh 首图与保存地图重启覆盖、GroundDirt 物理地污清扫、动态障碍，以及由外部正式 producer 提供的在线感知、急停恢复和清后新帧复核报告。它自己不启动后三个 producer；缺少其中任一报告时，组合结果必须为 `COMPETITION_SIM_ONLY_COMPOSITE_COMPONENT_EVIDENCE_BLOCKED`。这个入口不是同场因果闭环，也不能得到 `SIM_DEMO_VERIFIED` 或 Golden Mission 结论。

```bash
export FORMAL_COMPETITION_SIM_ONLY_RUN_ROOT=/root/autodl-tmp/tzcup-competition-sim-only-<UTC>/component-run
export FORMAL_COMPETITION_SIM_ONLY_RUNTIME_WS=/root/autodl-tmp/tzcup-competition-sim-only-<UTC>/runtime-ws
export FORMAL_COMPETITION_SIM_ONLY_CLOSURE_MANIFEST="$FORMAL_COMPETITION_SIM_ONLY_RUNTIME_WS/final_runtime_closure_manifest.json"
export FORMAL_COMPETITION_SIM_ONLY_PERCEPTION_REPORT=/absolute/path/to/fresh-online-perception.json
export FORMAL_COMPETITION_SIM_ONLY_EMERGENCY_BRAKING_REPORT=/absolute/path/to/fresh-estop-recovery.json
export FORMAL_COMPETITION_SIM_ONLY_POST_CLEAN_REPORT=/absolute/path/to/fresh-post-clean-frame.json
bash scripts/run_competition_sim_only_cleaning.sh
```

每次使用 fresh session、runtime、run-root、ROS domain 和 Gazebo partition。原 `run_formal_single_episode_cleaning_mission.sh` 强制 20 个立方体并要求真实腕部 RGB-D 抓取复核；在该复核完成前，它不能作为当天清扫闭环的 PASS 来源。清扫分项必须报告 `grasp_status=NOT_EXECUTED`，不得替代 A12。正式提交所需的同场“识别定位→规划靠近→清扫→清后复查→避障→急停”目前仍是待完成硬门。

板端报告必须把输入标记为 `simulation` 或 `replay`，同时记录板上实际进程、可执行文件、模型、输入输出时间戳和资源。PC 生成的 map、path 或控制量不能转发后计作板端算法计算。没有真实外设时不得标记 HIL 或实车验证。

当前冲刺暂不录制视频，但所有 runner 应保留以后录制所需的同源日志、rosbag、看板数据和版本清单。
