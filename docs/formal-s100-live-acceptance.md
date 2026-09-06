# RDK S100P / Journey 6P 真板正式门

该门只接受真 RDK S100P（征程 6P）上的产品运行证据。旧 S100/征程 6 仅作为平台族的历史泛称，不能关闭本项目最终板端门。PC、WSL、Gazebo、交叉编译成功、HBM 文件存在、官方示例截图或手工填写 JSON 均不能通过。采集器首先读取 Linux device-tree；架构、型号或 SoC 标识不符合时立即写出 `FORMAL_RDK_S100_LIVE_RUNTIME_BLOCKED` 并以退出码 4 拒绝继续。

## 证据链

1. 板上采集器固定读取 `/proc/device-tree/model`、`/proc/device-tree/compatible`、`/etc/os-release`、内核与 Horizon/D-Robotics 运行时版本，计算模型/词表 SHA-256。2026-08-30 接入的真板签名为 model `D-Robotics RDK S100P V1P0`、compatible `drobot,s100-rdk`、架构 `aarch64`；后者是平台族 token，并不会重复写出 `journey6p`，因此必须与明确的 S100P model 组合判定，不能单独放行普通 ARM64 设备。
2. DOSOD、EdgeSAM 和产品 adapter 必须在采集前由正式启动方式运行。采集器不会替项目启动、替换或降级任何节点。
3. 采集器保存 ROS 节点、话题、类型和逐节点 `ros2 node info`，持续采样 `/proc` 内存/进程映射与 `/sys/class/thermal` 温度。
4. 离线 validator 同时校验 raw schema、语义门槛、冻结 snapshot、当前 acceptance session 和 frozen runtime closure；只有全部满足才生成合同接受的 `FORMAL_RDK_S100_LIVE_PRODUCT_RUNTIME_PASSED`。采集时记录的 session 起始时间和 snapshot 必须与恢复时的同一 session 一致；session 从 `RUNNING` 进入 S100-pending 时状态文件可以改变，但不得借此复用另一 session。
5. JSON 和 SHA-256 链可发现修改，但不是 TPM 或远程密码学证明；如需对抗主动伪造，应另加受管设备密钥/TPM 签名。

## 产品 diagnostics 合同

`/perception/open_vocab/diagnostics` 必须是 `diagnostic_msgs/msg/DiagnosticArray`。每次真实推理各产生一条 DOSOD 或 EdgeSAM status；`status.name` 必须包含 `dosod` 或 `edgesam`，并带以下 `KeyValue`：

- `backend`: 只能是 `bpu` 或 `cpu`，代表该次实际执行后端；
- `model_sha256`: DOSOD 为实际加载的 HBM hash；EdgeSAM 为实际加载的 encoder、decoder HBM hash，以逗号分隔；
- `latency_ms`: 该次真实推理的正数耗时；
- `inference_ok`: `true`/`false`；失败必须如实报告。

正式节点名为 `hobot_dosod`、`mono_edgesam`、`open_vocab_product_adapter`。后端声明还必须和相应进程的 `/proc/PID/maps` 一致：BPU 需出现 HBRT/HB DNN 库；CPU 不得出现这些 BPU 库。缺诊断、缺模型 hash、节点消失、进程重启、推理失败或 ROS 图出现 simulator/evaluator truth surface 均保持 BLOCKED。

## 真板命令

在与冻结 snapshot 相同的代码检出中、source ROS 环境并启动正式产品节点后运行（路径必须替换成板上真实文件）：

```bash
python3 scripts/collect_formal_s100_live_runtime.py \
  --output artifacts/s100-live/raw-runtime.json \
  --snapshot reports/engineering/formal_vehicle_snapshot_manifest.json \
  --acceptance-session /opt/tzcup/evidence/formal_final_acceptance_session.json \
  --runtime-closure /opt/tzcup/evidence/final_runtime_closure_manifest.json \
  --dosod-hbm /opt/tzcup/models/dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm \
  --dosod-vocabulary /opt/tzcup/models/dosod/tzcup_offline_vocabulary.json \
  --edgesam-encoder-hbm /opt/tzcup/models/edgesam/edgesam_encoder_512.hbm \
  --edgesam-decoder-hbm /opt/tzcup/models/edgesam/edgesam_decoder_512.hbm \
  --dosod-compile-receipt /opt/tzcup/evidence/dosod_compile_receipt.json \
  --dosod-parity-report /opt/tzcup/evidence/dosod_parity_report.json \
  --dosod-metric-report /opt/tzcup/evidence/dosod_metric_report.json \
  --dosod-admission-bundle /opt/tzcup/evidence/dosod_admission_bundle \
  --duration-sec 1800 \
  --sample-period-sec 1
```

2026-08-30 的板端 smoke 已验证项目 overlay、RGB/BGR/NV12 fail-closed 转换、正式 5
节点图和官方参考 DOSOD BPU 推理。该 smoke 使用上游参考词表且没有非空项目目标，只能证明
链路；正式门还要求项目四类 HBM/词表、真实 RGB-D/TF/map、非空 detection/mask/target、
1800 秒资源与温度证据。原始边界摘要见
`artifacts/formal_s100p_board_smoke_20260830.json`。

将 raw artifact 原样复制回与同一 snapshot 对应的验收 worktree，再离线验证：

```bash
python3 scripts/validate_formal_s100_live_runtime.py \
  --raw artifacts/s100-live/raw-runtime.json \
  --snapshot reports/engineering/formal_vehicle_snapshot_manifest.json \
  --acceptance-session artifacts/formal_final_acceptance_session.json \
  --runtime-closure /path/to/final_runtime_closure_manifest.json \
  --dosod-admission-bundle /path/to/dosod_admission_bundle \
  --output artifacts/formal_s100_live_acceptance.json
```

`--acceptance-session` 与 `--runtime-closure` 是本次 session 创建后从验收工作树逐字节复制给板端的只读输入；采集器不接受缺失输入。回传的 raw 证据保存这两份输入的摘要，离线校验会与当前 session 的起始时间/snapshot 及当前 frozen closure 摘要重新比对。不得手工编辑、替换或以 PC/WSL/Gazebo 文件伪造这些记录。

正式最低门槛为持续 1800 秒、DOSOD 不低于 2 Hz、EdgeSAM 不低于 1 Hz、二者 p95 延迟均不高于 1000 ms、产品检测/分割/目标话题均有真实输出、节点消失/进程重启/推理失败均为 0、最低可用内存不少于总内存 5%、峰值温度不高于 85 °C。CPU 后端可以被如实采集，但 S100 正式产品 profile 指定 BPU，因此 CPU 运行会明确保持 BLOCKED，报告绝不会把它伪装成 BPU。
