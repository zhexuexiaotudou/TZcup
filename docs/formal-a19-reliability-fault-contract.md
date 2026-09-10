# A19 两小时长稳、故障注入与恢复

`config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json` 定义 A19 的固定产品门。
当前源码已经包含 canonical producer、独立 validator 和正式 runner，但尚未执行新鲜的
7200 秒产品长跑，因此状态是 `READY_FOR_FRESH_TWO_HOUR_RUNTIME`，不是 PASS。

`scripts/formal_a19_product_adapter.py` 现已提供真实产品图的传感器输入控制面：它强制
`product_demo.launch.py` 的 PC 感知只消费代理后的八路 RGB-D/CameraInfo 图像，并对
RGB/Depth 断流、时间戳偏移、CameraInfo 尺寸错配、不可达 TF frame 和全无效深度保留
实际消息变换及输入/输出计数 readback。`nav2_path_unavailable` 通过真实
`/planner_server/change_state` 使 `/compute_path_to_pose` endpoint 消失后再恢复；
`dynamic_obstacle_blocks_observation` 通过冻结 pedestrian schedule 的模型与 Gazebo
`/world/<world>/set_pose` 置位，并以 service 成功和导航 scan 回读确认。STOPPED/RECOVERED 还必须从产品 Safety、Nav2、
Perception 和 cleaning diagnostics 独立观察，定位误差由 `/odom` 与物理
`/odom/unfiltered` 计算，RSS 来自同一 PGID 的 `/proc`。proposal_flood、proposal_dropout、
classifier_exception、classifier_timeout、action_verifier_failure 和 reobserve_timeout
现通过受控主题实际改变 PC 推理或物理抓取消费者，并由这些节点自己的 diagnostics/status
回传消费计数；adapter 只在该计数增长后确认 STOPPED。
PC 感知节点还订阅受控的 `/formal_a19/perception_fault`：CUDA 故障只在实际 ORT provider
可用时创建该 provider 的会话，CPU-only runtime 如实回读没有 CUDA；DOSOD hash 对真实文件
重算后以错误期望值验证；EdgeSAM 只复制到临时 shadow 后篡改并交给真实 ORT loader，原模型
不会写入；慢推理在真实推理回调中 sleep 并报告观测延迟。四者均由产品 diagnostics 独立确认
active/recovered。profile 切换绝不停止、重启或替换产品进程：同一 PID/PGID 保持整个 7200 秒。
`wheel_slip_ratio` 和 `actuator_gain` 经现有 A300 ROS-to-Gazebo 原生桥进入同一 drivetrain plant：
前者改变实际轮端速度参考，后者缩放实际 `JointForceCmd` 扭矩。plant 的原生 status 回传原始/有效
轮速命令、未缩放/实际扭矩与实测轮速；adapter 只有在收到该独立回传且数值符合指定 profile 时才确认
切档。每个 non-nominal profile 还必须观察到非零轮端命令和非零未缩放扭矩，故写入配置或静止状态不能
充当物理证据；恢复 nominal 也重新下发并读回。传感器延迟/确定性 dropout 仍由代理入口处理，并由每档
ingress/egress 和样本中的 `unexpected_model_reload_count=0` 读回。
动态障碍恢复必须以 `SetEntityPose` 真实回原位，并同时以服务结果、原生 Gazebo `Pose_V` 和恢复后
的导航 scan 读回确认；仅清除本地状态不能报告 RECOVERED。18 项 fault 均有实际控制面与 readback；
完整 A19 仍须新鲜两小时运行才能通过。

## 不可缩短的正式口径

同一个冻结产品进程组必须连续运行至少 7200 个真实 monotonic 秒，1 Hz 采集 Coverage、
Perception、Tracking、DynamicTrashMap、Spot Cleaning、Post-Clean Verification 的原始
时序。nominal、transport_stress、wet_surface、degraded_drive 四档必须各有至少 1500 秒
可复核样本；最大采样间隔为 2.5 秒，至少保留 7000 个样本。fixture、虚拟时间、历史报告和
短跑均不能生成正式通过。

18 类故障由 producer 按合同固定时间表下发，不由 adapter 自行宣称已经注入。每项命令都带
随机 run nonce、唯一 command ID、profile 和完整参数；adapter 必须回显同一命令，并按合同的
fault-specific `DEGRADED` 或 `STOPPED` 期望及 `RECOVERED` 上报独立 readback。产品没有把某个
故障接入整车安全链时，不能借 operator disarm/arm 人为伪造 STOPPED；该故障必须验证其实际的
感知、Nav2 或执行器降级 readback，同时全程维持 crash/deadlock/unsafe cleaning 等全局零门。
只有已接入真实安全链的故障才可声明 STOPPED，且必须证明安全态、清扫抑制和制动读回。RECOVERED
后 Coverage 必须回到 RUNNING/RESUMED；operator 指令只允许产品启动时那一次显式记录的人工 arm，
不参与故障或自然恢复。

长稳硬门仍为 crash/deadlock/queue growth/意外模型重载/持续 TF 失败/不可恢复 watchdog/
不安全清扫均为零，首末 300 秒窗口的 RSS 中位数增长不超过 5%，定位 XY RMSE/P95 均不超过
50 mm。最终进程必须自然返回 0；producer 不允许用 INT/TERM/KILL 换取 PASS，并在 `/proc`
按精确 PGID 复核零 survivor。

## 证据链

`scripts/produce_formal_a19_reliability_fault.py` 监督一个冻结的非 fixture adapter。adapter 使用
逐行 JSON 的 `tzcup.formal_a19.adapter.v1` 协议；producer 自己记录接收 wall/monotonic 时间、
下发命令、启动 argv、PID/PGID、退出码、清理信号和 survivor。原始目录包含：

- `adapter_events.jsonl`：producer 封装的逐条原始事件与接收时间；
- `adapter_stderr.log`：未经摘要替换的 adapter stderr；
- `runtime_gate_binding.json`：当前 RUNNING session、canonical snapshot 和 frozen runtime closure；
- `raw_receipt.json`：上述文件的 SHA-256/字节数/行数、当前 Git commit/tree 与进程终态。

`scripts/validate_formal_a19_reliability_fault.py` 不信任 raw receipt 的 PASS 声明，而是重新读取
JSONL、重新计算时长/连续性/内存/定位/18 故障顺序与安全语义，再复核 producer、contract、
当前 Git、session、snapshot、closure 和全部原始文件摘要。正式输出为
`artifacts/formal_a19_reliability_fault_acceptance.json`，其 runtime-binding sidecar 也由最终
functional aggregate 再次读取并纳入 session 密封。

## 先做不占 Gazebo 锁的预检

adapter argv 使用 JSON 数组传递，禁止 shell `eval`。例如：

```bash
export FORMAL_A19_ADAPTER_ARGV_JSON='["/absolute/frozen/bin/tzcup-a19-product-adapter"]'
export FORMAL_VEHICLE_RUNTIME_WS=/absolute/fresh/final_runtime_ws
bash scripts/run_formal_a19_reliability_fault.sh --preflight
```

预检只验证合同、argv、干净 Git commit/tree、canonical snapshot、RUNNING session 和 frozen
runtime closure；不会获取 `/tmp/tzcup_formal_gazebo.lock`，不会启动 adapter 或 Gazebo，输出中的
`minimum_remaining_runtime_s` 固定为 7200。`scripts/fixtures/formal_a19_adapter_fixture.py` 只用于
POSIX 单元测试，正式入口按路径片段硬拒绝它。

## 独占窗口执行

总控分配独占 Gazebo 窗口后，在与预检完全相同的环境中执行：

```bash
bash scripts/run_formal_a19_reliability_fault.sh
```

runner 先取得统一 Gazebo 锁，再运行 snapshot 前检、producer、snapshot 后检和独立 validator。
已有 raw 目录、最终 receipt 或 runtime-binding sidecar 均会拒绝覆盖；失败 attempt 会保留为诊断
证据。A19 已作为 `a19_reliability` 接入正式 32 步编排，位于 multisite product 之后、S100 外部门
和 session finalize 之前。没有真实两小时 receipt 时，最终 aggregate 必须保持 pending。
