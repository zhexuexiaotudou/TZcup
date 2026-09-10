# A19 两小时长稳、故障注入与恢复

`config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json` 定义 A19 的固定产品门。
当前源码已经包含 canonical producer、独立 validator 和正式 runner，但尚未执行新鲜的
7200 秒产品长跑，因此状态是 `READY_FOR_FRESH_TWO_HOUR_RUNTIME`，不是 PASS。

`scripts/formal_a19_product_adapter.py` 现已提供真实产品图的传感器输入控制面：它强制
`product_demo.launch.py` 的 PC 感知只消费代理后的八路 RGB-D/CameraInfo 图像，并对
RGB/Depth 断流、时间戳偏移、CameraInfo 尺寸错配、不可达 TF frame 和全无效深度保留
实际消息变换及输入/输出计数 readback。STOPPED/RECOVERED 还必须从产品 Safety、Nav2、
Perception 和 cleaning diagnostics 独立观察，定位误差由 `/odom` 与物理
`/odom/unfiltered` 计算，RSS 来自同一 PGID 的 `/proc`。当前产品没有其余 12 类故障和
三种非 nominal profile 的运行时注入接口；adapter 会在正式 start 握手立即列出
`UNSUPPORTED` 并失败关闭，不会以命令回显冒充实测，也不会先浪费两小时再失败。因此
完整 A19 仍被产品 fault hooks 阻断。

## 不可缩短的正式口径

同一个冻结产品进程组必须连续运行至少 7200 个真实 monotonic 秒，1 Hz 采集 Coverage、
Perception、Tracking、DynamicTrashMap、Spot Cleaning、Post-Clean Verification 的原始
时序。nominal、transport_stress、wet_surface、degraded_drive 四档必须各有至少 1500 秒
可复核样本；最大采样间隔为 2.5 秒，至少保留 7000 个样本。fixture、虚拟时间、历史报告和
短跑均不能生成正式通过。

18 类故障由 producer 按合同固定时间表下发，不由 adapter 自行宣称已经注入。每项命令都带
随机 run nonce、唯一 command ID、profile 和完整参数；adapter 必须回显同一命令，并分别上报
时间有序的 `STOPPED` 和 `RECOVERED`。STOPPED 期间必须证明：安全态为 STOPPED，待执行清扫
已 CANCELLED/DEFERRED，感知为 DEGRADED/ERROR，Nav2 与 Watchdog 仍可运行，unsafe cleaning
计数为零，制动延迟不超过 1 秒。RECOVERED 后 Coverage 必须回到 RUNNING/RESUMED。

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
