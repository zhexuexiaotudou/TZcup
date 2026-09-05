# 正式整车最终验收编排

`scripts/run_formal_final_acceptance.py` 是正式整车的唯一总验收入口。它不把历史
单项报告拼接成“最终通过”，而是在最终源码和构建冻结后创建一个新 session，按
一车一 Gazebo 的顺序重新产生全部本地证据。

## 先冻结唯一的非符号链接运行闭包

正式验收不接受 `colcon build --symlink-install`，也不接受多个项目 overlay
叠加。Windows/WSL 主机必须从 Windows 侧使用冷启动保护包装器；不要先手工启动
WSL，也不要直接从 Windows 调用 `bash scripts/build_formal_final_runtime.sh`：

```powershell
pwsh.exe -NoProfile -File .\scripts\build_formal_final_runtime_windows.ps1 `
  -RuntimeWs /home/zhexu/tzcup_final_runtime_r27_20260830_ws `
  -EvidenceRoot .\.work\formal_final_runtime_windows_guard\r27_attempt `
  -BuildLog .\.work\formal_final_runtime_r27_build.log
```

证据目录、构建日志和运行时路径都必须是全新的。包装器在启动 WSL 前要求 Windows
可用提交内存至少 12.5 GiB、Docker private 不超过 4 GiB 且 `vmmemWSL=0`；12.5 GiB
最低值不能通过参数调低。通过后才调用 Linux 构建脚本，后者还会执行 Windows 和
Linux 双重内存预检与持续 watchdog，并把已验证的冷启动 JSON 以只读文件复制到
runtime workspace。统一 closure 会绑定该文件摘要，最终编排在每一步前后都要求
它存在且未漂移；本项目的正式证据不接受直接调用 Linux builder 的旁路。

构建脚本拒绝复用已有的 `build/install/log`，不得增加 `--symlink-install`。为避免
WSL/Windows 提交量尖峰，它强制 `--parallel-workers 1`，并把同一个上限导出为
`CMAKE_BUILD_PARALLEL_LEVEL=1` 和 `MAKEFLAGS=-j1`；正式内存恢复路径不接受通过
`FORMAL_COLCON_PARALLEL_WORKERS` 提高并行度。构建完成后脚本会从已安装
Xacro生成 `side_brush_sdf_surface_preflight.json`，并自动写入与同一 merged prefix
绑定的 `integrated_build_manifest.json`。随后记录统一闭包：

```bash
repo_root="$PWD"
runtime_ws="$repo_root/.work/final_frozen_runtime"
python3 scripts/formal_final_runtime_closure.py record \
  --repository-root "$repo_root" \
  --runtime-ws "$runtime_ws" \
  --perception-artifacts "$repo_root/.work/formal_perception_assets" \
  --onnx-pythonpath /home/zhexu/tzcup-ros-onnx \
  --manifest \
    "$runtime_ws/final_runtime_closure_manifest.json"
```

该清单绑定 16 个最终运行包的源码与安装字节、直接位于同一 merged ament
前缀下的资源标记、全部构建标记、12 个 Gazebo C++ 插件、DOSOD/EdgeSAM
`artifact_manifest.json` 及其声明模型，以及 ONNX Runtime Python 包。`install/`
内任意目录或文件为符号链接、存在 isolated package prefix、源码晚于构建标记、
模型摘要不符或 source/install 文件不一致时都会 fail-closed。

## 再做无仿真预检

在 Windows 主机且 WSL 尚未启动时，可先运行只读 dry-run。它不调用
`bash.exe`、`wsl.exe`、Gazebo、CadQuery 或 FreeCAD；报告会逐项列出 frozen
workspace 的 `src/build/install/log`、closure/build/install manifests、冷启动
证据和模型 hand-off 是否缺失，并按正式总编排中的顺序渲染四条物理链
（底盘、地污、积水、物理抓投）的最终 runner 命令。水回收命令中的 typed
subclosure 在 Windows 报告中只是形状占位，绝不能当作 closure 或运行证据：

```powershell
py -3 .\scripts\run_formal_final_acceptance.py --windows-dry-run `
  --runtime-ws .\.work\final_frozen_runtime `
  --integrated-build-manifest .\.work\final_frozen_runtime\integrated_build_manifest.json `
  --perception-artifacts .\.work\formal_perception_assets `
  --onnx-pythonpath C:\path\to\onnx-overlay `
  --output .\reports\engineering\formal_final_runtime_windows_dry_run.json
```

返回 `FORMAL_FINAL_RUNTIME_WINDOWS_DRY_RUN_BLOCKED` 只说明 hand-off 未齐全，
并非运行失败；即使 dry-run 为 ready，也必须先在原生 Linux/WSL 中执行下述
`--preflight`，并重新验证 frozen closure，才有资格执行任何 runner。

内存恢复后的正式顺序是固定的：Windows 冷启动资源门通过后，以单 worker 构建冻结 runtime，
记录并在 native preflight 中重新验证 closure，然后严格按底盘、地污、积水、物理抓投四链
串行启动。每一个重型阶段（所有 Gazebo 阶段及跨图 RL 训练）在启动其自身进程前都会重新
执行 Windows 资源门；上一阶段的通过绝不复用。任一步骤（包括资源门）失败即停止，不会尝试
其余链、不封存 session，并保留 RUNNING session 和失败报告供诊断/人工恢复。总编排没有 CAD
执行路径，Gazebo 全局锁将并行数限制为 1；它也绝不自动运行或合成 S100 板端验收。Windows
dry-run 只渲染上述契约，不查询资源、不启动 WSL/Gazebo/CAD/板端，也不产生运行证据。

预检只读取脚本、依赖、构建清单、snapshot 状态和目标产物路径，不创建 session，
也不启动 Gazebo。该命令必须在原生 Linux/WSL shell 内运行；Windows 侧的
`C:\Windows\System32\bash.exe` 不是正式运行环境，预检会明确拒绝且不会调用它：

```bash
python3 scripts/run_formal_final_acceptance.py --preflight \
  --runtime-ws "$PWD/.work/final_frozen_runtime" \
  --integrated-build-manifest \
    "$PWD/.work/final_frozen_runtime/integrated_build_manifest.json" \
  --runtime-closure-manifest \
    "$PWD/.work/final_frozen_runtime/final_runtime_closure_manifest.json" \
  --perception-artifacts "$PWD/.work/formal_perception_assets" \
  --onnx-pythonpath /home/zhexu/tzcup-ros-onnx \
  --run-root "$PWD/.work/formal_final_acceptance/session-001" \
  --output "$PWD/.work/formal_final_acceptance_preflight.json"
```

预检只读取已有 session、合同证据和总报告，并为它们生成保全计划；它不会移动、删除或
覆盖任何内容。`--execute` 仅在预检全绿后执行该计划：将既有内容按原仓库相对路径移入
`.work/formal_final_acceptance/archives/<timestamp>/`，再创建新 session 和新证据，因此历史证据
无需由操作者手工移走。预检仍拒绝已有运行目录、漂移的统一运行闭包或 source/install 构建
清单、缺失的 ROS/Gazebo/模型依赖，以及已被占用的全局 Gazebo 锁。snapshot 即使当前漂移也只
记录为“将在执行时重生成”，因为正式执行的第一步就是从冻结源码生成新 snapshot。
`--run-root` 必须是此前不存在、没有任何符号链接路径段的
`$PWD/.work/formal_final_acceptance/` 子目录；`--preflight`、`--execute` 与
`--resume-s100` 使用同一边界。正式合同共 26 门：25 个本地门和 1 个 S100 外部硬门；这样
25 个本地门结束后保留的 run root 一定仍可被安全恢复，
不接受外部临时目录或已存在目录。

完整 source/install 绑定会重新读取并哈希冻结源码与 merged install。在 Windows 挂载盘的
WSL/DrvFS 冷缓存环境中，该只读检查默认允许 300 秒，避免旧的 55 秒固定上限把慢盘误判为
源码漂移；它仍是有限且 fail-closed 的门。需要显式调整时使用
`--integrated-source-build-preflight-timeout-seconds`，只接受 60–900 秒，实际值会写入
preflight 和运行报告。该参数不是 Gazebo 步骤超时，也不能跳过或降级 source/install 校验。
30 个正式感知 episode 的默认 ROS domain 基准为 60，对应安全范围 60–89。

S100P 已连接时，`s100_live_runtime` 的**原始采集顺序**应是 board-first：snapshot 和
session 创建后即可直接在板端运行同版本模型/adapter 并保存证据，不等待 PC 推理或 24 个
本地门。总报告仍只会在全部 26 门均通过后完成聚合；`--resume-s100` 仅是保持旧 session
本地证据不重跑的恢复接口，不能被解读为“板端必须最后执行”。必须保留 Gazebo 的机械、接触、
清扫/水回收和随机环境物理门，具体边界见
[S100P板端优先边界](s100p-board-first-execution-boundary.md)。

## 正式串行执行

仅当预检全绿后，把 `--preflight` 改为 `--execute`。编排顺序为：snapshot、session、
部件台账、产品/检修十九视图、惯量与扫掠体积、传感器、底盘、安全、清扫机构和
电机、地污、积水、检修门、充排、机械臂、物理抓投、20 块抓投与动态载荷、综合
基础物理、新 episode、首次建图、saved-map 硬重启复用、同 episode 全覆盖基线、
随机场景感知、动态避障、fresh RL 训练/跨图验证、单 episode 产品闭环，最后再
封存 session 并聚合 38 个功能位置。

`start_session` 不只冻结 vehicle snapshot；它会先重新验证正式 closure manifest 与
non-symlink merged install，再把 manifest 路径/文件摘要、closure 摘要和 install root 写入
session。此后每个 runtime-binding sidecar 必须匹配该 session closure，不能在运行中换用另一份
overlay、旧 closure 或仅字段外观相似的 JSON。

随机场景感知步骤的正式参数是 `--perception-episodes 30`（也是默认值），并且不是
可降级的性能调试开关。30 个 episode 使用冻结 `val` split 的 8 张地图轮转，每张至少
3 个 mission；聚合器还会核对 episode ID、split、地图覆盖与每图样本数。3 个 episode
只可用于启动、传感器链路和模型诊断 smoke，不能生成
`FORMAL_DOSOD_EDGESAM_RANDOM_SCENE_ACCEPTANCE_PASSED`，也不能满足总功能合同。即使
30 个 PC/Gazebo episode 全部通过，该结论也仅限当前冻结模型在 Gazebo 相机 validation
matrix 的产品链路；不推导 S100P/Journey 6P 板端、实车精度或统计泛化。

前进/制动、物理抓取投箱、地面脏污清扫和积水/漏液回收是四条可分别诊断的物理链，但正式
总编排仍严格 fail-fast：任一步骤失败都会先保留失败行、日志与已有证据，然后停止本次
session；它不会把同一未完成 session 中后来单跑的结果拼接成最终通过。需要继续诊断其余
链时，使用各自 runner 和独立 attempt 目录；修复后仍须以全新 run-root 全量重跑。综合基础物理入口还会为每个场景写入当前 session、snapshot、source
build manifest 和 runtime-binding 的 attempt sidecar；侧车缺失、变更或不匹配时即使场景自身
返回成功也必须 fail-closed。失败的四场景会生成仅位于唯一 run 目录的
`INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_FAILED` attempt manifest，绝不发布为中心 PASS 汇总。

每个 Gazebo 阶段都使用 `/tmp/tzcup_formal_gazebo.lock`。有共享隔离工具的 runner
自行持锁；仍没有持锁的综合 runner 由总编排器外层持锁。传感器、检修门、充排、
辅助电源/急停和整车执行器联锁的直跑入口还会在 source overlay 和启动 Gazebo
之前，共同校验唯一 non-symlink closure、canonical snapshot 和 RUNNING fresh
session，并把这三者的摘要绑定写入本次报告；已有报告、日志或绑定文件均拒绝覆盖。
每个阶段结束
后必须释放锁，并再次校验 snapshot。31 个步骤的每一步开始前和结束后都会重新
计算统一运行闭包；即使 runner 返回失败，也先执行结束侧闭包检查。源码、安装树、
插件、模型或 ONNX Runtime 任一字节漂移，或运行过程中出现符号链接，都会立即停止。
旧证据不能在下一次运行中被复用。

包括 `cleaning_actuators`、首次建图/复用、随机场景感知、动态避障和单 episode
端到端任务在内的 21 个直接或复合物理运行门还必须携带独立
`.runtime_binding.json`。清扫机构位置 runner 在 source 冻结 overlay 之前先执行 snapshot
`--check`，再把当前 RUNNING session、无符号链接 closure 和 install 字节身份写入 sidecar；
validator 会重新核对 sidecar 后才允许发布刷盘、滚刷、升降、排水阀、泵和刮水柔顺性报告。
水回收 runner 的 `FORMAL_VEHICLE_RUNTIME_WS` 明确指向 closure 记录的 `install/` 根，避免把
workspace 根误当 install 根而产生无法验证的 binding。
同一阶段的 typed 电机诊断改用显式 `FORMAL_WATER_TYPED_RUNTIME_WS` 指向其父 workspace，
以便独立核验 `install/` 与 `INSTALL_SYMLINKS.txt`。
最终 `validate_formal_functional_acceptance_contract.py` 也会逐一重新读取这 21 个 sidecar，
要求报告内嵌 binding 与 sidecar 字节语义完全一致，并与当前 session 的 snapshot、开始时间和
runtime closure 完全一致；仅有历史报告、`skipped` 行、摘要哈希或已通过的单个 validator 都不能
使 aggregate/final complete。静态审计还强制合同中声明 runtime binding 的 gate 集合恰为这 21 个，
新增、遗漏或改名都在任何长时运行之前 fail-closed。

所有 Gazebo 步骤还经过同一个低内存保护层。冻结运行时构建前由 Windows 包装器
执行上述 12.5 GiB 冷启动门；WSL 已启动后的每个仿真步骤还会从 WSL 通过
`powershell.exe` 读取 Windows commit limit/charge 和所有
`com.docker.backend` 进程的 `PrivateMemorySize64` 总和；默认 commit 可用量不足
10 GiB 或 Docker private 超过 4 GiB 时以退出码 86 拒绝启动。运行中以 1 Hz
同时记录 `/proc/meminfo`、本项目步骤 PGID RSS、Windows commit、Docker private
和 `vmmemWSL` private；默认 Windows
commit 可用量低于 6 GiB、Docker private 超过 8 GiB、WSL MemAvailable 低于
3 GiB、SwapUsed 超过 1 GiB 或本项目 PGID RSS 超过 9 GiB 时，只对该精确 PGID
依次发送 INT、TERM。保护层不会终止、重启或修改 Docker，也不按进程名杀任何
进程；PowerShell 查询或解析失败时 fail-closed。正式验收不得降低上述阈值或关闭
保护；环境变量覆盖仅供独立的保护层测试夹具使用，不能作为正式运行证据。

探针还以只读本机 API 记录 Windows nonpaged pool，并尝试读取 `Nbuf`、`Nnbl`、
`Nnbf` 三个内核 pool tag 的 nonpaged 占用。仅当 nonpaged pool 至少 2 GiB 且这三项
合计至少 1 GiB 时，报告会标记 `suspected_ndis_nonpaged_pool_leak=true`；这是需要进一步
归因的高占用信号，不能据此指认具体驱动，但已经足以拒绝新的正式重任务。启动门以退出码
86 拒绝，运行中的 watchdog 只停止本次精确 PGID；它不会重启网络、变更适配器或停止
Docker/其他 WSL 工作负载。pool-tag 查询受系统版本、权限和内核实现影响；若不可用，证据会以
`pool_tag_diagnostics.status=unavailable` 及结构化失败类别记录，而不是把零值误报为正常。

正式单机运行同时锁死 ROS 2 与 Gazebo Transport 的接口选择：
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、
CycloneDDS `lo` 配置、`GZ_IP=127.0.0.1` 和 `IGN_IP=127.0.0.1` 均不可被调用环境放宽，
`ROS_LOCALHOST_ONLY`、`ROS_STATIC_PEERS`、`GZ_RELAY/IGN_RELAY` 会被清除；主仿真 launch
还会在创建 Gazebo/bridge action 前再次写入 loopback 地址。`GZ_PARTITION` 只负责命名空间
隔离，不能替代上述网卡绑定。修改此策略后必须重新构建冻结 runtime、记录新 closure，并在
NDIS 启动门恢复正常后用新 run-root 全量重跑，不能继续使用旧 frozen install。

重启释放历史 NDIS 非分页池后，不直接再次进入耗时的完整编排。先构建一份包含上述
loopback 修复的新冻结 runtime，再从原生 Windows PowerShell 运行一次全规格传感器
transport probe：

```powershell
pwsh.exe -NoProfile -File .\scripts\run_formal_sensor_transport_probe_windows.ps1 `
  -RuntimeWs /mnt/f/Project/TZcup-integrated-functional-acceptance/.work/final_frozen_runtime_rXX `
  -RunId rXX_sensor_transport_001 `
  -DomainId 81
```

该入口先在任何 WSL 启动前要求 12.5 GiB Windows commit 余量、Docker private 不超过
4 GiB、`vmmemWSL=0`、pool-tag 查询可用且无 NDIS suspect；随后只调用一次 WSL helper。
helper 在仓库 `.work/formal_sensor_transport_probe/<RunId>` 创建独立 RUNNING session，绑定
canonical snapshot 和指定 frozen closure，并原样执行正式高带宽传感器 runner：12 条
payload stream、完整 25-topic/8-group 合同、FOV、frame、分辨率、频率和三个样本要求都不
降级。运行中额外从真实 Gazebo 与 `ros_gz_bridge parameter_bridge` 子进程的 `/proc` 环境
证明 `GZ_IP/IGN_IP=127.0.0.1`、ROS/CycloneDDS localhost 且 relay 不存在；watchdog 日志
用于计算 NDIS/nonpaged first、peak、last，而不是只看退出时末样本。

runner 清理后还必须证明同一 `GZ_PARTITION` 无存活进程、watchdog 目标组无 survivor、锁已
释放。Windows wrapper 最后再次采集 pool evidence，并由
`finalize_formal_sensor_transport_probe.py` 把 before/peak/after、12 stream、session、snapshot、
closure、runtime binding、loopback 和 cleanup 统一 fail-closed 汇总。任一文件缺失、pool-tag
不可观测、NDIS suspect、bridge 逃出 loopback、topic 不达标或清理残留都会返回非零并保留
attempt。该 probe 只决定是否安全进入完整 31 步，不能替代最终 session 的 sensor gate。

若十九视图在进入新冻结构建前需要隔离 Gazebo Transport、`ros_gz_image` 与 DDS，
必须从 Windows 使用同样的冷启动门运行单相机诊断；不要先手工启动 WSL：

```powershell
pwsh.exe -NoProfile -File .\scripts\run_formal_visual_single_topic_diagnostic_windows.ps1 `
  -RuntimeWs /home/zhexu/tzcup_final_runtime_r32_20260830_ws `
  -OutputRoot /home/zhexu/tzcup_visual_single_topic_r32_diag_001 `
  -EvidenceRoot .\.work\formal_visual_single_topic_windows_guard\r32_diag_001 `
  -DiagnosticLog .\.work\formal_visual_single_topic_windows_guard\r32_diag_001.log
```

该包装器固定要求 `vmmemWSL=0`、Windows 可用提交内存至少 12.5 GiB、Docker
private 不超过 4 GiB，随后只启动一台 1600×1000 验收相机、一个
`ros_gz_image/image_bridge` 和一个 ROS 订阅探针。它保存 Gazebo/ROS 发现、单帧
元数据、bridge 可执行文件依赖和进程 `/proc` 映射；诊断通过只证明单 topic 传输链，
不替代 r33 冻结构建、十九视图或最终 31 步验收。

仓库内旧的 AUTO-16 仿真、frozen coverage trial 和 AUTO-17 可视化 PowerShell
入口不属于最终 31 步验收，但同样必须先经过 `formal_wsl_entry_memory_guard.ps1`：
WSL 未运行时执行 12.5 GiB 冷启动门并要求 `vmmemWSL=0`，WSL 已运行时执行
10 GiB 运行门并要求 `vmmemWSL>0`；状态要求在探针中互斥并二次确认。路径转换、
实际仿真、WSLg prepare、shutdown 后恢复及 retry 均使用独立预检证据，禁止在一次
轻量 WSL 调用后直接启动重任务，或复用 shutdown 前的热门证据。

运行态 Windows 探针保持严格 fail-closed：PowerShell 查询异常或连续两个无效样本仍以
`125 / FORMAL_WINDOWS_MEMORY_PROBE_FAILED_CLOSED` 停止且只停止本次精确 PGID，绝不把
“无法观测”当作内存充足。探针到 watchdog 的十字段记录使用一次原子 pipe 写入，避免
`read -t` 在半条记录中超时后把残段误判成第二个坏样本；PowerShell stderr 直接继承到
watchdog 日志，避免未读取管道积压。JSON 的 `windows_diagnostics.last_probe_failure`
区分 `stdout_timeout`、`probe_eof`、`field_count`、`non_uint`、
`non_monotonic_epoch`、`commit_invariant` 与 `pool_diagnostic_invariant`，用于判断是
Windows 查询链故障还是真实资源阈值越界。5 秒读取超时与一次瞬态容忍保持不变，资源
阈值、退出码和 exact-PGID 隔离合同均未放宽。
故障对象同时记录 `rejected_sequence` 与 `previous_accepted_sequence`，以便区分真实
重复记录、probe 重启和读取边界问题，不记录或回放原始管道内容。
十字段流的首字段只承担重复/陈旧样本检测，使用 Python probe 进程内从 1 开始严格
递增的序列；一阶段启动证据仍保存真实 UTC `epoch_ns`。因此 Windows/WSL 虚拟机
墙上时间或 `CLOCK_MONOTONIC` 重基准不会被误报成 `non_monotonic_epoch`，而真正重复
的 pipe 记录仍会被拒绝。一次性 Windows 查询允许且仅允许一次 0.25 秒后的有界重试；
第二次仍无严格合法样本就返回 125，并将 stdout/stderr 摘要写入日志，不会生成假 PASS。
对 Bash `read -t` 的边界行为只有一个窄例外：若一个 `stdout_timeout` 后紧接着出现与
上次接受序号相同或更旧的完整十字段行，watchdog 丢弃这一行并记为
`post_timeout_duplicate`，但不消耗第二个失败计数；再下一条必须严格递增且所有内存
不变量合法，否则仍按原一次瞬态预算返回 125。该规则不接受旧值作为当前内存样本。

传感器门不降低任何 URDF 名义规格。MID-360、双 D435、双 1920×1080 鱼眼仍按正式
分辨率和源帧率生成，但原始图像/点云从控制平面桥中分离到专用 YAML：方向固定为
`GZ_TO_ROS`，采用 `SENSOR_DATA` QoS、lazy 订阅和 ROS/Gazebo 两侧单帧队列。十九路
视觉验收相机固定为 0.2 fps；通用 `ros_gz_bridge` 已发现端点却不能转发实际
1600×1000 帧，故改用 ROS 官方的 `ros_gz_image/image_bridge`，其 `sensor_data`
预设为 BEST_EFFORT/VOLATILE/KEEP_LAST(depth 5)，且没有 lazy 或队列深度参数。
该专用桥由默认关闭的 `visual_acceptance_runtime` 开关隔离，只在十九相机视觉工作室中
显式启动；低帧率和持续内存看门狗限制其负载，不会改变正式高带宽传感器仍按需桥接的
约束。采集器先等待控制器和 robot description 就绪，
再以 best-effort/KEEP_LAST(1) 订阅合同流；每个流取得至少三帧、唯一源时间戳和元数据后
立即退订，使高带宽传感器的 lazy bridge 停止不再需要的转换。正式编排器已为该 runner 提供唯一外层
session/PGID，runner 不再二次 `setsid` 逃出 watchdog。以上仅是内存安全设计；仍须在
新冻结运行时中重跑完整并发传感器门，不能把静态测试写成 Gazebo 运行通过。

## S100 外部硬门

`s100_live_runtime` 只能来自真实 RDK S100P（Journey 6P）板端采集。PC、仿真、离线
日志或手写 JSON 均不能替代。板端采集应在 session/snapshot 就绪后优先进行；若全部本地门通过但缺少本 session 启动后的真实板端证据
时，编排器以退出码 `4` 和
`FORMAL_FINAL_ACCEPTANCE_LOCAL_GATES_PASSED_S100_EXTERNAL_BLOCKED` 收口；session
保持 `PENDING`，不能宣称最终整车通过。只有真实板端证据也通过时，退出码才为 `0`。

这不是让 25 个本地门重跑的理由。板端完成采集后，保留同一 session 的 `run-root`
及本地门产物，将**原始 collector JSON 和已验证的 final JSON 一起放在本仓库受控且
非符号链接的目录内**，然后只执行恢复收口：

```bash
python3 scripts/run_formal_final_acceptance.py --resume-s100 \
  --runtime-ws "$PWD/.work/final_frozen_runtime" \
  --integrated-build-manifest \
    "$PWD/.work/final_frozen_runtime/integrated_build_manifest.json" \
  --runtime-closure-manifest \
    "$PWD/.work/final_frozen_runtime/final_runtime_closure_manifest.json" \
  --perception-artifacts "$PWD/.work/formal_perception_assets" \
  --onnx-pythonpath /home/zhexu/tzcup-ros-onnx \
  --run-root "$PWD/.work/formal_final_acceptance/session-001" \
  --accept-operator-trusted-s100
```

`--resume-s100` 与其他模式互斥，必须显式提供既有 `run-root`。它不会归档、生成或
重跑任何本地/Gazebo 步骤；先以 session 级独占锁检查旧报告严格处于
`LOCAL_GATES_PASSED_S100_EXTERNAL_BLOCKED`、session 仍为 `PENDING` 且 failures 精确为
`{s100_live_runtime: missing}`，检查 snapshot 当前且身份未变、统一 runtime closure
仍有效，并逐一重新验证 25 个本地 gate 的文件、绑定和摘要仍与旧报告一致。任一漂移、
重复恢复、非预期 PENDING 原因或 COMPLETE session 都会拒绝恢复。

若 final 或 raw S100 证据不存在，恢复仍返回退出码 `4`，且不会改动 session。final 的
`raw_evidence.path` 必须是如 `artifacts/formal_s100_live_raw.json` 这样的**仓库根目录
相对路径**；绝不接受绝对路径、`..` 逃逸或任何符号链接。若两者存在，恢复会要求二者
都晚于 session start、摘要相符且路径受上述约束；
随后以 `validate_formal_s100_live_runtime.py` 从 raw、当前 snapshot、当前 session 身份和当前 frozen runtime closure 写出一份保留的
revalidation report，并要求它与提供的 final report 严格 JSON 一致，才会重新 finalize
session 和运行 `--require-all` 聚合。成功时既有 30 行报告中的 `s100_live`、
`finalize_session` 与 `functional_aggregate` 三行原位更新，记录上次状态与本次恢复起止
时间；报告的 `resume_history` 还保留三行的完整前后快照及其 SHA-256。其余本地步骤绝不重复。
若前一次恢复已经成功将 session finalize 为 `COMPLETE`，但 aggregate、closure 或报告写入
随后失败，保留报告仍处于 S100-blocked 状态时可进行第二阶段恢复：它重新验证 S100 与全部
25 个 session-bound evidence/digest，跳过第二次 finalize，只重跑 aggregate 和报告收口；
报告已经 `COMPLETE` 的情况仍拒绝重复恢复。

首次 `--execute` 已验证 S100 并已把 session 封存为 `COMPLETE` 后，若仅 aggregate、其后
closure 或报告收口失败，编排器会明确保留
`FORMAL_FINAL_ACCEPTANCE_S100_COMMITTED_AGGREGATE_PENDING`，而不是把它混同为普通
`ORCHESTRATION_FAILED`。恢复只接受该专用状态配合 failures 为空的 COMPLETE session，并仍会
重新验证 S100 raw/final 链和全部 25 项 session-bound evidence；普通编排失败一律拒绝恢复。
该状态允许末尾 `finalize_session` 或 `functional_aggregate` 报告行尚未写入：恢复只按
既有的 local-step 前缀原位补齐到 30 行，并在 `resume_history` 如实写入缺失行的
`before: null`/`before_sha256: null` 及实际 after 行摘要，绝不伪造旧行，也绝不重跑本地或
Gazebo 步骤。

`--accept-operator-trusted-s100` 是任何 S100 PASS 路径的显式确认：它既用于
`--resume-s100`，也用于在 `--execute` 期间已经提供并检测到 S100 final 证据的情况。
若 `--execute` 未发现 final，仍以退出码 `4` 保持 S100 pending；若发现 final 却没有
该 flag，则 fail-closed，绝不会悄悄忽略或把它当成缺失。该链条以 device-tree、collector
revision、raw digest、模型/运行时/ROS 图、snapshot source digest、session 起始身份和 runtime closure 摘要绑定检查一次板端采集，并会拒绝
PC/仿真字段或不一致的 raw/final 链。它仅是**operator-trusted、tamper-evident、
non-cryptographic** 的证据链，不是 TPM、远程签名或针对恶意持有者的密码学防伪；不得
把它表述为抗恶意 PC 伪造的硬件远程认证。没有受信操作员提供的真实证据时，S100 硬门
必须保持 pending。

静态覆盖审计见
`reports/engineering/formal_final_acceptance_orchestration_audit.json`。
