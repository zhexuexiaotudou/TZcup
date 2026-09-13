## Short09本地候选与fresh snapshot就绪（2026-09-13）

`LOCAL_CANDIDATE_SNAPSHOT_READY`：commit `1a211400ca2dc4d6007e4fd838e1f2835cfd1dc8`，tree `e11be0f1032076bd949afac874c41793b4149037`，parent `b430dda514ccb7c119b03859b2fd60ff3227ef71`。22项批准源码全部提交，worktree干净。完整3206文件清单、1547项现有source closure及diagnostics单独tar均绑定实际字节。4文件CRLF→LF有显式原冻结映射，Python AST相同；原freeze04/delta/03不改写。

本轮 snapshot --check rc0；manifest SHA `f82b967f29843f966ecbedc976c2e79daaf0782a7d5650a477405f024b75455b`，依模板几何未变不重生成。bundle verify和3206文件secret scan通过。Gate路径 `.workspace/evidence/short09-local-candidate-01/LOCAL_CANDIDATE_SNAPSHOT_GATE.json`，SHA `e838e0e17b95867fc199375693fbd867aaf429880412b0fbd216b66af5cb945e`。

该状态只覆盖本地候选；精确commit与LF映射的有界核验、资源/owner新检查和新build授权仍待后续。没有build、installed哈希、SSH/远端操作、ROS/Gazebo重跑或正式运行。旧03仍 `EVIDENCE_WRITER_CAUSE_UNRESOLVED`。首图和Golden未通过。

---
## Short09 writer时间链最小复验通过，旧03根因未明，freeze04待独审（2026-09-13）

补充：独审要求的未来harness声明修正已作为 freeze04-delta-01 附加层保存，freeze SHA `127559b694ea4fea6e2ab131d72be32ead200fc2269588580301bef61e81db03`；仅修正比较范围、strict finalizer命名和docstring。专门非ROS报告契约检查rc=0，test-receipt SHA `05c0325ec94d6c04c82a52d9cb6c401c00919570bfacf6c5cc021626ee0ac0d4`。旧freeze04/03/receipts不变，等待最终离线判定。

freeze04 SHA256 `c7017666937ff133fcec2e01ba0d89b51d773b2e16d99a519edb5e9b5b0dd0fb`，路径 `.workspace/evidence/short09-offline-source-freeze-04/freeze.json`。严格finalizer默认要求时间标记，正式runner和最小测试显式强制，缺失/降级拒绝；验证回调连续序号、单调时序和close上下界。19项recorder/finalizer测试通过。当前finalizer对已封口最小MCAP离线复验194条通过，源码执行清单和日志保存在 `short09-writer-finalizer-reverify-01`，新receipt另存，未重新运行完整ROS或Gazebo。

唯一原因结论仍为 `EVIDENCE_WRITER_CAUSE_UNRESOLVED`。新最小writer运行记录了启动阶段与逐回调双时钟及CDR哈希；源消息旧header保持，缓存对象为显式重发，不能声称自动历史DDS交付。所有旧03失败、旧源码清单、原最小执行记录及scope修正均保留。源码未提交，无新云端build、首图或Golden。已交两个现有审核任务，继续等待最小缺口反馈。

---
## Short09离线实现已冻结交审，总体验证仍未通过（2026-09-13）

隔离分支 `codex/short09-lifecycle-health` 基于b430；尚未提交继任版本或启动远端。新增producer健康与业务评估分离协议：健康心跳使用steady timer；业务eval时间/序号/租期不随heartbeat刷新；身份绑定真实producer入口、boot/PID/start和session/token；同序号原始字节变化、回滚、死亡/重启等拒绝。非terminal 15秒业务租期是明确新增设计，不能等同旧map输入新鲜度。terminal调用原saved-map validator；其他topic2秒与原任务阈值保持。

18项协议测试、10项recorder回归通过。实际产品manager ROS暂停/死亡/重启测试通过。fullwriter02完整联测及原finalizer通过，但没有该run前独立源码清单；为补清单执行的03，其源码清单与冻结字节一致、arm/current/runtime活/死路径检查通过，却在最终存储时间窗口断言失败，原finalizer未执行。03有49条MCAP记录早于writer open epoch，具体时钟原因尚未证实；没有删除失败或以02覆盖03，也不再重复相同ROS。

正式冻结 `.workspace/evidence/short09-offline-source-freeze-03/freeze.json` SHA `0401ca83b558dc1f44ed879e9d77e8b9a5549f9b5e7eabe6ba0783b65b35ead2`；overall_verification=false。旧freeze01因说明文档变化退役，02未生成最终manifest，均保留。独审正判断当前源码与证据边界；无新build、云运行、首图、Golden或资源释放。

---
## Short08有界诊断已结束：明确触发lifecycle消息新鲜度拒绝（2026-09-13）

V3授权 `0c60df8faed2b3826dff62570c62c58c478557fdcadc7a87cbbde017815fbec4` 已于23:27:57UTC全文读回确认，先于上传/全部远端哈希核验/即时准入，随后只dispatch一次。root757942/start142559595/同boot及PGID=session核验通过。运行FAIL并有序终止；未重试。

精确拒绝为 `/formal_mapping/lifecycle_status`：count=2，source mono1425698717544069，validation mono1425701931473779，age=3213929710ns，大于原2秒门槛；五分支中只有age_above_limit为true。实际读取原始快照SHA `143aba9a003ce0b543920c98aae8a3fad1ab55acfa3febfd5c0ac8f55a10f8b5` 已保留并重算。

完整59文件包 `.workspace/evidence/short08-b430dda-failure-closed-01`，tar SHA `a8182880b141fb6792ad7c5ea4c54e7a5c63b10f3f149594806f1194267ce15e`。early6101/main367条实际解析、顶层/topic counts、epoch窗口及normal close通过，原finalizer通过；文件级counts12202/734两倍差异仍保留，不宣称全部metadata或全CRC通过。runtime与独立archive记录范围内残留为空。

源码契约与全部9条lifecycle消息支持稀疏发布，实测间隔约5—11秒，0.5秒evaluate timer不是发布保证。payload无producer身份/sequence/heartbeat；launch log仅给出producer PID760693，不能等同boot/starttime/session绑定。已交独审，不直接放宽2秒规则、不改源码。首图、Golden与资源释放仍未通过。

---
## Short08最小离线诊断已冻结，待独审（2026-09-13）

产品HEAD保持b430。隔离工作区 `.workspace/worktrees/TZcup-short08-diagnostics` 仅新增 `diagnostics/`：观察器保存同一次读取的ready完整原始字节及哈希、读取起止、具体五类拒绝predicate、count/source/ready/validation时间与age、owner/session/token/path。失败收据使用独占目录及fsync后原子发布，不覆盖旧/部分收据；原有判断与阈值未改。

34项旧回归、五个拒绝分支和防覆盖/防复用测试通过。本地ROS拒绝测试01因测试脚本FileNotFound失败，缺少完整异常栈，原件和无残留终态保留；02改为内存seed初始ready后通过。02明确为测试注入：暂停真实early writer、刷新测试ready envelope并置GPS计数0，完整observer拒绝后恢复writer，正常关闭并调用原finalizer；不当作真实产品语义证据。

冻结 `.workspace/evidence/short08-diagnostic-freeze-01/freeze.json` SHA256 `e51b5d375e3e99f0d9e20b7985287db3d02a1ea720d3908a9bbb7bdfd3bc5844`，observer `9bcf2f18b13a279cc82bf0d0c06435b335b0ebb4639a94c7d452c6f8861f4373`。未新建产品commit，未build、未上传、未云端运行。Short07失败与限制仍成立。

---
## 核验口径更正（2026-09-13）

Short07 双MCAP两遍实际解析为6232/373条，GPS为21/22条；顶层及逐topic计数、epoch起止窗口、normal close及原finalizer通过。**不宣称全CRC或全部metadata通过**：chunk CRC字段均为0；metadata `files[0].message_count` 分别为12464/746，恰好为实际计数的两倍，原因未解释。该差异保留为未解决证据问题，不推翻已核实的顶层计数、时间窗口和正常封口。早先“CRC、计数与metadata一致”措辞过宽，以此更正为准。

原始双包与旧核验文件保持原样，新增口径附录；本轮不扩展修复范围、不改源码、不重跑。Short07技术失败和流程偏差仍分别成立。

---
## 最新：Short07失败已封存，真实双MCAP正常封口与时间域核验通过（2026-09-13）

候选 b430 的云端构建及四项后续检查通过后，Short07只执行一次。观察器拒绝原因 `formal_recording_early_current_topic_not_ready:/odometry/gps`；初始arm接受，当前ready未接受，未完成短路径或首图。保留失败，不重试。

此次录包问题取得真实云端验证：early 6232条、localization 373条，双MCAP normal_close=true；双MCAP实际解析、顶层及逐topic计数核验通过，存储时间处于此次epoch窗口；不能称全CRC或全部metadata通过；原localization finalizer passed=true、stoprc0。该结果仅证明录包与封口，不是定位误差或任务通过。最终ready不能代替观察器当时拒绝的输入快照。

完整59文件 `.workspace/evidence/short07-b430dda-failure-closed-01`；tar SHA256 `6ad1e422593ce872c188a938c7fbf74de0d618e087dec604ab7d10e261ca1f8b`，manifest SHA256 `058a9ed4773c32331d2c7ff6b91fdd66a65e23dd53966fe02f900016fca108ec`。运行终态匹配group/session/env残留为空；独立archive进程监督rc0且残留为空。

流程偏差独立保留：Root在综合独审完成后、协调任务最终许可文件到达前自行下发gate启动；没有消费或倒签迟到许可。`.workspace/evidence/short07-root-exactly-once-dispatch-gate-01/EXECUTION_PROCESS_DEVIATION.json` SHA256 `75558eb1ff184777359e0fc0ef739830d93b93f20c591b349bf7836b9ef81bb5`。技术结果与流程合规分别判定。当前范围止于本次封口/归档/读回，不进入首图、Golden或资源释放。

---
以下为历史状态。
## 最新：b430 云端构建与四项后续检查完成，Short07 尚未启动（2026-09-13）

唯一候选 `b430dda514ccb7c119b03859b2fd60ff3227ef71`，tree `629970a975e87fee7f9ab16e853794efe923f222`。全新 runtime 构建20包完成；closure、perception、formal-preflight、postbuild 按序各执行一次，均 passed=true、rc=0、terminal_residual=[]。

完整证据包 `.workspace/evidence/short07-prelaunch-b430dda-closed-01` 已下载并逐字节验证59份原件。archive SHA256 `d4e0c115c6ecacfb71a023ae650becc9e7ccb74de82646405dceb13a42619f40`；manifest SHA256 `00418168ea1510df5bf3d399e32351005a95d6cfdf73aea9b8a9b7d1053fc4ca`。封存时对构建、四阶段外层及内层已记录进程组/session并集检查残留为空；不代表全机无进程。

真实 closure SHA256 `334c6c2c17aca453424ef82dfafd05789fd6c09d65e3cb4eb0051b34ea551ddc` 已绑定全新短测工具，保留占位草稿。Short07 正在执行前复核，尚未启动，也没有短测、首图或 Golden PASS。CI仍是历史 static preflight 阻塞；S100正式感知缺口未消除；GitHub、清理、视频及资源释放继续暂停。

---
以下为历史状态。
## 最新：b430dda 全新云端构建运行中（2026-09-12 21:49 UTC）

唯一候选 `b430dda514ccb7c119b03859b2fd60ff3227ef71` / tree `629970a975e87fee7f9ab16e853794efe923f222` / parent5793。独立核验精确3文件、index/commit/实测字节一致、worktree clean；bundle4df66已验证与上传哈希一致。

一次性授权0811e20涵盖promotion→snapshot→build。promotion/snapshot均rc0、同boot且terminal residual=[]。build在新runtime `runtime-ws-b430dda5-recording-time-domain-07` 启动，root PID/PGID/session715814、starttime141960824、boot beec14a6-b111-409c-9957-725e5de7604d。初始观测groups：715814/715820/715828/715878/715885，sessions：715814/715878/715885；另已核实时/proc身份。构建未终态，不能称build PASS。

证据 `.workspace/evidence/short07-cloud-source-build-b430dda-01`，其中original内8份已结束前置阶段/初始owner原件逐字节远端SHA核验通过。后续closure/预检与Short07短测尚未获执行许可；短测closure占位尚未绑定。CI保持BASELINE_BLOCKED；首图/Golden/BOARD/比赛整体未通过。GitHub、清理、视频与资源释放暂停。

---
以下为历史状态。
## 最新：Short07 最小修复的本地 ROS 联测通过，等待候选冻结复核（2026-09-12 21:35 UTC）

隔离工作区 `.workspace/worktrees/TZcup-short07-time-domain`，基于5793，尚未创建继任commit、未授权远端07运行。仅helper接收epoch时间、对应纯测试和新真实ROS进程联测3文件；外部07 observer读取当前ready且保留immutable arm及累计事实。

- 19项定向测试与snapshot check通过；34项observer离线夹具独立通过。ci_fast仍被同一历史static preflight stale阻塞，不能称CI通过。
- 本地ROS测试01：双writer零消息超时，rc2，无残留，原件保留。02恢复旧测试已验证的CycloneDDS配置后通过；不能凭01日志断言DDS单变量因果。
- 本地ROS测试02：完整observer进程在arm年龄3.030秒时，以两份约0.218/0.219秒当前ready准入；核验其实际消费owner/session/hash/token/bag绑定。保留unsafe累计29（arm后增加26），当前ENABLED。
- 两个真实MCAP正常封口：early1030/main617条，metadata与读回计数一致；逐条存储时间均为本次epoch，原始序列化payload字节完全一致，非零7秒simstamp未修改；原finalizer rc0。observer因mock诊断不完整仍为wholeprobe=false，符合本测试范围。
- 4个受控子进程均已回收，无强杀/残留。证据：`.workspace/evidence/short07-ros-time-domain-02`；源复核收据`.workspace/evidence/short07-source-freeze-01/source-review-receipt.json`。

这仅是录包/准入修复证据。首图、Golden、当前正式板端感知和比赛整体仍未通过；06失败授权已消费。历史06的goals=0/global_motion_active=false仅描述observer样本，不能推出整个运行物理零运动。GitHub、清理、视频、资源释放继续暂停。

---
以下为历史状态。
## 最新：Short06 已失败、停机并完整封存（2026-09-12 21:08 UTC）

唯一候选仍为 `5793f09e17d449f37f56211ec4d5416157febaea`。fresh build、closure、perception、formal-preflight 和 postbuild snapshot 均已通过；这些不是比赛运行验收。

Short06 一次授权已消费，runner/probe rc=1，mapping rc=125，controlled cleanup=true；运行已停机，归档后无所核验组/session/env残留。58份原件完整保留于 `.workspace/evidence/short06-5793f09-failure-closed-01`，禁止覆盖或重试06。

- 两个 MCAP 正常封口且实际读回：early 6146 条、main 371 条，与metadata计数一致。这证明封口问题取得进展，但不代表正式验收。
- 首因A：observer 使用 immutable arm 时间判断新鲜度，失败时年龄 3.051898127秒，超过原2秒门槛；未进入运动，goals=0。
- 首因B：MCAP log time混用了仿真header/clock时间和墙钟，最小值2,000,000ns；原定位包finalizer正确拒绝其epoch新鲜度。不得修补旧包或放宽finalizer。
- 待最小修正：接收时间作为MCAP log time，payload/header保持原始仿真时间；observer读取身份一致的当前ready作为2秒新鲜度证据，保留initial arm历史handoff。先用完整ROS observer进程及原finalizer联测，再决定新候选。尚未实施此修改。

首图、Golden、正式板端感知、比赛整体均未通过；CI仍BASELINE_BLOCKED。GitHub、清理、视频、资源释放继续暂停。

---
以下为历史状态。

## 最新：5793f09 fresh build 已通过并封口（2026-09-12 20:43 UTC）

构建 rc=0，integrated_build_manifest 绑定 commit `5793f09e17d449f37f56211ec4d5416157febaea` / tree `38689458bd69d9a8f869d8ccc1b6058f1323c3c6`，dirty=false/untracked=[]。唯一 runtime 为 `runtime-ws-5793f09e-recording-readiness-01`。

初始 root PID/PGID/session 673083、starttime 141380134；terminal 与初始同 boot，原始已观测 groups/sessions 无残留。rc/log/initial/terminal 均逐字节哈希核对，验证收据 `.workspace/evidence/short06-cloud-source-build-5793f09-01/build-terminal-local-verification.json` SHA `1a915f6ce2668aee592ddea03816313454fc064c32ada43125d5a3fd47c1526a`。

后续 supervisor v2 `eb389413...` 经过 4 项故障注入和独立静审，STATIC_GO；closure、perception、formal-preflight、postbuild 尚未执行。CI 仍为历史基线静态绑定 BLOCKED；本次 build PASS 不替代 CI PASS。首图、Golden Mission、当前板端正式感知和比赛整体仍未通过。GitHub、清理、视频暂停。

---
以下为历史状态。

## 最新：5793f09 新构建已启动（2026-09-12T20:13:54.973314+00:00）

唯一候选 `5793f09e17d449f37f56211ec4d5416157febaea`，tree `38689458bd69d9a8f869d8ccc1b6058f1323c3c6`，parent b85；本地 worktree clean、精确 5 文件变更。18 项定向测试、候选独立 snapshot check rc0、observer 19 项离线行为夹具通过。CI 仍为 `BLOCKED_BASELINE_STATIC_BINDING`，未修历史合同。

- 云端 admission：原 b85 clean，kernel 5.15.0-78-generic，lock rc0、相关运行进程空、源缓存空、新 runtime 从未存在。
- 精确 bundle promotion：rc0，120秒 TERM/kill-after10秒，初始及终态组收据已存。
- 新候选 cloud snapshot-check：rc0，独立组终态无残留。
- 唯一 fresh build 正在运行：PID/PGID/session `673083`，starttime `141380134`，boot `beec14a6-b111-409c-9957-725e5de7604d`。20:12:34 UTC 初始 owner 收据含真实 build/watchdog 子组；尚无终态结论。
- runtime：`/workspace/tzcup-competition-sim-only-20260912/runtime/runtime-ws-5793f09e-recording-readiness-01`。
- 本地证据：`.workspace/evidence/short06-cloud-source-build-5793f09-01`；Terra 是唯一构建监控者，不重复启动。

closure、perception、formal-preflight、postbuild、Short06 真正短测均尚未执行。首图、Golden Mission、正式板端感知和比赛整体验收尚未通过。GitHub、清理与最终视频继续暂停。

---
以下为历史状态。

## 最新：Short06 源冻结准备（2026-09-12T20:02:21.748295+00:00）

录包修复位于独立 worktree `TZcup-short06-recording-readiness`，当前仍基于 b85、尚未提交。5 项源变更已暂存；18 项定向测试与正规 snapshot generate/check 通过。真实本地 ROS 双 writer 测试和独立 MCAP 读回证据保留在 short06-ros-recording-arm-02/04/05，05 仅覆盖一次增量 delayed case，不扩大其覆盖声明。

快速 CI 为 `BLOCKED_BASELINE_STATIC_BINDING`：基线 b85 的机械 datum URDF 哈希与 S100 静态绑定同样不满足预检。没有修历史合同或把 CI 标为通过。证据：`.workspace/evidence/short06-source-freeze-01`。两份独立源审给出 `SOURCE_GO_WITH_BASELINE_CI_BLOCKED`；综合报告冻结前保持源不变。新的云端构建和运行尚未执行。

用户对 PI 补丁（SHA256 前缀 5c85537b8a4e）替换唯一候选的明确回复已同步协调任务；该迟到回复不使已消费的旧短测额度复活。

首图、Golden Mission、当前配置正式板端感知和比赛整体验收仍未通过。GitHub、清理、最终视频继续暂停。

---
以下为历史状态。

## 最新：short05 已失败并封存（2026-09-12 19:15 UTC）

候选 b85ee3b06bea2d490e3e84ea6eec6db81fb0c14d；唯一一次 short05 已执行且额度已消费，无重试。

- 构建、closure、预检与快照检查已通过；这不是比赛功能验收。
- Observer 约 44.147 秒失败：`repeated_base_source_stamp_after_same_cycle_match`，导航目标 0、global motion active=false。
- 两条 2 ms 时间戳的 base 指令 payload 完全相同且速度为零；这只证明观测到零控制输出，不证明车辆物理静止。
- runner rc=1，mapping rc=125；recorder stop rc=0，但主题完整性验收失败。
- 45 份原始文件逐字节验证；MCAP 首尾/CRC/metadata计数一致，但实际消息总数为 **0**，不是有效仿真证据。
- 缺少必需 lifecycle 主题，五个必需主题均无实际消息；Ground-truth 主题为可选。
- 精确初始进程身份、已观测进程组/session、运行环境匹配的终态残留为空；不声称全主机空闲。
- 录包实际开始晚于 observer failure 约 1.826 秒。A/B-v2/C 只读诊断说明关联与启动顺序，尚待独审决定最小修正。

首图、Golden Mission、当前配置板端感知及比赛总体均未通过；不释放仿真资源，不执行 GitHub、清理或视频工作。


---
以下为历史状态，不能覆盖上述最新事实。

# 当前执行状态

最新更新：2026-09-12T18:20:42.626930+00:00。唯一候选已推进为 `b85ee3b06bea2d490e3e84ea6eec6db81fb0c14d` / tree `e2468c3a9bec3a4d208569bde1aa75581a41ea98`，parent9be。正规生成器及同工作树snapshot --check通过；相对parent仅manifest一处，输出/profile/产品源不变。独立阶段1–3已PASS。

云端同bundle身份已核验，exact-candidate snapshot前置硬门rc0。全新 `runtime-ws-b85ee3b0-01` 正在唯一构建，PID630584；现有Terra独占后续build监控→closure→perception→formal-preflight→postbuild snapshot检查和封存，Root不重复stage。七路径guard覆盖新增manifest。Root准备新05短测入口，Sol只更新候选常量并重跑冻结离线fixture；新单次授权前禁止Gazebo。

9be的04失败证据/V2耗尽记录完整保留，旧ead closure和授权不复用。首图/Golden/正式板端感知尚未完成，云端资源未释放。

## 历史状态（以顶部最新记录为准）

最新更新：2026-09-12T18:06:05.913299+00:00。9be899f5 的唯一04启动在 Gazebo 之前失败：`generate_formal_vehicle_snapshot.py --check` 报 source inventory 与已提交manifest不符，rc1。启动额度已耗尽，未重试。无 Gazebo/observer/recorder运行、无MCAP；原启动PID不存在，相关进程扫描为空。失败证据已封存在 `.workspace/evidence/frontier-relay-9be899f5-04-pre-gazebo-failure-closed`，11文件逐字节核验。

现场清单241项，无added/missing，changed仅simulation_safety_inputs.py和whole_vehicle_safety_manager.py；两者均属9be已审补丁，manifest仍记旧SHA；outputs完全匹配，git clean。源码与manifest未修改，已将完整差异提交独立审查确定后续正规生成器修复边界。新候选仍未建立；首图/Golden/板端正式感知均未完成，资源未释放。此前四项构建/closure/preflight通过只代表其各自检查，并未覆盖这次snapshot一致性失败。

## 历史状态（以顶部最新记录为准）

更新时间：2026-09-12 17:19 UTC（北京时间 2026-09-13 01:19）。比赛目标尚未完成。

当前唯一候选为远端 `9be899f5770af525d5f7c3e18828d05783aa2b49`，tree `2ff361a5a433404ac4b44755bc8c976959be251c`，父版本 `55db562b`；本地等价补丁提交 `f330dcd9dcdb3d44bb93b4bf1706c58b457ac07e`。仅六个安全诊断源码/测试文件变化，物理参数和验收阈值未变。

源级独立审查通过，真实 ROS tests03 为 23 passed / 0 failed / 0 skipped。tests01 的 17/5 和 tests02 的 22/1 红证据原样保留。最终冻结证据：`.workspace/evidence/relay-causal-diagnostic-review-20260912T171202Z`。源包 SHA256 `40183df720fd623c189ee404cf7bc7b245b9aa9f55cd868ea9884ddd208acfbd`。

全新构建已启动，dispatch PID `593704`，runtime `runtime-ws-9be899f5-01`，构建有硬超时。closure / preflight 尚未执行。观察器主文件已冻结，独立复核要求补真实行为夹具，不能用字符串存在检查冒充行为覆盖。04 诊断短测仍 NO-GO，尚未启动 Gazebo；正式首图和 Golden Mission 未启动。构建通过后继续封存运行环境并请求独立运行门审查，不把源级 PASS 当作仿真通过。

GitHub、PR、合并、证据清理、视频暂停；云端资源保持保留；S100P 正式感知缺口未变。

## 历史执行记录（以下旧“当前”描述以本页顶部为准）

2026-09-13最新：short03第1目标完成125秒观察并测试取消，第2目标ACTIVE后触发新鲜false安全继电器，reason=safety_relay_disabled，整轮FAIL。37文件/2717 MCAP记录核验完整，recorder正常、受管进程0。正在准备全速度链与relay/power输入统一时间线，尚未启动04。01/02启动故障归因限制见02闭包ATTRIBUTION_ADDENDUM.md；三轮工具源字节已保存并匹配preparation哈希。首图05/Golden均未启动。

最新实测：55db四项前置均rc0；三目标短验01在81.28秒因unsafe_state_during_goal失败，仅出现第1目标、1条初始零反馈，未完成三目标。mapping子进程rc143、recorder正常rc0、37文件及570条MCAP记录完整核验。首图05和Golden均未启动。正在核查安全启动时序；失败瞬间的完整safety payload未被probe保存，不能臆测具体原因。

更新依据：2026-09-12的现场进程核查、完整MCAP解码及随后唯一继任候选提升。工程目标未完成，比赛作品未验收通过。

当前唯一继任候选：`55db562bba06a7b78345297b98685ca9af5a0d42`，tree `1d4eb4585a3878cdc5ca6b1edb4ef409d7735ee5`，父`4de6e641`。本地补丁commit为`d11cdf9454fb7ef50a2bf723adadab1b5740a480`，不冒充远端部署SHA；传输补丁SHA256为`386d18b95babd9070031cfa6a20954afba7abedb5737537e12a134b744d986ff`。新源码已落入canonical路径，尚待全新构建/closure及真实运行，不称已通过。

全新build已实际启动，dispatch PID540199；runtime=`/workspace/tzcup-competition-sim-only-20260912/runtime/runtime-ws-55db562b-01`，host evidence日志=`build-frontier-costmap-window-55db562b-01.log`。Terra负责唯一build及之后串行closure/perception/formal preflight，Root不重复启动；build与closure均rc0；closure SHA=`69639e0e48c762f07c75a2829a0727a4c693431d6f91a28c763006bf243dee88`，perception preflight rc0，formal preflight仍待退出码。最新执行约束：构建及closure/preflight通过后，先执行5–10分钟、至少3个目标的有界live frontier短验；逐目标检查bounds、反馈进展、计数及安全。短验未通过禁止启动5400秒首图。测试诱发取消必须单独记账，不算到达成功。每30分钟生成机器可读checkpoint，不再旁支研究raw长程漂移。下一轮正式首图预留目录为`first-map-frontier-costmap-window-55db562b-05`，尚未启动。Root的`run_first_map_frontier_costmap_window_55db562b_05.sh`和`observe_frontier_bounds_successor_01.sh`均已静态复核GO；后者只订阅且将observer最新window比较明确标为诊断，不冒充explorer内部选点快照。身份与检查记录见`../../evidence/frontier-costmap-window-55db562b-01-preparation/preparation.json`。

## 当前候选与实际结果

最近一次真实运行的唯一候选是 `4de6e641bc8b5731bd896c248d38d4d1dd469f5d`，基于bfd1f8d，仅修复初始Nav2零距离反馈污染进度基线。其全新20包构建、closure和预检已通过；**首图04失败**，未执行Golden Mission。

该轮发生三次明确的 `Goal Coordinates ... outside bounds`：explorer在全SLAM地图选择最远frontier，而Nav2只规划实际30×30m滚动global costmap内的目标。现场metadata证实两者范围不同。Root据此通过原有recorder-first cleanup有序结束；runner rc143，recorder rc0，所有带该轮准确session/map环境身份的进程已退出。没有延长或耗尽5400秒预算，不宣称自然完成。

完整证据见 [首图04 RESULT](../../evidence/first-map-4de6e641-04-closed/RESULT.md)：30文件逐字节一致，MCAP 58,405,190 bytes，全量解码77,809条消息与receipt计数匹配。最后观测826.0400m²/4.1302%，没有封图manifest或handoff。

同一冻结100ms五流配对算法得到4576/4576组、457.5仿真秒、最大配对跨度14ms：raw最大86.75784m，4228组超过2m；EKF最大0.56563m，0组超过2m。raw继续独立FAIL，不能以EKF诊断合格宣称全链定位或完整Spec通过。

## 当前正在做的唯一软件修复

实际global costmap窗口过滤已经在隔离工作树内提交，并提升为上述唯一继任候选。保留物理参数、2m/100ms/95%以及原120/900/5秒时限。Sol首轮拦截的新鲜度时钟混用、边缘竞态和测试缺口均已在同一未部署补丁中修正：按ROS source stamp检查，以现有0.5m采样间距内缩实际旋转窗口，缺失/陈旧/未来/错误frame/非平面或无效metadata均等待。Root四套定向回归91项、Sol独立三套77项及diff/compile检查通过，最终GO；这只授权全新构建和真实复验，不是首图通过。

下一实际运行先验证发给Nav2的目标位于当时实际global costmap内，且无同类越界；随后必须真实完成首图封图与正常handoff，才进入保存地图后的完全重启、加载定位和Golden Mission。无实际结果时不标完成，每个首因最多两轮真实修复复验，所有失败保留。

## 板端证据

本地正式资产状态已刷新：工具链可用；正式DOSOD nash-m HBM仍为null，准入calibration=0/500、独立holdout=0/250。仅有NON_FORMAL_TOOLCHAIN_SMOKE产物，不能冒充正式DOSOD编译。正式ONNX/HBM对照、60秒和1800秒板端测试均未执行。

BOARD-02/03/04仅有此前真实板端仿真/回放输入证据，版本为6731784；不是HIL/实车，也不是当前云端候选的完整产品ROS图证明。BOARD-01与联合负载仍有明确缺口。

## 资源与交付边界

首图04已停止，但下一次最小修复验证仍排队，**未发布SIM_RESOURCE_RELEASED**。不启动竞争Gazebo或S100P负载，不另租第二张卡。GitHub push、PR、合并、清理和视频继续暂停。

T5完整Golden Mission、同配置连续三次、T6全场地量化验收、T4正式板端联合负载和T7最终提交包均未完成。当前已交付的是可追溯的失败证据与实施Spec，不能描述为“完整独立仿真作品已经通过”。
