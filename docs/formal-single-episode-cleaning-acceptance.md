# 单一随机 Episode 端到端清扫验收

该验收只接受一次冻结 session 中、同一 episode ID/seed、同一 Gazebo
cleaning 进程产生的在线数据。不得把感知、抓取、积水和动态避障的历史独立
JSON 拼成 `single_episode`。

## 运行链

1. 准备已通过硬重启定位的 saved-map，以及同一 saved-map cleaning 进程中由
   `coverage_probe` 完整执行后生成的 schema-v2 FullCoverage runtime 报告。仅有
   90 秒的 AMCL/coverage server readiness 报告不等于基线，不能填写或推断
   `successful_distance_m`。正式长跑使用自包含 runner：

   ```bash
   bash scripts/run_formal_same_map_full_coverage_baseline.sh \
     --episode-root <episode> --map-root <saved-map> \
     --session artifacts/formal_final_acceptance_session.json \
     --runtime-overlay <本次冻结源码构建的overlay> \
     --output <fresh-baseline-run-directory> \
     --formal-output artifacts/formal_same_map_full_coverage_baseline.json
   ```

   它从 saved-map 做独立 cleaning 硬重启，显式选择 `full_coverage`（不加载 RL
   checkpoint），启动一套 OpenNav coverage server 和一个 `coverage_probe`，持续
   运行到完整轨迹成功或 fail closed。覆盖配置由正式清扫宽度和 deployed footprint
   生成；Gazebo 命名实体位姿只进入 evaluator 的 `/ground_truth/odom`，产品控制节点
   不订阅该真值。默认超时 24 小时，可用 `--timeout` 显式修改。本文件更新本身没有
   启动该长时 Gazebo 任务。

   若完整 runtime 已由同一链路保留，可用下面的短命令将 mapping、hard-restart cleaning、
   lifecycle 和 FullCoverage runtime 按当前 session/snapshot 重新校验并封存：

   ```bash
   bash scripts/run_formal_same_map_baseline.sh \
     --episode-manifest <episode>/public/episode_manifest.json \
     --map-root <saved-map> \
     --mapping-runtime <saved-map>/mapping_runtime.json \
     --cleaning-runtime <cleaning-run>/cleaning_runtime.json \
     --lifecycle-acceptance artifacts/formal_map_lifecycle_acceptance.json \
     --coverage-runtime <cleaning-run>/coverage_probe.json \
     --session artifacts/formal_final_acceptance_session.json \
     --output artifacts/formal_same_map_full_coverage_baseline.json
   ```

   短封存器不启动 Gazebo，并拒绝覆盖旧报告。它要求固定起点与 episode 一致、
   map frame 锚定在固定起点、首次建图明确忽略脏污、map 文件哈希有效、mapping
   与 cleaning 的 robot description 等于 session 冻结 URDF、cleaning 是独立进程
   AMCL 硬重启、FullCoverage 全轨迹执行/覆盖质量/安全/定位均通过。实际任务轨迹
   长度作为基线，返航距离不计入；报告同时冻结 planned/actual 比较字段，供
   Q-learning 策略按同一指标比较。赛题的 `>=3500 m²/h` 是该次、source-bound
   FullCoverage 的 `covered_area_m2 / actual_duration_sec * 3600` 独立测量门：原始
   `competition_efficiency_pass` 必须为真，报告会复算并要求与
   `net_efficiency_m2_h` 一致，且明确 `return_distance_included=false`。它不能以
   active-dirt/RL 任务自身时长替代；RL 仍只按不含返航的任务轨迹长度相对同图
   FullCoverage 基线优化。随后准备冻结策略和 DOSOD/EdgeSAM 产物；策略必须是带
   非空 Q 表、`truth_access_used=false` 的 checkpoint。
2. 运行 `scripts/run_formal_single_episode_cleaning_mission.sh`。脚本只启动一个
   `product_demo.launch.py`，把 episode seed 显式传给 RL planner。必须通过
   `FORMAL_E2E_RUNTIME_OVERLAY` 或 `--runtime-overlay` 指定本次源码冻结后构建的唯一
   overlay；脚本只在 ROS Jazzy 基础环境之上 source 该 overlay，并拒绝非法
   `ROS_DOMAIN_ID`、已存在的 run 目录或已有正式报告。
   runner 会在创建输出目录和启动 Gazebo 前调用 baseline validator，从源证据
   重算报告；session、snapshot 或任何 baseline 源文件发生变化都会 fail closed。
3. runner 在启动 Gazebo 前冻结 public/evaluator manifest、ground truth、world、
   行人 schedule、session、baseline、policy 的文件哈希，以及 saved-map 和感知
   目录的逐文件确定性树哈希。collector 同时监听产品 planner/mission/path/grasp/
   odom 与 evaluator 话题；只有采到初始 evaluator 状态、实时参数和完整 ROS 节点
   图后才生成 readiness，runner 才允许通过 operator gate 开始任务。
4. aggregator 重新计算全部输入哈希，校验 session/episode/evaluator seed ledger/
   runtime/Gazebo PID，核对 live planner 的 policy 路径和 seed、map manager 的
   saved-map 路径、perception 的 artifact 路径，并要求每个话题都有在线样本。
5. 每块垃圾的成功 ID 必须与该 episode 的 20 个 truth object ID 一一相等；每个
   result 都必须包含目标位姿驱动的 MoveIt、IK、碰撞检查、腕部复核、物理夹持及
   投箱质量增量证据。干箱质量增量必须等于 20 块 truth 质量之和；污水的地面体积
   减量、累计回收增量和污水箱质量增量必须相互守恒。validator 会从 raw 重新聚合
   并逐字段比对，因此修改 aggregate 中的布尔值或指标不能形成通过证据。

产品控制图不订阅 `/evaluation/single_episode/*` 或行人真值；这一点来自运行中
`get_subscriptions_info_by_topic` 的节点图审计，不再由 JSON 中的固定空数组声明。
验收 collector 只读，不发布控制命令；runner 唯一动作是通过公开 operator gate
启动任务。
返回距离由 planner 在进入 return-home 前冻结的任务距离计算，不计入效率。

当前仅完成源码与静态门，尚未启动 Gazebo。正式通过必须保留同一输出目录中的
`raw_collection.json`、`aggregate.json`、`validation.json` 和启动日志。validation
通过后 runner 才会以 pending 文件原子发布到
`artifacts/formal_end_to_end_cleaning_mission_acceptance.json`；所有 Gazebo/bridge/
collector 均在独立进程组中，退出后还会按唯一 `GZ_PARTITION` 审计并清除残留。
