# TZcup 系统技术规范

## 1. 产品目标与状态

TZcup 在结构化道路与安全约束内完成建图定位、导航、覆盖清扫、垃圾发现、跟踪与地图融合、定点清扫、清扫后验证和人工监督。系统以 ROS 2 Jazzy、Gazebo Harmonic 与 x86 产品链为仿真基线，目标计算平台家族为地平线 Journey 6；在板卡与随板官方 SDK 完成识别前，SKU 与 BPU march 均保持 `auto`。

状态严格分层：

- `SIMULATION_PRODUCT_COMPLETE`：固定 A–P 仿真合同全部通过；
- `PRODUCT_INTEGRATION_READY`：仿真通过，并完成目标计算平台部署和板端性能验收；
- `PRODUCT_FIELD_READY`：冻结版本在真实车辆、真实传感器和真实道路独立通过。

下游状态不能由上游 smoke、单元测试或离线指标推导。

Journey 6 的 PC 先行状态独立记录为 `J6_PC_FUNCTIONAL_PASS`、`J6_X86_SIMULATION_READY`、`J6_LOOPBACK_HIL_READY` 与 `J6_DEPLOYMENT_BUNDLE_READY`。这些状态不能推导上述产品状态；RDK S100/S100P、J5 或其他平台的 SDK、BSP、预编译 HBM 与性能数据不能作为 Journey 6 证据。

## 2. 三平面架构

| 平面 | 职责 | 禁止事项 |
|---|---|---|
| Safety Plane | E-stop、Collision Monitor、keepout、边界/悬崖保护、速度安全门 | 不受 LLM、感知或清扫智能覆盖 |
| Autonomy Plane | 定位、地图、Nav2、Coverage、任务生命周期 | 不读取垃圾真值，不绕过 Safety |
| Cleaning Intelligence Plane | proposal、重观察、近距分类、ActionVerifier、Tracking、DynamicTrashMap、Scheduler、Post-Clean | 不直接写危险底盘命令，不把 CANDIDATE 当作可执行目标 |

任何输入过期、TF 缺失、深度无效、provider 失败、模型哈希不匹配、路径不可达或状态冲突都必须降级、取消或延后清扫。

## 3. 车辆与清扫机构

产品默认是 Ackermann：物理前轮转向、后轮牵引、单一 Gazebo AckermannSteering writer。冻结几何包括车长/宽、轴距、轮距、轮半径、转角、最小转弯半径、传感器安装、brush footprint、垃圾箱和清扫机构位姿。

当前基线：

- Xacro 默认 `drive_model=ackermann`；
- 默认有效清扫宽度 `1.32 m`；
- 垃圾箱几何 `0.50 × 0.40 × 0.20 m = 40 L`；
- Ackermann Nav2 禁止 rotate-to-heading，Coverage connector 禁止零速原地转向；
- `skid_steer_legacy` 只保留为显式历史回归，不能计入 V1 产品门。

正式验收不得缩小 footprint、虚增 brush width、缩小转弯半径或移动传感器。

## 4. 生产输入与真值隔离

允许进入生产链的输入只有 RGB、Depth、CameraInfo、LiDAR、IMU、wheel odometry、带时间戳 TF、Nav2 map 和静态清扫边界。

任务开始时：

```text
Production Target List = EMPTY
DynamicTrashMap = EMPTY
```

垃圾坐标、类别、instance ID、未来出现时间和真实清扫结果不得预载。Gazebo semantic/instance/world state 与 `/ground_truth/*` 只进入独立 post-run evaluator。生产链无观测时必须报告 unavailable/blocked，不能用 GT 补写。

## 5. 定位、导航与覆盖

- 定位使用 odom/IMU/GNSS/scan 的产品融合链，评测真值独立；
- Nav2 的 local/global costmap、Collision Monitor、Coverage 几何和路径预检使用同一 footprint；
- Coverage 分开记录规划路径、实际轨迹、brush-swept area、connector、repair 和刷盘状态；
- 正式 Ackermann 路径使用曲率连续的前进/显式倒车 connector，不使用 Spin/RotateInPlace；
- map save/load/relocalize/navigation、动态避障、E-stop 与 keepout 都由正式运行重算。

## 6. 感知与 DynamicTrashMap

离散链路：

```text
class-agnostic proposal
→ persistent CANDIDATE
→ OBSERVE_AGAIN / safe approach
→ close-range four-class decision
→ ActionVerifier
→ CONFIRMED
→ Scheduler
```

四类为 `plastic_bottle`、`metal_can`、`paper_litter`、`background_or_unknown`。Classifier 不能直接发出 `CLEAN_NOW`。ActionVerifier 至少检查类别置信、unknown、多帧/多视角一致性、depth、投影协方差、track persistence 和地图一致性。

DynamicTrashMap 使用可审计状态：`CANDIDATE`、`OBSERVE_AGAIN`、`CONFIRMED`、`DEFERRED`、`SCHEDULED`、`CLEANING`、`VERIFYING`、`CLEANED`、`REJECTED`、`EXPIRED`。动态插入只允许观察后建图；动态移除不得留下 stale cleaning action。

leaf 与 puddle 使用独立 Area segmentation 语义，逐类 IoU、macro mIoU、boundary F1 与 negative actionable FP/frame 分别报告。

## 7. Spot Cleaning 与 Post-Clean

完整链路是 Coverage 安全暂停、Nav2 approach、Pre-Clean Verification、执行、Post-Clean Verification 和 Coverage 恢复。Pre-Clean 必须重新确认目标存在、identity、定位、分类、ActionVerifier 与 Safety。

执行器成功不等于 `CLEANED`。离散目标只有在目标区域重新进入真实 camera FOV 且连续帧不存在后才能标记 CLEANED；Area remaining ratio 超过阈值时只允许一次 retry，否则 defer。

## 8. 多模态与 LLM

至少实现两个官方允许的交互模态。命令先经过固定 schema 与安全策略，再进入任务编排；LLM 只输出受限任务 DSL，不能直接控制底盘、执行器或 Safety。

## 9. 数据、冻结和 sealed final

数据分为 `TRAIN`、`DEVELOPMENT_HOLDOUT`、`SEALED_DEV_VAL`、`SEALED_FINAL`，要求 world/seed/exact RGB/pHash 零交叉泄漏。Final 一次访问、freeze-bound、原子记录、不可重考。

`MODEL_FREEZE_X86.json` 固定模型、权重、阈值、预/后处理、proposal、classifier、ActionVerifier、tracking、DynamicTrashMap、scheduler、re-observation、Area、projection、cadence、container 与 dependency。任何影响结果的修改都会使 freeze 失效。

## 10. 证据与发布

每个 Gate 同时提供 machine JSON、human Markdown、raw log 和 artifact SHA-256，并绑定 dataset、source commit、model/config/dataset SHA、container digest、dependency lock、seed、command 与 exit code。

最终 release 只从 CI 全绿且已合并的精确 `origin/main` 生成，包含 models、manifests、configs、launch、licenses、SBOM、SHA256SUMS、dependency lock、container digest、操作/健康检查/回滚说明和 evidence index。上线前完成 inactive warmup、healthcheck、atomic switch，并真实演练一次回滚。

固定机器合同为 [`config/product_acceptance_v1.json`](config/product_acceptance_v1.json)，完整自然语言规范为 [`docs/product-acceptance-spec-v1.md`](docs/product-acceptance-spec-v1.md)。
