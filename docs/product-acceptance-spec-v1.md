# TZcup 仿真产品级最终验收标准与产品准入要求


**文档性质：最终验收规范 / Product Acceptance Specification**
**适用对象：TZcup 智慧环卫无人清扫车项目**
**用途：作为后续 Codex/DeepSeek/人工开发的固定验收基准，不允许在看到测试结果后随意修改门槛。**

---

# 1. 三个最终状态

## 1.1 `SIMULATION_PRODUCT_COMPLETE=true`

表示：

> 在与目标实体车辆一致的运动学、传感器、计算资源和软件架构约束下，完整 Gazebo/x86 产品链已经通过正式仿真验收。

必须同时通过：

```text
车辆/传感器仿真一致性
定位
建图
覆盖规划
清扫效率
动态避障
离散垃圾感知
Area感知
Tracking
DynamicTrashMap
Spot Cleaning
Post-Clean Verification
安全
性能
稳定性
故障恢复
Replay
Release
```

该状态表示：

> 软件已经可以作为实体产品的软件基线，直接进入 J6/实体车部署与实机验证。

但不等于：

```text
PRODUCT_FIELD_READY=true
```

---

## 1.2 `PRODUCT_INTEGRATION_READY=true`

表示：

```text
SIMULATION_PRODUCT_COMPLETE=true
+
目标计算平台可部署
+
模型转换成功
+
板端性能通过
```

---

## 1.3 `PRODUCT_FIELD_READY=true`

表示：

> 冻结的软件版本已在真实车辆、真实传感器和真实道路环境中完成独立 field 验收。

只有这个状态才能宣称：

```text
实体产品级完成
```

---

# 2. 验收优先级

```text
P0：官方赛题明确要求
P1：产品安全硬门
P2：内部产品质量门
P3：优化目标
```

规则：

```text
官方要求优先
内部标准不得低于官方明确门槛
```

如果官方只写：

```text
“垃圾识别准确率 ≥95%”
```

必须同时报告：

```text
Precision
Recall
F1
per-class Precision
per-class Recall
Confusion Matrix
```

不得只挑一个最有利指标。

---

# 3. 总验收逻辑

以下主门全部 PASS 才允许：

```text
SIMULATION_PRODUCT_COMPLETE=true
```

主门：

```text
A. Vehicle / Simulation Fidelity
B. Localization & Mapping
C. Coverage & Efficiency
D. Navigation & Safety
E. Discrete Perception
F. Area Perception
G. Tracking & DynamicTrashMap
H. Spot Cleaning
I. Post-Clean Verification
J. Multimodal / LLM
K. Performance
L. Reliability / Soak
M. Fault Injection
N. Replay / Reproducibility
O. Freeze / Release / Supply Chain
P. Competition Mapping
```

任何硬门失败：

```text
SIMULATION_PRODUCT_COMPLETE=false
```

不得平均抵消。

---

# 4. Ground Truth 隔离：一票否决

所有正式产品任务必须：

```text
Production Target List starts EMPTY
DynamicTrashMap starts EMPTY
```

机器人不得预先知道：

```text
垃圾坐标
垃圾类别
垃圾 instance ID
未来垃圾出现时间
真实清扫结果
```

生产链允许输入：

```text
RGB
Depth
CameraInfo
LiDAR
IMU
wheel odometry
timestamped TF
Nav2 map
static cleaning boundary
```

GT 只允许 post-run evaluator 使用。

硬门：

```text
GT_control_violation = 0
preknown_target_coordinates = false
mission_start_target_count = 0
pre_FOV_false_target_creation = 0
```

任一失败：

```text
整体验收 FAIL
```

---

# 5. 数据与测试集隔离

正式数据：

```text
TRAIN
DEVELOPMENT_HOLDOUT
SEALED_DEV_VAL
SEALED_FINAL
```

要求：

```text
world overlap = 0
seed overlap = 0
exact RGB duplicate = 0
cross-split pHash duplicate = 0
final-set training access = 0
```

Final set：

```text
one-shot
freeze-bound
immutable access record
```

失败后不得：

```text
看结果
→ 调参数
→ 原 final set 重考
```

---

# 6. 车辆仿真一致性

仿真运动学必须与目标实体车一致。

若实体产品为 Ackermann：

```text
正式仿真也必须为 Ackermann
point-turn count = 0
zero-speed yaw command = 0
不得使用 Spin/RotateInPlace 冒充产品
```

必须冻结：

```text
vehicle length/width
wheelbase
track width
wheel radius
steering limit
minimum turning radius
sensor mounting pose
brush footprint
cleaning width
bin geometry
cleaning mechanism pose
```

不得为了通过验收：

```text
缩小 footprint
虚增 brush width
缩小 turning radius
移动传感器
```

---

# 7. 清扫机构基础产品门

```text
effective cleaning width >= 600 mm
bin capacity >= 40 L
```

建议：

```text
nominal cleaning width >= 650 mm
```

仿真必须按真实 footprint 计算；实体阶段重新实测。

---

# 8. 定位验收

官方要求：

```text
定位精度 <= 50 mm
```

若官方 evaluator 有明确算法，严格按官方算法。

内部正式仿真同时要求：

```text
XY RMSE <= 0.050 m
XY P95 <= 0.050 m
```

必须报告：

```text
mean
median
RMSE
P95
max
yaw error
```

测试不能只测静止点，必须包含：

```text
straight
turn
long mission
dynamic obstacle
different map region
```

---

# 9. 建图验收

官方能力目标：

```text
mapping area >= 20,000 m²
```

正式验收必须真实跑通：

```text
continuous mapping
map save
restart
map load
re-localization
Nav2 navigation
```

要求：

```text
>=20,000 m² 等效环境建图成功
无坐标系断裂
无明显拓扑破坏
保存/加载后可继续导航
```

“代码理论支持”不算 PASS。

---

# 10. 覆盖规划验收

覆盖率按实际 brush swept area 计算：

```text
empirical coverage >= 95%
>=98% preferred
```

重复覆盖：

```text
repeat coverage ratio <= 20%
<=15% preferred
```

Hard：

```text
keepout violation = 0
collision = 0
```

Ackermann connector 必须符合真实车辆运动学，不允许零速原地转向。

---

# 11. 清扫效率验收

官方要求：

```text
effective cleaning efficiency >= 3500 m²/h
```

正式定义：

```text
实际有效覆盖面积 / 实际任务总时间
```

总时间必须包含：

```text
转弯
连接段
避障
正常减速
感知推理
Tracking
DynamicTrashMap
Spot Cleaning
必要 re-observation
```

不得人为扣除正常算法耗时。

Hard：

```text
>=3500 m²/h
```

否则 FAIL。


---

# 12. 导航与动态安全验收

正式任务至少：

```text
30 independent seeds
```

场景覆盖：

```text
straight
turn
narrow passage
blocked route
dynamic obstacle
replan
recovery
keepout
```

要求：

```text
navigation success >= 95%
collision = 0
keepout violation = 0
```

Safety Plane 必须独立于 Cleaning Intelligence。

Cleaning Intelligence 不得：

```text
关闭 collision monitor
绕过 keepout
直接写危险底盘命令
```

动态人车场景至少包含：

```text
pedestrian crossing
stationary pedestrian
vehicle crossing
sudden obstacle
partial path blockage
sensor degradation
```

Hard：

```text
collision = 0
safety bypass = 0
```

---

# 13. 边界保护与 E-stop

边界：

```text
cleaning boundary violation = 0
forbidden region violation = 0
drop/cliff simulated violation = 0
```

E-stop：

```text
>=30 independent trials
command accepted = 100%
safe stop = 100%
P95 response <= 0.200 s
```

同时报告：

```text
median
P95
max
```

实体车需重新测实际制动距离。

---

# 14. 离散垃圾感知：最终产品语义

正式产品允许：

```text
远距离 class-agnostic/high-recall proposal
→ persistent candidate
→ OBSERVE_AGAIN / approach
→ close-range classification
→ ActionVerifier
→ CONFIRMED
→ scheduler
```

不要求一个远距离 tiny-object 单帧 detector 同时负责：

```text
发现 + 精确分类 + 清扫授权
```

但最终产品 hard gate 不降低。

---

# 15. Proposal 层验收

```text
eventual proposal recall >= 0.98
first-visible-small cohort proposal recall >= 0.95
```

产品负担目标：

```text
FP/frame <= 0.50 preferred
```

若：

```text
FP/frame > 1.0
```

默认 FAIL，除非正式证据证明后级处理、安全和性能仍全部达标。

---

# 16. Close-Range 分类验收

类别必须包含：

```text
plastic_bottle
metal_can
paper_litter
background_or_unknown
```

禁止强迫所有候选三分类。

以 candidate-level decision 为最终产品指标：

```text
macro F1 >= 0.98
each target precision >= 0.97
each target recall >= 0.97
background specificity >= 0.995
paper precision >= 0.98
metal recall >= 0.97
```

tight/context 单独指标必须报告，但最终 Gate 以产品真实 candidate 级决策为准。

---

# 17. Small-object cohort

定义：

> 目标第一次进入有效相机视野时属于 small bucket 的目标。

即使之后车辆靠近、目标变大：

```text
仍属于 small cohort
```

不得把成功靠近后的目标从 small 指标里删除。

Hard：

```text
first-visible-small eventual correct-class recall >= 0.90
```

---

# 18. ActionVerifier 验收

Classifier 不得直接 CLEAN_NOW。

必须：

```text
Classifier
→ ActionVerifier
→ CONFIRMED
→ Scheduler
```

ActionVerifier 至少检查：

```text
class confidence
background/unknown
multi-view agreement if used
multi-frame consistency
depth validity
projection covariance
track persistence
map consistency
```

Hard：

```text
correct-target acceptance recall >= 0.95
small acceptance recall >= 0.90

confirmed actionable precision >= 0.98 preferred
hard minimum >= 0.95

wrong confirmed actionable <= 0.01
negative-only confirmed actionable <= 0.01

false CLEAN_NOW = 0
wrong-class CLEAN_NOW = 0
```

---

# 19. Active Re-observation

允许：

```text
OBSERVE_AGAIN
```

Hard：

```text
max OBSERVE_AGAIN <= 2 / candidate
```

第二次仍不能确认：

```text
DEFER / REJECT
```

必须报告：

```text
mean reobserve count
P95 reobserve count
extra travel distance
extra time
```

这些成本全部计入清扫效率。

---

# 20. Tracking 验收

```text
track creation recall >= 0.98
far->close identity continuity >= 0.95
confirmed-track correct-class recall >= 0.95

ID consistency >= 0.97
duplicate track rate <= 0.01
fragmentation <= 0.03
```

不得因靠近后二次建 track 导致重复目标/重复清扫。

---

# 21. RGB-D 投影验收

对 correct + depth-valid observation：

```text
projection success >= 0.98
```

定位：

```text
median map localization error <= 0.05 m
P95 <= 0.15 m
```

DynamicTrashMap 总 RMSE：

```text
<= 0.10 m
```

---

# 22. DynamicTrashMap 验收

任务开始：

```text
EMPTY
```

推荐状态语义：

```text
CANDIDATE
OBSERVE_AGAIN
CONFIRMED
DEFERRED
SCHEDULED
CLEANING
VERIFYING
CLEANED
REJECTED
EXPIRED
```

或等价状态。

关键：

```text
CANDIDATE != actionable
CONFIRMED only after ActionVerifier
```

Hard：

```text
confirmed target precision >= 0.95
map coverage >= 0.95
map RMSE <= 0.10 m

duplicate target rate <= 0.01
pre-FOV target creation = 0
removed-target stale scheduling = 0
```

---

# 23. 动态插入/移除

Dynamic insertion：

```text
mission start target absent
target appears later
→ no target before FOV
→ create only after observation
```

Dynamic removal：

```text
target detected
target disappears before clean
→ no stale CLEAN_NOW
→ eventually REJECTED/EXPIRED/REMOVED
```

Hard：

```text
pre-FOV false target = 0
stale cleaning action = 0
```

---

# 24. Area 感知：落叶 / 积水

正式 Area Gate：

```text
macro mIoU >= 0.80
boundary F1 >= 0.80
negative actionable FP/frame <= 0.02
```

必须分别报告：

```text
leaf IoU
puddle IoU
macro IoU
boundary F1
negative FP/frame
```

不得只报 macro。

---

# 25. Negative-only 场景

必须包含：

```text
wet reflection
road paint
shadow
stones
cracks
seams
metallic glint
plastic-like clutter
paper-like flat marks
other realistic clutter
```

Hard：

```text
wrong confirmed actionable <= 0.01
false CLEAN_NOW = 0
```

---

# 26. Spot Cleaning 端到端验收

至少：

```text
30 formal seeds
```

完整链：

```text
Coverage
→ Proposal
→ Candidate
→ Track
→ Re-observe if needed
→ Close-range classify
→ ActionVerifier
→ CONFIRMED
→ Scheduler
→ Safe pause
→ Nav2 approach
→ Pre-clean verify
→ Clean
→ Post-clean verify
→ Resume Coverage
```

Hard：

```text
mission success >= 0.90
confirmed-target cleaning success >= 0.90

wrong-target cleaning = 0
false-candidate cleaning = 0
duplicate cleaning <= 0.01

Coverage safe pause = 100%
Coverage resume >= 0.90

collision = 0
keepout violation = 0
```

---

# 27. Pre-Clean Verification

清扫执行前必须重新确认：

```text
target still valid
track identity stable
map localization healthy
class confidence healthy
ActionVerifier still ACCEPT
Safety healthy
```

任一失败：

```text
cancel / DEFER
```

不得因历史 CONFIRMED 状态盲目执行。

---

# 28. Post-Clean Verification

```text
actuator success != CLEANED
```

离散垃圾只有：

```text
目标区域重新进入真实 camera FOV
+
目标连续 N 帧不存在
```

才允许：

```text
CLEANED
```

Hard：

```text
camera-backed CLEANED claims = 100%
false CLEANED = 0
```

Area：

```text
remaining_area / before_area <= 0.10
```

否则：

```text
RETRY / DEFER
```

最多：

```text
1 retry
```


---

# 29. 多模态交互验收

项目最终至少实现官方要求中的：

```text
>= 2 种交互模态
```

例如：

```text
APP
Speech
BCI
```

具体组合以最终赛题允许形式为准。

正式要求：

```text
command reception success >= 95%
unsafe command cannot bypass Safety
```

---

# 30. LLM 任务分解验收

官方目标：

```text
task decomposition accuracy >= 95%
```

测试集必须固定、版本化、不可在看到结果后修改。

至少覆盖：

```text
开始清扫
指定区域
暂停
继续
返回
点清扫
任务状态查询
非法/危险请求
模糊请求
```

Hard：

```text
task decomposition accuracy >= 0.95
unsafe action bypass = 0
```

LLM 不得直接写底盘控制。

---

# 31. 官方垃圾识别指标映射

若官方只给出：

```text
垃圾识别准确率 >= 95%
```

至少同时满足/报告：

```text
Precision >= 0.95
Recall >= 0.95
F1 >= 0.95
```

并报告：

```text
per-class Precision
per-class Recall
confusion matrix
small cohort
negative specificity
```

内部更严格产品门仍按前文执行。

---

# 32. 正式场景对象覆盖

至少包含：

```text
plastic bottle
metal can
paper litter
leaf
puddle
```

以及赛题明确规定的：

```text
小积水
落叶堆
```

若官方给出具体物理尺寸/高度条件，正式环境必须按官方尺寸构造，不得使用更容易的替代场景。

---

# 33. 综合正式仿真场景

最终正式仿真至少：

```text
30 independent seeds
```

场景矩阵必须覆盖：

```text
normal
turn
behind-FOV
partial occlusion
reappearance
wet
reflection
dark
bright
shadow
road paint
clutter
small object
dynamic insertion
dynamic removal
negative-only
dynamic obstacle
all target classes
```

开发集不能替代 final。

---

# 34. 性能验收

## 34.1 Unpaced Service Capacity

完整最终 pipeline：

```text
>=1200 frames
```

Hard：

```text
sustainable throughput >= 10 Hz
>=12 Hz preferred
```

## 34.2 Real-Time

输入：

```text
source >= 15 Hz
duration >= 10 min
```

Hard：

```text
processed rate >= 10 Hz
P95 end-to-end latency <= 200 ms
drop rate <= 1%
queue bounded
no latency accumulation
```

必须测完整 pipeline，不得只测 detector。

---

# 35. 资源占用

必须记录：

```text
GPU utilization
VRAM
CPU
RAM
```

不得出现：

```text
持续 RAM 增长
持续 VRAM 增长
queue 持续增长
频繁 model reload
```

---

# 36. 两小时 Soak

冻结后的完整产品 pipeline：

```text
continuous runtime >= 2 h
```

任务中必须真实包含：

```text
Coverage
Perception
Tracking
DynamicTrashMap
Spot Cleaning
Post-Clean Verification
```

Hard：

```text
crash = 0
deadlock = 0
memory growth <= 5%
queue growth = 0
unexpected model reload = 0
persistent TF failure = 0
unrecoverable watchdog event = 0
```

---

# 37. Fault Injection

至少测试：

```text
RGB freeze
Depth freeze
timestamp skew
CameraInfo mismatch
TF unavailable
invalid depth

proposal flood
proposal dropout
classifier exception
classifier timeout
ActionVerifier failure
re-observe timeout

CUDA/provider failure
model hash mismatch
corrupt model
sustained slow inference

Nav2 path unavailable
dynamic obstacle blocks observation
```

预期：

```text
no unsafe CLEAN_NOW
unsafe pending clean cancelled/deferred
perception health = DEGRADED/ERROR
Safety/Nav2 remains operational
safe Coverage recovery where possible
```

Hard：

```text
unsafe cleaning action = 0
```

---

# 38. MCAP / rosbag Replay

最终至少：

```text
5 formal bags
```

必须真实：

```text
ros2 bag play
```

重新计算：

```text
observations
tracks
DynamicTrashMap
scheduler
clean decisions
post-clean states
metrics
```

关键结果差异：

```text
<= 1%
```

---

# 39. 可复现性

所有正式结果必须绑定：

```text
source commit
model SHA256
config SHA256
dataset SHA256
container digest
dependency lock
seed
command
exit code
```

无法追溯的结果：

```text
不计正式验收证据
```

---

# 40. CI / Build 验收

Release commit 必须：

```text
git diff --check = pass
fast CI = pass
ROS build = pass
ROS tests = pass
secret scan = pass
```

只有明确、已记录的 optional dependency skip 才允许存在。

---

# 41. Model Freeze

Final/sealed test 前必须创建：

```text
MODEL_FREEZE_X86.json
```

冻结：

```text
models
weight hashes
thresholds
preprocessing
postprocessing
proposal policy
classifier
ActionVerifier
tracking
DynamicTrashMap
scheduler
re-observation
Area
projection
runtime cadence
container
dependencies
```

Freeze 后任何影响结果的修改：

```text
freeze invalid
```

必须重新走开发门。

---

# 42. Sealed Final / G5_V2

最终集合：

```text
one-shot
freeze-bound
immutable
```

最低建议：

```text
>=4 unseen worlds
>=100 scenes
>=1000 frames
>=20 moving sequences
```

覆盖所有主要 hard domains。

第一次访问：

```text
G5_V2_ACCESS_RECORD.json
```

必须原子写。

Final fail：

```text
final set consumed
```

不得调参后重考。

---

# 43. Release Bundle

最终生成：

```text
release/TZcup_<version>_<commit>.zip
```

至少包含：

```text
models/
manifests/
configs/
launch/
licenses/
SBOM
SHA256SUMS
dependency lock
container digest
operation guide
healthcheck
rollback guide
evidence index
```

---

# 44. Rollback

必须验证：

```text
new release stage
→ hash verify
→ inactive warmup
→ healthcheck
→ atomic switch
```

异常时：

```text
rollback to last-known-good release
```

至少真实测试一次 rollback。

---

# 45. 第三方许可 / Supply Chain

所有进入产品包的：

```text
code
models
weights
runtime
dataset-derived weights
```

必须记录：

```text
upstream
tag/commit
license
model license
source
SHA256
local modifications
redistribution status
```

未知许可：

```text
不得进入最终 Release
```

---

# 46. 仿真资产真实性

严禁通过以下方式提高指标：

```text
类别固定高饱和颜色
文字标签
二维码
隐藏 marker
扩大垃圾到不真实尺寸
人为降低背景复杂度
删除 hard negatives
修改相机到实体产品不具备的条件
```

允许修复：

```text
错误 PBR
missing texture
错误 transparency
错误 scale
错误 material binding
明显 rendering bug
```

所有 asset 修改必须版本化。

---

# 47. 正式产品运行时禁止 Debug/Oracle

Release 中必须确认：

```text
GT subscriber = disabled/absent
semantic GT input = disabled
instance GT input = disabled
known target coordinate loader = disabled
test oracle = disabled
```

由自动审计确认。

---

# 48. 产品安全分层

正式架构至少逻辑分为：

```text
Safety Plane
Autonomy Plane
Cleaning Intelligence Plane
```

Cleaning Intelligence 不得覆盖：

```text
E-stop
collision monitor
Nav2 safety
keepout
boundary protection
```

---

# 49. 最终比赛 Demo 链

最终实时 Demo 必须真实展示：

```text
建图/定位
全覆盖路径
动态避障
实时垃圾发现
实时跟踪
RGB-D地图投影
目标接近
清扫/抓取
清扫后验证
继续覆盖
```

不得使用：

```text
预设垃圾坐标
GT触发
剪辑视频替代核心实时过程
```

---

# 50. 证据格式

每个正式 Gate 必须同时产生：

```text
machine-readable JSON
human-readable Markdown
raw logs
artifact SHA256
```

每个指标必须可追溯到：

```text
数据
代码
模型
命令
环境
```



---

# 51. 正式最终状态 JSON

最终必须生成类似：

```json
{
  "SIMULATION_PRODUCT_COMPLETE": true,
  "PRODUCT_X86_PERCEPTION_READY": true,
  "PRODUCT_INTEGRATION_READY": false,
  "PRODUCT_FIELD_READY": false,

  "GT_CONTROL_VIOLATION": 0,

  "LOCALIZATION_PASS": true,
  "MAPPING_20000M2_PASS": true,
  "COVERAGE_PASS": true,
  "EFFICIENCY_3500M2H_PASS": true,

  "NAVIGATION_PASS": true,
  "DYNAMIC_OBSTACLE_PASS": true,
  "ESTOP_PASS": true,

  "DISCRETE_PERCEPTION_PASS": true,
  "AREA_PERCEPTION_PASS": true,
  "TRACKING_PASS": true,
  "DYNAMIC_TRASH_MAP_PASS": true,

  "SPOT_CLEAN_PRODUCT_PASS": true,
  "POST_CLEAN_VERIFICATION_PASS": true,

  "MULTIMODAL_PASS": true,
  "LLM_TASK_DECOMPOSITION_PASS": true,

  "PERFORMANCE_PASS": true,
  "SOAK_2H_PASS": true,
  "FAULT_INJECTION_PASS": true,
  "MCAP_REPLAY_PASS": true,

  "MODEL_FREEZE_CREATED": true,
  "SEALED_FINAL_PASS": true,
  "RELEASE_BUNDLE_PASS": true,
  "LICENSE_AUDIT_PASS": true,
  "CI_GREEN": true
}
```

任何必需字段不是 true：

```text
SIMULATION_PRODUCT_COMPLETE=false
```

---

# 52. 一票否决项

以下任一出现，最终直接 FAIL：

```text
GT 控制生产链
任务开始预加载垃圾坐标
错误目标被实际清扫
false candidate 被实际清扫
false CLEAN_NOW > 0
wrong-class CLEAN_NOW > 0
collision > 0
keepout violation > 0
E-stop 不可靠
sealed final 泄漏
final test 后调参再重考
release model hash 不匹配
silent CPU fallback 未披露
Safety 被 Cleaning Intelligence 绕过
无法提供正式证据 hash
```

---

# 53. 仿真最终判定函数

```text
function ACCEPT_SIMULATION_PRODUCT():

    if any official competition hard gate fails:
        return FAIL

    if GT_CONTROL_VIOLATION != 0:
        return FAIL

    if wrong_target_cleaning != 0:
        return FAIL

    if false_candidate_cleaning != 0:
        return FAIL

    if collision != 0:
        return FAIL

    if false_CLEAN_NOW != 0:
        return FAIL

    if wrong_class_CLEAN_NOW != 0:
        return FAIL

    if localization fails:
        return FAIL

    if mapping_20000m2 fails:
        return FAIL

    if cleaning_efficiency < 3500m2/h:
        return FAIL

    if coverage < 95%:
        return FAIL

    if perception gates fail:
        return FAIL

    if tracking/map gates fail:
        return FAIL

    if spot-clean/post-clean gates fail:
        return FAIL

    if performance fails:
        return FAIL

    if 2h soak fails:
        return FAIL

    if fault injection fails:
        return FAIL

    if replay fails:
        return FAIL

    if sealed final fails:
        return FAIL

    if release/license/hash audit fails:
        return FAIL

    return PASS
```

只有：

```text
ACCEPT_SIMULATION_PRODUCT() == PASS
```

才允许：

```text
SIMULATION_PRODUCT_COMPLETE=true
```

---

# 54. 仿真 PASS 后允许进入产品的含义

当：

```text
SIMULATION_PRODUCT_COMPLETE=true
```

允许：

```text
合并仿真产品 PR
创建 release tag
发布冻结的 x86 software baseline
将同一冻结版本移植到 J6 / 实体车
开始实体产品集成验证
```

此时该版本应被定义为：

```text
Product Software Baseline
```

而不是“实验代码”。

但是：

```text
Simulation PASS
!=
Field Product PASS
```

实体平台仍需要单独准入。

---

# 55. J6 / 目标计算平台准入

若最终产品使用 Horizon J6，至少要求：

```text
官方 toolchain version locked
PTQ success
compile success
unsupported critical op = 0
silent CPU fallback = 0
```

板端连续：

```text
>=30 min
```

建议性能：

```text
FPS >= 10
P95 model inference <= 100 ms
```

要求：

```text
crash = 0
memory leak = 0
thermal failure = 0
```

记录：

```text
BPU utilization
CPU
RAM
temperature
power
FPS
latency
```

通过后：

```text
PRODUCT_INTEGRATION_READY=true
```

---

# 56. 实体传感器与标定准入

实体产品必须重新完成：

```text
camera intrinsic calibration
camera-depth alignment
camera-to-base extrinsic
LiDAR-to-base extrinsic
IMU alignment
wheel odometry calibration
TF audit
timestamp synchronization
```

不得直接把 Gazebo 完美标定参数复制到真实产品。

---

# 57. Real RGB-D / Field Validation

最低建议数据：

```text
>=20 scenes
>=1000 qualifying RGB-D frames
>=10 moving sequences
```

覆盖：

```text
bottle
metal can
paper
leaf
puddle
wet ground
reflection
shadow
road paint
clutter
negative-only
small/distant
```

Ground Truth 必须来自：

```text
manual review
known placement
fiducial
independent measurement
```

不得用模型自己的输出作为 GT。

最低 Field Gate：

```text
object precision >= 0.90
object recall >= 0.90
each class recall >= 0.85
paper precision >= 0.85

area mIoU >= 0.75
negative specificity >= 0.95

map RMSE <= 0.15 m
pre-FOV target creation = 0
wrong clean action = 0
```

实体产品可根据实际使用要求继续收紧。

---

# 58. 实体安全准入

必须重新测试：

```text
真实刹停距离
真实动态避障
真实急停
真实行人横穿
真实车辆/障碍物
真实轮胎打滑
真实坡度
真实低附着
真实定位异常
真实感知断流
```

正式产品禁止只引用 Gazebo 安全指标。

---

# 59. 实体清扫准入

必须重新实测：

```text
实际 cleaning width >= 600 mm
实际 bin capacity >= 40 L
实际 cleaning efficiency >= 3500 m²/h
```

同时验证：

```text
真实 bottle
真实 metal can
真实 paper
真实 leaves
真实 puddle / wet condition
```

以及：

```text
wrong-target cleaning = 0
false-candidate cleaning = 0
```

---

# 60. 最终证据包

正式仿真完成时必须至少存在：

```text
FINAL_ACCEPTANCE_STATUS.json
FINAL_ACCEPTANCE_MATRIX.json
FINAL_EVIDENCE_INDEX.md

MODEL_FREEZE_X86.json
PERCEPTION_X86_FREEZE_MANIFEST.json

30SEED_DYNAMIC_DISCOVERY_REPORT.json
30SEED_SPOT_CLEAN_REPORT.json
POST_CLEAN_VERIFICATION_REPORT.json

PERFORMANCE_REPORT.json
SOAK_2H_REPORT.json
FAULT_MATRIX_REPORT.json
MCAP_REPLAY_REPORT.json

COMPETITION_GATE_MAPPING.json
THIRD_PARTY_NOTICES.md
SBOM.json
SHA256SUMS
release ZIP
```

---

# 61. 推荐 Acceptance Matrix 字段

每个 Gate 统一记录：

```text
gate_id
requirement
metric
threshold
measured_value
unit
dataset/scenario
source_commit
model_sha
config_sha
evidence_path
status
```

示例：

```json
{
  "gate_id": "PERCEPTION-07",
  "requirement": "wrong confirmed actionable <= 1%",
  "metric": "wrong_confirmed_actionable_rate",
  "threshold": 0.01,
  "measured_value": 0.004,
  "unit": "ratio",
  "dataset": "SEALED_FINAL",
  "source_commit": "...",
  "model_sha": "...",
  "config_sha": "...",
  "evidence_path": "...",
  "status": "PASS"
}
```

---

# 62. 后续开发不得移动门槛

从本文件正式采用后：

> 本文件作为 TZcup 仿真产品级最终验收的固定基准。

任何标准修改必须：

```text
显式创建新版本
记录修改原因
记录旧值/新值
记录影响范围
在读取 sealed final 之前完成
```

禁止：

```text
测试失败后降低 threshold
删除失败 metric
改变统计单位
删除 hard scene
更换 final set 后不记录
```

---

# 63. 最终成功定义

本项目最终成功不再定义为：

```text
“某个模型准确率提高”
“Gazebo 能跑”
“demo 看起来正常”
```

最终成功定义为：

```text
车辆在未知垃圾坐标条件下
从 EMPTY DynamicTrashMap 开始

自主完成：

定位/建图
→ 全覆盖清扫
→ 动态避障
→ 实时垃圾发现
→ Candidate/Tracking
→ 安全确认
→ DynamicTrashMap
→ Spot Cleaning
→ Post-Clean Verification
→ Resume Coverage
→ 安全停车

并在：

准确性
安全性
效率
性能
稳定性
可复现性
发布完整性

全部达到固定门槛。
```

只有此时：

```text
SIMULATION_PRODUCT_COMPLETE=true
PRODUCT_X86_PERCEPTION_READY=true
```

然后使用**同一冻结软件版本**进入：

```text
J6 deployment
实体车 integration
Field Validation
```

不允许移植前重新修改核心算法而不重新验收。

---

# 64. 最终原则

```text
仿真不是“能运行”就完成。

仿真产品级完成 =
功能闭环
+ 未知目标真实在线发现
+ 无GT作弊
+ 感知/地图准确
+ 0错误清扫
+ 安全闭环
+ 3500m²/h效率
+ 10Hz产品性能
+ 2h稳定运行
+ 故障安全
+ Replay可复现
+ Freeze可追溯
+ Release可回滚
+ 可直接进入实体产品部署。
```

**最终判定只接受机器证据，不接受主观判断。**
