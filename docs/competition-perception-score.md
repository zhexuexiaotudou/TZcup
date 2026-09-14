# C-perception：独立受控评分，2026-09-13

**识别质量 FAIL；固定相机 RGB-D 消息链冒烟 PASS；完整功能可靠性 PARTIAL；官方 R01 保持 NOT_MEASURED。** 本任务不评价定位，不宣称 map 目标准确、导航成功或由识别驱动的清扫闭环。

只运行了一次小型 Gazebo 相机场景，没有完整车辆任务、大地图、定位、视频、HBM、新模型训练、PR 或全量 CI。原定位候选与已通过清扫修复保留；原 `run_competition_motion_cleaning_probe.sh` 未改变。

## 实施与冻结参数

基线 `210c4dd39af5d39a354e41e0b39316b517e96d33`；感知修复提交 `da3beec86733fe73bc45c39bd40562abfce9979e`。C 补丁移植后补齐：CameraInfo 到达时重试待处理图像；相同/过期图像时间戳不重复增加观测次数；同帧多个连通分量不能重复确认同一 track；输出过滤前的原始候选框、策略框、内部状态计数和拒绝原因。默认不把 TENTATIVE 发布为正式目标。

TF 查询使用图像时间；RGB/depth/CameraInfo 要求同坐标系、同尺寸，时间差 ≤0.03 s。32FC1 按米使用，16UC1 转米，不缩放深度图冒充配准。本次实际验证的是 32FC1，未声称实测了 16UC1 相机。

运行前冻结五类阈值均为 0.8、最小连通区域 24 像素，base_footprint ROI 为 `[0.4,3.0,-0.8,0.8,-0.08,0.30]`。z 上限由候选 0.15 改为 0.30 是运行前依据注册表瓶高 0.24 m 的决定，避免俯视瓶顶被排除；没有看 holdout 后调参、重采或选最好结果。

模型为现有 `stage5b_learned_perception.onnx`，SHA-256 `6858863df0588f33083779f15c87a37e7f280d06d8241c37363277d4c9ecc328`。模型卡注明 D1 procedural rendered 训练域，不是实时 Gazebo 相机域，更不是已验证真实垃圾模型。场景使用已有几何/颜色资产与积水代理面，结果只能解释这个受控场景，不能外推真实环境识别率。

自带 12 项测试通过；补齐同帧确认检查后，与相关跟踪测试合计 15 项通过。只重建 `sanitation_perception` 独立 overlay，构建、导入和 Python/Bash 语法检查通过。最终评分脚本对封存 MCAP 完整读取及复算返回 0。

## 输入、可见性与独立标注

既有 competition-integrated 包无 RGB-D 图像；旧 smoke 无完整独立标签，不能独立评分。因此采用一次静止俯视 RGB-D 相机采集，五类为 plastic_bottle、metal_can、paper_litter、leaf_pile、puddle，负样本包含箱体、地面标记及背景。相机和 base/map 的关系是公开固定标定，不是从定位 GT 回灌。产品节点只读取相机、标定及 TF；对象身份/几何仅用于构建测试环境和离线评分。

首张暖机图为黑帧，未据此启动评分。在同一 Gazebo 进程内继续暖机后，检查真实 RGB，确认五类及负样本可见，随后启动评分段。没有重启第二次 Gazebo。评分起点为 3.198 s，终点为 64.199 s；固定起点后每 2 s 一帧，共 30 个时刻，按预定 ±0.26 s 选择最近原始图像。实际评分图像时间为 5、7、…、63 s，未移动时间戳。

30 个 RGB 帧全部存在。每帧 RGB/depth/CameraInfo 均为 848×480、同 optical frame，深度编码 32FC1，配对时间差为 0。CameraInfo 的 fx/fy 约 619.759 像素。标注根据已封存 RGB 的逐帧可见对象、资产身份与独立几何投影生成，每张图像 SHA 和可见 object_id 保存在 `visibility-annotations.json`；有对象的 76 次实例均通过独立深度表面可见性检查，不采用模型预测作为真值。

**第 16 帧是过渡正样本，不是空背景。** 尽管隐藏指令已经按计划发出，35 s 图像中积水仍可见。逐帧复核后将其计入积水真值，没有删掉该帧。最终为 15 帧五类全可见、1 帧仅积水可见、14 帧空垃圾背景，总计 76 个可见实例。原先按计划推测背景的 `score/` 初算被保留并明确作废；权威结果为 `score-reviewed/`。这是纠正独立标签，不是调阈值或重选样本。

这些是高度相关的静态重复画面，不是 30 个独立场景；不能据此给总体识别率置信区间。

## 两层检测结果

原始层是每个已执行推理帧的类别连通区域，最小 24 像素，位于 confidence/depth/ROI/tracking 过滤之前；策略层是过滤后的 `/detections_2d`。两层都使用同一 30 帧、相同独立标签、类别一致且 IoU ≥0.5 的一对一匹配。13 个固定帧没有原始/策略检测输出，均按空预测计入，没有删除。

| 类别 | 真值数 | 原始 TP / FP / FN | 原始 P / R | 策略 TP / FP / FN | 策略 P / R |
|---|---:|---|---|---|---|
| plastic_bottle | 15 | 0 / 92 / 15 | 0% / 0% | 0 / 0 / 15 | 未定义 / 0% |
| metal_can | 15 | 0 / 195 / 15 | 0% / 0% | 0 / 0 / 15 | 未定义 / 0% |
| paper_litter | 15 | 0 / 109 / 15 | 0% / 0% | 0 / 8 / 15 | 0% / 0% |
| leaf_pile | 15 | 0 / 196 / 15 | 0% / 0% | 0 / 0 / 15 | 未定义 / 0% |
| puddle | 16 | 0 / 62 / 16 | 0% / 0% | 0 / 0 / 16 | 未定义 / 0% |
| 合计 | 76 | **0 / 654 / 76** | **0% / 0%** | **0 / 8 / 76** | **0% / 0%** |

无预测类别的 precision 分母为零，所以报告未定义，不伪装为 100%。保守宏平均 P/R 均为 0。没有达到 IoU 门的正确类别匹配，故匹配框 IoU/位置误差列表为空；原始分割对独立几何 mask 的逐类 IoU 另存 JSON。误检和漏检均附 frame_id、原图路径、框与类别，30 张原图和评价叠图全部保留。

过滤将 654 个原始误检减少到 8 个策略误检，但召回仍为 0，不能称作识别质量改善。83 次 RGB-D context 拒绝是回调事件计数，不是 83 个独立丢帧；13/30 的实际缺输出说明同步缓存/回调顺序仍需离线排查。已有实际输出的正样本帧同样为 0 TP，不能把全部失败归因于缺帧。没有继续在这个 holdout 上调参。

## 功能与质量分开

实际相机→ONNX→2D/3D→带 map frame 坐标的目标消息路径能够执行；正式目标消息中 CONFIRMED 出现 32 次、TENTATIVE 为 0，仅 1 个唯一 track ID。内部去重诊断的状态×帧计数为 TENTATIVE 2、CONFIRMED 32、LOST 34。这些计数不是正确识别次数或独立物体数量；该目标未获得正确类别/框的评分支持。

所以只将**固定相机消息链冒烟**标为 PASS。完整感知功能可靠性为 PARTIAL（原始输出覆盖仅 17/30），识别质量为 FAIL。官方“95%识别准确率”的精确定义未确认，工程 P/R 不擅自替代它，R01 保持 NOT_MEASURED。由于定位仍未闭环，map 目标准确性及感知驱动动作闭环均为 NOT_MEASURED，不提供定位精度通过声明。

## 封口、资源与回滚

runner 返回 4：录包进程在首个 10 s 退出等待窗口内未被判定退出。此失败标记保留。录包日志确有 `Recording stopped`，metadata 存在；随后对 MCAP summary、CRC 和全部消息的读取成功，最终评分返回 0。因此数据封口通过不等于 runner 命令通过。未重新采集以消除这条失败记录。

`final-resource-release.json` 确认唯一任务域 93/partition 无残留进程、仿真锁可获取，**SIM_RESOURCE_RELEASED=true**。所有失败尝试、黑帧、初算与复核结果均保留。

云端证据：`/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/perception-score-20260913-01`。本地封存副本：`.work/perception-score-20260913-01/evidence-reviewed`。权威评分目录为 `score-reviewed`，并有逐图哈希标注、完整 MCAP、参数/模型绑定、模型卡、源码、构建日志及释放证明。

回滚点为 `210c4dd`；只反向撤销本次感知提交并省略感知 overlay，不撤销定位候选或清扫提交。工作树、分支、临时资产及证据均保留，等待用户确认后再考虑清理。项目文档已按 neat-freak 同步，未写入全局记忆。

## 2026-09-14 离线根因复核

原始结论保持 FAIL。复核未修改 30 帧标签、76 个失败分母、IoU 0.5 门、0.8 类阈值或 holdout。

根因不是简单的类别索引错位，而是受控彩色夹具误用了错误的模型档案：原运行使用已在其冻结 test 上失败（离散 macro F1 `0.0076769537847382165`）的 `stage5b_learned_perception.onnx`。该夹具使用固定、简单颜色对象，模型域应绑定已有 `synthetic_perception_pass=true` 的 Stage5A 色彩原型模型，而不是 Stage5B 程序化渲染学习模型。

修复将受控夹具绑定到 `controlled_primitive_color_fixture` 档案，并在启动前校验模型 SHA-256；默认不再接受任意 `PERCEPTION_MODEL` 绕过域绑定。没有训练、采样或调参。

同一 `score-reviewed` 的 30 张 RGB 离线复放中，当前有可追溯原始指标的是 raw 层。前文表中的 `0 / 8 / 76` 是 2026-09-13 原运行捕获的策略层结果，不是本轮离线复放产物。

| 模型与过滤 | 证据状态 | TP / FP / FN |
|---|---|---:|
| Stage5B 冻结模型 raw 离线复放 | measured retained | 0 / 654 / 76 |
| Stage5A controlled profile raw 离线复放，保留原 17/30 原始输出帧 | measured retained | **33 / 50 / 43** |
| Stage5A controlled profile policy 离线复放 | NOT_RUN | NOT_MEASURED |

Stage5A 在原有输出的 17 帧内恢复了瓶、罐、纸和积水；13 个没有原始输出的帧继续按空预测计分，没有删除或补造输出。黄色叶堆仍失败。原因是 Stage5A 固定背景原型近似黑色，而 controlled fixture 背景为灰色，整幅灰色地面被判为 `leaf_pile`，因此叶堆框的 IoU 不过门。修复没有查看 holdout 后重调背景原型。

策略层没有被实际复放。提交的 `replay_competition_perception_model.py` 只计算 raw RGB 候选，并明确写入 `policy_replay_status=NOT_RUN` 和 `policy_metrics=null`；上轮提交的离线 JSON 中没有足够的 depth、ROI 与 tracking 状态。因此此前表中的 Stage5A 策略三元组不可复现并已撤回。2026-09-14 后续已从原始 MCAP 的嵌入式 schema 与消息中恢复 depth、CameraInfo 和静态 TF，并单独执行策略层离线复放，见下一节。完整识别仍为 FAIL。

下一步需要在独立开发集上预先冻结 Stage5A 阈值及背景拒绝规则，生成单独的策略层 replay 产物并在启动前校验模型与输入哈希，再采集新的独立 holdout；现有 15 个重复正样本和 14 个背景样本只能支持此受控域诊断，不能证明 95%，也不能外推真实域。

未验证边界：没有重跑 Gazebo、ROS、定位或板端；上轮 policy 状态为 NOT_RUN，后续 MCAP 离线复放单独记录在下一节；`competition_R01` 和完整功能可靠性仍保持 NOT_MEASURED/PARTIAL。机器可读状态与原始文件哈希见 `artifacts/perception_replay_20260914_review/replay_status.json`。

## 2026-09-14 GPU 有界恢复尝试

本轮只在隔离 worktree `codex/day1-perception-gpu@3ae009a` 工作，使用租用 RTX 3080 Ti 做训练和校准，没有启动 Gazebo、S100P 或其他仿真。冻结输入仍为同一 30 帧、76 个实例、IoU 0.5、五类阈值 0.8、最小区域 24 像素和原 17/30 raw-output presence mask。

现有仓库没有当前可用的五类真实/在线训练图像集。Stage5A 仅保留确定性的合成 smoke 定义：12 train、4 val、4 test scene；因此本轮没有把 holdout 用于训练或选模，而是从同一生成器另行生成 4096 个 train scene 和 512 个 val scene，显式加入灰色背景与受控颜色扰动，只配置一次 24 epoch 的模型尝试。前两次执行均在训练完成后因 harness 的 tensor device/uint8 转换错误而未产出 artifact；修复 harness 后以完全相同的数据、超参、模型和阈值重跑，未更换候选。该训练数据仍属于合成颜色域，不是独立真实域数据。

GPU 训练只在训练循环内计时 `12.205 s`，`nvidia-smi` 每 0.5 s 采样得到平均利用率 `66.55%`、最高 `91%`，峰值训练显存 `1821.24 MiB`；数据生成时间未计入该数值。模型为 `10342` 参数的 CNN，ONNX SHA-256 为 `00f4f96f3eb925e202266506d22892a1df84838d08dbf6541d204c3e1d089c6a`。

| 阶段 | Stage5A 基线 | 灰色背景校准候选 | 边界 |
|---|---:|---:|---|
| 合成 validation foreground macro-F1 | 0.754595 | **0.999935** | 同生成器、非独立真实域 |
| 合成 validation background-to-leaf FP rate | 0.187741 | **0.00000187** | 同生成器、非独立真实域 |
| 冻结 30 帧 raw TP / FP / FN | 33 / 50 / 43 | **41 / 0 / 35** | 继续保留原 17/30 输出帧 |
| 冻结 30 帧 raw macro P / R | 0.552941 / 0.432500 | **1.000000 / 0.539167** | 受控重复帧，不证明 95% |
| 原始 MCAP policy replay TP / FP / FN | 0 / 0 / 76 | **33 / 0 / 43** | offline replay，非官方 R01 |
| 原始 MCAP policy replay macro P / R | 0.000000 / 0.000000 | **0.800000 / 0.432500** | metal can 全被策略门拒绝 |

策略层使用 `/sensors/front_rgbd/depth/image_rect_raw/{image,depth_image,camera_info}` 与 `/tf_static`，匹配到 129 个图像时刻、30 个深度帧、129 个 CameraInfo 和 2 个静态 TF，无缺失绑定。它仍不是原运行内策略层的位级复现：本轮从 MCAP 原始消息恢复输入并重新应用冻结 ROI/阈值；基线策略三元组由原来的 `0 / 8 / 76` 复算为 `0 / 0 / 76`，因此两者分别标注，不互相冒充。

结论是灰色背景叶堆误分类已在本受控域内被有效抑制，raw precision 提升到 1.0，policy replay 从全漏检变为 `33/0/43`。这不表示识别率达到 95%，也不表示真实域、官方 R01、J6 板端或完整清扫闭环通过。全部中间失败、固定配置、模型、raw/policy replay 与哈希见 `artifacts/perception_gpu_recovery_20260914/`；完整大体积证据保留在远程 `.../.work/gpu-perception-recovery-20260914`。
