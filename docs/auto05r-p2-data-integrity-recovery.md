# AUTO-05R P2 数据完整性恢复

## 结论

P2 教师训练首先发现 G4 采集链的场景状态泄漏；第一次修复后并行重采又发现
跨容器中间件串流。两批历史数据均不得继续用于产品模型训练。使用独立 ROS
domain 与 Gazebo partition 的 v3 已完成 12 world / 300 scene / 3000 frame
全量重采并通过严格 QA，当前只解锁官方 FCOS-R50 teacher，不提前解锁 student。

## 根因与影响

每个 Gazebo world 连续复用 25 个 scene。旧 `g4_scene.randomize()` 只把本场
选中的资产移动到相机前，没有把此前 scene 已经移动过的资产放回场外。
正目标因此逐场累积，scene manifest 与语义/实例像素真值逐渐分离。

新增 QA 对历史 300 scene / 3000 frame 全量复查得到：

- scene pose-reset 合同有效率 `0 / 300`；
- manifest—像素目标一致率 `987 / 3000 = 32.9%`；
- `2013 / 3000` 帧存在 manifest 未声明的额外正目标；
- QA 共记录 2313 项错误，数据门和质量门都为 false。

因此 P1 的架构淘汰结论仅保留为污染数据上的诊断；P2 FCOS-R50 教师在第
3 轮后停止，不能形成教师能力结论，也禁止启动 FCOS-lite student。

## 修复

每次随机化现在为全部 166 个目标变体与 84 个困难负样本资产生成且只生成
一个 pose：本场选中项进入观测区域，其余项全部回收到场外。scene manifest
记录 pose-reset 合同，QA 新增两个不可跳过的机器门：

1. 全部资产都有唯一 pose，且无重复名称；
2. 每帧每类像素实例数不得超过 manifest 声明数，negative-only 帧出现任何
   正目标都会失败。

真实 Gazebo `1 world × 2 scene × 10 frame` 烟测中，两门均为 100%，相关错误
为 0。完整 `12 × 25 × 10` 重采仍必须通过严格 QA，之后才允许重启官方
Torchvision FCOS-R50 教师门。

第一次三路分片完整重采虽然三个进程都正常结束，但 wrapper 未隔离 ROS 2
domain 与 Gazebo partition。三台容器的桥接 topic 因此发生跨容器串流：v2
全量 QA 的 manifest—像素一致率只有 `0.731333`，并检测到 303 组跨 split
exact duplicate 与 483 组跨 split pHash duplicate。该批数据同样作废并保留
为失败证据。v3 分片必须分别使用互异的 `ROS_DOMAIN_ID`、`GZ_PARTITION` 和
`IGN_PARTITION`。

## v3 正式恢复结果

v3 三个分片分别使用 `ROS_DOMAIN_ID=100/104/108` 与互异 Gazebo/Ignition
partition，从空目录采集后只读合并。严格 QA 实测：

- 12 worlds、300 scenes、3000 frames，split 为 8/2/2；
- scene pose-reset 合同有效率 `1.0`；
- manifest—像素目标一致率 `1.0`；
- annotation、四传感器同步、CameraInfo、TF 有效率均为 `1.0`；
- semantic/instance error rate `0.0`；
- 跨 split exact duplicate `0`，pHash duplicate `0`；
- `G4_dataset_gate_pass=true`、`quality_gates_pass=true`，失败门与错误均为空。

正式 QA SHA-256 为
`5da1a06fff93e9545a2b98412eb8d76ee889e0f4a92ae0e776de09d968d89eae`
（5695 bytes）；raw 数据与完整 QA 保留在仓库外，repo 内 compact evidence 已同步。
下一步按合同只读取 train/val 并重新训练 teacher；若 teacher 不过门，返回数据/
标注/相机尺度，不启动 student。

紧凑证据见
`artifacts/auto05r_p2_evidence/P2_DATA_INTEGRITY_RECOVERY.json`；原始帧、完整
QA、中止训练日志及 v2 串流失败证据继续留在仓库外。

## v3 双向可见性复审（2026-08-09）

后续 teacher 像素尺度审计触发了更严格的反向检查：原 QA 只拒绝画面出现
manifest 未声明的额外目标，却允许 manifest 声明的目标完全离开画面。对 v3
全部 3000 帧按五类逐帧比较 declared 与 semantic-instance observed 后，仅
1164 帧完全一致，一致率 `0.388`；1836 帧至少缺少一个声明实例，额外实例仍为
0。因此此前 `G4_dataset_gate_pass=true` 被正式撤销，2× teacher 在 epoch 8
中止，未运行 formal val，也未启动 student。

第一次修复把 QA 改为逐帧双向相等，但真实运动相机并不要求所有世界目标在每帧
同时可见；同时 C0 水平相机烟测显示纸屑最短边中位数仅 3 px、100% 小于 8 px。
因此最终合同收敛为：逐帧不得出现 manifest 未声明目标；每个声明类别须在十帧序列
中至少有 2 帧达到声明实例数。AUTO-05R 新增独立产品相机配置，以显式 Xacro 覆盖
复用 `V5_retracted` 的 `[0.36, 0, 0.66] m`、俯角 50° 机械位姿，不修改历史默认
C0；五类单实例沿 1.2–3.2 m 轨迹分层布置，横向避开扫掠走廊。旧 v3 完整复审 QA SHA-256
为 `3fe950473267210052f662dcd4919433ce1f99dcefbaf49a5ebed80e5ce1f713`，
紧凑证据见 `P2_BIDIRECTIONAL_VISIBILITY_FAILURE.json`。只有从空目录完成 v4
全量重采并通过严格 QA 后，才允许重新启动 teacher。专用配置见
`starter_ws/src/sanitation_learning/config/auto05r_product_camera.yaml`。

V5 第二轮真实 Gazebo 烟测已在全新外部目录完成：4/4 场景、40/40 帧采集通过，
逐帧未声明目标一致率和序列声明可见率均为 1.0，15 个正场景类别检查全部通过，
QA 错误为 0。纸屑最短边 p10/p50 为 19.9/30 px，离散三类总体 p50 为 32 px；
这只通过相机与场景几何选择门，不是正式 G4 数据门。紧凑证据为
`artifacts/auto05r_p2_evidence/P2_V5_CAMERA_SMOKE.json`。正式 G4 已从四个分别配置
`ROS_DOMAIN_ID`、`GZ_PARTITION` 和 `IGN_PARTITION` 的空分片目录开始重采。

正式重采与定向修复现已完成。四个基础分片各 75 场景；首次统一 QA 在 1070 个
“场景×声明类别”检查中发现 14 个只完整出现 1 帧的缺口，涉及 11 个场景，其中
leaf_pile 11 次。没有把门槛降为 1 帧，而是将最近目标车道从 1.2 m 后移到 1.8 m，
仅定向重采这 11 个场景（110 帧）；修复集逐帧一致率、序列可见率均为 1.0、错误 0。
唯一跨 split 64-bit pHash 相同的图像对经独立像素审计为不同图：SHA 不同、RGB
MAE 23.149、像素完全相同比例 5.59%、RMSE 34.003；QA 因此改为 pHash 命中后
必须再通过 64×48 RGB MAE/RMSE 确认，而不是删除样本或忽略重复门。

最终 `merged-v3` 严格 QA：12 world / 300 scene / 3000 frame、8/2/2，所有门为
true，失败门与错误均为空；scene reset、逐帧声明一致、序列声明可见、四传感器同步、
CameraInfo、TF 均为 1.0，semantic-instance error 为 0，跨 split exact/pHash 重复均
为 0。QA SHA-256 为 `72baf192e70c59d369c284c8141dcc6e2c03350dca930212ae97cf2182d1ab01`。
完整 val 离散三类最短边 p50 为 31 px、paper p50 为 29 px，预注册尺度规则选择
1× 输入。官方 FCOS-R50 teacher 已通过：正式 val recall `0.955357`、AP50
`0.950495`、precision `1.0`、false candidate/min `0`，checkpoint SHA-256 为
`a5884ac9bfa4e89f2ae8a25f4cae0521e263dd951ef895fa1185f013b2f04ee5`。紧凑证据见
`artifacts/auto05r_p2_evidence/P2_V5_FORMAL_DATA_RECOVERY.json` 与
`P2_TEACHER_PASS.json`。

teacher 通过后新增 D1–D5 单因素原生 Gazebo 诊断，每类 10 scene / 100 frame：
D1 已见世界/未见资产、D2 未见世界/已见资产、D3 已见几何/未见材质、D4 已见
资产/未见光照、D5 纯未见负样本。五类独立 QA 的同步、CameraInfo、TF、语义/
实例一致、pose reset、逐帧零额外目标和序列声明可见门均为 100%，错误为空；扩展
screening 视图共 3500 帧，保留正式 G4 QA SHA 不变，未读 legacy D6 或 G5。

A1 FCOS-lite ResNet18-FPN 首轮训练暴露 checkpoint 选择 bug：recall/IoU 仍为 0
时，零误报使约束成立，selector 在第 1 个 epoch 冻结；随后 validation loss 持续
改善却未重置 patience。现已按冻结合同改为“硬约束优先、任务指标次之、同任务指标
再比较 validation loss”，并补齐 discovery ONNX parity 的 `passed` 字段。

选择修复后的旧单网格融合基线仍严格失败：classifier 的 macro F1、paper precision、
background/hard-negative specificity 均为 1.0，四个模型的 task-specific ONNX parity
和零自定义算子门通过，D1–D5 报告完整；但 in-domain/cross-world discovery recall
均为 0，leaf IoU 为 0，puddle IoU 约为 0.22，边界 F1 近 0。该结果只作为诊断
基线保留在 `artifacts/auto05r_p4_evidence/P4_A1_SELECTION_FIXED_FAILURE.json`，不冒充
P4。当前 A1 已按产品合同改为真正保留 P3/P4/P5 三层输出、跨层 NMS、EMA warmup、
稀疏边界加权 BCE+Dice 后从零重训，不复用任何旧 checkpoint。

该重训的中间诊断证明多尺度修复有效：discovery 在固定阈值 0.35 下已恢复到
validation recall `0.9583`、AP50 `0.9157`，但仍有约 3024 false candidates/min，
因此不能通过。进一步审计发现两处协议问题：所有阈值仍是 CLI 固定值而非只在 VAL
选择；单任务 leaf/puddle selector 使用双通道平均 boundary F1，而另一个空通道使其
理论上最高只有 0.5，却要求 0.7。现已加入预注册 VAL-only discovery/classifier/
leaf/puddle 阈值网格、约束优先选择、`selected_models_product_eligible` 硬门，并改用
任务自身 boundary F1。双分支 ResNet18 area 中间结果仍明显退化（leaf IoU `0.0725`、
boundary F1 `0.0861`、negative FP/frame `0.2167`），故保留为失败诊断；下一面积
对照采用曾有强结果依据的 DeepLabV3-ResNet50，同时保持原始 RGB stem、零初始化
浅层 geometry 分支，并从 256-channel decoder feature 生成独立 boundary head。

四模型训练完成后的首次评估还暴露了预测框边界合同：右/下边界可略超出固定模型
画布，严格 model→native 变换因此拒绝继续；decoder 现统一裁剪到 640×480 并丢弃
空框，回归测试覆盖最右下网格。复用已保留 checkpoint 的 fail-closed 恢复评估完成，
固定阈值下 in-domain/cross-world candidate recall 为 `0.9672/0.9717`，但 false
candidates/min 为 `3336.5/3562.8`。更重要的是，cross-world leaf/puddle IoU 达到
`0.9043/0.9527`，而 in-domain leaf 仅 `0.0452`。审计确认 `max_train_frames=600`
此前按 manifest 顺序截断，实际只覆盖 8 个 train world 中的前三个（220/180/200
帧），却用全部 8 个世界生成 holdout；该证据不能称为 in-domain。现改为按
world×positive/negative 分层确定性抽样 600 帧，并在报告中写出逐 world 计数；
旧结果紧凑保存在 `P4_A1_PYRAMID_DIAGNOSTIC_FAILURE.json`。
