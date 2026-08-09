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
