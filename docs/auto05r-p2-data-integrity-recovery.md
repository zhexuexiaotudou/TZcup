# AUTO-05R P2 数据完整性恢复

## 结论

P2 教师训练没有暴露出一个需要继续调参的普通收敛问题，而是发现了 G4
采集链的场景状态泄漏。历史 G4 数据不得继续用于产品模型训练，原有
`G4_dataset_gate_pass=true` 已撤销。

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

紧凑证据见
`artifacts/auto05r_p2_evidence/P2_DATA_INTEGRITY_RECOVERY.json`；原始帧、完整
QA 和中止训练日志继续留在仓库外。
