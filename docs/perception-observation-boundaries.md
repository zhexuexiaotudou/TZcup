# 产品感知观测边界与离线回放

本页说明 DOSOD/EdgeSAM 产品输入到地图观测的拒绝条件和诊断回放。
源码与离线测试不构成真实模型精度、ROS/Gazebo 运行、S100P 或实车验收。

## 输入和输出

`pc_open_vocab_adapter` 消费注册到同一光学坐标系的 RGB、深度、CameraInfo，
以及采集时刻的 `map <- camera` TF。RGB-D 与 CameraInfo 必须尺寸一致；深度
编码必须为 `16UC1`（毫米）或 `32FC1`（米），内参必须有限且焦距为正。
RGB 与深度时间差、非零时间戳 CameraInfo 的时间差沿用 `depth_max_age_s`；
零时间戳 CameraInfo 可作为固定标定。此门不等于硬件同步或外参实测标定通过。

地面脏污只有实际 EdgeSAM 掩膜才能进入投影。因提示面积或数量限制未进入
EdgeSAM 的检测使本帧拒绝发布，诊断缺失分割；不能用空掩膜把该区域标成干净，
也不回退为整幅检测框。离散 `litter_cube` 仍使用检测框内的
有效深度生成候选位置；这不能证明目标确为 30 mm，也不能代替腕部几何测量。
缺少实测立方体几何时，腕部诊断为
`wrist_grasp_recheck_not_ready / measured_cube_geometry_missing`，不发送抓取复核。

地图投影拒绝无效内参、变换、掩膜尺寸、置信度及非有限几何；完全越界或
反向的目标框不产生目标。禁止用真值或名义尺寸补齐缺失观测。

`formal_observation_bridge` 接收 `/perception/ground_dirt/masks` 和
`/perception/garbage/targets`，发布 belief、过滤目标与
`/active_cleaning/observation_ready`。就绪需要两路消息的采集时间和到达时间
均在 `max_observation_age_sec` 内。零、未来、陈旧或重复/倒序的采集时间
被拒绝；无效消息立即撤销相应来源的新鲜状态。仿真 launch 已传
`use_sim_time=true`，单独启动节点时必须使用与消息一致的 ROS 时钟。
目标数组与每个目标必须位于 `map`；目标采集和最后观测时间也必须新鲜。
重复 UUID 整组丢弃，避免按 ID 转发时把已拒绝对象替换进输出。
`CLEANED` 与 `IN_BIN` 被解释为已清除。公共 PGM 中只有低于 `free_thresh`
的像素可通行，未知像素不可作为已知自由空间；旋转地图原点显式拒绝。

## 复用产品采集包

现有 `ProductIntermediateCapture` 可由 PC 节点的
`intermediate_capture_root` 参数启用，保存最先到达的固定频率 front 产品帧。
只有取得运行资源窗口后才可采集，回放命令本身不启动 ROS、Gazebo 或模型。

```text
capture_root/
  frames/frame-0000/{metadata.json,arrays.npz,manifest.json}
  maps/<map_content_sha256>/{metadata.json,arrays.npz,manifest.json}
```

帧包含 RGB、深度、内参、TF、检测框/类别/置信度、提示索引、EdgeSAM 掩膜与
质量值及历史投影栅格。地图快照包含公共占据数据和几何。不得用隐藏测试集
或评价真值补齐这些产品输入。

```powershell
py -3 scripts/replay_product_observation_capture.py C:/evidence/product_capture --output C:/evidence/product_replay.json
```

工具校验文件 SHA-256、地图绑定与内容哈希、时间差、标定/图像尺寸和提示索引，
然后调用当前产品投影代码。`--max-skew-s` 默认 0.5 s，复现时应与采集节点的
冻结配置一致。缺包、坏哈希、非法输入或没有可用投影输出为 `not_ready`，
退出码 2；完整回放为 `replayed`，退出码 0。

当前采集格式未记录独立 RGB/depth frame ID 和 TF 时间戳，因此回放不能
重验实时适配器的全部消息元数据门；这些仍需要原始 ROS 消息和真实运行验证。

`changed_raster_cells` 仅表示当前实现与历史栅格的差异；历史输出不是正确性
标准。哈希证明包内字节一致，不能证明数据确来自相机。报告中的
`runtime_verified`、`recognition_accuracy_verified`、`model_inference_executed`
和 `capture_authenticity_verified` 始终为 false。单元测试生成的采集包明确仅为
合成输入边界测试，不能登记为真实数据复跑或正式 session 证据。

## 验证与交付边界

快速 CI 包含 `test_rgbd_projection_boundaries.py`、
`test_formal_observation_core.py` 与 `test_perception_observation_replay.py`。
正式合并还需要受影响 ROS 包构建/测试和真实运行路径验证；部署后必须使用
精确合并版本重新验证，不能以这些离线用例替代 Stage 门。

2026-09-11 本任务没有新采集真实输入，没有运行 Gazebo，也未取得可确认的
历史原始采集包路径。因此真实输入回放、实测腕部几何、ROS/Gazebo 验收及
部署保持 blocked；分支、PR 与测试日志保留给后续资源窗口复验。
