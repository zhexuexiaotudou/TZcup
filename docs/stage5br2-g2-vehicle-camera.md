# Stage5BR2：G2 车载相机感知恢复

> 历史阶段说明：本页记录 G2 基础恢复时的边界；80/800 数据与模型筛选已在 Stage5BR3 执行，当前结论以 `docs/stage5br3-g2-screening.md` 为准。

Stage5BR2 不把 G1 顶视相机结果继续外推成部署结论。本轮新增 G2 数据域，其相机外参、视场、原生分辨率、频率和 ROS 话题均从当前车辆 Xacro 与 `sim.launch.py` 提取。训练世界内的 RGB-D、semantic GT 和 instance GT 传感器共位、同内参、同频率；GT 仅用于离线标注，生产启动文件没有被修改，也禁止 GT 进入控制或在线感知输入。

## 已完成的基础恢复

- 将历史单世界指标明确更名为 `cross_asset_same_world`；只有至少两个不同世界时才能填 `cross_world`，否则必须为 `null`。
- 实例尺寸由 instance-id 掩码逐实例计算，报告 bbox 宽高、最短边、掩码面积、深度距离和可用时的遮挡率。零像素物体记为 `not_visible`，不再作为面积 0 样本混入分位数。
- 生成四个不同 SHA 的 G2 世界：沥青校园、浅色混凝土、湿暗地面、混合路缘植被；训练使用两个世界，验证和测试各自使用一个未见世界。
- G2 资产使用项目自制 Apache-2.0 程序化几何，并保留真实物理尺寸，未沿用 G1 对瓶子和易拉罐的 1.50/1.35 放大。
- 固化四档分辨率扫描和两模型路线的筛选门；正式扩量只允许在 80 scene/800 frame 的 G2 QA 与所有筛选门通过后发生。

## 当前硬边界

四世界 SDF 和相机/GT 契约只是数据生产基础，不等于 G2 数据集已通过。车体运动轨迹、0–3 个每类实例、负样本、遮挡/不可达区域、四分辨率实测、独立 detector 与 area segmenter 的训练评估仍需实际采集结果。若 80 scene/800 frame 数据或任一筛选门未通过，`READY_FOR_GPT_REVIEW_STAGE5B` 和 `READY_FOR_STAGE5C` 必须保持 `false`，不得启动 500 scene/5000 frame、live 或 Nav2 门。
