# Task Reformulation and Close-Range Verification V10

TRCRV10 继承 TGARV9 的完整失败事实，但重构产品决策链，不再要求远距 detector 同时完成发现、三分类和清扫授权。冻结链路是：远距 class-agnostic proposal → 持久候选 → RGB-D 地图候选 → 安全接近或重观察 → 近距四分类 → 独立 ActionVerifier → 多帧一致 → 确认 → 调度 → 清扫。

`OBSERVATION`、`CANDIDATE`、`OBSERVE_AGAIN` 和 `CLASSIFIED` 均不可直接触发清扫。只有独立 `ACTION_VERIFIED` 且满足多帧一致性后，目标才可进入 `CONFIRMED` 和 scheduler clean path。GT 类别、坐标、semantic mask 和 instance ID 只用于训练或独立 evaluator。

开发顺序固定为视觉可辨识性/资产审计、G10 approach sequences、class-agnostic proposal、近距分类器、ActionVerifier、主动重观察和综合 HOLDOUT。综合 HOLDOUT 通过并冻结全部阈值前，`G10_DEV_VAL_SEALED`、历史 `VAL_NEW`、`G5_V2` 和正式 30-seed 数据保持未读。禁止 T4/T5/T6 或无界 detector 搜索。

允许停止的结果只有仿真产品全门通过，或协议定义的可辨识性/资产、综合 HOLDOUT、一次针对性恢复、一次性 DEV-VAL 硬阻断。所有失败均 fail-closed，不得以未执行或不支持的指标冒充通过。

视觉资产审计保留 G4/G8/G9 原域不变，并为 G10 创建独立 `g10_physical_close_range_v1` 域。该域沿用既有跨类别调色板、程序纹理、PBR 参数和真实物理尺寸，只补足塑料瓶的透明体/瓶肩/瓶颈/瓶盖、金属罐的上下 rim/顶面 inset，以及纸张的不规则边缘/浅折痕。禁止固定类别色、文字、二维码、类别标记或不真实尺寸放大；结构审计通过后仍须由真实 Gazebo 近距图像和独立 HOLDOUT 证明可辨识性。
