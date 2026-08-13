# Task Reformulation and Close-Range Verification V10

TRCRV10 继承 TGARV9 的完整失败事实，但重构产品决策链，不再要求远距 detector 同时完成发现、三分类和清扫授权。冻结链路是：远距 class-agnostic proposal → 持久候选 → RGB-D 地图候选 → 安全接近或重观察 → 近距四分类 → 独立 ActionVerifier → 多帧一致 → 确认 → 调度 → 清扫。

`OBSERVATION`、`CANDIDATE`、`OBSERVE_AGAIN` 和 `CLASSIFIED` 均不可直接触发清扫。只有独立 `ACTION_VERIFIED` 且满足多帧一致性后，目标才可进入 `CONFIRMED` 和 scheduler clean path。GT 类别、坐标、semantic mask 和 instance ID 只用于训练或独立 evaluator。

开发顺序固定为视觉可辨识性/资产审计、G10 approach sequences、class-agnostic proposal、近距分类器、ActionVerifier、主动重观察和综合 HOLDOUT。综合 HOLDOUT 通过并冻结全部阈值前，`G10_DEV_VAL_SEALED`、历史 `VAL_NEW`、`G5_V2` 和正式 30-seed 数据保持未读。禁止 T4/T5/T6 或无界 detector 搜索。

允许停止的结果只有仿真产品全门通过，或协议定义的可辨识性/资产、综合 HOLDOUT、一次针对性恢复、一次性 DEV-VAL 硬阻断。所有失败均 fail-closed，不得以未执行或不支持的指标冒充通过。
