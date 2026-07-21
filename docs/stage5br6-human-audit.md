# Stage5BR6-A：双人盲审封闭与停止边界

## 结论

本阶段只完成真人评审的安全交付准备。V4 是人工审计开始前固定的候选，不是人工答题后重新在 V1/V2/V4 中选优。当前没有两份完整真人 response，因此不会冻结相机、policy v2 或 candidate footprint，也不会运行 Oracle 主动观察和模型训练。

## 审计集

每位评审收到相同的 270 个视觉样本，但顺序和 opaque sample ID 独立：200 个正样本来自 Stage5BR5 V4 审计集，五类各 40；70 个负样本来自 Stage5BR6 训练专用 label=0 几何世界的真实 V4 Gazebo camera 采集。七类负样本各 10 张，四传感器精确同步，裁剪前后均不依赖类别预测，并用 semantic mask 确认目标像素为 0；生产世界未修改。

负样本覆盖同色非垃圾、瓶/罐形障碍、湿地面但非积水、阴影、落叶背景但非目标区、车辆自身结构和裁剪边界伪影。负样本来源、world、frame、模型名和答案只存在于 sealed truth，不进入评审包。

## 隔离与完整性

Reviewer A/B 包仅包含图片、说明、空白 JSON/CSV response 模板和内部文件清单。PNG 由像素重新编码，不含 EXIF/text chunks；文件名只使用 opaque ID。外部 handoff manifest 记录 package ID、ZIP SHA-256、sample ID 集合摘要和 sealed truth SHA。包内不存在 world ID、camera ID、原 Git 路径或 sample truth。

回收脚本只执行 package、身份声明、时间、字段类型、sample ID 完整集合与两位评审独立性检查。它不作答、不补值、不改 response；只有完整性通过后才允许揭盲评分。

## 恢复条件

两名互相独立的真人分别完成 Reviewer A/B 后，将 JSON 原样放到：

```text
external_reviews/stage5br6/reviewer_A_completed.json
external_reviews/stage5br6/reviewer_B_completed.json
```

首次回收时先固定两份 response 的 SHA-256，再执行完整性与人工指标。任何缺失、串包、重复 sample ID、相同 pseudonym、无效声明或时间倒置都会 fail-closed。失败时不得使用同一审计集调阈值、删困难样本或改选 V1/V2。

## 当前机器状态

```text
AWAITING_HUMAN_REVIEW=true
READY_FOR_STAGE5BR6_ORACLE=false
READY_FOR_GPT_REVIEW_STAGE5BR6=false
READY_FOR_STAGE5BR7=false
READY_FOR_GPT_REVIEW_STAGE5B=false
READY_FOR_STAGE5C=false
```

固定边界继续为：竞赛感知、真实域评测、J6 runtime 和竞赛效率均未通过，理论效率仍为 `1053 m²/h < 3500 m²/h`。
