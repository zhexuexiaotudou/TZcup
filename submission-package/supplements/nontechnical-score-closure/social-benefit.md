# 社会效益参数模型

## 1. 目的

社会效益不能只写“减少人工、提高效率”。本包把人工工时、监护工时、返工工时、清洁质量和能源分开计算，并要求每个输入标注来源。

当前状态：`MODEL_ONLY_NOT_FIELD_MEASURED`。

## 2. 计算逻辑

```text
baseline_total_hours =
  manual_cleaning_hours + manual_supervision_hours + manual_rework_hours

autonomous_total_hours =
  autonomous_supervision_hours + autonomous_rework_hours

person_hour_reduction =
  baseline_total_hours - autonomous_total_hours

person_hour_reduction_rate =
  person_hour_reduction / baseline_total_hours

energy_delta_kwh =
  vehicle_kwh - manual_equipment_kwh
```

清洁质量必须单独报告，例如残留率、一次通过率、返工面积和投诉率。若质量输入缺失，则 `quality_claimable=false`，即使工时下降也不能声称社会效益已实现。

## 3. 证据分层

| 来源等级 | 含义 | 可支持 |
|---|---|---|
| `OFFICIAL` | 政府、标准或赛题原文 | 背景和口径 |
| `FIELD_MEASURED` | 同场景、同班次、同质量口径的真实记录 | 项目效果声明 |
| `VENDOR` | 供应商规格或报价 | 能耗、寿命、单价输入 |
| `ASSUMPTION` | 用于演算和敏感性分析的假设 | 只支持模型演示 |
| `UNSET` | 尚无输入 | 不能算出正式结果 |

## 4. 当前情景文件

`data/social-benefit-scenarios.json` 的三个情景均为 `ASSUMPTION`，只用于验证模型和展示敏感性。它们不是园区实测值，也不能写成项目已经减少 50%-67% 人力。

## 5. 正式试点最小数据

同一区域至少采集一个前后对照周期：

- 清扫面积、路线长度、班次和天气；
- 人工作业人数、工时、监护和返工；
- 车辆耗电、充电、耗材和故障停机；
- 清洁质量抽检、一次通过率、投诉和人工补扫；
- 采集时间和负责人员。

## 6. 当前可写入报告

> 项目已建立将人工工时、监护、返工、能耗和清洁质量分开审计的社会效益模型。当前情景只用于方法演示，尚无同场景前后对照数据，因此不声称已经产生可量化社会效益。
