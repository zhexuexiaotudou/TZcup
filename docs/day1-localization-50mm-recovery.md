# 定位 <= 50 mm 因果恢复候选

日期：2026-09-14

状态：`OFFLINE_CANDIDATE_PASS`

## 官方边界

官方材料只写“定位精度 <= 50 mm”，没有明确采用 RMSE、P95 还是最大值。
因此本结果只声明：对同一封存 run 使用在线可见输入做因果复算后，RMSE、
P95、max 三项都满足严格 `<= 50 mm`。它不是新的 Gazebo 验收，也不代表
已经获得官方 PASS。

## 输入与冻结

- 原始 MCAP：
  `localization-day1-final-20260914-02/run-01/bag/bag_0.mcap`
- MCAP SHA-256：
  `8e64c0b7dd6a29247ad0d2f61ac719ba7f8c4f6b1957edf8bffc88aa38308047`
- 原始 focus JSON SHA-256：
  `ca147bdc54824263b7c05c9bb16f4be542b59f2bbb1d81258b13659fd96ac939`
- 动态参考样本 `2711`，严格配对样本 `2711`，配对覆盖率 `1.0`。
- 基线结果与冻结 focus JSON 的 `accuracy` 完全一致。

## 根因观察

原始链路：

```text
T_map_base = T_map_odom * T_odom_base
```

其中 `T_odom_base` 对同帧真值的误差很小：

| 输入 | RMSE | P95 | max |
|---|---:|---:|---:|
| `/odom` 局部里程计 | 0.373 mm | 0.670 mm | 0.689 mm |
| 原始 `T_map_odom * T_odom_base` | 39.271 mm | 83.780 mm | 126.328 mm |
| `/localization/fused_odom` | 39.649 mm | 83.525 mm | 126.055 mm |

误差主要来自全局 `map -> odom` 修正随时间出现的低频漂移；局部运动链
本身没有同等量级误差。

## 在线因果候选

对 `map -> odom` 使用一阶因果低通：

```text
alpha = 1 - exp(-dt / tau)
y[k] = y[k-1] + alpha * (raw[k] - y[k-1])
```

然后继续保持原有变换组合：

```text
T_candidate_base = lowpass(T_map_odom) * T_odom_base
```

- 默认 `tau = 1.5 s`
- `dt` 上限 `0.1 s`
- 在线输入只有 `/tf map->odom` 和 `/tf odom->base_footprint`
- 每个评分时刻只取不晚于该时刻的最后一条 TF，不使用未来数据
- 不使用 ground truth、不做 truth-based 对齐、不插值跨越缺失段
- ground truth 只在滤波完成后参与评分

## 同分母结果

| 指标 | 改进前 | 改进后 | 变化 |
|---|---:|---:|---:|
| RMSE | 39.271 mm | 29.192 mm | -10.079 mm |
| P95 | 83.780 mm | 40.285 mm | -43.495 mm |
| max | 126.328 mm | 47.566 mm | -78.762 mm |
| 严格配对样本 | 2711 | 2711 | 0 |

改进后的最大误差出现在仿真时间 `21.08 s`，值为 `47.566 mm`；原始最大
误差发生在 `79.40 s`，值为 `126.328 mm`。全部有效运动样本均属于
`straight` 类，因为该 run 的评分样本没有静止或转向样本：
`2711 / 2711`。

## 稳定性检查

预定义参数敏感性：

| `tau` | RMSE | P95 | max | 严格 max |
|---:|---:|---:|---:|---|
| 0.75 s | 26.682 mm | 39.148 mm | 49.651 mm | PASS |
| 1.00 s | 27.518 mm | 39.324 mm | 48.964 mm | PASS |
| 1.25 s | 28.361 mm | 39.840 mm | 48.145 mm | PASS |
| 1.50 s | 29.192 mm | 40.285 mm | 47.566 mm | PASS |
| 2.00 s | 30.782 mm | 41.073 mm | 44.667 mm | PASS |
| 3.00 s | 33.688 mm | 46.550 mm | 48.191 mm | PASS |

这说明结果不是某个单点参数偶然通过；`0.75--3.0 s` 都存在严格 max 通过区。
`1.5 s` 被选为当前折中值，避免过度延迟真实的 `map -> odom` 修正。

## 失败尾巴与相关边界

- 原始最大误差尾部仍保留在 CSV 中，没有删除任何样本。
- 改进后误差仍具有时间相关性，说明低通抑制的是高频修正扰动，不是零均值
  白噪声。
- 该 run 的局部里程计本身非常接近真值，因此本候选主要证明“不要被全局
  修正链路的高频漂移带偏”，不能替代长期漂移、重定位跳变或多地图验证。

## 复现

```powershell
$env:PYTHONPATH = (Resolve-Path .work\python).Path
py -3.13 scripts\day1_localization_50mm_recovery.py `
  --mcap .work\localization-recovery\remote-day1-final-20260914-02\run-01\bag\bag_0.mcap `
  --focus-result .work\localization-recovery\remote-day1-final-20260914-02\run-01\localization_focus.json `
  --output-dir .work\localization-recovery\recovery-output-final
```

如果 Python 环境没有 MCAP 依赖：

```powershell
py -3.13 -m pip install --target .work\python mcap mcap-ros2-support
```

## 实时采用边界

要变成真实运行证据，还需要在唯一 TF 权威链中加入同等的 `map -> odom`
稳定器，并重新跑一次有界动态验收。当前不要直接把离线 receipt 当官方
PASS；也不要在未重跑的情况下把参数平滑逻辑标记为 Gazebo 已验证。

实时接线、排他 domain/partition、六类验收门和一键 harness 见
`docs/day1-localization-stabilizer-live.md`。

回滚基线：`47e3cb3a7ecc01edd82aa23a3b54cbeaffc418bc`。
