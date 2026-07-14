# 仿真评测计划

## 1. 运行档位

- `smoke`：1–3 分钟，验证启动和接口；
- `functional`：10–20 分钟，验证建图、导航和覆盖；
- `stress`：1–8 小时，验证稳定性；
- `batch`：headless 多随机种子运行。

## 2. 场景集

| ID | 场景 | 主要指标 |
|---|---|---|
| S00 | 空场直线/转弯 | 控制稳定、里程计 |
| S01 | 静态障碍绕行 | 成功率、最小距离 |
| S02 | 规则矩形覆盖 | 覆盖率、重复率 |
| S03 | 非凸区域覆盖 | 覆盖率、恢复次数 |
| S04 | 内部禁行区 | 越界次数 |
| S05 | 窄通道 | 通过率、横向误差 |
| S06 | 低摩擦积水区 | 打滑代理量、轨迹偏差 |
| S07 | 落叶/离散垃圾 | 感知率、清扫完成率 |
| S08 | 动态行人横穿 | 避障率、制动距离 |
| S09 | 急停注入 | 响应时间 |
| S10 | 任务中断与恢复 | 恢复成功率 |
| S11 | 大地图 headless | 实时率、CPU/GPU、内存 |

## 3. 随机化

每次 batch run 至少随机化：

- 垃圾位置和类别；
- 障碍位置；
- 传感器噪声；
- 轮地摩擦；
- 初始位姿；
- 动态障碍速度；
- 网络/推理延迟（后续）。

所有随机种子写入报告，确保可复现。

## 4. 报告 JSON 最小字段

```json
{
  "run_id": "2026-xx-xxTxx-xx-xx_seed42",
  "git_commit": "",
  "ros_distro": "jazzy",
  "gazebo_version": "",
  "scenario_id": "S02",
  "seed": 42,
  "success": true,
  "coverage_ratio": 0.0,
  "miss_ratio": 0.0,
  "overlap_ratio": 0.0,
  "cleaned_area_m2": 0.0,
  "duration_s": 0.0,
  "effective_rate_m2_h": 0.0,
  "path_length_m": 0.0,
  "collisions": 0,
  "recoveries": 0,
  "localization_rmse_m": null,
  "emergency_stop_latency_s": null,
  "artifacts": []
}
```
