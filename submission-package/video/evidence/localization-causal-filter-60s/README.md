# 60 秒定位因果滤波成功可视化

本目录保存定位离线因果滤波候选的可视化结果，用于支持算法优化和定位章节的复核。

## 成果摘要

- 同一 session、同一 `2711` 点配对分母。
- 两段 Nav2 目标结果均为 `[4,4]`，完成 `2/2`。
- 因果滤波后 RMSE `29.192 mm`、P95 `40.285 mm`、max `47.566 mm`。
- 滤波器只读取当前及过去 TF，评分阶段才使用 ground truth。

## 边界

- 这是离线因果复算和可视化，不是 live Gazebo 或 official PASS。
- 原始 live run 的 max `126.328 mm` 失败边界仍保留。
- 在同一冻结实时 TF 链中完成 live 复跑前，不升级官方定位状态。

## 文件

- `localization_tracking_success_1080p.mp4`：60.0 s、1920x1080、30 fps 成功可视化。
- `localization_trajectory_success.png`：轨迹。
- `localization_error_success.png`：误差曲线。
- `tracking_goals_success.png`：双目标完成状态。
- `recovery_receipt.json`：离线因果复算回执。
- `render-manifest.json`：输出与来源哈希。
