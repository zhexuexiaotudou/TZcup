# 监督台回放与过期状态

适用入口是 `human_visualization.launch.py` 启动的 `sanitation_hmi_server`，
浏览器访问 `/`（`index.html` / `app.js`）。它仍由
[监督台指南](human-visualization.md)引用；AUTO-17 的 `demo.html` 是另一个入口。

## 操作者看到的事实

- 任务状态和覆盖率来源超过 10 秒未更新后显示不可用。API 的
  `mission.last_observed` 保留最后观测值，必须结合 `sources` 的年龄查看，
  不能将旧 `COMPLETED` 作为当前任务成功。
- 刷盘来源超过 3 秒或报错后，新里程计样本的 `brush` 为 `null`；未知刷盘状态
  不累计已清扫轨迹。原有历史样本保持原值。
- 历史轨迹回放不包含同步地图、规划、感知和影像，因此这些图层不显示；
  当前系统、定位、安全来源单独标为“当前实时”。回放不会改变安全指令接口。
- 原始回放报告的成功、失败或未知结论原样显示；它不是当前 session 验收。
  播放按样本步进，时间标签使用记录时间，不保证实时速度复现。
- 服务断联清空当前状态、指标、图像并撤销旧能力；恢复连接后只使用新响应。
  已下载的回放可继续查看。

## 离线检查入口

在仓库根目录运行（不启动 ROS / Gazebo，也不发送控制指令）：

```powershell
$env:PYTHONPATH = "$PWD/starter_ws/src/sanitation_hmi"
py -3 -m sanitation_hmi.server --port 18765
```

访问 `http://127.0.0.1:18765`。没有 ROS 时，离线、来源不可用和操作按钮禁用
是预期结果。此入口只检查 HTTP 与浏览器链路，不是正式仿真或产品完成证明。

```powershell
py -3 scripts/ci_fast.py
node --test starter_ws/src/sanitation_hmi/test/test_app_replay.cjs
```

Node 测试执行实际 `app.js`，验证回放与实时轮询隔离、HTTP/网络断联、
恢复、非法样本拒绝和空地图；浏览器检查另行覆盖 1920×1080 与 390×844。
离线测试夹具始终标注“非正式运行证据”，不能代替真实来源验收。

## 当前交付边界

完整建图→固图→重启→定位→清扫→抓投/回收的正式展示需要对应生产遥测、
结果和同一 session 证据。本次经典监督台修复没有新增这些生产事件，
也没有改动控制状态机、安全或正式门聚合器。正式运行资源由主任务串行管理，
缺少新 session 的真实 ROS/仿真检查时，运行部署与正式验收仍为 blocked。
