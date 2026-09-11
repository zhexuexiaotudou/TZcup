# HMI 离线浏览器检查证据

日期：2026-09-11。来源是本任务隔离 worktree 的真实 HTTP 服务和 Chromium
页面，所有过期/失败回放响应由明确标注的离线 UI 测试夹具提供。
**不是 ROS、Gazebo、正式 session 或比赛成功证据。** 没有发送控制 POST。

- `stale-desktop.png`：1920×1080，任务/刷盘来源过期，当前指标不可用。
- `replay-failure-mobile.png`：390×844 全页，历史标签在轮询后保持，原始报告失败，
  未记录的地图/规划/影像不显示；手机地图工具栏完整可见。
- `disconnected-desktop.png`：1920×1080，注入 HTTP 503，当前任务未知、旧能力撤销。
- `screenshot_hashes.json`：上述 PNG 的 SHA-256 与字节数。

行为检查执行实际 `app.js`，6 项 Node 测试通过；HMI Python 测试 20 项通过。
浏览器还覆盖 390×844 断联和 1920×1080 失败回放。没有 JS 运行异常；503 是
故意注入的故障。初次浏览器检查发现并修复缺失 favicon 的 404。

本机完整脚本和原始截图保留在 `output/playwright/`，其中服务日志可能含自动生成的
本地操作令牌，不进入 Git。专题说明见 `docs/hmi-replay-truthfulness.md`。
