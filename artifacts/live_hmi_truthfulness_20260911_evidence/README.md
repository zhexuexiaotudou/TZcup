# Live HMI 离线状态真实性证据

2026-09-11，真实 Chromium + 独立 HTTP 页面，使用明确 UI fixture 响应。
不是 ROS/Gazebo、正式 session 或比赛验收。没有发送控制请求。

- `stale-desktop.png`：1920×1080，`status=stale` 且最后验收 true，显示当前未知。
- `disconnected-mobile.png`：390×844 全页，HTTP 503 后卡片警示、当前值撤销、
  图片隐藏、移动端标题正常。截图中的 `UI fixture` 标明测试来源。
- 哈希、字节数见 `screenshot_hashes.json`。

另以真实浏览器验证失联后重新收到响应可恢复；503/首次无 API 的 404 是
离线注入的预期资源错误，无 JS 运行异常。程序和原始截图保留在
`output/playwright/`。真实生产遥测与部署尚未验证。
