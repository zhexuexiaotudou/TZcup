"use strict";

const consoleRoot = document.getElementById("console");
const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");
const stage = document.getElementById("map-stage");
const layers = {
  operation: true,
  keepout: true,
  truth: true,
  perception: true,
  obstacles: true,
  planned: true,
  local: false,
  trajectory: true,
  vehicle: true,
  slam: true,
};
const layerNames = {
  operation: "作业区域", keepout: "禁行与排除区", truth: "仿真参考真值",
  perception: "感知预测", obstacles: "障碍物", planned: "全局规划",
  local: "局部规划", trajectory: "实际轨迹", vehicle: "车辆姿态", slam: "SLAM 占据栅格",
};
let snapshot = null;
let mapView = "operation";
let replay = null;
let replayIndex = 0;
let replayTimer = null;
let imageRevision = 0;
let dragging = null;
const view = { cx: 0, cy: 0, scale: 22, fitted: false };

const $ = (id) => document.getElementById(id);
const hasNumber = (value) => typeof value === "number" && Number.isFinite(value);
const percent = (value) => hasNumber(value) ? `${(value * 100).toFixed(1)}%` : "--";
const safeText = (value, fallback = "数据不可用") => value === null || value === undefined || value === "" ? fallback : String(value);
const statusClass = (value) => `status-${value === "ready" || value === "live" ? "ready" : value === "degraded" || value === "stale" ? "degraded" : "offline"}`;

function setText(id, value, fallback) {
  const node = $(id);
  if (node) node.textContent = safeText(value, fallback);
}

function setStatus(id, label, status) {
  const node = $(id);
  node.textContent = label;
  node.className = statusClass(status);
}

function sourceLabel(source) {
  const map = { live: "实时", stale: "过期", error: "错误", unavailable: "不可用" };
  return map[source?.status] || "不可用";
}

function systemLabel(status) {
  return ({ ready: "就绪", degraded: "降级", offline: "离线" })[status] || "未知";
}

function safetyLabel(safety) {
  return ({ ready: "安全就绪", emergency_stopped: "急停中", unknown: "接口就绪·状态未知" })[safety?.status] || "未知";
}

function refreshState(data) {
  snapshot = data;
  const sources = data.sources || {};
  setText("scene-name", data.scene?.name, "场景未配置");
  setText("source-mode", data.mode === "live" ? "实时模式" : "历史回放");
  setStatus("system-status", systemLabel(data.system_status), data.system_status);
  setStatus("localization-status", sourceLabel(sources.slam_map), sources.slam_map?.status);
  setStatus("safety-status", safetyLabel(data.safety), data.safety?.status === "ready" ? "ready" : data.safety?.status === "emergency_stopped" ? "offline" : "degraded");
  setText("mission-state", data.mission?.coverage_state);
  setText("mission-name", data.reference?.mission?.id ? `任务 ${data.reference.mission.id}` : "等待任务配置");
  setText("mission-boundary", data.capabilities?.task_dispatch ? "安全任务编排器已连接。" : "任务编排器尚未接入，任务按钮保持失败关闭；监测与急停能力分开显示。");
  const metrics = data.mission?.coverage_metrics || {};
  setText("actual-coverage", percent(metrics.actual_ratio));
  setText("planned-coverage", percent(metrics.planned_ratio));
  setText(
    "target-count",
    data.sources?.perception?.status === "live"
      ? (data.targets?.predictions?.length ?? 0)
      : "--",
  );
  setText("current-action", currentAction(data));
  setText("next-action", data.mission?.next_action);
  setText("route-reason", routeReason(data));
  setText("map-difference", mapDifference(data));
  setText("truth-boundary", `${data.scene?.truth_boundary || "参考真值只用于显示和评测"}。当前界面不代表真实车辆或 J6 验证通过。`);
  refreshSources(sources);
  refreshEvents(data.events || []);
  refreshCapabilities(data.capabilities || {});
  refreshImages(sources);
  draw();
}

function currentAction(data) {
  if (data.safety?.emergency_stop === true) return "紧急停止，车辆控制输出被抑制";
  const coverage = String(data.mission?.coverage_state || "").toUpperCase();
  if (coverage.includes("RUN") || coverage.includes("FOLLOW")) return "沿规划路径执行覆盖清扫";
  if (coverage.includes("PAUSE")) return "覆盖任务已暂停";
  if (data.system_status === "offline") return "等待 ROS 数据源";
  if (data.system_status === "degraded") return "监测数据降级，等待源恢复";
  return "安全监测已连接，等待任务编排器";
}

function routeReason(data) {
  if ((data.planned_path || []).length) return "显示 /plan 的全局规划结果；实际轨迹单独绘制，不把计划当作执行。";
  return "尚未收到全局规划路径。";
}

function mapDifference(data) {
  if (!data.slam_map) return "未收到 /map，无法与参考配置对比。";
  const known = data.slam_map.data.filter((v) => v >= 0).length;
  const total = data.slam_map.data.length || 1;
  return `SLAM 栅格已接入，已知单元占 ${(known / total * 100).toFixed(1)}%；参考配置与 SLAM 仍保持来源分离。`;
}

function refreshSources(sources) {
  const list = $("source-health-list");
  list.replaceChildren();
  Object.entries(sources).forEach(([name, source]) => {
    const row = document.createElement("div");
    row.className = "health-row";
    const label = document.createElement("span");
    label.textContent = name;
    const status = document.createElement("b");
    status.className = source.status;
    status.textContent = source.age_s === null ? sourceLabel(source) : `${sourceLabel(source)} ${source.age_s.toFixed(1)}s`;
    row.append(label, status);
    list.append(row);
  });
}

function refreshEvents(events) {
  const list = $("event-list");
  list.replaceChildren();
  if (!events.length) {
    const empty = document.createElement("li");
    empty.innerHTML = "<div></div><div><strong>暂无运行事件</strong><p>数据源连接后将在此显示。</p></div>";
    list.append(empty);
    return;
  }
  events.slice(0, 20).forEach((event) => {
    const row = document.createElement("li");
    row.className = event.severity || "info";
    const time = document.createElement("time");
    time.textContent = new Date(event.at * 1000).toLocaleTimeString("zh-CN", { hour12: false });
    const body = document.createElement("div");
    const title = document.createElement("strong");
    const detail = document.createElement("p");
    title.textContent = event.title;
    detail.textContent = event.detail;
    body.append(title, detail);
    row.append(time, body);
    list.append(row);
  });
}

function refreshCapabilities(capabilities) {
  document.querySelectorAll("[data-needs-dispatch]").forEach((button) => {
    button.disabled = !capabilities.task_dispatch;
    button.title = capabilities.task_dispatch ? "" : "安全任务编排器尚未接入";
  });
  document.querySelectorAll(".estop,.release-estop").forEach((button) => {
    button.disabled = !capabilities.emergency_stop;
    button.title = capabilities.emergency_stop ? "" : "急停 ROS 接口当前不可用";
  });
  $("replay-button").disabled = !capabilities.replay;
}

function refreshImages(sources) {
  imageRevision += 1;
  refreshImage("gazebo_overview", sources.gazebo_overview, "overview-image", "overview-empty", "overview-source");
  refreshImage("camera", sources.camera, "camera-image", "camera-empty", "camera-source");
}

function refreshImage(name, source, imageId, emptyId, labelId) {
  const image = $(imageId);
  const empty = $(emptyId);
  setText(labelId, sourceLabel(source));
  if (!source || !["live", "stale"].includes(source.status)) {
    image.style.display = "none";
    empty.style.display = "block";
    return;
  }
  image.onload = () => { image.style.display = "block"; empty.style.display = "none"; };
  image.onerror = () => { image.style.display = "none"; empty.style.display = "block"; };
  image.src = `/api/v1/images/${name}?v=${imageRevision}`;
}

function worldToScreen(x, y, clip = null) {
  const width = clip?.width || canvas.clientWidth;
  const height = canvas.clientHeight;
  const offset = clip?.offset || 0;
  return [offset + width / 2 + (x - view.cx) * view.scale, height / 2 - (y - view.cy) * view.scale];
}

function allWorldPoints(data, includeAll = false) {
  const points = [];
  const add = (p) => Array.isArray(p) && hasNumber(+p[0]) && hasNumber(+p[1]) && points.push([+p[0], +p[1]]);
  (data?.reference?.mission?.outer_polygon || []).forEach(add);
  (data?.reference?.mission?.keepout_polygons || []).flat().forEach(add);
  (data?.planned_path || []).forEach(add);
  (data?.trajectory || []).forEach(add);
  if (hasNumber(data?.vehicle?.x)) add([data.vehicle.x, data.vehicle.y]);
  if (includeAll) {
    (data?.reference?.truth_targets || []).forEach((item) => add(item.position));
    (data?.reference?.obstacles || []).forEach((item) => add(item.position));
  }
  if (data?.slam_map && (includeAll || mapView === "slam" || mapView === "compare")) {
    const map = data.slam_map;
    add(map.origin);
    add([map.origin[0] + map.width * map.resolution, map.origin[1] + map.height * map.resolution]);
  }
  return points;
}

function fitMap(includeAll = false) {
  const points = allWorldPoints(snapshot, includeAll);
  if (!points.length) { view.cx = 0; view.cy = 0; view.scale = 22; draw(); return; }
  const xs = points.map((p) => p[0]);
  const ys = points.map((p) => p[1]);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  view.cx = (minX + maxX) / 2;
  view.cy = (minY + maxY) / 2;
  view.scale = Math.max(4, Math.min((canvas.clientWidth - 80) / Math.max(4, maxX - minX), (canvas.clientHeight - 70) / Math.max(4, maxY - minY)));
  view.fitted = true;
  draw();
}

function drawGrid() {
  const width = canvas.clientWidth, height = canvas.clientHeight;
  ctx.fillStyle = "#0a0f13";
  ctx.fillRect(0, 0, width, height);
  const stepMeters = view.scale > 45 ? 1 : view.scale > 18 ? 2 : 5;
  const step = stepMeters * view.scale;
  const origin = worldToScreen(0, 0);
  ctx.strokeStyle = "#182228";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = origin[0] % step; x < width; x += step) { ctx.moveTo(x, 0); ctx.lineTo(x, height); }
  for (let y = origin[1] % step; y < height; y += step) { ctx.moveTo(0, y); ctx.lineTo(width, y); }
  ctx.stroke();
  $("scale-label").textContent = `${stepMeters} m`;
  document.querySelector(".map-scale span").style.width = `${step}px`;
}

function polygon(points, style, clip) {
  if (!points || points.length < 2) return;
  ctx.beginPath();
  points.forEach((point, index) => {
    const [x, y] = worldToScreen(+point[0], +point[1], clip);
    if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.closePath();
  if (style.fill) { ctx.fillStyle = style.fill; ctx.fill(); }
  if (style.stroke) { ctx.strokeStyle = style.stroke; ctx.lineWidth = style.width || 1; ctx.setLineDash(style.dash || []); ctx.stroke(); ctx.setLineDash([]); }
}

function path(points, color, width, dash, clip) {
  if (!points || points.length < 2) return;
  ctx.beginPath();
  points.forEach((point, index) => {
    const [x, y] = worldToScreen(+point[0], +point[1], clip);
    if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []); ctx.stroke(); ctx.setLineDash([]);
}

function drawSlam(map, clip) {
  if (!map || !layers.slam) return;
  const stride = Math.max(1, Math.floor(Math.max(map.width, map.height) / 220));
  const cell = Math.max(1, map.resolution * view.scale * stride);
  for (let y = 0; y < map.height; y += stride) {
    for (let x = 0; x < map.width; x += stride) {
      const value = map.data[y * map.width + x];
      if (value < 0) continue;
      ctx.fillStyle = value > 65 ? "#8999a1b8" : "#24333aad";
      const [sx, sy] = worldToScreen(map.origin[0] + x * map.resolution, map.origin[1] + y * map.resolution, clip);
      ctx.fillRect(sx, sy - cell, cell + .5, cell + .5);
    }
  }
}

function drawReference(data, clip) {
  const mission = data.reference?.mission || {};
  if (layers.operation) polygon(mission.outer_polygon, { fill: "#18303695", stroke: "#45a8b7", width: 1.4 }, clip);
  if (layers.keepout) {
    (mission.keepout_polygons || []).forEach((area) => polygon(area, { fill: "#6d263077", stroke: "#ef6570", width: 1.4, dash: [5, 4] }, clip));
    (mission.exclusion_polygons || []).forEach((area) => polygon(area, { fill: "#54263066", stroke: "#c95a66", width: 1, dash: [3, 3] }, clip));
  }
  if (layers.truth) (data.reference?.truth_targets || []).forEach((target) => {
    const [x, y] = worldToScreen(+target.position[0], +target.position[1], clip);
    const size = Math.max(5, (+target.size?.[0] || .2) * view.scale);
    ctx.fillStyle = "#d7a63e"; ctx.fillRect(x - size / 2, y - size / 2, size, size);
  });
  if (layers.obstacles) (data.reference?.obstacles || []).forEach((obstacle) => {
    const [x, y] = worldToScreen(+obstacle.position[0], +obstacle.position[1], clip);
    ctx.strokeStyle = obstacle.dynamic ? "#ff8f55" : "#9aa7ae"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, Math.max(5, +obstacle.radius_m * view.scale), 0, Math.PI * 2); ctx.stroke();
  });
}

function pointInPolygon(point, polygonPoints) {
  let inside = false;
  for (let i = 0, j = polygonPoints.length - 1; i < polygonPoints.length; j = i++) {
    const xi = +polygonPoints[i][0], yi = +polygonPoints[i][1];
    const xj = +polygonPoints[j][0], yj = +polygonPoints[j][1];
    if (((yi > point[1]) !== (yj > point[1])) && point[0] < (xj - xi) * (point[1] - yi) / ((yj - yi) || 1e-9) + xi) inside = !inside;
  }
  return inside;
}

function drawCoverage(data, clip) {
  const area = data.reference?.mission?.outer_polygon || [];
  const raw = replay
    ? replay.samples.slice(0, replayIndex + 1).filter((row) => row.brush).map((row) => [row.x, row.y])
    : (data.trajectory || []).filter((row) => row[4]).map((row) => [row[0], row[1]]);
  if (area.length < 3 || raw.length < 1 || !layers.operation) return;
  const xs = area.map((p) => +p[0]), ys = area.map((p) => +p[1]);
  const step = Math.max(.22, Math.min(.55, 7 / view.scale));
  const radius = Math.max(.15, +(data.reference?.mission?.operation_width_m || .65) / 2);
  const buckets = new Map();
  let previousKey = null;
  raw.forEach((point) => {
    const key = `${Math.floor(point[0] / step)},${Math.floor(point[1] / step)}`;
    if (key !== previousKey) buckets.set(key, (buckets.get(key) || 0) + 1);
    previousKey = key;
  });
  const reach = Math.ceil(radius / step);
  for (let y = Math.min(...ys); y <= Math.max(...ys); y += step) {
    for (let x = Math.min(...xs); x <= Math.max(...xs); x += step) {
      if (!pointInPolygon([x, y], area)) continue;
      let visits = 0;
      const cellX = Math.floor(x / step), cellY = Math.floor(y / step);
      for (let dx = -reach; dx <= reach; dx += 1) {
        for (let dy = -reach; dy <= reach; dy += 1) {
          if (Math.hypot(dx * step, dy * step) <= radius + step) {
            visits += buckets.get(`${cellX + dx},${cellY + dy}`) || 0;
          }
        }
      }
      const [sx, sy] = worldToScreen(x, y, clip);
      ctx.fillStyle = visits === 0 ? "#8e343a70" : visits === 1 ? "#2ca86c91" : "#d9853799";
      ctx.fillRect(sx - step * view.scale / 2, sy - step * view.scale / 2, step * view.scale + .5, step * view.scale + .5);
    }
  }
}

function drawDynamic(data, clip) {
  drawCoverage(data, clip);
  if (layers.perception) (data.targets?.predictions || []).forEach((target) => {
    const position = target.position || target.centroid || [target.x, target.y];
    if (!hasNumber(+position?.[0]) || !hasNumber(+position?.[1])) return;
    const [x, y] = worldToScreen(+position[0], +position[1], clip);
    ctx.strokeStyle = "#4dd7ea"; ctx.lineWidth = 2; ctx.setLineDash([4, 3]);
    ctx.beginPath(); ctx.arc(x, y, 8, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
  });
  if (layers.planned) path(data.planned_path, "#6499f5", 2, [7, 5], clip);
  if (layers.local) path(data.local_path, "#b785f6", 1.5, [3, 3], clip);
  const trajectory = replay ? replay.samples.slice(0, replayIndex + 1).map((row) => [row.x, row.y]) : data.trajectory;
  if (layers.trajectory) path(trajectory, "#53d5de", 2.4, [], clip);
  const vehicle = replay ? replay.samples[replayIndex] : data.vehicle;
  if (layers.vehicle && hasNumber(vehicle?.x) && hasNumber(vehicle?.y)) drawVehicle(vehicle, clip);
}

function drawVehicle(vehicle, clip) {
  const [x, y] = worldToScreen(+vehicle.x, +vehicle.y, clip);
  const yaw = -(+vehicle.yaw || 0);
  ctx.save(); ctx.translate(x, y); ctx.rotate(yaw);
  ctx.fillStyle = "#e7f2f3"; ctx.strokeStyle = "#51d5df"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(13, 0); ctx.lineTo(-9, -8); ctx.lineTo(-5, 0); ctx.lineTo(-9, 8); ctx.closePath(); ctx.fill(); ctx.stroke();
  ctx.restore();
}

function drawHalf(data, kind, clip) {
  ctx.save(); ctx.beginPath(); ctx.rect(clip.offset, 0, clip.width, canvas.clientHeight); ctx.clip();
  if (kind === "slam") drawSlam(data.slam_map, clip); else drawReference(data, clip);
  drawDynamic(data, clip); ctx.restore();
}

function draw() {
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, canvas.clientWidth), height = Math.max(1, canvas.clientHeight);
  if (canvas.width !== Math.floor(width * dpr) || canvas.height !== Math.floor(height * dpr)) {
    canvas.width = Math.floor(width * dpr); canvas.height = Math.floor(height * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  drawGrid();
  if (!snapshot) { showMapMessage("等待监督数据"); return; }
  if (!view.fitted) { fitMap(mapView !== "operation"); return; }
  if (mapView === "compare") {
    const half = width / 2;
    drawHalf(snapshot, "reference", { offset: 0, width: half });
    drawHalf(snapshot, "slam", { offset: half, width: half });
    ctx.strokeStyle = "#788a93"; ctx.beginPath(); ctx.moveTo(half, 0); ctx.lineTo(half, height); ctx.stroke();
    ctx.fillStyle = "#a9bbc4"; ctx.font = "10px sans-serif"; ctx.fillText("参考配置", 12, 19); ctx.fillText("SLAM 实测", half + 12, 19);
  } else {
    if (mapView === "slam") drawSlam(snapshot.slam_map);
    else drawReference(snapshot);
    if (mapView === "operation" || mapView === "slam") drawDynamic(snapshot);
  }
  if (mapView === "slam" && !snapshot.slam_map) showMapMessage("SLAM 地图不可用，未以参考地图替代");
  else if (replay) showMapMessage("历史真实记录回放，不代表当前车辆状态");
  else showMapMessage("");
}

function showMapMessage(message) {
  const node = $("map-message");
  node.hidden = !message;
  node.textContent = message;
}

async function poll() {
  try {
    const response = await fetch("/api/v1/state", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    refreshState(await response.json());
  } catch (error) {
    setStatus("system-status", "连接失败", "offline");
    showMapMessage(`监督服务不可用：${error.message}`);
  }
}

async function sendCommand(command) {
  const token = $("operator-token").value.trim();
  const result = $("command-result");
  if (!token) { showCommandResult("需要本地操作员令牌", true); return; }
  try {
    const response = await fetch("/api/v1/commands", {
      method: "POST",
      headers: {
        "Authorization": `Bearer ${token}`,
        "Idempotency-Key": globalThis.crypto?.randomUUID?.() || `${Date.now()}-${Math.random()}`,
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ command }),
    });
    const data = await response.json();
    showCommandResult(response.ok ? `已受理：${data.dsl?.intent || command}` : `未执行：${data.reason || data.status}`, !response.ok);
    poll();
  } catch (error) { showCommandResult(`请求失败：${error.message}`, true); }
}

function showCommandResult(message, error = false) {
  const result = $("command-result");
  result.textContent = message; result.className = `visible ${error ? "status-critical" : "status-ready"}`;
  window.setTimeout(() => result.classList.remove("visible"), 5000);
}

async function enterReplay() {
  try {
    const response = await fetch("/api/v1/replay", { cache: "no-store" });
    if (!response.ok) throw new Error("未装载真实历史记录");
    replay = await response.json(); replayIndex = 0; view.fitted = false;
    $("replay-range").max = Math.max(0, replay.samples.length - 1);
    $("replay-range").value = 0; $("replay-warning").textContent = replay.warning;
    $("replay-bar").hidden = false; setText("source-mode", "历史回放"); updateReplayLabel(); draw();
  } catch (error) { showCommandResult(error.message, true); }
}

function exitReplay() {
  window.clearInterval(replayTimer); replayTimer = null; replay = null; $("replay-bar").hidden = true;
  $("replay-toggle").textContent = "播放"; view.fitted = false; setText("source-mode", "实时模式"); draw();
}

function toggleReplay() {
  if (!replay) return;
  if (replayTimer) { window.clearInterval(replayTimer); replayTimer = null; $("replay-toggle").textContent = "播放"; return; }
  $("replay-toggle").textContent = "暂停";
  replayTimer = window.setInterval(() => {
    replayIndex += 1;
    if (replayIndex >= replay.samples.length) { replayIndex = replay.samples.length - 1; toggleReplay(); }
    $("replay-range").value = replayIndex; updateReplayLabel(); draw();
  }, 120);
}

function updateReplayLabel() {
  if (!replay) return;
  const first = replay.samples[0]?.t || 0, current = replay.samples[replayIndex]?.t || first;
  const seconds = Math.max(0, current - first);
  $("replay-time").textContent = `${String(Math.floor(seconds / 60)).padStart(2, "0")}:${String(Math.floor(seconds % 60)).padStart(2, "0")}`;
}

Object.entries(layerNames).forEach(([key, label]) => {
  const row = document.createElement("label");
  const checkbox = document.createElement("input");
  checkbox.type = "checkbox"; checkbox.checked = layers[key];
  checkbox.addEventListener("change", () => { layers[key] = checkbox.checked; draw(); });
  row.append(checkbox, document.createTextNode(label)); $("layer-controls").append(row);
});

document.querySelectorAll(".mode-button").forEach((button) => button.addEventListener("click", () => {
  document.querySelectorAll(".mode-button").forEach((item) => item.classList.toggle("active", item === button));
  consoleRoot.dataset.uiMode = button.dataset.uiMode;
}));
document.querySelectorAll(".map-view").forEach((button) => button.addEventListener("click", () => {
  document.querySelectorAll(".map-view").forEach((item) => item.classList.toggle("active", item === button));
  mapView = button.dataset.mapView; view.fitted = false; draw();
}));
document.querySelectorAll("[data-command]").forEach((button) => button.addEventListener("click", () => sendCommand(button.dataset.command)));
$("fit-map").addEventListener("click", () => { view.fitted = false; fitMap(true); });
$("toggle-layers").addEventListener("click", () => {
  const drawer = $("layer-drawer"); drawer.hidden = !drawer.hidden; $("toggle-layers").setAttribute("aria-expanded", String(!drawer.hidden));
});
$("replay-button").addEventListener("click", enterReplay);
$("exit-replay").addEventListener("click", exitReplay);
$("replay-toggle").addEventListener("click", toggleReplay);
$("replay-range").addEventListener("input", (event) => { replayIndex = +event.target.value; updateReplayLabel(); draw(); });
$("operator-token").addEventListener("change", (event) => sessionStorage.setItem("tzcup_operator_token", event.target.value));
$("operator-token").value = sessionStorage.getItem("tzcup_operator_token") || "";

canvas.addEventListener("pointerdown", (event) => { dragging = { x: event.clientX, y: event.clientY, cx: view.cx, cy: view.cy }; canvas.setPointerCapture(event.pointerId); });
canvas.addEventListener("pointermove", (event) => {
  const rect = canvas.getBoundingClientRect();
  const wx = view.cx + (event.clientX - rect.left - rect.width / 2) / view.scale;
  const wy = view.cy - (event.clientY - rect.top - rect.height / 2) / view.scale;
  $("map-cursor").textContent = `坐标 ${wx.toFixed(2)}, ${wy.toFixed(2)} m`;
  if (dragging) { view.cx = dragging.cx - (event.clientX - dragging.x) / view.scale; view.cy = dragging.cy + (event.clientY - dragging.y) / view.scale; view.fitted = true; draw(); }
});
canvas.addEventListener("pointerup", () => { dragging = null; });
canvas.addEventListener("pointercancel", () => { dragging = null; });
canvas.addEventListener("wheel", (event) => { event.preventDefault(); view.scale = Math.max(2, Math.min(180, view.scale * (event.deltaY > 0 ? .88 : 1.14))); view.fitted = true; draw(); }, { passive: false });

new ResizeObserver(draw).observe(stage);
window.setInterval(() => setText("current-time", new Date().toLocaleTimeString("zh-CN", { hour12: false })), 1000);
window.setInterval(poll, 1000);
poll();
