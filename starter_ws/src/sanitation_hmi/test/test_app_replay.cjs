"use strict";

// Browser-independent behavior checks; real rendering is verified separately.
// Run: node --test starter_ws/src/sanitation_hmi/test/test_app_replay.cjs
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");
const vm = require("node:vm");

const app = fs.readFileSync(path.join(__dirname, "../web/app.js"), "utf8");
const html = fs.readFileSync(path.join(__dirname, "../web/index.html"), "utf8");

function element() {
  return {
    textContent: "", className: "", style: {}, dataset: {}, value: "", children: [],
    hidden: true, disabled: false, clientWidth: 800, clientHeight: 500,
    classList: { remove() {}, toggle() {} },
    append(...nodes) { this.children.push(...nodes); },
    replaceChildren(...nodes) { this.children = nodes; },
    addEventListener() {}, setAttribute(name, value) { this[name] = value; },
    removeAttribute(name) { delete this[name]; },
  };
}

function liveState(ratio = 0.8) {
  return {
    mode: "live", system_status: "ready", scene: { name: "CURRENT LIVE SCENE" },
    sources: Object.fromEntries(["slam_map", "perception", "camera", "gazebo_overview"].map(
      (name) => [name, { status: "live", age_s: 0 }])),
    safety: { status: "ready", emergency_stop: false },
    capabilities: { task_dispatch: true, emergency_stop: true, replay: true },
    mission: { coverage_state: "RUNNING", coverage_metrics: { actual_ratio: ratio, planned_ratio: 0.9 } },
    reference: { mission: { id: "CURRENT", outer_polygon: [[0, 0], [1, 0], [1, 1]] },
      truth_targets: [{ position: [1, 1] }], obstacles: [{ position: [1, 1], radius_m: 1 }] },
    targets: { predictions: [{ position: [2, 2] }] },
    planned_path: [[0, 0], [2, 2]], local_path: [[0, 0], [1, 1]],
    trajectory: [[0, 0], [1, 1]], vehicle: { x: 1, y: 1, yaw: 0 },
    slam_map: { width: 1, height: 1, resolution: 1, origin: [0, 0], data: [100] },
    events: [{ at: 1, title: "LIVE EVENT", detail: "LIVE DETAIL" }],
  };
}

const record = {
  samples: [{ x: 10, y: 20, t: 1, yaw: 0 }, { x: 11, y: 21, t: 3, yaw: 0 }],
  success: false, warning: "历史记录", execution_boundary: "离线证据，不能证明当前执行",
};

async function browser(initial = liveState()) {
  const nodes = new Map([...html.matchAll(/id="([^"]+)"/g)].map((match) => [match[1], element()]));
  const strokes = [];
  const context2d = new Proxy({}, {
    get(target, key) {
      if (key in target) return target[key];
      return (...args) => { strokes.push({ method: key, args, color: target.strokeStyle }); };
    },
  });
  nodes.get("map-canvas").getContext = () => context2d;
  const dispatch = [element(), element()];
  const emergency = [element(), element()];
  const scale = element();
  let nextResponse = { ok: true, json: async () => initial };
  const sandbox = vm.createContext({
    document: {
      getElementById(id) { assert.ok(nodes.has(id), `Unknown DOM id ${id}`); return nodes.get(id); },
      createElement: element, createTextNode: (textContent) => ({ textContent }),
      querySelector: () => scale,
      querySelectorAll: (selector) => selector === "[data-needs-dispatch]" ? dispatch
        : selector === ".estop,.release-estop" ? emergency : [],
    },
    window: { devicePixelRatio: 1, setInterval() { return 1; }, clearInterval() {}, setTimeout() {} },
    sessionStorage: { getItem() { return null; } },
    ResizeObserver: class { observe() {} },
    fetch: async () => { if (nextResponse instanceof Error) throw nextResponse; return nextResponse; },
  });
  vm.runInContext(app, sandbox, { filename: "app.js" });
  await new Promise(setImmediate); // Let the unmodified app's initial poll settle.
  return {
    node: (id) => nodes.get(id), dispatch, emergency, strokes,
    run: (code) => vm.runInContext(code, sandbox),
    reply(data) { nextResponse = { ok: true, json: async () => data }; },
    fail(error = new Error("connection lost")) { nextResponse = error; },
    httpFailure() { nextResponse = { ok: false, status: 503 }; },
  };
}

test("replay remains historical across polls and excludes unrecorded live overlays", async () => {
  const ui = await browser();
  assert.equal(ui.node("actual-coverage").textContent, "80.0%");
  assert.match(ui.node("camera-image").src, /api\/v1\/images\/camera/);
  const queuedLiveImageLoad = ui.node("camera-image").onload;
  ui.reply(record);
  await ui.run("enterReplay()");
  queuedLiveImageLoad();
  assert.equal(ui.node("camera-image").style.display, "none", "Queued live image callback cannot revive an image during replay");
  assert.equal(ui.node("camera-empty").style.display, "block");
  ui.reply(liveState(0.99));
  await ui.run("poll()");
  assert.match(ui.node("source-mode").textContent, /历史/);
  assert.equal(ui.node("actual-coverage").textContent, "--");
  assert.equal(ui.node("planned-coverage").textContent, "--");
  assert.equal(ui.node("target-count").textContent, "--");
  assert.match(ui.node("current-action").textContent, /历史/);
  assert.match(ui.node("replay-warning").textContent, /失败/);
  assert.match(ui.node("replay-warning").textContent, /不能证明当前执行/);
  assert.match(ui.node("safety-status").textContent, /当前实时/);
  assert.ok(ui.emergency.every((button) => !button.disabled), "Live emergency controls remain available");
  for (const id of ["camera-image", "overview-image"]) {
    assert.equal(ui.node(id).style.display, "none");
    assert.equal(ui.node(id).src, undefined);
    assert.equal(ui.node(id).onload, null);
  }
  for (const mode of ["operation", "slam", "compare", "reference"]) {
    ui.strokes.length = 0;
    ui.run(`mapView = ${JSON.stringify(mode)}; replayIndex = 1; draw()`);
    assert.ok(!ui.strokes.some((call) => call.method === "arc"), "No live obstacles or predictions");
    assert.ok(!ui.strokes.some((call) => ["#6499f5", "#b785f6"].includes(call.color)), "No live plans");
    assert.ok(ui.strokes.some((call) => call.method === "stroke" && call.color === "#53d5de"), "Recorded trajectory remains drawn");
    assert.match(ui.node("map-message").textContent, /历史/);
  }
  ui.run("exitReplay()");
  assert.equal(ui.node("source-mode").textContent, "实时模式");
  assert.equal(ui.node("actual-coverage").textContent, "99.0%", "Exit restores the most recent live poll");
  assert.match(ui.node("camera-image").src, /api\/v1\/images\/camera/);
});

for (const kind of ["network", "http"]) {
  test(`${kind} failure revokes stale capabilities, metrics and image state`, async () => {
    const ui = await browser();
    assert.ok(ui.dispatch.every((button) => !button.disabled));
    const queuedLiveImageLoad = ui.node("camera-image").onload;
    if (kind === "network") ui.fail(); else ui.httpFailure();
    await ui.run("poll()");
    queuedLiveImageLoad();
    assert.equal(ui.node("camera-empty").style.display, "block", "Queued image callback cannot conceal the disconnected placeholder");
    assert.ok([...ui.dispatch, ...ui.emergency].every((button) => button.disabled));
    assert.equal(ui.node("replay-button").disabled, true);
    assert.equal(ui.node("actual-coverage").textContent, "--");
    assert.equal(ui.node("target-count").textContent, "--");
    assert.match(ui.node("mission-state").textContent, /未知/);
    assert.match(ui.node("system-status").textContent, /连接失败/);
    assert.equal(ui.node("camera-image").style.display, "none");
    assert.equal(ui.node("camera-image").src, undefined);
    ui.reply(liveState());
    await ui.run("poll()");
    assert.ok(ui.emergency.every((button) => !button.disabled));
    assert.equal(ui.node("actual-coverage").textContent, "80.0%");
  });
}

test("disconnect during replay cannot restore stale live data when exiting", async () => {
  const ui = await browser();
  ui.reply(record);
  await ui.run("enterReplay()");
  ui.fail();
  await ui.run("poll()");
  assert.match(ui.node("source-mode").textContent, /历史/);
  assert.ok(ui.emergency.every((button) => button.disabled));
  ui.run("exitReplay()");
  assert.equal(ui.node("actual-coverage").textContent, "--");
  assert.ok(ui.emergency.every((button) => button.disabled));
});

test("invalid replay samples are rejected without switching out of live mode", async () => {
  for (const samples of [[], [{ x: 0, y: 0 }], [{ x: "1", y: 0, t: 1 }]]) {
    const ui = await browser();
    ui.reply({ ...record, samples });
    await ui.run("enterReplay()");
    assert.equal(ui.node("source-mode").textContent, "实时模式");
    assert.match(ui.node("command-result").textContent, /无有效样本/);
  }
});

test("empty offline scene renders without recursive fit failure", async () => {
  const ui = await browser({ mode: "live", system_status: "offline" });
  assert.match(ui.node("current-action").textContent, /等待 ROS/);
  assert.doesNotThrow(() => ui.run("view.fitted = false; draw()"));
});
