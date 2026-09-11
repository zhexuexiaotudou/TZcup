"use strict";

// Browser-independent checks for the shipped inline dashboard script.
// Run: node --test starter_ws/src/sanitation_hmi/test/test_demo_disconnect.cjs
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");
const vm = require("node:vm");

const html = fs.readFileSync(path.join(__dirname, "../web/demo.html"), "utf8");
const inlineScript = html.match(/<script>([\s\S]*?)<\/script>/)?.[1];
assert.ok(inlineScript, "demo.html must retain an executable inline script");
assert.match(html, /#front-camera-image\[hidden\]\s*\{\s*display:\s*none;/, "hidden camera images need an explicit display rule");

function element() {
  return {
    className: "", dataset: {}, hidden: false, innerHTML: "", style: {}, textContent: "",
    addEventListener() {},
    classList: { remove() {}, toggle() {} },
    getBoundingClientRect() { return { width: 800, height: 500 }; },
    removeAttribute(name) { this[name] = undefined; },
  };
}

function liveTelemetry() {
  const liveInput = (topic) => ({ status: "live", topic, age_sec: 0 });
  return {
    status: "RUNNING", elapsed_sec: 12,
    progress: { ratio: 0.6, active_component_number: 1, expected_components: 4, current_component: "A" },
    vehicle: {
      estimated_pose_map: [1, 2, 0], commanded_linear_speed_m_s: 0.4,
      measured_linear_speed_m_s: 0.3, command_source: "/cmd_vel",
    },
    cleaning: { brush_enabled: true, emergency_stop: false },
    final_demo: { status: "live", stage: "COVERAGE", formal_product_acceptance: true, age_sec: 0 },
    live_inputs: {
      commanded_speed: liveInput("/cmd_vel"), measured_speed: liveInput("/odom"),
      brush: liveInput("/brush"), emergency_stop: liveInput("/emergency_stop"),
      front_camera: { ...liveInput("/front_camera"), width: 640, height: 480 },
      perception_targets: { ...liveInput("/targets"), count: 1 },
      perception_diagnostics: { ...liveInput("/diagnostics"), statuses: [] },
      mapping_lifecycle: liveInput("/mapping/lifecycle"), mapping_map_ready: liveInput("/mapping/ready"),
      mapping_explorer: liveInput("/mapping/explorer"), saved_map_coverage: liveInput("/coverage"),
      safety_status: liveInput("/safety"), drivetrain_status: liveInput("/drivetrain"),
      cleaning_motor_status: liveInput("/cleaning_motor"), map: liveInput("/map"), map_pose: liveInput("/map_pose"),
    },
    visualization: {
      geometry: { outer_polygon: [[0, 0], [3, 0], [3, 3]] },
      evaluation_only_trajectory: [[0, 0], [1, 1]],
      planned_path: [[1, 1], [2, 2]],
    },
    events: [{ elapsed_sec: 1, label: "LIVE" }],
  };
}

function response(data) {
  return { ok: true, json: async () => data };
}

async function dashboard(firstResponse = response(liveTelemetry())) {
  const nodes = new Map([...html.matchAll(/id="([^"]+)"/g)].map((match) => [match[1], element()]));
  const strokes = [];
  const context2d = {
    beginPath() {}, clearRect() {}, closePath() {}, fill() {}, fillRect() {}, lineTo() {}, moveTo() {},
    restore() {}, rotate() {}, save() {}, setLineDash() {}, setTransform() {}, translate() {},
    stroke() { strokes.push(this.strokeStyle); },
  };
  nodes.get("map").getContext = () => context2d;
  let nextResponse = firstResponse;
  const sandbox = vm.createContext({
    ResizeObserver: class { observe() {} },
    document: {
      querySelector(selector) {
        assert.match(selector, /^#[\w-]+$/, `Unsupported selector: ${selector}`);
        const node = nodes.get(selector.slice(1));
        assert.ok(node, `Missing DOM node: ${selector}`);
        return node;
      },
    },
    fetch: async () => {
      if (nextResponse instanceof Error) throw nextResponse;
      return nextResponse;
    },
    setInterval() { return 1; },
    window: { addEventListener() {}, devicePixelRatio: 1 },
  });
  vm.runInContext(inlineScript, sandbox, { filename: "demo.html:inline-script" });
  await new Promise(setImmediate);
  return {
    node: (id) => nodes.get(id), strokes,
    refresh: () => vm.runInContext("refresh()", sandbox),
    reply(data) { nextResponse = response(data); },
    networkFailure() { nextResponse = new Error("connection lost"); },
    httpFailure() { nextResponse = { ok: false, statusText: "Service Unavailable" }; },
  };
}

for (const [name, disconnect] of [["network", "networkFailure"], ["HTTP", "httpFailure"]]) {
  test(`${name} failure clears live telemetry and recovers only from fresh data`, async () => {
    const ui = await dashboard();
    assert.equal(ui.node("final-demo-acceptance").textContent, "通过");
    assert.equal(ui.node("front-camera-image").hidden, false);
    assert.equal(ui.node("brush").textContent, "开启");
    assert.equal(ui.node("safety").textContent, "正常");
    assert.equal(ui.node("progress-value").textContent, "60%");
    assert.ok(ui.strokes.includes("#536d65"), "live trajectory is initially rendered");

    ui.strokes.length = 0;
    ui[disconnect]();
    await assert.doesNotReject(() => ui.refresh());

    assert.equal(ui.node("final-demo-acceptance").textContent, "过期 · 当前验收未知");
    assert.equal(ui.node("final-demo-acceptance").className, "warn");
    assert.match(ui.node("final-demo-stage").textContent, /^最后观测：COVERAGE$/);
    assert.equal(ui.node("commanded-speed").textContent, "过期");
    assert.equal(ui.node("commanded-speed").className, "warn");
    assert.equal(ui.node("brush").textContent, "过期");
    assert.equal(ui.node("brush").className, "warn");
    assert.equal(ui.node("safety").textContent, "过期");
    assert.equal(ui.node("safety").className, "warn");
    assert.equal(ui.node("progress-value").textContent, "未知");
    assert.equal(ui.node("state").className, "state warn");
    assert.equal(ui.node("front-camera-image").hidden, true);
    assert.equal(ui.node("front-camera-image").src, undefined);
    assert.ok(!ui.strokes.some((color) => ["#536d65", "#69d7ff"].includes(color)), "old trajectories are not repainted");

    ui.reply(liveTelemetry());
    await ui.refresh();
    assert.equal(ui.node("final-demo-acceptance").textContent, "通过");
    assert.equal(ui.node("final-demo-acceptance").className, "ok");
    assert.equal(ui.node("commanded-speed").className, "ok");
    assert.equal(ui.node("brush").textContent, "开启");
    assert.equal(ui.node("safety").textContent, "正常");
    assert.equal(ui.node("progress-value").textContent, "60%");
    assert.equal(ui.node("state").className, "state");
    assert.equal(ui.node("front-camera-image").hidden, false);
  });
}

test("stale acceptance=true never renders as a passing current acceptance", async () => {
  const stale = liveTelemetry();
  stale.final_demo = { ...stale.final_demo, status: "stale", formal_product_acceptance: true };
  const ui = await dashboard(response(stale));
  assert.equal(ui.node("final-demo-acceptance").textContent, "过期 · 当前验收未知");
  assert.equal(ui.node("final-demo-acceptance").className, "warn");
});

for (const [status, color] of [["FAILED", "danger"], ["CANCELED", "warn"]]) {
  test(`${status} task state uses its terminal-status color`, async () => {
    const terminal = liveTelemetry();
    terminal.status = status;
    const ui = await dashboard(response(terminal));
    assert.equal(ui.node("state").textContent, status);
    assert.equal(ui.node("state").className, `state ${color}`);
  });
}

test("the first telemetry request may fail without throwing", async () => {
  const ui = await dashboard(new Error("initial connection lost"));
  assert.equal(ui.node("final-demo-acceptance").textContent, "过期 · 当前验收未知");
  assert.equal(ui.node("final-demo-acceptance").className, "warn");
  assert.equal(ui.node("state").textContent, "连接失败 · 当前任务状态未知");
  assert.equal(ui.node("state").className, "state warn");
  assert.equal(ui.node("progress-value").textContent, "未知");
  assert.equal(ui.node("front-camera-image").hidden, true);
  assert.equal(ui.node("commanded-speed").textContent, "不可用");
  assert.equal(ui.node("brush").textContent, "不可用");
  assert.equal(ui.node("safety").textContent, "不可用");

  ui.reply(liveTelemetry());
  await assert.doesNotReject(() => ui.refresh());
  assert.equal(ui.node("final-demo-stage").textContent, "COVERAGE");
  assert.equal(ui.node("final-demo-acceptance").textContent, "通过");
  assert.equal(ui.node("progress-value").textContent, "60%");
  assert.equal(ui.node("state").className, "state");
  assert.equal(ui.node("front-camera-image").hidden, false);
});
