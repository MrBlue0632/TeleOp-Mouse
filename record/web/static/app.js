import { createRobotView } from "./robot_view.js";

const state = {
  telemetry: null,
  camera: "wrist",
  configFile: "global",
  robotModel: null,
  robotView: null,
};

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => Number.isFinite(Number(v)) ? Number(v).toFixed(d) : "--";

function setText(id, text) {
  const el = $(id);
  if (el) el.textContent = text;
}

function renderKV(el, entries) {
  el.innerHTML = entries.map(([k, v]) => `<dt>${k}</dt><dd>${v ?? "--"}</dd>`).join("");
}

function renderGrid(el, labels, values, digits = 1) {
  el.innerHTML = labels.map((label, i) => `
    <div class="joint-cell"><span>${label}</span><strong>${fmt(values?.[i], digits)}</strong></div>
  `).join("");
}

function renderTorque(snapshot) {
  const torque = snapshot.torque || {};
  const api = torque.api || [];
  const ext = torque.external || [];
  const bars = Array.isArray(torque.external_bars) && torque.external_bars.length === 6
    ? torque.external_bars
    : ext.map((v) => ({ value: Math.abs(v), ratio: Math.min(Math.abs(v) / 10, 1), color: "#d7a735" }));
  $("torqueBars").innerHTML = ["J1", "J2", "J3", "J4", "J5", "J6"].map((joint, i) => {
    const bar = bars[i] || { value: 0, ratio: 0, color: "#d7a735" };
    return `<div class="torque-row">
      <strong>${joint}</strong>
      <span class="api">API ${fmt(api[i], 2)}</span>
      <div class="track"><div class="fill" style="width:${Math.round((bar.ratio || 0) * 100)}%; background:${bar.color || "#d7a735"}"></div></div>
      <span class="ext">${fmt(bar.value, 2)} Nm</span>
    </div>`;
  }).join("");
  setText("torqueMode", torque.external_mode || "unknown");
  setText("torqueNote", torque.external_note || "");
}

function renderRobot(snapshot) {
  if (!state.robotView) return;
  state.robotView.update(snapshot);
}

function render(snapshot) {
  state.telemetry = snapshot;
  setText("connStatus", "LIVE");
  $("connStatus").classList.remove("warn");
  setText("robotState", `state ${snapshot.robot?.state ?? "--"} / err ${snapshot.robot?.error ?? "--"}`);
  setText("episodeChip", `ep ${snapshot.dataset?.num_episodes ?? 0} / frame ${snapshot.dataset?.total_frames ?? 0}`);
  setText("coordFrame", snapshot.robot?.coord_frame || "BASE");
  $("resetBtn").disabled = !snapshot.robot?.reset_enabled;

  renderKV($("datasetGrid"), [
    ["Repo", snapshot.dataset?.repo_id],
    ["Task", snapshot.dataset?.task],
    ["Episodes", snapshot.dataset?.num_episodes],
    ["Frames", snapshot.dataset?.total_frames],
    ["Buffer", snapshot.dataset?.episode_buffer_size],
    ["FPS", snapshot.dataset?.rate_hz],
    ["Root", snapshot.dataset?.root],
  ]);
  renderGrid($("stateGrid"), ["J1", "J2", "J3", "J4", "J5", "J6"], snapshot.state?.joints_deg || [], 1);
  renderGrid($("actionGrid"), ["vx", "vy", "vz", "vrx", "vry", "vrz"], snapshot.action?.velocity_cmd || [], 1);
  const pose = snapshot.state?.pose_xyzrpy_deg || [];
  setText("poseSummary", `xyz ${fmt(pose[0], 0)}, ${fmt(pose[1], 0)}, ${fmt(pose[2], 0)} | rpy ${fmt(pose[3], 1)}, ${fmt(pose[4], 1)}, ${fmt(pose[5], 1)}`);

  const cam = snapshot.camera?.[state.camera] || {};
  setText("cameraOverlay", `${state.camera.toUpperCase()} ${cam.status || "unknown"} age ${fmt(cam.age_ms, 0)} ms`);
  setText("cameraStats", ["wrist", "base", "target"].map((name) => `${name}: ${snapshot.camera?.[name]?.status || "--"}`).join("   "));
  renderTorque(snapshot);
  renderRobot(snapshot);
}

function connectTelemetry() {
  const source = new EventSource("/api/telemetry");
  source.addEventListener("telemetry", (event) => {
    try { render(JSON.parse(event.data)); } catch (err) { console.error(err); }
  });
  source.onerror = () => {
    setText("connStatus", "DISCONNECTED");
    $("connStatus").classList.add("warn");
  };
}

async function loadConfig(file) {
  state.configFile = file;
  const res = await fetch(`/api/config?file=${encodeURIComponent(file)}`);
  const data = await res.json();
  if (data.ok) {
    $("configText").value = data.text;
    setText("configStatus", `${file} config loaded. 保存后相关连接/URDF/payload 配置下次启动生效。`);
  } else {
    setText("configStatus", data.error || "load failed");
  }
}

async function validateConfig() {
  const res = await fetch("/api/config/validate", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ text: $("configText").value }) });
  const data = await res.json();
  setText("configStatus", data.ok ? "YAML 校验通过。" : `YAML 错误: ${data.error}`);
  return data.ok;
}

async function saveConfig() {
  const ok = await validateConfig();
  if (!ok) return;
  if (!confirm(`保存 ${state.configFile} 配置？会生成 .bak 备份，部分配置下次启动生效。`)) return;
  const res = await fetch(`/api/config?file=${encodeURIComponent(state.configFile)}`, { method: "PUT", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ text: $("configText").value }) });
  const data = await res.json();
  setText("configStatus", data.ok ? `已保存，备份: ${data.backup || "none"}` : `保存失败: ${data.error}`);
}

async function resetHome() {
  if (!confirm("确认让机械臂回到 Home？请确保工作空间安全。")) return;
  const res = await fetch("/api/reset-home", { method: "POST" });
  const data = await res.json();
  setText("connStatus", data.ok ? "RESET REQUESTED" : `RESET BLOCKED: ${data.error}`);
  $("connStatus").classList.add("warn");
}

async function loadRobotModel() {
  try {
    const res = await fetch("/api/robot-model");
    const data = await res.json();
    state.robotModel = data;
    if (state.robotView) {
      state.robotView.setRobotModel(data);
    }
    if (!data.ok) {
      setText("urdfStatus", data.error || "URDF unavailable");
      return;
    }
    const label = data.render_mode === "mesh"
      ? "URDF mesh model"
      : data.render_mode === "mesh_missing"
        ? "URDF mesh refs missing"
        : "URDF primitive model";
    setText("urdfStatus", `${label}: ${data.movable_joint_count}/${data.joint_count} movable joints, ${data.geometry_types?.join("+") || "geometry"}`);
    if (state.telemetry) renderRobot(state.telemetry);
  } catch (err) {
    setText("urdfStatus", "URDF metadata unavailable");
  }
}

function initRobotView() {
  const canvas = $("robotCanvas");
  if (!canvas) return;
  try {
    state.robotView = createRobotView(canvas, { forceHud: $("forceHud") });
  } catch (err) {
    setText("urdfStatus", `3D renderer unavailable: ${err.message || err}`);
  }
}

function setupUI() {
  document.querySelectorAll("[data-camera]").forEach((btn) => {
    btn.addEventListener("click", () => {
      state.camera = btn.dataset.camera;
      document.querySelectorAll("[data-camera]").forEach((b) => b.classList.toggle("active", b === btn));
      $("cameraFeed").src = `/stream/${state.camera}.mjpg?t=${Date.now()}`;
      if (state.telemetry) render(state.telemetry);
    });
  });
  document.querySelectorAll("[data-config]").forEach((btn) => btn.addEventListener("click", () => loadConfig(btn.dataset.config)));
  $("validateConfig").addEventListener("click", validateConfig);
  $("saveConfig").addEventListener("click", saveConfig);
  $("resetBtn").addEventListener("click", resetHome);
  window.addEventListener("resize", () => {
    if (state.robotView) state.robotView.resize();
  });
}

setupUI();
initRobotView();
loadRobotModel();
connectTelemetry();
loadConfig("global");
