const state = {
  telemetry: null,
  camera: 'wrist',
  configFile: 'global',
  robotModel: null,
};

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => Number.isFinite(Number(v)) ? Number(v).toFixed(d) : '--';

function setText(id, text) {
  const el = $(id);
  if (el) el.textContent = text;
}

function renderKV(el, entries) {
  el.innerHTML = entries.map(([k, v]) => `<dt>${k}</dt><dd>${v ?? '--'}</dd>`).join('');
}

function renderGrid(el, labels, values, digits = 1) {
  el.innerHTML = labels.map((label, i) => `
    <div class="joint-cell"><span>${label}</span><strong>${fmt(values?.[i], digits)}</strong></div>
  `).join('');
}

function renderTorque(snapshot) {
  const torque = snapshot.torque || {};
  const api = torque.api || [];
  const ext = torque.external || [];
  const bars = Array.isArray(torque.external_bars) && torque.external_bars.length === 6
    ? torque.external_bars
    : ext.map((v) => ({ value: Math.abs(v), ratio: Math.min(Math.abs(v) / 10, 1), color: '#d7a735' }));
  $('torqueBars').innerHTML = ['J1','J2','J3','J4','J5','J6'].map((joint, i) => {
    const bar = bars[i] || { value: 0, ratio: 0, color: '#d7a735' };
    return `<div class="torque-row">
      <strong>${joint}</strong>
      <span class="api">API ${fmt(api[i], 2)}</span>
      <div class="track"><div class="fill" style="width:${Math.round((bar.ratio || 0) * 100)}%; background:${bar.color || '#d7a735'}"></div></div>
      <span class="ext">${fmt(bar.value, 2)} Nm</span>
    </div>`;
  }).join('');
  setText('torqueMode', torque.external_mode || 'unknown');
  setText('torqueNote', torque.external_note || '');
}

function renderRobot(snapshot) {
  const canvas = $('robotCanvas');
  const rect = canvas.getBoundingClientRect();
  const scale = window.devicePixelRatio || 1;
  canvas.width = Math.max(360, Math.floor(rect.width * scale));
  canvas.height = Math.max(260, Math.floor(rect.height * scale));
  const ctx = canvas.getContext('2d');
  ctx.scale(scale, scale);
  const w = rect.width;
  const h = rect.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#060504';
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(215,167,53,0.12)';
  ctx.lineWidth = 1;
  for (let x = 24; x < w; x += 48) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke(); }
  for (let y = 24; y < h; y += 48) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }

  const model = state.robotModel || {};
  const modelMode = model.render_mode === 'mesh'
    ? 'MESH'
    : model.render_mode === 'mesh_missing'
      ? 'MESH MISSING'
      : 'PRIMITIVE URDF';
  ctx.fillStyle = 'rgba(245,234,216,0.64)';
  ctx.font = '700 11px ui-sans-serif, sans-serif';
  ctx.fillText(modelMode, 14, 22);

  const joints = snapshot.state?.joints_deg || [0, 0, 0, 0, 0, 0];
  const base = { x: w * 0.18, y: h * 0.76 };
  const lengths = [70, 88, 78, 56, 44, 34].map((v) => v * Math.min(w / 520, h / 320));
  let angle = -Math.PI / 2 + (joints[0] || 0) * Math.PI / 720;
  let p = base;
  const pts = [p];
  joints.forEach((deg, i) => {
    angle += (Number(deg) || 0) * Math.PI / 520;
    p = { x: p.x + Math.cos(angle) * lengths[i], y: p.y + Math.sin(angle) * lengths[i] };
    pts.push(p);
  });

  const floorY = h * 0.82;
  const baseGradient = ctx.createLinearGradient(base.x - 44, floorY - 28, base.x + 44, floorY + 16);
  baseGradient.addColorStop(0, '#2a2118');
  baseGradient.addColorStop(0.45, '#c49432');
  baseGradient.addColorStop(1, '#5f3a12');
  ctx.fillStyle = 'rgba(0,0,0,0.35)';
  ctx.beginPath();
  ctx.ellipse(base.x + 34, floorY + 18, 82, 18, 0, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = baseGradient;
  ctx.fillRect(base.x - 46, floorY - 30, 92, 30);
  ctx.beginPath();
  ctx.ellipse(base.x, floorY - 30, 46, 14, 0, 0, Math.PI * 2);
  ctx.fill();

  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  for (let i = 0; i < pts.length - 1; i += 1) {
    const start = pts[i];
    const end = pts[i + 1];
    const linkGradient = ctx.createLinearGradient(start.x, start.y, end.x, end.y);
    linkGradient.addColorStop(0, '#6b4b1c');
    linkGradient.addColorStop(0.45, '#f0c768');
    linkGradient.addColorStop(1, '#b97820');
    ctx.shadowColor = 'rgba(240,138,36,0.3)';
    ctx.shadowBlur = 14;
    ctx.strokeStyle = linkGradient;
    ctx.lineWidth = Math.max(12 - i, 7);
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(end.x, end.y);
    ctx.stroke();
  }
  ctx.shadowBlur = 0;
  pts.forEach((pt, i) => {
    ctx.fillStyle = i === 0 ? '#f08a24' : '#14100b';
    ctx.strokeStyle = i === pts.length - 1 ? '#f0c768' : '#f5ead8';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, i === 0 ? 14 : 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  });
}

function render(snapshot) {
  state.telemetry = snapshot;
  setText('connStatus', 'LIVE');
  $('connStatus').classList.remove('warn');
  setText('robotState', `state ${snapshot.robot?.state ?? '--'} / err ${snapshot.robot?.error ?? '--'}`);
  setText('episodeChip', `ep ${snapshot.dataset?.num_episodes ?? 0} / frame ${snapshot.dataset?.total_frames ?? 0}`);
  setText('coordFrame', snapshot.robot?.coord_frame || 'BASE');
  $('resetBtn').disabled = !snapshot.robot?.reset_enabled;

  renderKV($('datasetGrid'), [
    ['Repo', snapshot.dataset?.repo_id],
    ['Task', snapshot.dataset?.task],
    ['Episodes', snapshot.dataset?.num_episodes],
    ['Frames', snapshot.dataset?.total_frames],
    ['Buffer', snapshot.dataset?.episode_buffer_size],
    ['FPS', snapshot.dataset?.rate_hz],
    ['Root', snapshot.dataset?.root],
  ]);
  renderGrid($('stateGrid'), ['J1','J2','J3','J4','J5','J6'], snapshot.state?.joints_deg || [], 1);
  renderGrid($('actionGrid'), ['vx','vy','vz','vrx','vry','vrz'], snapshot.action?.velocity_cmd || [], 1);
  const pose = snapshot.state?.pose_xyzrpy_deg || [];
  setText('poseSummary', `xyz ${fmt(pose[0],0)}, ${fmt(pose[1],0)}, ${fmt(pose[2],0)} | rpy ${fmt(pose[3],1)}, ${fmt(pose[4],1)}, ${fmt(pose[5],1)}`);

  const cam = snapshot.camera?.[state.camera] || {};
  setText('cameraOverlay', `${state.camera.toUpperCase()} ${cam.status || 'unknown'} age ${fmt(cam.age_ms, 0)} ms`);
  setText('cameraStats', ['wrist','base','target'].map((name) => `${name}: ${snapshot.camera?.[name]?.status || '--'}`).join('   '));
  renderTorque(snapshot);
  renderRobot(snapshot);
}

function connectTelemetry() {
  const source = new EventSource('/api/telemetry');
  source.addEventListener('telemetry', (event) => {
    try { render(JSON.parse(event.data)); } catch (err) { console.error(err); }
  });
  source.onerror = () => {
    setText('connStatus', 'DISCONNECTED');
    $('connStatus').classList.add('warn');
  };
}

async function loadConfig(file) {
  state.configFile = file;
  const res = await fetch(`/api/config?file=${encodeURIComponent(file)}`);
  const data = await res.json();
  if (data.ok) {
    $('configText').value = data.text;
    setText('configStatus', `${file} config loaded. 保存后相关连接/URDF/payload 配置下次启动生效。`);
  } else {
    setText('configStatus', data.error || 'load failed');
  }
}

async function validateConfig() {
  const res = await fetch('/api/config/validate', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ text: $('configText').value }) });
  const data = await res.json();
  setText('configStatus', data.ok ? 'YAML 校验通过。' : `YAML 错误: ${data.error}`);
  return data.ok;
}

async function saveConfig() {
  const ok = await validateConfig();
  if (!ok) return;
  if (!confirm(`保存 ${state.configFile} 配置？会生成 .bak 备份，部分配置下次启动生效。`)) return;
  const res = await fetch(`/api/config?file=${encodeURIComponent(state.configFile)}`, { method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ text: $('configText').value }) });
  const data = await res.json();
  setText('configStatus', data.ok ? `已保存，备份: ${data.backup || 'none'}` : `保存失败: ${data.error}`);
}

async function resetHome() {
  if (!confirm('确认让机械臂回到 Home？请确保工作空间安全。')) return;
  const res = await fetch('/api/reset-home', { method: 'POST' });
  const data = await res.json();
  setText('connStatus', data.ok ? 'RESET REQUESTED' : `RESET BLOCKED: ${data.error}`);
  $('connStatus').classList.add('warn');
}

async function loadRobotModel() {
  try {
    const res = await fetch('/api/robot-model');
    const data = await res.json();
    state.robotModel = data;
    if (!data.ok) {
      setText('urdfStatus', data.error || 'URDF unavailable');
      return;
    }
    const label = data.render_mode === 'mesh'
      ? 'URDF mesh model'
      : data.render_mode === 'mesh_missing'
        ? 'URDF mesh refs missing'
        : 'URDF primitive model';
    setText('urdfStatus', `${label}: ${data.movable_joint_count}/${data.joint_count} movable joints, ${data.mesh_count} mesh`);
    if (state.telemetry) renderRobot(state.telemetry);
  } catch (err) {
    setText('urdfStatus', 'URDF metadata unavailable');
  }
}

function setupUI() {
  document.querySelectorAll('[data-camera]').forEach((btn) => {
    btn.addEventListener('click', () => {
      state.camera = btn.dataset.camera;
      document.querySelectorAll('[data-camera]').forEach((b) => b.classList.toggle('active', b === btn));
      $('cameraFeed').src = `/stream/${state.camera}.mjpg?t=${Date.now()}`;
      if (state.telemetry) render(state.telemetry);
    });
  });
  document.querySelectorAll('[data-config]').forEach((btn) => btn.addEventListener('click', () => loadConfig(btn.dataset.config)));
  $('validateConfig').addEventListener('click', validateConfig);
  $('saveConfig').addEventListener('click', saveConfig);
  $('resetBtn').addEventListener('click', resetHome);
  window.addEventListener('resize', () => state.telemetry && renderRobot(state.telemetry));
}

setupUI();
loadRobotModel();
connectTelemetry();
loadConfig('global');
