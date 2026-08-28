/* ============================================================
   WildNav Mission Control — client logic
   ============================================================ */

const S = {
  ws: null,
  drone: null,               // {name, ip, port}
  modes: [],
  selectedMode: 'safety',
  waypoints: [],             // [{lat, lon}]
  approxStart: null,         // {lat, lon} | null
  pinMode: 'waypoint',       // 'waypoint' | 'start'
  mapImg: null,
  mapReady: false,           // true once a valid GeoTIFF is uploaded + rendered
  // Geo footprint of the uploaded GeoTIFF (top-left / bottom-right corners in
  // WGS84). Populated from the /api/upload_map response — never hardcoded. The
  // SAME uploaded map is what the drone localises against, so these corners are
  // authoritative for both picking and the flight.
  geo: null,

  // ---- connection mode: local WiFi (default, unchanged) vs cloud (5G) ----
  connMode: 'wifi',          // 'wifi' | 'cloud'
  droneId: null,             // from /api/info — matches WILDNAV_DRONE_ID
  mqtt: null,                 // MQTT.js client, only used in cloud mode
  cloudCfg: null,             // {host, ws_port, path, drone_id} from /api/cloud_config
  pendingCmds: new Map(),     // cmd_id -> {resolve, reject, timeoutHandle}
};

/* ---------- boot ---------- */
window.addEventListener('DOMContentLoaded', () => {
  // On a drone-served page, /api/info tells us who we are. The discovery list
  // is populated by beacons relayed through this same drone's hub, but to keep
  // the agent lightweight we discover via the drone's own /api/info plus any
  // peers it knows. For simplicity here we show THIS drone + allow manual IPs.
  bootstrap();
});

async function bootstrap() {
  try {
    const info = await fetch('/api/info').then(r => r.json());
    S.modes = info.modes;
    S.droneId = info.name;
    renderDiscoveredSelf(info);
  } catch (e) {
    document.getElementById('droneList').innerHTML =
      '<div class="empty">Could not reach agent. Open this page from a drone IP.</div>';
  }
  // Also try the discovery hub websocket if present (when served by hub).
  tryHubDiscovery();
  // Open the agent WS immediately (needed for settings tool logs even before
  // a drone is selected — we're served by the drone anyway).
  openWebSocket();
}

/* ---------- discovery ---------- */
function renderDiscoveredSelf(info) {
  const list = document.getElementById('droneList');
  const busy = info.mission_active
    ? '<span class="busy-badge">BUSY</span>' : '';
  list.innerHTML = `
    <div class="drone-item" onclick='selectDrone(${JSON.stringify(JSON.stringify(
      {name: info.name, ip: info.ip, port: info.port}))})'>
      <div class="left">
        <span class="live-pip"></span>
        <div>
          <div class="dname">${info.name}</div>
          <div class="dip">${info.ip}:${info.port}</div>
        </div>
      </div>
      <div style="display:flex;align-items:center;gap:12px">
        ${busy}<span class="arrow">Select →</span>
      </div>
    </div>`;
}

function tryHubDiscovery() {
  // If a discovery_hub is broadcasting on the same host:8080, subscribe.
  // Non-fatal if absent.
  try {
    const hub = new WebSocket(`ws://${location.hostname}:8080/ws`);
    hub.onmessage = (e) => {
      const d = JSON.parse(e.data);
      if (d.drones && d.drones.length) appendDrones(d.drones);
    };
    hub.onerror = () => hub.close();
  } catch (e) { /* no hub, fine */ }
}

function appendDrones(drones) {
  const list = document.getElementById('droneList');
  const existing = new Set(
    [...list.querySelectorAll('.dip')].map(x => x.textContent));
  drones.forEach(x => {
    const key = `${x.ip}:${x.port}`;
    if (existing.has(key)) return;
    const div = document.createElement('div');
    div.className = 'drone-item';
    div.onclick = () => selectDrone(JSON.stringify(x));
    div.innerHTML = `
      <div class="left"><span class="live-pip"></span>
        <div><div class="dname">${x.name}</div>
        <div class="dip">${key}</div></div></div>
      <span class="arrow">Select →</span>`;
    list.appendChild(div);
  });
}

function connectManual() {
  const ip = document.getElementById('manualIp').value.trim();
  if (!ip) return;
  // If we're not already served by this IP, redirect to it.
  if (location.hostname !== ip) {
    location.href = `http://${ip}:8000`;
    return;
  }
  selectDrone(JSON.stringify({ name: ip, ip, port: 8000 }));
}

function selectDrone(json) {
  S.drone = JSON.parse(json);
  document.getElementById('cfgDroneName').textContent = S.drone.name;
  document.getElementById('liveDroneName').textContent = S.drone.name;
  document.getElementById('connModeDroneName').textContent = S.drone.name;
  show('stageConnMode'); hide('stageDiscovery'); hide('stageSettings');
  renderModes();
  if (!S.ws || S.ws.readyState !== 1) openWebSocket();
}

/* ---------- connection mode: local WiFi vs cloud (5G) ---------- */
function chooseConnMode(mode) {
  S.connMode = mode;
  document.getElementById('connModeWifiCard').classList.toggle('active', mode === 'wifi');
  document.getElementById('connModeCloudCard').classList.toggle('active', mode === 'cloud');
  document.getElementById('cloudConnPanel').classList.toggle('hidden', mode !== 'cloud');
  document.getElementById('connModeExplain').innerHTML = mode === 'wifi'
    ? "Commands and telemetry use the local network directly — fastest, but only works while you're on the drone's WiFi."
    : 'Commands and telemetry travel over the internet via the cloud broker — works from anywhere, including after you leave WiFi range.';
  const btn = document.getElementById('connModeContinueBtn');
  if (mode === 'wifi') {
    btn.disabled = false;
  } else {
    btn.disabled = !(S.mqtt && S.mqtt.connected);
    loadCloudConfig();
  }
}

async function loadCloudConfig() {
  if (!S.cloudCfg) {
    try { S.cloudCfg = await fetch('/api/cloud_config').then(r => r.json()); }
    catch (e) { S.cloudCfg = { host: '' }; }
    document.getElementById('ccHost').value = S.cloudCfg.host || '(not configured)';
    document.getElementById('ccPort').value = S.cloudCfg.ws_port || '8884';
    document.getElementById('ccPath').value = S.cloudCfg.path || '/mqtt';
  }
  checkCloudLink();
}

async function checkCloudLink() {
  const note = document.getElementById('cloudLinkNote');
  try {
    const st = await fetch('/api/cloud_status').then(r => r.json());
    if (!st.enabled) {
      note.textContent = "Cloud relay isn't configured on this drone (no WILDNAV_MQTT_HOST) — cloud mode unavailable.";
      note.style.color = 'var(--red)';
    } else if (!st.connected) {
      note.textContent = 'Drone is not currently connected to the broker — check its internet connection.';
      note.style.color = 'var(--red)';
    } else {
      note.textContent = 'Drone is connected to the broker ✓ — enter operator credentials below to connect this browser too.';
      note.style.color = 'var(--phosphor)';
    }
  } catch (e) {
    note.textContent = 'Could not check — the local agent is unreachable.';
    note.style.color = 'var(--red)';
  }
}

function connectCloud() {
  if (!S.cloudCfg || !S.cloudCfg.host) { alert('Cloud relay is not configured on this drone.'); return; }
  const user = document.getElementById('ccUser').value;
  const pass = document.getElementById('ccPass').value;
  if (S.mqtt) { try { S.mqtt.end(true); } catch (e) {} }

  const statusEl = document.getElementById('cloudConnStatus');
  statusEl.textContent = 'connecting…';
  const url = `wss://${S.cloudCfg.host}:${S.cloudCfg.ws_port}${S.cloudCfg.path}`;
  S.mqtt = mqtt.connect(url, {
    username: user || undefined,
    password: pass || undefined,
    clientId: 'wildnav-operator-' + Math.random().toString(16).slice(2, 10),
    clean: true,
    reconnectPeriod: 3000,
  });

  const id = () => S.cloudCfg.drone_id || S.droneId;

  S.mqtt.on('connect', () => {
    statusEl.textContent = 'connected ✓';
    S.mqtt.subscribe(`wildnav/${id()}/status`, { qos: 1 });
    S.mqtt.subscribe(`wildnav/${id()}/lwt`, { qos: 1 });
    S.mqtt.subscribe(`wildnav/${id()}/events`, { qos: 1 });
    S.mqtt.subscribe(`wildnav/${id()}/cmd_ack`, { qos: 1 });
    if (S.connMode === 'cloud') document.getElementById('connModeContinueBtn').disabled = false;
  });
  S.mqtt.on('reconnect', () => { statusEl.textContent = 'reconnecting…'; });
  S.mqtt.on('close', () => {
    statusEl.textContent = 'disconnected';
    if (S.connMode === 'cloud') document.getElementById('connModeContinueBtn').disabled = true;
  });
  S.mqtt.on('error', (e) => { statusEl.textContent = 'error: ' + (e && e.message || e); });
  S.mqtt.on('message', onCloudMessage);
}

function onCloudMessage(topic, payloadBuf) {
  const sub = topic.split('/')[2];
  let data;
  try { data = JSON.parse(payloadBuf.toString()); } catch (e) { return; }
  if (sub === 'status' || sub === 'lwt') {
    setConn(data.status === 'online' ? 'live' : 'dead',
            data.status === 'online' ? 'drone online (cloud)' : 'drone offline (cloud)');
    return;
  }
  // events carries the exact same {kind: ...} shape the local WebSocket
  // sends — handleMessage()/updateTelemetry()/routeEvent() below are
  // transport-agnostic, no cloud-specific rendering path needed.
  if (sub === 'events') { handleMessage(data.event || {}); return; }
  if (sub === 'cmd_ack') { handleCmdAck(data); return; }
}

function handleCmdAck(ack) {
  if (ack.status === 'received') {
    logLine('setup', '» command received by drone, executing…');
    return;
  }
  const pending = S.pendingCmds.get(ack.cmd_id);
  if (!pending) return;
  clearTimeout(pending.timeoutHandle);
  S.pendingCmds.delete(ack.cmd_id);
  if (ack.status === 'done') pending.resolve(ack);
  else pending.reject(new Error(ack.detail || 'rejected'));
}

function sendCloudCommand(action, params) {
  return new Promise((resolve, reject) => {
    if (!S.mqtt || !S.mqtt.connected) { reject(new Error('not connected to broker')); return; }
    const cmd_id = 'cmd-' + Date.now() + '-' + Math.random().toString(16).slice(2, 8);
    const timeoutHandle = setTimeout(() => {
      S.pendingCmds.delete(cmd_id);
      reject(new Error('no response from drone within 5s'));
    }, 5000);
    S.pendingCmds.set(cmd_id, { resolve, reject, timeoutHandle });
    const id = S.cloudCfg.drone_id || S.droneId;
    S.mqtt.publish(`wildnav/${id}/cmd`, JSON.stringify({
      cmd_id, action, params, ts: Date.now() / 1000,
    }), { qos: 1 });
  });
}

function confirmConnMode() {
  hide('stageConnMode'); show('stageConfig');
}

/* ---------- stage switching ---------- */
function show(id) { document.getElementById(id).classList.remove('hidden'); }
function hide(id) { document.getElementById(id).classList.add('hidden'); }

/* ---------- modes ---------- */
function renderModes() {
  const wrap = document.getElementById('modeCards');
  wrap.innerHTML = S.modes.map(m => {
    const seed = m.seed_from_gps
      ? '<span class="mc-flag on">GPS SEED</span>'
      : '<span class="mc-flag off">VISUAL SEED</span>';
    const corr = m.gps_correction
      ? '<span class="mc-flag on">GPS CORRECT</span>'
      : '<span class="mc-flag off">NO GPS CORRECT</span>';
    const active = m.name === S.selectedMode ? 'active' : '';
    return `<div class="mode-card ${active}" onclick="selectMode('${m.name}')">
      <div class="mc-name">${m.name}</div>
      <div class="mc-flags">${seed}${corr}</div>
    </div>`;
  }).join('');
  updateModeExplain();
}

function selectMode(name) {
  S.selectedMode = name;
  renderModes();
  refreshLaunchState();
}

function updateModeExplain() {
  const m = S.modes.find(x => x.name === S.selectedMode);
  if (!m) return;
  const seedTxt = m.seed_from_gps
    ? 'Start location comes from <b>GPS</b> at takeoff.'
    : 'Start location comes from the <b>first visual fix</b> after the vertical climb to altitude.';
  const corrTxt = m.gps_correction
    ? 'GPS may <b>take over course correction</b> if vision drifts for too long.'
    : 'Course correction is <b>vision-only</b> — GPS is never used to steer.';
  document.getElementById('modeExplain').innerHTML = `${seedTxt} ${corrTxt}`;
  // Approx-start pin only meaningful when seed is visual.
  const startBtn = document.getElementById('startModeBtn');
  startBtn.style.opacity = m.seed_from_gps ? '.4' : '1';
  startBtn.style.pointerEvents = m.seed_from_gps ? 'none' : 'auto';
}

/* ---------- map upload ---------- */
document.getElementById('mapFile').addEventListener('change', async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  setMapError('');
  setMapBusy(true);

  const fd = new FormData(); fd.append('file', file);
  let res;
  try {
    res = await fetch('/api/upload_map', { method: 'POST', body: fd })
      .then(r => r.json());
  } catch (err) {
    setMapBusy(false);
    setMapError('Upload failed — could not reach the drone.');
    return;
  }
  setMapBusy(false);

  if (!res.ok) {
    // Bad map (not a georeferenced GeoTIFF). Keep launch disabled.
    S.mapReady = false; S.geo = null;
    refreshLaunchState();
    setMapError(res.error || 'Invalid map file.');
    return;
  }

  // Valid GeoTIFF. The server gives us the real footprint corners (WGS84);
  // these drive picking and match exactly what the drone localises against.
  S.geo = res.bounds;
  S.mapMeta = { filename: res.filename, crs: res.crs,
                width_px: res.width_px, height_px: res.height_px };
  S.mapReady = true;
  S.mapImg = null;

  document.getElementById('mapPlaceholder').classList.add('hidden');
  const cv = document.getElementById('mapCanvas');
  cv.classList.remove('hidden');
  renderMapMeta();
  refreshLaunchState();

  // Load the server-rendered satellite preview (downsampled from the TIF).
  // Picking works with or without it; if it loads we paint it under the grid.
  if (res.preview_url) {
    const img = new Image();
    img.onload = () => { S.mapImg = img; drawMap(); };
    img.onerror = () => { S.mapImg = null; drawMap(); };
    img.src = res.preview_url;
  }
  drawMap();
});

function setMapError(msg) {
  const el = document.getElementById('mapError');
  if (!el) return;
  el.textContent = msg;
  el.classList.toggle('hidden', !msg);
}
function setMapBusy(on) {
  const btn = document.querySelector('.upload-btn');
  if (btn) btn.textContent = on ? 'Reading map…' : 'Upload GeoTIFF map';
}
function renderMapMeta() {
  const el = document.getElementById('mapMeta');
  if (!el || !S.mapMeta) return;
  const g = S.geo;
  el.innerHTML =
    `<span class="mm-file">${S.mapMeta.filename}</span>` +
    `<span class="mm-crs">${S.mapMeta.crs}</span>` +
    `<span class="mm-bounds">${g.tlLat.toFixed(5)}, ${g.tlLon.toFixed(5)} → ` +
    `${g.brLat.toFixed(5)}, ${g.brLon.toFixed(5)}</span>`;
  el.classList.remove('hidden');
}

function drawMap() {
  if (!S.geo) return;
  const cv = document.getElementById('mapCanvas');
  const frame = document.getElementById('mapFrame');
  const rect = frame.getBoundingClientRect();
  cv.width = rect.width; cv.height = rect.height;
  const ctx = cv.getContext('2d');
  ctx.clearRect(0, 0, cv.width, cv.height);

  // The map panel represents the GeoTIFF footprint as a coordinate-accurate
  // plane (browsers can't decode GeoTIFF pixels; the drone matches the raster
  // server-side). We letterbox the footprint's aspect ratio into the canvas.
  const geoW = Math.abs(S.geo.brLon - S.geo.tlLon);
  const geoH = Math.abs(S.geo.tlLat - S.geo.brLat);
  // Use the TIF's pixel aspect if available, else geographic aspect.
  const ar = (S.mapMeta && S.mapMeta.width_px && S.mapMeta.height_px)
    ? (S.mapMeta.width_px / S.mapMeta.height_px)
    : (geoW / geoH);
  const cr = cv.width / cv.height;
  let dw, dh, dx, dy;
  if (ar > cr) { dw = cv.width; dh = dw / ar; dx = 0; dy = (cv.height - dh) / 2; }
  else { dh = cv.height; dw = dh * ar; dy = 0; dx = (cv.width - dw) / 2; }
  S._draw = { dx, dy, dw, dh };

  // Footprint fill + border. If the satellite preview loaded, paint it into
  // the footprint rect (it's already north-up and matches the corner bounds).
  if (S.mapImg) {
    ctx.drawImage(S.mapImg, dx, dy, dw, dh);
  } else {
    ctx.fillStyle = '#0c141c';
    ctx.fillRect(dx, dy, dw, dh);
  }
  ctx.strokeStyle = 'rgba(61,155,255,.45)'; ctx.lineWidth = 1.5;
  ctx.strokeRect(dx, dy, dw, dh);

  // lat/lon graticule (fainter over imagery so it doesn't fight the map)
  const gridA = S.mapImg ? 0.28 : 0.13;
  const labelA = S.mapImg ? 0.9 : 0.55;
  ctx.strokeStyle = `rgba(61,155,255,${gridA})`; ctx.lineWidth = 1;
  ctx.font = '9px JetBrains Mono';
  ctx.textBaseline = 'top';
  const N = 4;
  for (let i = 0; i <= N; i++) {
    const fx = i / N, x = dx + fx * dw;
    ctx.beginPath(); ctx.moveTo(x, dy); ctx.lineTo(x, dy + dh); ctx.stroke();
    const lon = S.geo.tlLon + fx * (S.geo.brLon - S.geo.tlLon);
    if (i > 0 && i < N) {
      ctx.textAlign = 'center';
      if (S.mapImg) { ctx.fillStyle = 'rgba(0,0,0,.55)';
        ctx.fillText(lon.toFixed(4), x + 0.5, dy + 3.5); }
      ctx.fillStyle = `rgba(190,215,235,${labelA})`;
      ctx.fillText(lon.toFixed(4), x, dy + 3);
    }
    const fy = i / N, y = dy + fy * dh;
    ctx.beginPath(); ctx.moveTo(dx, y); ctx.lineTo(dx + dw, y); ctx.stroke();
    const lat = S.geo.tlLat + fy * (S.geo.brLat - S.geo.tlLat);
    if (i > 0 && i < N) {
      ctx.textAlign = 'left';
      if (S.mapImg) { ctx.fillStyle = 'rgba(0,0,0,.55)';
        ctx.fillText(lat.toFixed(4), dx + 3.5, y + 2.5); }
      ctx.fillStyle = `rgba(190,215,235,${labelA})`;
      ctx.fillText(lat.toFixed(4), dx + 3, y + 2);
    }
  }

  // waypoint path + pins
  ctx.strokeStyle = '#3d9bff'; ctx.lineWidth = 2; ctx.setLineDash([6, 5]);
  ctx.beginPath();
  S.waypoints.forEach((wp, i) => {
    const p = geoToPx(wp.lat, wp.lon);
    if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
  });
  ctx.stroke(); ctx.setLineDash([]);
  S.waypoints.forEach((wp, i) => {
    const p = geoToPx(wp.lat, wp.lon);
    ctx.fillStyle = '#3d9bff';
    ctx.beginPath(); ctx.arc(p.x, p.y, 11, 0, 7); ctx.fill();
    ctx.fillStyle = '#04121f'; ctx.font = '700 12px JetBrains Mono';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(i + 1, p.x, p.y);
  });
  if (S.approxStart) {
    const p = geoToPx(S.approxStart.lat, S.approxStart.lon);
    ctx.fillStyle = '#f0a830';
    ctx.beginPath(); ctx.arc(p.x, p.y, 10, 0, 7); ctx.fill();
    ctx.strokeStyle = '#1a1204'; ctx.lineWidth = 2; ctx.stroke();
    ctx.fillStyle = '#f0a830'; ctx.font = '700 9px JetBrains Mono';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('S', p.x, p.y);
  }
}

/* pixel <-> geo mapping via the footprint draw rect + GeoTIFF corners */
function pxToGeo(x, y) {
  const d = S._draw;
  const fx = (x - d.dx) / d.dw;   // 0..1 across footprint
  const fy = (y - d.dy) / d.dh;   // 0..1 down footprint
  const lat = S.geo.tlLat + fy * (S.geo.brLat - S.geo.tlLat);
  const lon = S.geo.tlLon + fx * (S.geo.brLon - S.geo.tlLon);
  return { lat, lon };
}
function geoToPx(lat, lon) {
  const d = S._draw;
  const fy = (lat - S.geo.tlLat) / (S.geo.brLat - S.geo.tlLat);
  const fx = (lon - S.geo.tlLon) / (S.geo.brLon - S.geo.tlLon);
  return { x: d.dx + fx * d.dw, y: d.dy + fy * d.dh };
}

document.getElementById('mapCanvas').addEventListener('click', (e) => {
  if (!S.mapReady) return;
  const rect = e.target.getBoundingClientRect();
  const x = e.clientX - rect.left, y = e.clientY - rect.top;
  const g = pxToGeo(x, y);
  if (S.pinMode === 'start') {
    S.approxStart = g; renderApproxStart();
  } else {
    S.waypoints.push(newWaypoint(g.lat, g.lon)); renderWaypoints();
  }
  drawMap();
});

// A waypoint carries the params for the leg ARRIVING at it. New waypoints
// inherit the current default speed/altitude; edit per-row afterwards.
function newWaypoint(lat, lon) {
  return {
    lat, lon,
    speed: parseFloat(document.getElementById('speedInput').value) || 2.5,
    alt: parseFloat(document.getElementById('altSelect').value) || 120,
  };
}

function setPinMode(mode) {
  S.pinMode = mode;
  document.getElementById('pinModeBtn').classList.toggle('active', mode === 'waypoint');
  document.getElementById('startModeBtn').classList.toggle('active', mode === 'start');
}

/* ---------- waypoint list ---------- */
function renderWaypoints() {
  const ul = document.getElementById('wpList');
  ul.innerHTML = S.waypoints.map((wp, i) => `
    <li>
      <span class="wp-idx">${i + 1}</span>
      <span class="wp-coords">${wp.lat.toFixed(6)}, ${wp.lon.toFixed(6)}</span>
      <button class="x" onclick="removeWaypoint(${i})">×</button>
    </li>`).join('');
  document.getElementById('wpCount').textContent = S.waypoints.length;
  refreshLaunchState();
}
function addManualWaypoint() {
  const lat = parseFloat(document.getElementById('wpLat').value);
  const lon = parseFloat(document.getElementById('wpLon').value);
  if (isNaN(lat) || isNaN(lon)) return;
  S.waypoints.push({ lat, lon });
  document.getElementById('wpLat').value = '';
  document.getElementById('wpLon').value = '';
  renderWaypoints();
  if (S.mapReady) drawMap();
}
function removeWaypoint(i) {
  S.waypoints.splice(i, 1); renderWaypoints();
  if (S.mapReady) drawMap();
}
function clearWaypoints() {
  S.waypoints = []; renderWaypoints();
  if (S.mapReady) drawMap();
}
function renderApproxStart() {
  const row = document.getElementById('approxStartRow');
  if (S.approxStart) {
    row.classList.remove('hidden');
    document.getElementById('approxStartText').textContent =
      `${S.approxStart.lat.toFixed(6)}, ${S.approxStart.lon.toFixed(6)}`;
  } else {
    row.classList.add('hidden');
  }
}
function clearApproxStart() {
  S.approxStart = null; renderApproxStart();
  if (S.mapReady) drawMap();
}

/* ---------- launch gating ---------- */
function refreshLaunchState() {
  const btn = document.getElementById('launchBtn');
  const note = document.getElementById('launchNote');
  if (!S.mapReady) {
    btn.disabled = true;
    note.textContent = 'Upload a GeoTIFF map to begin.';
  } else if (S.waypoints.length === 0) {
    btn.disabled = true;
    note.textContent = 'Add at least one waypoint to launch.';
  } else {
    btn.disabled = false;
    const m = S.modes.find(x => x.name === S.selectedMode);
    note.textContent = m && !m.seed_from_gps
      ? 'Drone will climb vertically to altitude, acquire its start by vision, then fly the route.'
      : 'Drone will seed its start from GPS, then fly the route.';
  }
}

/* ---------- launch ---------- */
async function launch() {
  // The map is not sent here — the server uses the GeoTIFF already uploaded
  // via /api/upload_map for both picking and the drone's localisation.
  const payload = {
    waypoints: S.waypoints.map(w => [w.lat, w.lon]),
    mode: S.selectedMode,
    takeoff_alt: parseFloat(document.getElementById('altSelect').value),
    nav_speed: parseFloat(document.getElementById('speedInput').value),
    min_valid_alt_m: parseFloat(document.getElementById('minAltInput').value),
    approx_start: S.approxStart ? [S.approxStart.lat, S.approxStart.lon] : null,
  };
  if (S.connMode === 'cloud') {
    const btn = document.getElementById('launchBtn');
    btn.disabled = true; btn.innerHTML = '<span class="launch-icon">▲</span> Sending…';
    try {
      await sendCloudCommand('start_mission', payload);
      hide('stageConfig'); show('stageLive');
      logLine('ok', 'Mission command sent via cloud — confirmed by drone.');
    } catch (e) {
      alert('Launch failed: ' + e.message);
    } finally {
      btn.disabled = false; btn.innerHTML = '<span class="launch-icon">▲</span> Start mission';
    }
    return;
  }

  const res = await fetch('/api/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  }).then(r => r.json());
  if (!res.ok) { alert('Launch failed: ' + (res.error || res.message)); return; }
  hide('stageConfig'); show('stageLive');
  logLine('ok', 'Mission command sent — running setup then flight.');
}

async function stopMission() {
  if (S.connMode === 'cloud') {
    try {
      await sendCloudCommand('stop', {});
      logLine('err', 'STOP sent via cloud — confirmed by drone.');
    } catch (e) {
      logLine('err', 'STOP via cloud failed: ' + e.message);
    }
    return;
  }
  await fetch('/api/stop', { method: 'POST' }).catch(() => {});
  logLine('err', 'STOP sent — drone commanded to LOITER/LAND.');
}

/* ---------- websocket: live events + telemetry ---------- */
function openWebSocket() {
  const url = `ws://${location.host}/ws`;
  S.ws = new WebSocket(url);
  S.ws.onopen = () => setConn('live', 'connected');
  S.ws.onclose = () => setConn('dead', 'disconnected');
  S.ws.onerror = () => setConn('dead', 'error');
  S.ws.onmessage = (e) => handleMessage(JSON.parse(e.data));
}

function setConn(cls, txt) {
  const dot = document.getElementById('connDot');
  dot.className = 'conn-dot ' + cls;
  document.getElementById('connText').textContent = txt;
}

function handleMessage(m) {
  if (m.kind === 'heartbeat') { updateTrtState(!!m.trt_running); return; }
  if (m.kind === 'telem') { updateTelemetry(m); return; }
  if (m.kind === 'tool_log') { appendTrtLog(m.line); return; }
  routeEvent(m);
}

/* ---------- settings panel ---------- */
let _prevStage = 'stageDiscovery';
function toggleSettings() {
  const s = document.getElementById('stageSettings');
  if (s.classList.contains('hidden')) {
    for (const id of ['stageDiscovery', 'stageConfig', 'stageLive']) {
      if (!document.getElementById(id).classList.contains('hidden')) _prevStage = id;
      hide(id);
    }
    show('stageSettings');
    loadSettings();
  } else {
    hide('stageSettings');
    show(_prevStage);
  }
}

async function loadSettings() {
  try {
    const s = await fetch('/api/settings').then(r => r.json());
    document.getElementById('setName').value = s.name || '';
    document.getElementById('autostartToggle').checked = !!s.autostart;
    document.getElementById('autostartText').textContent =
      s.autostart ? 'enabled — agent starts at boot' : 'disabled';
    updateTrtState(!!s.trt_running);
    if (s.trt_exists) {
      document.getElementById('trtStatus').textContent = 'engine present ✓';
    }
  } catch (e) { /* agent unreachable; fields stay blank */ }
}

async function saveName() {
  const name = document.getElementById('setName').value.trim();
  const fb = document.getElementById('nameFeedback');
  const res = await fetch('/api/settings', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  }).then(r => r.json()).catch(() => ({ ok: false, error: 'unreachable' }));
  fb.classList.remove('hidden', 'ok', 'err');
  if (res.ok) {
    fb.classList.add('ok');
    fb.textContent = `✓ saved as "${res.name}" — ${res.note}`;
  } else {
    fb.classList.add('err');
    fb.textContent = '✗ ' + (res.error || 'save failed');
  }
}

async function setAutostart(enabled) {
  const fb = document.getElementById('autostartFeedback');
  const res = await fetch('/api/autostart', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled }),
  }).then(r => r.json()).catch(() => ({ ok: false, error: 'unreachable' }));
  fb.classList.remove('hidden', 'ok', 'err');
  if (res.ok) {
    fb.classList.add('ok');
    fb.textContent = enabled ? '✓ agent will start at boot'
                             : '✓ autostart disabled';
    document.getElementById('autostartText').textContent =
      enabled ? 'enabled — agent starts at boot' : 'disabled';
  } else {
    fb.classList.add('err');
    fb.textContent = '✗ ' + (res.error || 'failed');
    document.getElementById('autostartToggle').checked = !enabled; // revert
  }
}

async function startTrt() {
  const width = parseInt(document.getElementById('trtWidth').value, 10);
  const height = parseInt(document.getElementById('trtHeight').value, 10);
  const log = document.getElementById('trtLog');
  log.classList.remove('hidden'); log.textContent = '';
  const res = await fetch('/api/convert_trt', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ width, height }),
  }).then(r => r.json()).catch(() => ({ ok: false, message: 'unreachable' }));
  appendTrtLog(res.ok ? `» ${res.message}` : `✗ ${res.message || res.error}`);
}

function appendTrtLog(line) {
  const log = document.getElementById('trtLog');
  if (!log) return;
  log.classList.remove('hidden');
  log.textContent += line + '\n';
  log.scrollTop = log.scrollHeight;
}

function updateTrtState(running) {
  const btn = document.getElementById('trtBtn');
  const st = document.getElementById('trtStatus');
  if (!btn) return;
  btn.disabled = running;
  btn.textContent = running ? 'Converting…' : 'Convert';
  if (running) st.textContent = 'running — takes 5–10 min';
}

function updateTelemetry(t) {
  if (t.phase === 'takeoff') {
    document.getElementById('livePhase').textContent =
      `Vertical climb — ${(t.agl_m||0).toFixed(0)} / ${t.target_alt} m`;
    setTel('telAgl', (t.agl_m||0).toFixed(0), 'm');
    return;
  }
  if (t.phase === 'landing') {
    document.getElementById('livePhase').textContent =
      `Descending — ${(t.agl_m||0).toFixed(0)} m`;
    setTel('telAgl', (t.agl_m||0).toFixed(0), 'm');
    return;
  }
  if (t.phase !== 'nav') return;
  document.getElementById('livePhase').textContent =
    `${t.state} · leg ${t.leg}/${t.total_legs}`;
  setTel('telState', t.state);
  setTel('telLeg', `${t.leg}/${t.total_legs}`);
  setTel('telAgl', (t.agl_m||0).toFixed(0), 'm');
  setTel('telDist', (t.dist_to_target_m||0).toFixed(0), 'm');
  setTel('telCte', (t.cte_m||0).toFixed(1), 'm', Math.abs(t.cte_m) > 15 ? 'warn' : '');
  setTel('telHdg', (t.heading_deg||0).toFixed(0), '°');
  setTel('telErr', (t.error_m||0).toFixed(1), 'm', t.error_m > 10 ? 'warn' : '');
  setTel('telInliers', t.n_inliers ?? '—', '',
    (t.status === 'accepted') ? 'hot' : '');
}

function setTel(id, val, unit = '', cls = '') {
  const el = document.getElementById(id);
  el.className = 'tel-value' + (cls ? ' ' + cls : '');
  el.innerHTML = val + (unit ? `<span class="tel-unit">${unit}</span>` : '');
}

function routeEvent(m) {
  const k = m.kind;
  const okKinds = ['setup_complete', 'camera_ready', 'telemetry_ok',
    'takeoff_complete', 'start_acquired', 'first_fix_ok', 'navigator_ready',
    'waypoint_reached', 'landed', 'mission_complete', 'yaw_locked_north'];
  const warnKinds = ['timeout', 'safety_loiter', 'gps_takeover', 'gps_reset',
    'gps_arrival', 'first_fix_wait', 'first_fix_retry', 'ap_status', 'engine_warn'];
  const errKinds = ['abort'];

  let cls = 'evt';
  if (okKinds.includes(k)) cls = 'ok';
  else if (warnKinds.includes(k)) cls = 'warn';
  else if (errKinds.includes(k)) cls = 'err';
  else if (k === 'setup_log' || k === 'setup_start') cls = 'setup';

  logLine(cls, formatEvent(m));

  if (k === 'phase') {
    document.getElementById('livePhase').textContent = phaseLabel(m.phase);
  }
  if (k === 'mission_complete' || k === 'engine_stopped') {
    document.getElementById('logStatus').style.color = '#6b7885';
  }
}

function phaseLabel(p) {
  const map = {
    GUIDED: 'Entering GUIDED mode…', ARM: 'Arming…',
    TAKEOFF: 'Vertical takeoff…',
    ACQUIRE_START_VISUAL: 'Acquiring start location by vision…',
    NAVIGATING: 'Navigating route…',
  };
  return map[p] || p;
}

function formatEvent(m) {
  const k = m.kind;
  switch (k) {
    case 'setup_start': return '$ activating conda env…';
    case 'setup_log': return '  ' + m.line;
    case 'setup_complete': return '✓ setup complete';
    case 'engine_start': return `engine start · mode=${m.mode}`;
    case 'gimbal_locked': return '✓ gimbal locked to nadir (-90°)';
    case 'mavlink_connecting': return `MAVLink → ${m.conn}`;
    case 'camera_ready': return '✓ camera stream live';
    case 'telemetry_ok': return `✓ telemetry · AGL ${m.agl?.toFixed(1)}m · GPS ${m.lat?.toFixed(5)},${m.lon?.toFixed(5)}`;
    case 'phase': return `» ${phaseLabel(m.phase)}`;
    case 'climbing': return `climbing · AGL ${m.agl?.toFixed(1)}m`;
    case 'takeoff_complete': return `✓ reached altitude · AGL ${m.agl?.toFixed(1)}m`;
    case 'yaw_locked_north': return '✓ yaw locked NORTH';
    case 'first_fix_wait': return `first-fix waiting · AGL ${m.agl?.toFixed(1)}m < floor ${m.floor}m`;
    case 'first_fix_fullmap_search': return 'first-fix: full-map search (no start pin)…';
    case 'first_fix_retry': return `first-fix retry · inliers=${m.inliers}`;
    case 'first_fix_ok': return `✓ first fix · inliers=${m.inliers} @ ${m.agl?.toFixed(1)}m`;
    case 'start_acquired': return `✓ START acquired (${m.source}) · ${m.lat?.toFixed(6)},${m.lon?.toFixed(6)}`;
    case 'navigator_ready': return `✓ navigator ready · ${m.legs} leg(s)`;
    case 'waypoint_reached': return `◎ waypoint ${m.leg}/${m.total} reached → next ${m.next_target?.map(x=>x.toFixed(5)).join(',')}`;
    case 'gps_takeover': return `⚠ GPS takeover · EKF↔GPS err ${m.error_m?.toFixed(1)}m`;
    case 'gps_reset': return `⚠ GPS reset #${m.count}`;
    case 'gps_arrival': return `GPS arrival backstop · ${m.dist?.toFixed(1)}m`;
    case 'landing': return `landing · ${m.dist_to_final?.toFixed(1)}m to final`;
    case 'landed': return `✓ LANDED · AGL ${m.agl?.toFixed(1)}m`;
    case 'safety_loiter': return `⚠ SAFETY → LOITER · ${m.reason}`;
    case 'ap_status': return `[AP] ${m.text}`;
    case 'timeout': return `⚠ timeout waiting for ${m.desc}`;
    case 'abort': return `✗ ABORT · ${m.reason}`;
    case 'mission_complete': return `✓ MISSION COMPLETE · ${m.frames} frames · ${m.gps_resets} GPS resets`;
    case 'engine_stopped': return 'engine stopped';
    default: return `${k} ${JSON.stringify(dropKeys(m))}`;
  }
}
function dropKeys(m) {
  const c = { ...m }; delete c.kind; delete c.ts; return c;
}

function logLine(cls, msg) {
  const body = document.getElementById('logBody');
  const t = new Date().toLocaleTimeString('en-GB', { hour12: false });
  const line = document.createElement('div');
  line.className = 'log-line ' + cls;
  line.innerHTML = `<span class="log-time">${t}</span><span class="log-msg">${escapeHtml(msg)}</span>`;
  body.appendChild(line);
  body.scrollTop = body.scrollHeight;
}
function escapeHtml(s) {
  return String(s).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
}

window.addEventListener('resize', () => { if (S.mapReady) drawMap(); });
