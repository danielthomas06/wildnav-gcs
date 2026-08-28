/* cloud.js — WildNav cloud dashboard (view-only, v1)
 *
 * Subscribes to wildnav/+/status, /lwt, /events over MQTT-over-WebSocket
 * and renders them. The `event` formatting helpers below (formatEvent,
 * phaseLabel, updateTelemetry) are intentionally mirrored from
 * static/app.js's local-WiFi dashboard, since both render the exact same
 * event `kind` vocabulary produced by drone_agent.py/flight_engine.py —
 * keep new `kind`s in sync in both files.
 */
const S = {
  client: null,
  drones: new Map(),       // drone_id -> { lastSeen, status }
  missions: new Map(),     // drone_id -> last mission_config payload (waypoints, mode, ...)
  selectedDrone: null,
  missionActive: false,    // true once a mission_config arrives or our own launch is confirmed
  pendingCmds: new Map(),  // cmd_id -> {resolve, reject, timeoutHandle}
  plan: {                  // waypoints picked on the map, not yet launched
    waypoints: [],          // [[lat, lon]]
    mode: 'safety',
    pinMode: 'waypoint',    // 'waypoint' | 'start' — which pin the next map click drops
    approxStart: null,      // [lat, lon] | null
  },
  availableMaps: [],       // filenames pre-staged on the Jetson (uploaded over local WiFi)
  activeMap: null,         // which one is currently selected for the next launch
  mapsByDrone: new Map(),  // drone_id -> last maps_list payload, cached regardless of selection
};

// Same 4 modes nav_modes.py serves via /api/info on the local page — that
// endpoint isn't reachable from here (mixed content, see DESIGN.md), so
// this is a small fixed duplicate rather than a live fetch. Keep in sync if
// the mode set ever changes.
const NAV_MODES = [
  { name: 'safety', seed_from_gps: true, gps_correction: true },
  { name: 'cold_start', seed_from_gps: false, gps_correction: true },
  { name: 'safe_start', seed_from_gps: true, gps_correction: false },
  { name: 'full_gps_denied', seed_from_gps: false, gps_correction: false },
];

/* ---------- map ---------- */
const M = { map: null, waypoints: null, trail: null, trailPts: [], gps: null, ekf: null, approxStart: null };

function initMap() {
  if (M.map) return;
  M.map = L.map('mapView', { attributionControl: false, zoomControl: true }).setView([20, 0], 2);
  L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    { maxZoom: 19 }
  ).addTo(M.map);
  M.waypoints = L.layerGroup().addTo(M.map);
  M.trail = L.polyline([], { color: '#f0a830', weight: 2, opacity: 0.7 }).addTo(M.map);
  M.gps = L.circleMarker([0, 0], { radius: 6, color: '#35e08a', fillColor: '#35e08a', fillOpacity: 1 });
  M.ekf = L.circleMarker([0, 0], { radius: 6, color: '#f0a830', fillColor: '#f0a830', fillOpacity: 1 });
  M.approxStart = L.circleMarker([0, 0], { radius: 10, color: '#1a1204', weight: 2, fillColor: '#f0a830', fillOpacity: 1 })
    .bindTooltip('S', { permanent: true, direction: 'center', className: 'wp-label' });
  // Same map, dual purpose: no mission active -> click to plan waypoints;
  // once one's confirmed running, clicks are ignored (see onMapClickForPlanning)
  // so you can't confuse "picked but not sent" with "actually flying".
  M.map.on('click', onMapClickForPlanning);
  updateMapCursor();
}

// Leaflet's default container cursor is "grab" (pan affordance), which gives
// no visual hint that a click drops a pin during planning — swap to a
// crosshair whenever a click would actually place one, back to the normal
// grab cursor once a mission is live (clicks are ignored then, see
// onMapClickForPlanning).
function updateMapCursor() {
  if (!M.map) return;
  M.map.getContainer().classList.toggle('planning-cursor', !S.missionActive);
}

function resetMap() {
  if (!M.map) return;
  M.waypoints.clearLayers();
  M.trailPts = [];
  M.trail.setLatLngs([]);
  M.map.removeLayer(M.gps);
  M.map.removeLayer(M.ekf);
  M.map.removeLayer(M.approxStart);
}

// Shared by both the "confirmed by drone" mission view and the "picked but
// not launched yet" planning view — same layer group, same drawing code,
// whichever one is currently relevant just calls this with its own points.
function drawWaypointsOnMap(pts) {
  if (!M.map) return;
  M.waypoints.clearLayers();
  if (!pts.length) return;
  L.polyline(pts, { color: '#6b7885', weight: 2, dashArray: '4 5' }).addTo(M.waypoints);
  pts.forEach((ll, i) => {
    L.circleMarker(ll, { radius: 5, color: '#dfe7ee', fillColor: '#dfe7ee', fillOpacity: 0.9 })
      .bindTooltip(String(i + 1), { permanent: true, direction: 'top', className: 'wp-label', offset: [0, -4] })
      .addTo(M.waypoints);
  });
  M.map.fitBounds(pts, { padding: [24, 24] });
}

function renderMission(m) {
  if (!m) return;
  drawWaypointsOnMap((m.waypoints || []).map(([lat, lon]) => [lat, lon]));
}

/* ---------- mission planning: click the map to add waypoints or set approx start ---------- */
function onMapClickForPlanning(e) {
  if (S.missionActive) return; // don't let clicks during a live mission look like a route change
  if (S.plan.pinMode === 'start') {
    S.plan.approxStart = [e.latlng.lat, e.latlng.lng];
    renderPlanApproxStart();
    return;
  }
  S.plan.waypoints.push([e.latlng.lat, e.latlng.lng]);
  renderPlanWaypoints();
}

function setPlanPinMode(mode) {
  S.plan.pinMode = mode;
  document.getElementById('cloudPinModeBtn').classList.toggle('active', mode === 'waypoint');
  document.getElementById('cloudStartModeBtn').classList.toggle('active', mode === 'start');
  updateMapCursor();
}

function renderPlanApproxStart() {
  const row = document.getElementById('cloudApproxStartRow');
  const text = document.getElementById('cloudApproxStartText');
  if (!row || !text || !M.map) return;
  if (!S.plan.approxStart) {
    row.classList.add('hidden');
    M.map.removeLayer(M.approxStart);
    return;
  }
  const [lat, lon] = S.plan.approxStart;
  text.textContent = `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
  row.classList.remove('hidden');
  M.approxStart.setLatLng([lat, lon]);
  if (!M.map.hasLayer(M.approxStart)) M.approxStart.addTo(M.map);
}

function clearPlanApproxStart() {
  S.plan.approxStart = null;
  renderPlanApproxStart();
}

function canLaunch() {
  return S.plan.waypoints.length > 0 && !S.missionActive && !!S.activeMap;
}

function renderPlanWaypoints() {
  const countEl = document.getElementById('planWpCount');
  const btn = document.getElementById('planLaunchBtn');
  if (countEl) countEl.textContent = S.plan.waypoints.length;
  if (btn) btn.disabled = !canLaunch();
  drawWaypointsOnMap(S.plan.waypoints);
}

function clearPlanWaypoints() {
  S.plan.waypoints = [];
  renderPlanWaypoints();
}

function renderPlanModes() {
  const wrap = document.getElementById('planModeCards');
  if (!wrap) return;
  wrap.innerHTML = NAV_MODES.map(m => {
    const active = m.name === S.plan.mode ? 'active' : '';
    return `<div class="mode-card ${active}" onclick="selectPlanMode('${m.name}')">
      <div class="mc-name">${m.name}</div>
    </div>`;
  }).join('');
}

function selectPlanMode(name) {
  S.plan.mode = name;
  renderPlanModes();
  updateStartPinGating();
}

// Approx-start pin only meaningful when the mode seeds from vision, not GPS.
function updateStartPinGating() {
  const m = NAV_MODES.find(x => x.name === S.plan.mode);
  const startBtn = document.getElementById('cloudStartModeBtn');
  if (!m || !startBtn) return;
  startBtn.style.opacity = m.seed_from_gps ? '.4' : '1';
  startBtn.style.pointerEvents = m.seed_from_gps ? 'none' : 'auto';
  if (m.seed_from_gps && S.plan.pinMode === 'start') setPlanPinMode('waypoint');
}

function updatePlanControlsEnabled() {
  const note = document.getElementById('planNote');
  const btn = document.getElementById('planLaunchBtn');
  updateMapCursor();
  if (!note || !btn) return;
  btn.disabled = !canLaunch();
  if (S.missionActive) {
    note.textContent = 'A mission is currently active — plan the next one once it ends, or STOP first.';
  } else if (!S.activeMap) {
    note.textContent = 'No map selected — pick a pre-staged one above, or upload via local WiFi first.';
  } else {
    note.textContent = 'Click the map to add waypoints, then Start mission.';
  }
}

async function launchFromCloud() {
  const payload = {
    waypoints: S.plan.waypoints,
    mode: S.plan.mode,
    takeoff_alt: parseFloat(document.getElementById('planAlt').value),
    nav_speed: parseFloat(document.getElementById('planSpeed').value),
    min_valid_alt_m: parseFloat(document.getElementById('planMinAlt').value),
    approx_start: S.plan.approxStart || null,
  };
  const btn = document.getElementById('planLaunchBtn');
  btn.disabled = true;
  btn.innerHTML = '<span class="launch-icon">▲</span> Sending…';
  try {
    await sendCloudCommand('start_mission', payload);
    S.missionActive = true;
    updatePlanControlsEnabled();
    logLine('ok', 'Mission command sent — confirmed by drone.');
  } catch (e) {
    alert('Launch failed: ' + e.message);
  } finally {
    btn.innerHTML = '<span class="launch-icon">▲</span> Start mission';
    btn.disabled = S.plan.waypoints.length === 0 || S.missionActive;
  }
}

async function stopFromCloud() {
  try {
    await sendCloudCommand('stop', {});
    S.missionActive = false;
    updatePlanControlsEnabled();
    logLine('err', 'STOP sent — confirmed by drone.');
  } catch (e) {
    logLine('err', 'STOP failed: ' + e.message);
  }
}

/* ---------- command channel: send + ack (mirrors static/app.js) ---------- */
function sendCloudCommand(action, params) {
  return new Promise((resolve, reject) => {
    if (!S.client || !S.client.connected) { reject(new Error('not connected to broker')); return; }
    if (!S.selectedDrone) { reject(new Error('no drone selected')); return; }
    const cmd_id = 'cmd-' + Date.now() + '-' + Math.random().toString(16).slice(2, 8);
    const timeoutHandle = setTimeout(() => {
      S.pendingCmds.delete(cmd_id);
      reject(new Error('no response from drone within 5s'));
    }, 5000);
    S.pendingCmds.set(cmd_id, { resolve, reject, timeoutHandle });
    S.client.publish(`wildnav/${S.selectedDrone}/cmd`, JSON.stringify({
      cmd_id, action, params, ts: Date.now() / 1000,
    }), { qos: 1 });
  });
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

function updateMapPosition(t) {
  if (!M.map) return;
  if (typeof t.gps_lat === 'number' && typeof t.gps_lon === 'number') {
    M.gps.setLatLng([t.gps_lat, t.gps_lon]);
    if (!M.map.hasLayer(M.gps)) M.gps.addTo(M.map);
  }
  if (typeof t.ekf_lat === 'number' && typeof t.ekf_lon === 'number') {
    M.ekf.setLatLng([t.ekf_lat, t.ekf_lon]);
    if (!M.map.hasLayer(M.ekf)) M.ekf.addTo(M.map);
    M.trailPts.push([t.ekf_lat, t.ekf_lon]);
    if (M.trailPts.length > 400) M.trailPts.shift();
    M.trail.setLatLngs(M.trailPts);
  }
}

/* ---------- connection panel ---------- */
function toggleConnPanel() {
  document.getElementById('stageConn').classList.toggle('hidden');
}

function loadSavedConn() {
  try {
    const saved = JSON.parse(localStorage.getItem('wildnav_cloud_conn') || '{}');
    if (saved.host) document.getElementById('cHost').value = saved.host;
    if (saved.port) document.getElementById('cPort').value = saved.port;
    if (saved.path) document.getElementById('cPath').value = saved.path;
    if (saved.user) document.getElementById('cUser').value = saved.user;
    if (typeof saved.tls === 'boolean') document.getElementById('cTls').checked = saved.tls;
  } catch (e) { /* ignore */ }
}

function connectBroker() {
  const host = document.getElementById('cHost').value.trim();
  const port = document.getElementById('cPort').value.trim() || '8884';
  const path = document.getElementById('cPath').value.trim() || '/mqtt';
  const useTls = document.getElementById('cTls').checked;
  const user = document.getElementById('cUser').value;
  const pass = document.getElementById('cPass').value;
  if (!host) { alert('Broker host is required.'); return; }

  localStorage.setItem('wildnav_cloud_conn', JSON.stringify({ host, port, path, user, tls: useTls }));

  if (S.client) { try { S.client.end(true); } catch (e) {} }

  const proto = useTls ? 'wss' : 'ws';
  const url = `${proto}://${host}:${port}${path}`;
  setConn('connecting', 'connecting…');

  S.client = mqtt.connect(url, {
    username: user || undefined,
    password: pass || undefined,
    clientId: 'wildnav-dashboard-' + Math.random().toString(16).slice(2, 10),
    clean: true,
    reconnectPeriod: 3000,
  });

  S.client.on('connect', () => {
    setConn('live', 'connected');
    S.client.subscribe('wildnav/+/status', { qos: 1 });
    S.client.subscribe('wildnav/+/lwt', { qos: 1 });
    S.client.subscribe('wildnav/+/events', { qos: 1 });
    S.client.subscribe('wildnav/+/mission', { qos: 1 });
    S.client.subscribe('wildnav/+/cmd_ack', { qos: 1 });
    S.client.subscribe('wildnav/+/maps', { qos: 1 });
    document.getElementById('stageConn').classList.add('hidden');
    document.getElementById('stageLive').classList.remove('hidden');
    initMap();
    renderPlanModes();
    updateStartPinGating();
    updatePlanControlsEnabled();
    setTimeout(() => M.map && M.map.invalidateSize(), 50); // container was display:none at init
  });
  S.client.on('reconnect', () => setConn('connecting', 'reconnecting…'));
  S.client.on('close', () => setConn('dead', 'disconnected'));
  S.client.on('error', (e) => setConn('dead', 'error: ' + (e && e.message || e)));
  S.client.on('message', onMqttMessage);
}

function disconnectBroker() {
  if (S.client) { S.client.end(true); S.client = null; }
  setConn('dead', 'disconnected');
}

function setConn(cls, txt) {
  document.getElementById('connDot').className = 'conn-dot ' + cls;
  document.getElementById('connText').textContent = txt;
}

/* ---------- message routing ---------- */
function onMqttMessage(topic, payloadBuf) {
  const parts = topic.split('/'); // wildnav/<id>/<sub>
  if (parts.length < 3 || parts[0] !== 'wildnav') return;
  const droneId = parts[1];
  const sub = parts[2];
  let data;
  try { data = JSON.parse(payloadBuf.toString()); } catch (e) { return; }

  registerDrone(droneId, sub, data);
  if (sub === 'mission') S.missions.set(droneId, data); // cache regardless of selection
  if (sub === 'maps') S.mapsByDrone.set(droneId, data); // same — a wildcard sub sees every drone
  if (droneId !== S.selectedDrone) return;

  // status and lwt carry the same {status, ts, mission_active?} shape —
  // cloud_relay.py publishes a retained "online" birth message to BOTH on
  // every connect, so a stale retained "offline" from a past ungraceful
  // disconnect never outlives a fresh reconnect. Read the payload, don't
  // assume the topic.
  if (sub === 'status' || sub === 'lwt') {
    updateLastSeen(data.ts);
    setTel('telLink', data.status === 'online' ? 'ONLINE' : 'OFFLINE');
    // This 5Hz retained heartbeat is the only authoritative "is a mission
    // actually running" signal — see the `mission` handler below for why
    // that topic can't be trusted for this.
    if (typeof data.mission_active === 'boolean' && data.mission_active !== S.missionActive) {
      S.missionActive = data.mission_active;
      updatePlanControlsEnabled();
    }
    return;
  }
  if (sub === 'events') { updateLatency(data.ts); handleEvent(data.event || {}); return; }
  if (sub === 'mission') {
    // Retained on its own topic so a dashboard connecting mid-mission gets
    // the waypoint list immediately — but it's retained forever, so it still
    // holds the *last* mission ever started even long after that one ended.
    // Never infer "active" from its mere presence; trust the status
    // heartbeat's mission_active instead (see above).
    renderMission(data);
    return;
  }
  if (sub === 'cmd_ack') { handleCmdAck(data); return; }
  if (sub === 'maps') {
    S.availableMaps = data.files || [];
    S.activeMap = data.active || null;
    renderMapSelect();
    updatePlanControlsEnabled();
    return;
  }
}

function renderMapSelect() {
  const sel = document.getElementById('planMapSelect');
  if (!sel) return;
  if (!S.availableMaps.length) {
    sel.innerHTML = '<option value="">No map pre-staged — upload via local WiFi first</option>';
    return;
  }
  sel.innerHTML = S.availableMaps.map(f =>
    `<option value="${f}" ${f === S.activeMap ? 'selected' : ''}>${f}${f === S.activeMap ? ' (active)' : ''}</option>`
  ).join('');
}

function selectMapRemote(filename) {
  if (!filename) return;
  sendCloudCommand('select_map', { filename })
    .then(() => logLine('ok', `Active map set to ${filename} — confirmed by drone.`))
    .catch(e => alert('Could not select map: ' + e.message));
}

// `data.ts` is when cloud_relay.py published this message to the broker
// (set right before the MQTT publish call) — comparing it to this browser's
// clock when the message arrives gives the real broker + WAN + render
// latency for this specific deployment, not a generic estimate. Assumes the
// Jetson's clock is roughly correct (NTP) — see DESIGN.md §9 clock sync.
// Lightly smoothed (EMA) since single-message jitter is noisy.
let _latEma = null;
function updateLatency(publishedTs) {
  if (!publishedTs) return;
  const ms = Date.now() - publishedTs * 1000;
  if (ms < -2000 || ms > 60000) return; // clock skew or a stale/replayed message — don't show nonsense
  _latEma = _latEma === null ? ms : _latEma * 0.7 + ms * 0.3;
  setTel('telLatency', Math.max(0, Math.round(_latEma)), 'ms');
}

function registerDrone(droneId, sub, data) {
  const now = Date.now();
  const existing = S.drones.get(droneId) || {};
  existing.lastSeen = now;
  if (sub === 'status' || sub === 'lwt') {
    existing.status = data.status;
    if (typeof data.mission_active === 'boolean') existing.missionActive = data.mission_active;
  }
  S.drones.set(droneId, existing);
  refreshDronePicker();
  if (!S.selectedDrone) selectDrone(droneId);
}

function refreshDronePicker() {
  const picker = document.getElementById('dronePicker');
  const current = picker.value;
  picker.innerHTML = '';
  for (const id of S.drones.keys()) {
    const opt = document.createElement('option');
    opt.value = id;
    opt.textContent = id;
    picker.appendChild(opt);
  }
  if (current && S.drones.has(current)) picker.value = current;
  else if (S.selectedDrone) picker.value = S.selectedDrone;
}

function selectDrone(id) {
  S.selectedDrone = id;
  document.getElementById('liveDroneName').textContent = id;
  document.getElementById('logBody').innerHTML = '';
  refreshDronePicker();
  const d = S.drones.get(id);
  setTel('telLink', (d && d.status === 'online') ? 'ONLINE' : 'OFFLINE');
  _latEma = null;
  setTel('telLatency', '—');
  resetMap();
  S.plan.waypoints = [];
  S.plan.approxStart = null;
  setPlanPinMode('waypoint');
  // Seed from the last status heartbeat we've seen for this drone, if any —
  // that's the authoritative live signal (see onMqttMessage); don't infer
  // it from the mission topic, which stays retained long after a mission ends.
  S.missionActive = !!(d && d.missionActive);
  const cachedMaps = S.mapsByDrone.get(id);
  S.availableMaps = cachedMaps ? (cachedMaps.files || []) : [];
  S.activeMap = cachedMaps ? (cachedMaps.active || null) : null;
  renderMapSelect();
  updatePlanControlsEnabled();
  const cachedMission = S.missions.get(id);
  if (cachedMission) renderMission(cachedMission);
}

/* ---------- rendering (mirrors static/app.js) ---------- */
function handleEvent(m) {
  if (m.kind === 'heartbeat') return;
  if (m.kind === 'telem') { updateTelemetry(m); return; }
  if (m.kind === 'mission_complete' || m.kind === 'abort' || m.kind === 'engine_stopped') {
    S.missionActive = false;
    updatePlanControlsEnabled();
  }
  routeEvent(m);
}

function updateLastSeen(ts) {
  if (!ts) return;
  const d = new Date(ts * 1000);
  setTel('telLastSeen', d.toLocaleTimeString('en-GB', { hour12: false }));
}

function updateTelemetry(t) {
  if (t.phase === 'takeoff') {
    document.getElementById('livePhase').textContent = `Vertical climb — ${(t.agl_m || 0).toFixed(0)} / ${t.target_alt} m`;
    setTel('telAgl', (t.agl_m || 0).toFixed(0), 'm');
    return;
  }
  if (t.phase === 'landing') {
    document.getElementById('livePhase').textContent = `Descending — ${(t.agl_m || 0).toFixed(0)} m`;
    setTel('telAgl', (t.agl_m || 0).toFixed(0), 'm');
    return;
  }
  if (t.phase !== 'nav') return;
  document.getElementById('livePhase').textContent = `${t.state} · leg ${t.leg}/${t.total_legs}`;
  setTel('telState', t.state);
  setTel('telLeg', `${t.leg}/${t.total_legs}`);
  setTel('telAgl', (t.agl_m || 0).toFixed(0), 'm');
  setTel('telDist', (t.dist_to_target_m || 0).toFixed(0), 'm');
  setTel('telCte', (t.cte_m || 0).toFixed(1), 'm', Math.abs(t.cte_m) > 15 ? 'warn' : '');
  setTel('telHdg', (t.heading_deg || 0).toFixed(0), '°');
  updateMapPosition(t);
}

function setTel(id, val, unit = '', cls = '') {
  const el = document.getElementById(id);
  if (!el) return;
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
  else if (k === 'proc_log') cls = 'setup';

  logLine(cls, formatEvent(m));

  if (k === 'phase') document.getElementById('livePhase').textContent = phaseLabel(m.phase);
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
    case 'waypoint_reached': return `◎ waypoint ${m.leg}/${m.total} reached`;
    case 'gps_takeover': return `⚠ GPS takeover · EKF↔GPS err ${m.error_m?.toFixed(1)}m`;
    case 'gps_reset': return `⚠ GPS reset #${m.count}`;
    case 'landing': return `landing · ${m.dist_to_final?.toFixed(1)}m to final`;
    case 'landed': return `✓ LANDED · AGL ${m.agl?.toFixed(1)}m`;
    case 'safety_loiter': return `⚠ SAFETY → LOITER · ${m.reason}`;
    case 'ap_status': return `[AP] ${m.text}`;
    case 'timeout': return `⚠ timeout waiting for ${m.desc}`;
    case 'abort': return `✗ ABORT · ${m.reason}`;
    case 'mission_complete': return `✓ MISSION COMPLETE · ${m.frames} frames · ${m.gps_resets} GPS resets`;
    case 'engine_stopped': return 'engine stopped';
    case 'proc_log': return `[${m.stream}] ${m.line}`;
    case 'mission_config': return `» mission started · mode=${m.mode} · ${(m.waypoints || []).length} waypoint(s)`;
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

loadSavedConn();
