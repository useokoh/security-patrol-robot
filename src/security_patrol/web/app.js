// SENTINEL 관제 앱 (1단계: 레이더 화면, 2단계: 수동 주행, 3단계: 코스 선택·자동 순찰)
// 서버(app_server)가 WebSocket(/ws)으로 지도(map)와 상태(state)를 보낸다.
//   state.scan   : 로봇 기준 라이다 점 [[x, y], ...]  (x 앞, y 왼쪽, m)
//   state.robot  : 지도 기준 로봇 위치 {x, y, yaw}  (위치추정 전에는 null)
//   state.manual : 수동 주행 {owner, linear, angular, max_linear, blocked, reason}
//   state.patrol_app : 앱이 켠 순찰 {running, course, laps, stopping, owner('app'|'external'|null), last}
// 앱 → 서버: {type:'drive', lin, ang} (-1~1, 누르고 있는 동안 0.1초마다), {type:'stop'},
//            {type:'patrol_start', course, laps}, {type:'patrol_stop'}, {type:'courses'} (목록 새로 고침)
'use strict';

const COLORS = {
  unknown: [11, 18, 32], free: [24, 36, 60], wall: [104, 124, 160],
  scan: '#5fd0ff', robot: '#34d399', caution: '#f5a524', urgent: '#f87171', ring: 'rgba(95, 208, 255, 0.18)',
};
const RADAR_RANGES = [1, 2, 4, 8];          // 레이더 화면 반지름 단계 [m]

const state = {
  map: null, mapImage: null, last: null, lastAt: 0, connected: false, myId: null,
  courses: [], selectedCourse: null, laps: 0,
  mode: 'radar', radarIndex: 2, mapZoom: 1, pan: { x: 0, y: 0 },
};

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const $ = (id) => document.getElementById(id);

// ---------- 연결 ----------

let socket = null;
function connect() {
  socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  socket.onopen = () => { state.connected = true; };
  socket.onclose = () => { state.connected = false; state.myId = null; releaseAll(); setTimeout(connect, 2000); };
  socket.onerror = () => socket.close();
  socket.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'hello') state.myId = msg.id;
    else if (msg.type === 'courses') setCourses(msg.courses);
    else if (msg.type === 'notice') showToast(msg.text, msg.level);
    else if (msg.type === 'map') setMap(msg);
    else if (msg.type === 'state') { state.last = msg; state.lastAt = performance.now(); }
  };
}

function send(msg) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(msg));
}

// 지도(OccupancyGrid)를 화면용 그림으로 한 번만 만들어 둔다
function setMap(msg) {
  const raw = atob(msg.data);
  const img = new ImageData(msg.width, msg.height);
  for (let i = 0; i < raw.length; i++) {
    const v = raw.charCodeAt(i);
    let c;
    if (v === 255) c = COLORS.unknown;
    else {
      const t = Math.min(v, 100) / 100;          // 0 빈칸 ~ 1 벽
      c = COLORS.free.map((f, k) => Math.round(f + (COLORS.wall[k] - f) * t));
    }
    img.data.set([c[0], c[1], c[2], 255], i * 4);
  }
  const off = document.createElement('canvas');
  off.width = msg.width; off.height = msg.height;
  off.getContext('2d').putImageData(img, 0, 0);
  state.map = msg; state.mapImage = off;
}

// ---------- 그리기 ----------

function resize() {
  const r = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(r.width * dpr); canvas.height = Math.round(r.height * dpr);
}

function drawMapImage() {
  if (!state.mapImage) return;
  const m = state.map, res = m.resolution;
  ctx.save();
  ctx.translate(m.origin[0], m.origin[1]);
  ctx.rotate(m.origin[2]);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(state.mapImage, 0, 0, m.width * res, m.height * res);
  ctx.restore();
}

// 로봇 기준 좌표계에서 그리기: 라이다 점과 로봇 모양
function drawScan(points, size) {
  ctx.fillStyle = COLORS.scan;
  for (const [x, y] of points) ctx.fillRect(x - size / 2, y - size / 2, size, size);
}

// 로봇 위에서 본 모습 (로봇 기준 좌표: x 앞, y 왼쪽, m). 실제 크기 0.26 x 0.20 m.
// 멀리 축소해도 화면에서 26픽셀보다 작아지지 않게 키운다.
function roundRect(x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y); ctx.lineTo(x + w - r, y); ctx.arcTo(x + w, y, x + w, y + r, r);
  ctx.lineTo(x + w, y + h - r); ctx.arcTo(x + w, y + h, x + w - r, y + h, r);
  ctx.lineTo(x + r, y + h); ctx.arcTo(x, y + h, x, y + h - r, r);
  ctx.lineTo(x, y + r); ctx.arcTo(x, y, x + r, y, r);
  ctx.closePath();
}

function drawRobot(px) {
  const dpr = window.devicePixelRatio || 1;
  const sc = Math.max(1, (26 * dpr * px) / 0.26);
  const lw = px / sc;                                   // 화면 1픽셀 두께
  ctx.save();
  ctx.scale(sc, sc);
  // 바닥에 은은한 빛
  const glow = ctx.createRadialGradient(0, 0, 0.03, 0, 0, 0.32);
  glow.addColorStop(0, 'rgba(52, 211, 153, 0.30)'); glow.addColorStop(1, 'rgba(52, 211, 153, 0)');
  ctx.fillStyle = glow; ctx.beginPath(); ctx.arc(0, 0, 0.32, 0, Math.PI * 2); ctx.fill();
  // 바퀴 4개
  ctx.fillStyle = '#0b1220'; ctx.strokeStyle = '#64748b'; ctx.lineWidth = lw;
  for (const wx of [0.075, -0.075]) {
    for (const wy of [0.1, -0.1]) { roundRect(wx - 0.036, wy - 0.016, 0.072, 0.032, 0.012); ctx.fill(); ctx.stroke(); }
  }
  // 몸체: 앞쪽이 밝은 청록 그라디언트
  const body = ctx.createLinearGradient(0.13, 0, -0.13, 0);
  body.addColorStop(0, '#2dd4a7'); body.addColorStop(1, '#0f5f4c');
  ctx.fillStyle = body; ctx.strokeStyle = '#a7f3d0'; ctx.lineWidth = 1.5 * lw;
  roundRect(-0.13, -0.085, 0.26, 0.17, 0.045); ctx.fill(); ctx.stroke();
  // 뒤쪽 홈 무늬
  ctx.strokeStyle = 'rgba(11, 18, 32, 0.45)'; ctx.lineWidth = 1.2 * lw;
  for (const gx of [-0.09, -0.105]) { ctx.beginPath(); ctx.moveTo(gx, -0.045); ctx.lineTo(gx, 0.045); ctx.stroke(); }
  // 앞 범퍼 불빛
  ctx.fillStyle = '#e6fffa'; roundRect(0.108, -0.055, 0.014, 0.11, 0.007); ctx.fill();
  // 라이다: 어두운 원 + 돌아가는 빛
  ctx.fillStyle = '#0b1220'; ctx.strokeStyle = '#5fd0ff'; ctx.lineWidth = 1.2 * lw;
  ctx.beginPath(); ctx.arc(0.01, 0, 0.042, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  const a = (performance.now() / 300) % (Math.PI * 2);
  ctx.fillStyle = 'rgba(95, 208, 255, 0.35)';
  ctx.beginPath(); ctx.moveTo(0.01, 0); ctx.arc(0.01, 0, 0.036, a - 0.6, a); ctx.closePath(); ctx.fill();
  ctx.fillStyle = '#5fd0ff'; ctx.beginPath(); ctx.arc(0.01, 0, 0.012, 0, Math.PI * 2); ctx.fill();
  // 진행 방향 표시 (몸 앞 작은 화살표)
  ctx.fillStyle = '#34d399';
  ctx.beginPath(); ctx.moveTo(0.215, 0); ctx.lineTo(0.16, 0.04); ctx.lineTo(0.172, 0); ctx.lineTo(0.16, -0.04); ctx.closePath(); ctx.fill();
  ctx.restore();
}

// 수동 주행 중 앞/뒤가 막혀 멈췄으면 막힌 구역을 빨갛게 (서버 guard_speed 와 같은 크기)
function drawBlocked(m) {
  if (!m || !m.blocked) return;
  const x0 = m.blocked === 'front' ? 0.13 : -0.28;
  ctx.fillStyle = 'rgba(248, 113, 113, 0.35)';
  ctx.fillRect(x0, -0.14, 0.15, 0.28);
}

// 지도 기준 좌표계에서 그리기: 감지 물체
function drawObjects(objects, px, labels) {
  for (const o of objects || []) {
    if (o.x === undefined) continue;
    const color = o.moving ? COLORS.urgent : COLORS.caution;
    const r = Math.max((o.width || 0.2) / 2, 0.12);
    ctx.globalAlpha = o.visible ? 1 : 0.35;
    ctx.fillStyle = color + '55'; ctx.strokeStyle = color; ctx.lineWidth = 2 * px;
    ctx.setLineDash(o.visible ? [] : [6 * px, 4 * px]);
    ctx.beginPath(); ctx.arc(o.x, o.y, r, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.setLineDash([]); ctx.globalAlpha = 1;
    labels.push({ x: o.x, y: o.y + r, text: `#${o.id}${o.person ? ' 사람 후보' : ''}`, color });
  }
}

function drawLabels(labels, toScreen) {
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.font = `${12 * (window.devicePixelRatio || 1)}px -apple-system, sans-serif`;
  ctx.textAlign = 'center';
  for (const l of labels) {
    const [sx, sy] = toScreen(l.x, l.y);
    ctx.textBaseline = l.center ? 'middle' : 'alphabetic';
    ctx.font = `${l.center ? 'bold ' : ''}${(l.center ? 10 : 12) * (window.devicePixelRatio || 1)}px -apple-system, sans-serif`;
    ctx.fillStyle = l.color; ctx.fillText(l.text, sx, l.center ? sy : sy - 6 * (window.devicePixelRatio || 1));
  }
  ctx.textBaseline = 'alphabetic';
}

// 지도 기준 좌표계에서 그리기: 순찰 코스 (점선 경로 + 번호 원). 지금 가는 지점은 주황색으로 깜박인다
function drawCourse(course, current, px, labels) {
  if (!course || !course.points.length) return;
  const pts = course.points, dpr = window.devicePixelRatio || 1;
  const r = Math.max(0.07, 9 * dpr * px);
  ctx.strokeStyle = 'rgba(95, 208, 255, 0.45)'; ctx.lineWidth = 2 * dpr * px;
  ctx.setLineDash([6 * dpr * px, 5 * dpr * px]);
  ctx.beginPath();
  pts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
  ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
  pts.forEach((p, i) => {
    const now = i === current;
    if (now) {                                          // 깜박이는 바깥 고리
      const pulse = 0.5 + 0.5 * Math.sin(performance.now() / 250);
      ctx.strokeStyle = `rgba(245, 165, 36, ${0.3 + 0.5 * pulse})`; ctx.lineWidth = 3 * dpr * px;
      ctx.beginPath(); ctx.arc(p.x, p.y, r * (1.5 + 0.3 * pulse), 0, Math.PI * 2); ctx.stroke();
    }
    ctx.fillStyle = now ? '#f5a524' : '#12305a'; ctx.strokeStyle = now ? '#ffd79a' : '#5fd0ff';
    ctx.lineWidth = 1.5 * dpr * px;
    ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    labels.push({ x: p.x, y: p.y, text: String(i + 1), color: now ? '#1a1205' : '#e8eef9', center: true });
  });
}

// 한 번 그리다 오류가 나도 다음 화면은 계속 그린다
function render() {
  try {
    const r = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    if (Math.round(r.width * dpr) !== canvas.width || Math.round(r.height * dpr) !== canvas.height) resize();
    if (canvas.width > 80 && canvas.height > 80) draw();
    updateStatus();
  } catch (e) {
    console.error(e);
  }
  requestAnimationFrame(render);
}

function draw() {
  const W = canvas.width, H = canvas.height, dpr = window.devicePixelRatio || 1;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const s = state.last, robot = s && s.robot;
  const labels = [];

  if (state.mode === 'radar') {
    // 로봇 중심, 로봇 앞쪽이 화면 위. 화면 = (cx - y*k, cy - x*k)
    const range = RADAR_RANGES[state.radarIndex], cx = W / 2, cy = H / 2;
    const k = (Math.min(W, H) / 2 - 16 * dpr) / range, px = 1 / k;
    // 거리 링
    ctx.strokeStyle = COLORS.ring; ctx.lineWidth = dpr; ctx.font = `${11 * dpr}px sans-serif`;
    ctx.fillStyle = 'rgba(143, 161, 194, 0.8)'; ctx.textAlign = 'left';
    const stepM = range <= 2 ? 0.5 : range <= 4 ? 1 : 2;
    for (let m = stepM; m <= range + 1e-6; m += stepM) {
      ctx.beginPath(); ctx.arc(cx, cy, m * k, 0, Math.PI * 2); ctx.stroke();
      ctx.fillText(`${m} m`, cx + 4 * dpr, cy - m * k - 3 * dpr);
    }
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke();

    ctx.setTransform(0, -k, -k, 0, cx, cy);          // 로봇 기준 좌표계
    if (robot && state.mapImage) {                   // 지도도 로봇 기준으로 돌려서 흐리게
      ctx.save();
      ctx.globalAlpha = 0.55;
      ctx.rotate(-robot.yaw); ctx.translate(-robot.x, -robot.y);
      drawMapImage();
      ctx.globalAlpha = 1;
      drawCourse(activeCourse(), currentPoint(), px, labels);
      drawObjects(s.objects, px, labels);
      ctx.restore();
    }
    if (s) drawScan(s.scan, 3 * dpr * px);
    drawBlocked(s && s.manual);
    drawRobot(px);
    const toScreen = (x, y) => {                     // 지도 좌표 → 화면 (라벨용)
      const dx = x - robot.x, dy = y - robot.y, c = Math.cos(-robot.yaw), n = Math.sin(-robot.yaw);
      const rx = c * dx - n * dy, ry = n * dx + c * dy;
      return [cx - ry * k, cy - rx * k];
    };
    if (robot) drawLabels(labels, toScreen);
    $('zoomLabel').textContent = `${range} m`;
  } else {
    // 지도 화면: 지도 전체가 보이게 맞춘 뒤 확대·이동
    const m = state.map;
    let vx = 0, vy = 0, k = 60 * dpr;
    if (m) {
      const w = m.width * m.resolution, h = m.height * m.resolution;
      vx = m.origin[0] + w / 2; vy = m.origin[1] + h / 2;
      k = Math.min(W / (w + 0.6), H / (h + 0.6));
    } else if (robot) { vx = robot.x; vy = robot.y; }
    k *= state.mapZoom; vx -= state.pan.x; vy -= state.pan.y;
    const px = 1 / k;
    ctx.setTransform(k, 0, 0, -k, W / 2 - vx * k, H / 2 + vy * k);
    drawMapImage();
    drawCourse(activeCourse(), currentPoint(), px, labels);
    drawObjects(s && s.objects, px, labels);
    if (robot) {
      ctx.save(); ctx.translate(robot.x, robot.y); ctx.rotate(robot.yaw);
      drawScan(s.scan, 3 * dpr * px);
      drawBlocked(s.manual);
      drawRobot(px);
      ctx.restore();
    }
    drawLabels(labels, (x, y) => [W / 2 + (x - vx) * k, H / 2 - (y - vy) * k]);
    $('zoomLabel').textContent = `×${state.mapZoom.toFixed(1)}`;
  }
}

// ---------- 상태 글자 ----------

const PATROL_STATES = { moving: '이동 중', watching: '감시 중', skipped: '건너뜀', stopped: '정지' };

function setChip(id, text, cls) {
  const el = $(id);
  el.querySelector('b').textContent = text;
  el.classList.remove('ok', 'bad', 'warn');
  if (cls) el.classList.add(cls);
}

function updateStatus() {
  const s = state.last, fresh = s && performance.now() - state.lastAt < 2000;
  setChip('conn', !state.connected ? '연결 끊김' : fresh ? '연결됨' : '데이터 없음', state.connected && fresh ? 'ok' : 'bad');
  setChip('battery', s && s.battery != null ? `${s.battery.toFixed(2)} V` : '-', s && s.battery != null && s.battery < 7.0 ? 'warn' : '');
  setChip('wifi', s && s.rssi != null ? `${s.rssi} dBm` : '-', s && s.rssi != null && s.rssi < -75 ? 'warn' : '');
  setChip('loc', s ? (s.localized ? '추정 중' : '모름') : '-', s && !s.localized ? 'warn' : '');

  const banner = $('banner');
  let warn = '';
  if (state.connected && !fresh) warn = '로봇 데이터가 오지 않습니다';
  else if (s && s.scan.length === 0) warn = '라이다 데이터 없음';
  else if (state.mode === 'map' && s && !s.localized) warn = '위치추정 전이라 지도 위 로봇 위치를 모릅니다';
  banner.hidden = !warn; banner.textContent = warn;

  const p = s && s.patrol, running = s && s.patrol_running;
  const where = p ? ` · ${p.lap}바퀴 · ${p.index + 1}번째 지점 <b>${p.name}</b>` : '';
  if (running) $('patrol').innerHTML = p ? `순찰 <b>${PATROL_STATES[p.state] || p.state}</b>${where}` : '순찰 <b>시작 중</b>';
  else $('patrol').innerHTML = `순찰 <b>꺼짐</b>${p ? ' (마지막' + where + ')' : ''}`;
  updateDrive(s, fresh);
  updateAuto(s, fresh);
}

// 수동 주행 탭 글자와 조이스틱 잠금
const STOP_REASONS = { timeout: '명령이 끊겨 자동으로 멈췄습니다', disconnected: '조종하던 화면의 연결이 끊겨 멈췄습니다',
  patrol: '순찰이 켜져서 수동 주행을 멈췄습니다' };

function updateDrive(s, fresh) {
  const m = s && s.manual, mine = m && m.owner != null && m.owner === state.myId;
  let text = '조이스틱으로 조종하세요', cls = '', locked = false;
  if (!state.connected || !fresh) { text = '로봇과 연결되어 있지 않습니다'; cls = 'bad'; locked = true; }
  else if (s.patrol_running && s.patrol_app && s.patrol_app.owner === 'app') { text = '자동 순찰 중입니다'; cls = 'warn'; locked = true; }
  else if (s.patrol_running) { text = '터미널에서 켠 순찰 중에는 수동 주행을 할 수 없습니다'; cls = 'warn'; locked = true; }
  else if (m && m.owner != null && !mine) { text = '다른 화면에서 조종 중입니다'; cls = 'warn'; locked = true; }
  else if (mine && m.blocked === 'front') { text = '앞이 막혀 전진을 멈췄습니다 (회전·후진 가능)'; cls = 'bad'; }
  else if (mine && m.blocked === 'back') { text = '뒤가 막혀 후진을 멈췄습니다 (회전·전진 가능)'; cls = 'bad'; }
  else if (mine) { text = '조종 중'; cls = 'ok'; }
  else if (m && STOP_REASONS[m.reason]) { text = STOP_REASONS[m.reason]; cls = 'warn'; }
  const el = $('driveStatus');
  el.textContent = text; el.className = 'drive-status ' + cls;
  $('stick').classList.toggle('locked', locked);
  $('stopPatrolForManual').hidden = !(s && s.patrol_app && s.patrol_app.owner === 'app' && !s.patrol_app.stopping);
  if (locked && joy.active) releaseStick();
  $('driveSpeed').textContent = m
    ? `지금 ${(m.blocked ? 0 : m.linear).toFixed(2)} m/s · 회전 ${m.angular.toFixed(2)}`
    : '-';
  updateSpeedLabel();
}

// ---------- 조작 ----------

document.querySelectorAll('#viewMode button').forEach((b) => b.addEventListener('click', () => {
  state.mode = b.dataset.mode;
  document.querySelectorAll('#viewMode button').forEach((x) => x.classList.toggle('on', x === b));
}));
$('zoomIn').addEventListener('click', () => {
  if (state.mode === 'radar') state.radarIndex = Math.max(0, state.radarIndex - 1);
  else state.mapZoom = Math.min(8, state.mapZoom * 1.5);
});
$('zoomOut').addEventListener('click', () => {
  if (state.mode === 'radar') state.radarIndex = Math.min(RADAR_RANGES.length - 1, state.radarIndex + 1);
  else { state.mapZoom = Math.max(0.5, state.mapZoom / 1.5); if (state.mapZoom <= 1) state.pan = { x: 0, y: 0 }; }
});
document.querySelectorAll('#tabs button').forEach((b) => b.addEventListener('click', () => {
  if (b.dataset.tab !== 'manual') releaseAll();          // 수동 주행 탭을 떠나면 멈춘다
  document.querySelectorAll('#tabs button').forEach((x) => x.classList.toggle('on', x === b));
  document.querySelectorAll('.tab-body').forEach((x) => { x.hidden = x.dataset.body !== b.dataset.tab; });
}));

// ---------- 수동 주행 ----------
// 누르고 있는 동안 0.1초마다 값을 보낸다. 손을 떼면 바로 정지를 보내고,
// 정지 신호가 유실돼도 서버가 0.5초 동안 값을 못 받으면 스스로 멈춘다.

const DEAD_ZONE = 0.10;                    // 조이스틱 가운데 작은 흔들림은 무시
const CURVE = 2.0;                         // 감도 곡선: 반쯤 밀면 25% 속도 → 가운데 근처를 섬세하게
const RAMP_STEP = 0.15;                    // 0.1초마다 늘어나는 최대 값 (0 → 최고 속도 약 0.7초). 줄이는 건 바로
const KEYMAP = { ArrowUp: [1, 0], KeyW: [1, 0], ArrowDown: [-1, 0], KeyS: [-1, 0],
  ArrowLeft: [0, 1], KeyA: [0, 1], ArrowRight: [0, -1], KeyD: [0, -1] };     // [앞뒤, 회전]
const joy = { active: false, pointerId: null, lin: 0, ang: 0 };
const keys = new Set();
let speedLevel = 0.5;                      // 속도 슬라이더 10~100% (서버 최대 속도에 곱함)
const out = { lin: 0, ang: 0 };            // 실제로 보내는 값 (서서히 따라감)
const stick = $('stick');

function shape(v) {                        // 데드존을 뺀 나머지를 0~1 로 펼친 뒤 곡선을 준다
  const a = Math.abs(v);
  if (a < DEAD_ZONE) return 0;
  return Math.sign(v) * Math.pow(Math.min(1, (a - DEAD_ZONE) / (1 - DEAD_ZONE)), CURVE);
}

// 목표 값으로 서서히 다가간다. 줄이거나 멈추는 건 바로, 방향이 바뀌면 먼저 0 으로 (안전)
function ramp(cur, target) {
  if (cur !== 0 && Math.sign(target) !== Math.sign(cur)) cur = 0;
  if (Math.abs(target) <= Math.abs(cur)) return target;
  return cur + Math.sign(target) * Math.min(RAMP_STEP, Math.abs(target - cur));
}

function moveKnob(e) {
  const r = stick.getBoundingClientRect(), R = r.width / 2 - 31;   // 손잡이가 테두리 안에 머물게
  let dx = e.clientX - (r.left + r.width / 2), dy = e.clientY - (r.top + r.height / 2);
  const d = Math.hypot(dx, dy);
  if (d > R) { dx *= R / d; dy *= R / d; }
  $('knob').style.transform = `translate(${dx}px, ${dy}px)`;
  joy.lin = shape(-dy / R);                // 위로 밀면 앞으로
  joy.ang = shape(-dx / R);                // 왼쪽으로 밀면 왼쪽으로 회전 (반시계)
}

function driving() { return joy.active || keys.size > 0; }

function currentCommand() {
  if (joy.active) return [joy.lin, joy.ang];
  let lin = 0, ang = 0;
  for (const k of keys) { lin += KEYMAP[k][0]; ang += KEYMAP[k][1]; }
  return [Math.max(-1, Math.min(1, lin)), Math.max(-1, Math.min(1, ang))];
}

function sendDrive() {
  if (!driving()) return;
  const [lin, ang] = currentCommand();
  out.lin = ramp(out.lin, lin); out.ang = ramp(out.ang, ang);
  send({ type: 'drive', lin: out.lin * speedLevel, ang: out.ang * speedLevel });
}

function stopNow() { out.lin = out.ang = 0; send({ type: 'stop' }); }

function releaseStick() {
  if (!joy.active) return;
  joy.active = false; joy.pointerId = null; joy.lin = joy.ang = 0;
  stick.classList.remove('active');
  $('knob').style.transform = '';
  if (!driving()) stopNow();
}

function releaseAll() {
  const was = driving();
  keys.clear();
  if (joy.active) releaseStick();
  else if (was) stopNow();
}

stick.addEventListener('pointerdown', (e) => {
  if (stick.classList.contains('locked') || joy.active) return;
  e.preventDefault();
  stick.setPointerCapture(e.pointerId);
  joy.active = true; joy.pointerId = e.pointerId;
  stick.classList.add('active');
  if (navigator.vibrate) navigator.vibrate(12);              // 안드로이드: 잡았다는 짧은 진동
  moveKnob(e);
  sendDrive();
});
stick.addEventListener('pointermove', (e) => { if (joy.active && e.pointerId === joy.pointerId) moveKnob(e); });
['pointerup', 'pointercancel', 'lostpointercapture'].forEach((type) => stick.addEventListener(type, (e) => {
  if (e.pointerId === joy.pointerId) releaseStick();
}));

window.addEventListener('keydown', (e) => {
  if (e.code === 'Space') { e.preventDefault(); releaseAll(); stopNow(); return; }
  if (e.code === 'BracketLeft' || e.code === 'BracketRight') {           // [ ] 로 속도 10% 씩
    setSpeed(speedLevel * 100 + (e.code === 'BracketRight' ? 10 : -10)); return;
  }
  const manualOpen = !document.querySelector('[data-body="manual"]').hidden;
  if (!KEYMAP[e.code] || !manualOpen || stick.classList.contains('locked')) return;
  e.preventDefault();
  if (!keys.has(e.code)) { keys.add(e.code); sendDrive(); }
});
window.addEventListener('keyup', (e) => {
  if (!keys.delete(e.code)) return;
  if (driving()) sendDrive(); else stopNow();
});
window.addEventListener('blur', releaseAll);                  // 다른 창으로 가면 키가 눌린 채 남지 않게
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });

$('estop').addEventListener('pointerdown', (e) => { e.preventDefault(); releaseAll(); stopNow(); });
function setSpeed(percent) {
  const p = Math.max(10, Math.min(100, Math.round(percent / 10) * 10));
  speedLevel = p / 100;
  $('speedRange').value = p;
  $('speedRange').style.setProperty('--fill', `${((p - 10) / 90) * 100}%`);
  try { localStorage.setItem('sentinel.speed', String(p)); } catch (e) { /* 저장 안 돼도 동작 */ }
  updateSpeedLabel();
}
function updateSpeedLabel() {
  const m = state.last && state.last.manual, p = Math.round(speedLevel * 100);
  $('speedLabel').textContent = m ? `${p}% · ${(m.max_linear * speedLevel).toFixed(2)} m/s` : `${p}%`;
}
$('speedRange').addEventListener('input', (e) => setSpeed(Number(e.target.value)));
let savedSpeed = 50;
try { savedSpeed = Number(localStorage.getItem('sentinel.speed')) || 50; } catch (e) { /* 기본값 */ }
setSpeed(savedSpeed);
setInterval(sendDrive, 100);

// ---------- 자동 주행 (코스 선택·순찰 시작/정지) ----------

const PATROL_WORDS = { moving: '이동 중', watching: '감시 중', skipped: '건너뜀', stopped: '중지됨' };
let coursesKey = '';

function courseById(id) { return state.courses.find((c) => c.id === id) || null; }

// 순찰 중이면 그 코스, 아니면 고른 코스 (지도에 미리 보여 준다)
function activeCourse() {
  const pa = state.last && state.last.patrol_app;
  if (pa && pa.running && pa.course) return courseById(pa.course);
  if (state.last && state.last.patrol_running) return null;    // 터미널에서 켠 순찰: 코스를 모름
  return courseById(state.selectedCourse);
}

function currentPoint() {
  const s = state.last;
  return s && s.patrol_running && s.patrol && s.patrol.index != null ? s.patrol.index : -1;
}

function setCourses(list) {
  state.courses = list || [];
  let saved = null;
  try { saved = localStorage.getItem('sentinel.course'); } catch (e) { /* 저장 안 돼도 동작 */ }
  if (!courseById(state.selectedCourse)) state.selectedCourse = courseById(saved) ? saved : (state.courses[0] || {}).id || null;
  coursesKey = '';                                              // 목록을 다시 그리게
}

function renderCourses(disabled) {
  const key = JSON.stringify([state.courses.map((c) => c.id), state.selectedCourse, disabled]);
  if (key === coursesKey) return;                               // 바뀐 것이 없으면 다시 만들지 않는다
  coursesKey = key;
  const box = $('courses');
  box.innerHTML = '';
  if (!state.courses.length) {
    box.innerHTML = '<p class="soon">코스 파일이 없습니다. route_recorder 로 순찰 지점을 기록하세요.</p>';
    return;
  }
  for (const c of state.courses) {
    const b = document.createElement('button');
    b.className = 'course' + (c.id === state.selectedCourse ? ' on' : '');
    b.disabled = disabled;
    const name = document.createElement('b'); name.textContent = c.name;
    const info = document.createElement('span'); info.textContent = `지점 ${c.points.length}개 · ${c.dwell}초 감시`;
    b.append(name, info);
    b.addEventListener('click', () => {
      state.selectedCourse = c.id;
      try { localStorage.setItem('sentinel.course', c.id); } catch (e) { /* 저장 안 돼도 동작 */ }
      coursesKey = '';
    });
    box.append(b);
  }
}

function updateAuto(s, fresh) {
  const pa = (s && s.patrol_app) || {}, running = !!(s && s.patrol_running), mine = pa.owner === 'app';
  renderCourses(running);
  document.querySelectorAll('#lapChoice button').forEach((b) => { b.disabled = running; });

  const btn = $('patrolBtn');
  btn.classList.toggle('stop', mine);
  if (!state.connected || !fresh) { btn.textContent = '연결 안 됨'; btn.disabled = true; }
  else if (mine && pa.stopping) { btn.textContent = '멈추는 중…'; btn.disabled = true; }
  else if (mine) { btn.textContent = '순찰 정지'; btn.disabled = false; }
  else if (running) { btn.textContent = '터미널에서 켠 순찰이 도는 중'; btn.disabled = true; }
  else { btn.textContent = '순찰 시작'; btn.disabled = !state.selectedCourse; }

  // 진행 상황: n바퀴째 · k/N 지점, 막대는 정한 바퀴 전체(계속 반복이면 한 바퀴) 기준
  const course = activeCourse(), p = s && s.patrol;
  let text = '', ratio = 0;
  if (running && p && course) {
    const total = course.points.length, laps = pa.laps || 0;
    const done = (p.lap - 1) * total + p.index + (p.state === 'moving' ? 0 : 1);
    ratio = laps ? done / (laps * total) : ((p.index + (p.state === 'moving' ? 0 : 1)) / total);
    text = `<b>${p.lap}바퀴째</b>${laps ? ` / ${laps}바퀴` : ' (계속 반복)'} · ${p.index + 1}/${total} 지점 `
      + `'${escapeHtml(p.name)}' ${PATROL_WORDS[p.state] || p.state}`;
  } else if (running) {
    text = mine ? '순찰 준비 중 (위치 확인·Nav2 대기)…' : '터미널에서 켠 순찰 진행 중';
  } else if (pa.last) {
    text = pa.last.stopped ? '지난 순찰: 앱에서 멈춤' : pa.last.code === 0 ? '지난 순찰: 정한 바퀴를 모두 돎' : '지난 순찰: 중간에 멈춤 (연속 실패 등)';
  }
  $('progress').hidden = !text;
  $('progressText').innerHTML = text;
  $('progressBar').style.width = `${Math.round(Math.min(1, Math.max(0, ratio)) * 100)}%`;
}

function escapeHtml(v) {
  return String(v).replace(/[&<>"']/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]));
}

let toastTimer = null;
function showToast(text, level) {
  const el = $('toast');
  el.textContent = text; el.className = 'toast ' + (level || ''); el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.hidden = true; }, 3500);
}

document.querySelectorAll('#lapChoice button').forEach((b) => b.addEventListener('click', () => {
  state.laps = Number(b.dataset.laps);
  document.querySelectorAll('#lapChoice button').forEach((x) => x.classList.toggle('on', x === b));
}));
$('patrolBtn').addEventListener('click', () => {
  const pa = state.last && state.last.patrol_app;
  if (pa && pa.owner === 'app') send({ type: 'patrol_stop' });
  else if (state.selectedCourse) { releaseAll(); send({ type: 'patrol_start', course: state.selectedCourse, laps: state.laps }); }
});
$('stopPatrolForManual').addEventListener('click', () => send({ type: 'patrol_stop' }));
$('refreshCourses').addEventListener('click', () => send({ type: 'courses' }));

// 지도 화면에서 끌어서 이동
let drag = null;
canvas.addEventListener('pointerdown', (e) => { if (state.mode === 'map') drag = { x: e.clientX, y: e.clientY }; });
window.addEventListener('pointerup', () => { drag = null; });
canvas.addEventListener('pointermove', (e) => {
  if (!drag || !state.map) return;
  const m = state.map, dpr = window.devicePixelRatio || 1;
  const k = Math.min(canvas.width / (m.width * m.resolution + 0.6), canvas.height / (m.height * m.resolution + 0.6)) * state.mapZoom;
  state.pan.x += (e.clientX - drag.x) * dpr / k;
  state.pan.y -= (e.clientY - drag.y) * dpr / k;
  drag = { x: e.clientX, y: e.clientY };
});

window.addEventListener('resize', resize);
resize();
connect();
requestAnimationFrame(render);
window.SENTINEL = { send };       // 다음 단계(자동 주행)와 디버깅에서 쓴다
