// Robot Dog 2.0 web teleop. Inputs (touch pads, keyboard, browser gamepad)
// are merged every 50 ms into one normalized drive command. While anything is
// pressed the command is streamed at 20 Hz; the server stops the robot if the
// stream stalls (tab hidden, Wi-Fi drop).
'use strict';

const $ = (id) => document.getElementById(id);
const SEND_HZ = 20;
let ws = null;
let limits = null;
let wasMoving = false;

// ------------------------------------------------------------- connection
function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => setConn(true);
  ws.onclose = () => { setConn(false); setTimeout(connect, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'hello') limits = msg.limits;
    else if (msg.type === 'state') setMode(msg.mode);
    else if (msg.type === 'error') showError(msg.message);
  };
}
function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}
function setConn(on) {
  const el = $('conn');
  el.textContent = on ? 'связь есть' : 'нет связи';
  el.className = 'pill ' + (on ? 'on' : 'off');
}
const MODE_RU = {
  passive: 'выключен', standing_up: 'встаёт', stand: 'стоит', walk: 'идёт',
  lying_down: 'ложится', lying: 'лежит', estop: 'E-STOP', unknown: '—',
};
function setMode(mode) {
  const el = $('mode');
  el.textContent = MODE_RU[mode] || mode;
  el.className = 'pill ' + (mode === 'estop' ? 'estop' : mode === 'walk' ? 'walk' : '');
}
function showError(text) {
  const el = $('error');
  el.textContent = text;
  el.hidden = false;
  setTimeout(() => { el.hidden = true; }, 3000);
}

// ---------------------------------------------------------------- buttons
function estop() { send({ type: 'estop', active: true }); }
$('estop').addEventListener('click', estop);
$('release').addEventListener('click', () => send({ type: 'estop', active: false }));
$('stand').addEventListener('click', () => send({ type: 'command', name: 'stand' }));
$('lie').addEventListener('click', () => send({ type: 'command', name: 'lie' }));

const speed = $('speed');
speed.addEventListener('input', () => { $('speed-out').textContent = speed.value + '%'; });
const height = $('height');
let pitch = 0;
function sendPose() { send({ type: 'pose', pitch, height: height.value / 1000 }); }
height.addEventListener('input', () => {
  $('height-out').textContent = height.value + ' мм';
  sendPose();
});

// ------------------------------------------------------------ touch pads
function makePad(el) {
  const knob = el.querySelector('.knob');
  const state = { x: 0, y: 0, active: false, id: null };
  function update(ev) {
    const r = el.getBoundingClientRect();
    const radius = r.width / 2;
    let dx = (ev.clientX - r.left - radius) / radius;
    let dy = (ev.clientY - r.top - radius) / radius;
    const len = Math.hypot(dx, dy);
    if (len > 1) { dx /= len; dy /= len; }
    state.x = dx; state.y = -dy;  // up = +y
    knob.style.left = `${33 + dx * 33}%`;
    knob.style.top = `${33 + dy * 33}%`;
  }
  function reset() {
    state.x = state.y = 0; state.active = false; state.id = null;
    knob.style.left = knob.style.top = '33%';
  }
  el.addEventListener('pointerdown', (ev) => {
    state.active = true; state.id = ev.pointerId;
    el.setPointerCapture(ev.pointerId);
    update(ev);
  });
  el.addEventListener('pointermove', (ev) => { if (state.active && ev.pointerId === state.id) update(ev); });
  el.addEventListener('pointerup', reset);
  el.addEventListener('pointercancel', reset);
  el.addEventListener('lostpointercapture', reset);
  return state;
}
const movePad = makePad($('pad-move'));
const turnPad = makePad($('pad-turn'));

// --------------------------------------------------------------- keyboard
const keys = new Set();
const KEYMAP = {
  KeyW: 'fwd', ArrowUp: 'fwd', KeyS: 'back', ArrowDown: 'back',
  KeyA: 'left', KeyD: 'right', KeyQ: 'ccw', ArrowLeft: 'ccw', KeyE: 'cw', ArrowRight: 'cw',
};
window.addEventListener('keydown', (ev) => {
  if (ev.target.tagName === 'INPUT') return;
  if (ev.code === 'Escape' || ev.code === 'KeyX') { estop(); ev.preventDefault(); return; }
  if (ev.repeat) { if (KEYMAP[ev.code]) ev.preventDefault(); return; }
  if (ev.code === 'KeyR') send({ type: 'estop', active: false });
  else if (ev.code === 'Digit1') send({ type: 'command', name: 'stand' });
  else if (ev.code === 'Digit2') send({ type: 'command', name: 'lie' });
  if (KEYMAP[ev.code]) { keys.add(KEYMAP[ev.code]); ev.preventDefault(); }
  if (ev.key === 'Shift') keys.add('turbo');
});
window.addEventListener('keyup', (ev) => {
  if (KEYMAP[ev.code]) keys.delete(KEYMAP[ev.code]);
  if (ev.key === 'Shift') keys.delete('turbo');
});
window.addEventListener('blur', () => keys.clear());
document.addEventListener('visibilitychange', () => { if (document.hidden) keys.clear(); });

// ---------------------------------------------------------- browser gamepad
// Standard mapping: axes 0/1 left stick, 2/3 right stick (down = +),
// buttons 0 A, 1 B, 4 LB (deadman), 5 RB (turbo), 8 Back, 9 Start.
const padPrev = {};
function readGamepad() {
  const gp = [...(navigator.getGamepads ? navigator.getGamepads() : [])].find((g) => g && g.connected);
  if (!gp) return null;
  const b = (i) => !!(gp.buttons[i] && gp.buttons[i].pressed);
  const edge = (i) => { const now = b(i); const was = padPrev[i]; padPrev[i] = now; return now && !was; };
  if (edge(8)) estop();
  if (edge(9)) send({ type: 'estop', active: false });
  if (edge(0)) send({ type: 'command', name: 'stand' });
  if (edge(1)) send({ type: 'command', name: 'lie' });
  if (!b(4)) return null;
  const dz = (v) => (Math.abs(v) < 0.08 ? 0 : v);
  return { vx: -dz(gp.axes[1]), vy: -dz(gp.axes[0]), wz: -dz(gp.axes[2]), pitch: dz(gp.axes[3]), turbo: b(5) };
}

// -------------------------------------------------------------- send loop
const clamp = (v) => Math.max(-1, Math.min(1, v));
setInterval(() => {
  let vx = movePad.y, vy = -movePad.x, wz = -turnPad.x;
  let newPitch = -turnPad.y * 0.8;  // pad up = nose up
  let turbo = keys.has('turbo');
  if (keys.has('fwd')) vx += 1;
  if (keys.has('back')) vx -= 1;
  if (keys.has('left')) vy += 1;
  if (keys.has('right')) vy -= 1;
  if (keys.has('ccw')) wz += 1;
  if (keys.has('cw')) wz -= 1;
  const gp = readGamepad();
  if (gp) { vx += gp.vx; vy += gp.vy; wz += gp.wz; newPitch = gp.pitch * 0.8; turbo = turbo || gp.turbo; }

  const scale = turbo ? 1 : speed.value / 100;
  vx = clamp(vx) * scale; vy = clamp(vy) * scale; wz = clamp(wz) * scale;
  const moving = Math.abs(vx) + Math.abs(vy) + Math.abs(wz) > 1e-3;
  if (moving) send({ type: 'drive', vx, vy, wz });
  else if (wasMoving) send({ type: 'stop' });
  wasMoving = moving;

  if (Math.abs(newPitch - pitch) > 0.02 || (newPitch === 0 && pitch !== 0)) {
    pitch = clamp(newPitch);
    sendPose();
  }
  if (limits) {
    $('readout').textContent =
      `vx ${(vx * limits.max_vx).toFixed(2)} · vy ${(vy * limits.max_vy).toFixed(2)} · wz ${(wz * limits.max_wz).toFixed(2)}`;
  }
}, 1000 / SEND_HZ);

connect();
