/* RobotDogQwen - Common JS */

const JOINT_NAMES = [
  "lf_hip_joint","lf_thigh_joint","lf_calf_joint",
  "rf_hip_joint","rf_thigh_joint","rf_calf_joint",
  "lr_hip_joint","lr_thigh_joint","lr_calf_joint",
  "rr_hip_joint","rr_thigh_joint","rr_calf_joint",
];

const LEG_MAP = {
  'LF': ['lf_hip_joint','lf_thigh_joint','lf_calf_joint'],
  'RF': ['rf_hip_joint','rf_thigh_joint','rf_calf_joint'],
  'LR': ['lr_hip_joint','lr_thigh_joint','lr_calf_joint'],
  'RR': ['rr_hip_joint','rr_thigh_joint','rr_calf_joint'],
};

// API helper
async function api(endpoint, data = null) {
  try {
    const opts = { method: data ? 'POST' : 'GET', headers: { 'Content-Type': 'application/json' } };
    if (data) opts.body = JSON.stringify(data);
    const res = await fetch(endpoint, opts);
    return await res.json();
  } catch (e) {
    console.error('API error:', e);
    return null;
  }
}

// WebSocket manager
class StateWS {
  constructor(onState) {
    this.onState = onState;
    this.ws = null;
    this.connect();
  }

  connect() {
    const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
    this.ws = new WebSocket(`${proto}//${location.host}/ws`);
    this.ws.onopen = () => {
      console.log('WS connected');
      if (document.getElementById('ws-dot')) {
        document.getElementById('ws-dot').className = 'dot on';
      }
    };
    this.ws.onmessage = (ev) => {
      try { this.onState(JSON.parse(ev.data)); } catch(e) {}
    };
    this.ws.onclose = () => {
      console.log('WS disconnected, reconnecting...');
      if (document.getElementById('ws-dot')) {
        document.getElementById('ws-dot').className = 'dot off';
      }
      setTimeout(() => this.connect(), 2000);
    };
  }
}

// Navigation highlight
document.addEventListener('DOMContentLoaded', () => {
  const path = location.pathname;
  document.querySelectorAll('nav a').forEach(a => {
    if (a.getAttribute('href') === path) a.classList.add('active');
  });
});
