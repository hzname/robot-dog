#!/usr/bin/env python3
"""
Servo Calibration Web Server - Clean rewrite
Runs on Banana Pi host (port 8081).
Stops ROS2 container, provides I2C direct control via web API.
Speed limited to 9 deg/sec for safety.
Movement thread auto-restarts on crash.
"""
import json, os, subprocess, time, math, threading
from fastapi import FastAPI, HTTPException, UploadFile, File
from fastapi.responses import FileResponse
from pydantic import BaseModel
from typing import Any
import smbus2

CONFIG_PATH = "/home/sg/robot-dog/servo_config.json"
TEMPLATE_DIR = "/home/sg/robot-dog/robot_dog_ws/src/dog_web/templates"
I2C_BUS = 0
PCA9685_ADDR = 0x40
SERVO_MIN_US = 520
SERVO_MAX_US = 2220
MAX_SPEED_DEG_S = 9.0

app = FastAPI()

pca = None
container_was_running = False
calibration_active = False
current_positions = [0.0] * 16
target_positions = [0.0] * 16
servo_initialized = [False] * 16
move_thread = None
alive = threading.Event()

LEG_JOINTS = {"lf": [0,1,2], "rf": [3,4,5], "lr": [6,7,8], "rr": [9,10,11]}

POSES = {
    "reference": {"label": "Reference (ноль)", "desc": "Полуприсед", "legs": {"lf":[0,0,0],"rf":[0,0,0],"lr":[0,0,0],"rr":[0,0,0]}},
    "tiptoe_front": {"label": "На цыпочках (перед)", "desc": "Передние лапы вверх", "legs": {"lf":[0,-30,20],"rf":[0,-30,20]}},
    "tiptoe_rear": {"label": "На цыпочках (зад)", "desc": "Задние лапы вверх", "legs": {"lr":[0,-30,20],"rr":[0,-30,20]}},
    "tiptoe_all": {"label": "На цыпочках (все)", "desc": "Все лапы вверх", "legs": {"lf":[0,-30,20],"rf":[0,-30,20],"lr":[0,-30,20],"rr":[0,-30,20]}},
    "lean_forward": {"label": "Наклон вперёд", "desc": "Перед назад, зад вперёд", "legs": {"lf":[-15,10,-10],"rf":[-15,10,-10],"lr":[15,-10,10],"rr":[15,-10,10]}},
    "sit_back": {"label": "Сесть на задние", "desc": "Зад согнуты, перед вытянуты", "legs": {"lf":[0,-20,20],"rf":[0,-20,20],"lr":[0,30,-40],"rr":[0,30,-40]}},
    "stand_tall": {"label": "Полный stand", "desc": "Максимально высоко", "legs": {"lf":[0,-30,15],"rf":[0,-30,15],"lr":[0,-30,15],"rr":[0,-30,15]}},
    "crouch": {"label": "Полный присед", "desc": "Максимально низко", "legs": {"lf":[0,40,-50],"rf":[0,40,-50],"lr":[0,40,-50],"rr":[0,40,-50]}},
    "front_reach": {"label": "Передние вперёд", "desc": "Передние вытянуты", "legs": {"lf":[20,-20,20],"rf":[20,-20,20]}},
    "rear_kick": {"label": "Задние назад", "desc": "Задние вытянуты назад", "legs": {"lr":[-20,20,-20],"rr":[-20,20,-20]}},
}

# ─── PCA9685 ──────────────────────────────────────────

class PCA9685:
    def __init__(self, bus=I2C_BUS, addr=PCA9685_ADDR, reset=False):
        self.bus = smbus2.SMBus(bus)
        self.addr = addr
        if reset:
            self.bus.write_byte_data(self.addr, 0x00, 0x00)
            time.sleep(0.01)
            old = self.bus.read_byte_data(self.addr, 0x00)
            self.bus.write_byte_data(self.addr, 0x00, (old & 0x7F) | 0x10)
            self.bus.write_byte_data(self.addr, 0xFE, 121)  # 50Hz
            self.bus.write_byte_data(self.addr, 0x00, old)
            time.sleep(0.01)
            self.bus.write_byte_data(self.addr, 0x00, old | 0x20)
        # else: chip already configured by ROS2, keep PWM as-is

    def set_pwm(self, ch, on, off):
        reg = 0x06 + 4 * ch
        self.bus.write_i2c_block_data(self.addr, reg, [on&0xFF,(on>>8)&0xFF,off&0xFF,(off>>8)&0xFF])

    def get_pwm_off(self, ch):
        data = self.bus.read_i2c_block_data(self.addr, 0x06 + 4*ch, 4)
        return data[2] | (data[3] << 8)

    def set_us(self, ch, us):
        tick = max(0, min(4095, int(us / 20000.0 * 4096)))
        self.set_pwm(ch, 0, tick)

    def get_us(self, ch):
        off = self.get_pwm_off(ch)
        return off / 4096.0 * 20000.0 if off else 0

    def disable(self, ch):
        self.set_pwm(ch, 0, 0)

    def disable_all(self):
        for ch in range(16):
            self.disable(ch)

    def close(self):
        self.disable_all()
        self.bus.close()

# ─── Helpers ──────────────────────────────────────────

def load_cfg():
    with open(CONFIG_PATH) as f:
        return json.load(f)

def save_cfg(cfg):
    with open(CONFIG_PATH, 'w') as f:
        json.dump(cfg, f, indent=2)

def deg_to_us(deg, sign, zero_deg, min_deg, max_deg):
    deg = max(min_deg, min(max_deg, deg))
    adj = (deg + zero_deg) * sign
    us = SERVO_MIN_US + (adj + 90) / 180.0 * (SERVO_MAX_US - SERVO_MIN_US)
    return max(SERVO_MIN_US, min(SERVO_MAX_US, us))

def us_to_deg(us, sign, zero_deg):
    if us < 10:
        return 0.0
    adj = (us - SERVO_MIN_US) / (SERVO_MAX_US - SERVO_MIN_US) * 180.0 - 90.0
    return adj / sign - zero_deg if sign != 0 else 0.0

def apply_pos(sid, angle_deg, jcfg):
    us = deg_to_us(angle_deg, jcfg['sign'], jcfg['zero_deg'], jcfg['min_deg'], jcfg['max_deg'])
    pca.set_us(sid, us)

def find_joint(cfg, name):
    for i, j in enumerate(cfg['joints']):
        if j['name'] == name:
            return i, j
    return -1, None

def container_running():
    r = subprocess.run(["docker","inspect","-f","{{.State.Running}}","robot_dog"], capture_output=True, text=True)
    return "true" in r.stdout.lower()

# ─── Movement Thread ─────────────────────────────────

lock = threading.Lock()

def movement_loop():
    import sys, traceback
    dt = 0.05
    max_step = MAX_SPEED_DEG_S * dt
    while alive.is_set():
        try:
            cfg = load_cfg()
            with lock:
                for j in cfg['joints']:
                    sid = j['servo_id']
                    tgt = target_positions[sid]
                    if not servo_initialized[sid]:
                        continue
                    cur = current_positions[sid]
                    diff = tgt - cur
                    if abs(diff) < 0.1:
                        continue
                    step = max_step if diff > 0 else -max_step
                    if abs(step) > abs(diff):
                        step = diff
                    current_positions[sid] = cur + step
                    apply_pos(sid, cur + step, j)
        except Exception as e:
            print(f"[movement_thread ERROR] {e}", file=sys.stderr, flush=True)
            traceback.print_exc(file=sys.stderr)
            time.sleep(0.5)  # back off, then retry
            continue
        time.sleep(dt)

def watch_thread():
    """Auto-restart movement thread if it dies."""
    import sys
    while True:
        time.sleep(1)
        if calibration_active and alive.is_set():
            if move_thread is None or not move_thread.is_alive():
                print("[watcher] Movement thread died, restarting...", file=sys.stderr, flush=True)
                t = threading.Thread(target=movement_loop, daemon=True)
                t.start()

# ─── Models ───────────────────────────────────────────

class ServoCmd(BaseModel):
    joint_name: str
    angle_deg: float

class JointFieldCmd(BaseModel):
    joint_name: str
    field: str
    value: Any

class LegCmd(BaseModel):
    leg: str
    hip: float = 0
    thigh: float = 0
    calf: float = 0

class FullConfig(BaseModel):
    body_dimensions: dict
    joints: list
    aux_servos: list

# ─── Routes ───────────────────────────────────────────

@app.get("/")
def index():
    return FileResponse(os.path.join(TEMPLATE_DIR, "calibration2.html"))

@app.get("/api/cal/status")
def status():
    return {"calibration_active": calibration_active,
            "container_running": container_running(),
            "current_positions": current_positions[:12],
            "speed_limit": MAX_SPEED_DEG_S}

@app.post("/api/cal/start")
def start():
    global pca, calibration_active, container_was_running
    if calibration_active:
        return {"status": "already_active"}
    container_was_running = container_running()
    if container_was_running:
        subprocess.run(["docker","compose","down"], cwd="/home/sg/robot-dog", capture_output=True)
        time.sleep(2)
    pca = PCA9685()
    calibration_active = True
    # No movement - servos stay where they are
    for i in range(16):
        current_positions[i] = 0.0
        target_positions[i] = 0.0
        servo_initialized[i] = False
    # Start thread
    alive.set()
    move_thread = threading.Thread(target=movement_loop, daemon=True)
    move_thread.start()
    return {"status": "started", "container_was_running": container_was_running}

@app.post("/api/cal/stop")
def stop():
    global pca, calibration_active, container_was_running
    if not calibration_active:
        return {"status": "not_active"}
    alive.clear()
    if pca:
        pca.disable_all()
        pca.close()
        pca = None
    calibration_active = False
    for i in range(16):
        current_positions[i] = 0.0
        target_positions[i] = 0.0
        servo_initialized[i] = False
    if container_was_running:
        subprocess.run(["docker","compose","up","-d"], cwd="/home/sg/robot-dog", capture_output=True)
        time.sleep(3)
    return {"status": "stopped"}

@app.get("/api/cal/config")
def get_config():
    return load_cfg()

@app.post("/api/cal/config")
def set_field(data: JointFieldCmd):
    cfg = load_cfg()
    idx, j = find_joint(cfg, data.joint_name)
    if idx < 0:
        raise HTTPException(404)
    if data.field not in ('sign','zero_deg','min_deg','max_deg','coupled_to','coupling_coeff'):
        raise HTTPException(400, f"Bad field: {data.field}")
    j[data.field] = data.value
    save_cfg(cfg)
    return {"status": "ok"}

@app.post("/api/cal/config/save")
def save_full(cfg: FullConfig):
    save_cfg(cfg.model_dump())
    return {"status": "ok"}

@app.post("/api/cal/config/replace")
def replace_full(cfg: FullConfig):
    save_cfg(cfg.model_dump())
    return {"status": "ok"}

@app.post("/api/cal/config/upload")
async def upload(file: UploadFile = File(...)):
    content = await file.read()
    cfg = json.loads(content)
    save_cfg(cfg)
    return {"status": "ok"}

@app.post("/api/cal/move")
def move(cmd: ServoCmd):
    if not calibration_active:
        raise HTTPException(400, "Not active")
    cfg = load_cfg()
    idx, j = find_joint(cfg, cmd.joint_name)
    if idx < 0:
        raise HTTPException(404)
    angle = max(j['min_deg'], min(j['max_deg'], cmd.angle_deg))
    sid = j['servo_id']
    with lock:
        target_positions[sid] = angle
        if not servo_initialized[sid]:
            # First move: teleport directly
            current_positions[sid] = angle
            apply_pos(sid, angle, j)
            servo_initialized[sid] = True
    eta = abs(angle - current_positions[sid]) / MAX_SPEED_DEG_S
    return {"status": "moving", "servo_id": sid, "target": angle, "eta_s": round(eta, 1)}

@app.post("/api/cal/move_leg")
def move_leg(cmd: LegCmd):
    if not calibration_active:
        raise HTTPException(400, "Not active")
    cfg = load_cfg()
    if cmd.leg not in LEG_JOINTS:
        raise HTTPException(400, f"Bad leg: {cmd.leg}")
    angles = [cmd.hip, cmd.thigh, cmd.calf]
    with lock:
        for i, ji in enumerate(LEG_JOINTS[cmd.leg]):
            j = cfg['joints'][ji]
            a = max(j['min_deg'], min(j['max_deg'], angles[i]))
            target_positions[j['servo_id']] = a
    return {"status": "moving"}

@app.post("/api/cal/move_all_hips")
def move_all_hips(angle: float):
    """Move all hip joints to same angle."""
    if not calibration_active:
        raise HTTPException(400, "Not active")
    cfg = load_cfg()
    with lock:
        for j in cfg['joints']:
            if "hip" in j['name']:
                a = max(j['min_deg'], min(j['max_deg'], angle))
                target_positions[j['servo_id']] = a
    return {"status": "moving"}

@app.post("/api/cal/pose/{name}")
def pose(name: str):
    if not calibration_active:
        raise HTTPException(400, "Not active")
    if name not in POSES:
        raise HTTPException(404, f"Unknown pose: {name}")
    cfg = load_cfg()
    with lock:
        for leg, indices in LEG_JOINTS.items():
            if leg in POSES[name]['legs']:
                for i, ji in enumerate(indices):
                    j = cfg['joints'][ji]
                    a = max(j['min_deg'], min(j['max_deg'], POSES[name]['legs'][leg][i]))
                    target_positions[j['servo_id']] = a
    return {"status": "moving", "pose": name, "label": POSES[name]['label']}

@app.get("/api/cal/poses")
def poses():
    return {k: {"label": v["label"], "desc": v["desc"]} for k, v in POSES.items()}

@app.post("/api/cal/disable_all")
def dis_all():
    if pca:
        pca.disable_all()
    for i in range(16):
        current_positions[i] = 0.0
        target_positions[i] = 0.0
        servo_initialized[i] = False
    return {"status": "ok"}

@app.post("/api/cal/disable/{name}")
def dis_one(name: str):
    if not calibration_active:
        raise HTTPException(400)
    cfg = load_cfg()
    idx, j = find_joint(cfg, name)
    if idx < 0:
        raise HTTPException(404)
    pca.disable(j['servo_id'])
    servo_initialized[j['servo_id']] = False
    return {"status": "ok"}

@app.post("/api/cal/set_zero")
def set_zero(data: ServoCmd):
    if not calibration_active:
        raise HTTPException(400)
    cfg = load_cfg()
    idx, j = find_joint(cfg, data.joint_name)
    if idx < 0:
        raise HTTPException(404)
    sid = j['servo_id']
    cur = current_positions[sid]
    j['zero_deg'] = round(cur + j['zero_deg'], 2)
    save_cfg(cfg)
    with lock:
        current_positions[sid] = 0.0
        target_positions[sid] = 0.0
    # No apply_pos needed - physical position unchanged, just shifted zero point
    return {"status": "ok", "zero_deg": j['zero_deg']}

@app.post("/api/cal/reapply/{name}")
def reapply(name: str):
    if not calibration_active:
        raise HTTPException(400)
    cfg = load_cfg()
    idx, j = find_joint(cfg, name)
    if idx < 0:
        raise HTTPException(404)
    pos = current_positions[j['servo_id']]
    apply_pos(j['servo_id'], pos, j)
    return {"status": "ok"}

if __name__ == "__main__":
    import uvicorn
    threading.Thread(target=watch_thread, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8081)
