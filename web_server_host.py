"""
Robot Dog Web Server - Host version
Runs on Banana Pi host, communicates with ROS2 via docker exec
"""

import asyncio
import json
import subprocess
import threading
from pathlib import Path
from datetime import datetime

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

BASE = Path(__file__).parent / "robot_dog_ws" / "src" / "dog_web"

JOINT_NAMES = [
    "lf_hip_joint", "lf_thigh_joint", "lf_calf_joint",
    "rf_hip_joint", "rf_thigh_joint", "rf_calf_joint",
    "lr_hip_joint", "lr_thigh_joint", "lr_calf_joint",
    "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint",
]

CALIBRATION_FILE = Path.home() / "robot-dog" / "calibration.json"
PROFILES_DIR = Path.home() / "robot-dog" / "calibration_profiles"

# --- ROS2 via docker exec ---

def ros2_cmd(cmd: str, timeout=5):
    """Run ROS2 command inside the robot_dog container"""
    full = f'docker exec robot_dog bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && {cmd}" 2>/dev/null'
    try:
        r = subprocess.run(full, shell=True, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip(), r.returncode
    except subprocess.TimeoutExpired:
        return "", 1

def ros2_pub(topic, msg_type, msg):
    """Publish a ROS2 message"""
    return ros2_cmd(f'ros2 topic pub {topic} {msg_type} "{msg}" --once', timeout=3)

# --- State ---

class RobotState:
    def __init__(self):
        self.joint_positions = [0.0] * 12
        self.emergency_stopped = False
        self.calibration = self._load_calibration()
        self.connected = False
        self._stop = False

    def _load_calibration(self):
        if CALIBRATION_FILE.exists():
            try:
                with open(CALIBRATION_FILE) as f:
                    return json.load(f)
            except Exception:
                pass
        return {n: 0.0 for n in JOINT_NAMES}

    def _save_calibration(self):
        CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(self.calibration, f, indent=2)

    def refresh(self):
        """Refresh state from ROS2"""
        out, code = ros2_cmd("ros2 topic echo /joint_states --once", timeout=2)
        if code == 0 and out:
            try:
                positions = []
                for line in out.split('\n'):
                    line = line.strip()
                    if line and not line.startswith(('header', 'name', '---')):
                        try:
                            val = float(line)
                            positions.append(val)
                        except ValueError:
                            continue
                if len(positions) == 12:
                    self.joint_positions = positions
                self.connected = True
            except Exception:
                pass
        else:
            self.connected = False

        out, code = ros2_cmd("ros2 topic echo /emergency_stop --once", timeout=1)
        if code == 0:
            self.emergency_stopped = "true" in out.lower()

    def get_state(self):
        return {
            "joint_positions": self.joint_positions,
            "joint_names": JOINT_NAMES,
            "emergency_stopped": self.emergency_stopped,
            "calibration": self.calibration,
            "connected": self.connected,
        }

state = RobotState()

# Background state poller
def state_poller():
    while True:
        try:
            state.refresh()
        except Exception:
            state.connected = False
        import time
        time.sleep(1)

# --- FastAPI ---

app = FastAPI(title="Robot Dog Web Interface")

static_dir = BASE / "static"
templates_dir = BASE / "templates"

app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")


def render(template_name):
    """Read and return HTML template directly"""
    path = templates_dir / template_name
    return HTMLResponse(content=path.read_text(), status_code=200)


# Pages

@app.get("/")
async def root():
    return RedirectResponse("/dashboard")

@app.get("/dashboard")
async def dashboard():
    return render("dashboard.html")

@app.get("/control")
async def control_page():
    return render("control.html")

@app.get("/state")
async def state_page():
    return render("state.html")

@app.get("/calibration")
async def calibration_page():
    return render("calibration.html")


# API - Control

@app.post("/api/control/cmd_vel")
async def cmd_vel(data: dict):
    lx = data.get("linear_x", 0.0)
    ly = data.get("linear_y", 0.0)
    az = data.get("angular_z", 0.0)
    ros2_pub("/cmd_vel", "geometry_msgs/msg/Twist",
             f"linear: {{x: {lx}, y: {ly}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {az}}}")
    return {"status": "ok"}

@app.post("/api/control/stop")
async def stop_robot():
    ros2_pub("/cmd_vel", "geometry_msgs/msg/Twist",
             "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}")
    return {"status": "ok"}

@app.post("/api/control/servo_enable")
async def servo_enable(data: dict):
    en = str(data.get("enable", True)).lower()
    ros2_pub("/servo_enable", "std_msgs/msg/Bool", f"data: {en}")
    return {"status": "ok"}

@app.post("/api/control/emergency_stop")
async def emergency_stop(data: dict):
    stop = str(data.get("stop", True)).lower()
    ros2_pub("/emergency_stop_trigger", "std_msgs/msg/Bool", f"data: {stop}")
    return {"status": "ok"}


# API - State

@app.get("/api/state")
async def get_state():
    return state.get_state()

@app.get("/api/state/joints")
async def get_joints():
    return {"joints": state.joint_positions}

@app.get("/api/state/system")
async def get_system():
    return {"ros_connected": state.connected}


# API - Calibration

@app.get("/api/calibration")
async def get_calibration():
    return {"calibration": state.calibration}

@app.post("/api/calibration/set")
async def set_calibration(data: dict):
    joint = data.get("joint")
    offset = data.get("offset", 0.0)
    if joint in JOINT_NAMES:
        state.calibration[joint] = offset
        state._save_calibration()
        return {"status": "ok"}
    return {"error": "Invalid joint"}

@app.post("/api/calibration/apply")
async def apply_calibration():
    vals = ",".join(str(state.calibration.get(n, 0.0)) for n in JOINT_NAMES)
    ros2_pub("/joint_position_command", "std_msgs/msg/Float64MultiArray", f"data: [{vals}]")
    return {"status": "ok"}

@app.post("/api/calibration/save")
async def save_profile(data: dict):
    name = data.get("profile_name", "default")
    calib = data.get("calibration", state.calibration)
    PROFILES_DIR.mkdir(parents=True, exist_ok=True)
    path = PROFILES_DIR / f"{name}.json"
    with open(path, 'w') as f:
        json.dump({"profile_name": name, "calibration": calib}, f, indent=2)
    return {"status": "ok", "profile": name}

@app.post("/api/calibration/load")
async def load_profile(data: dict):
    name = data.get("profile_name")
    if not name:
        return {"error": "profile_name required"}
    path = PROFILES_DIR / f"{name}.json"
    if not path.exists():
        return {"error": f"Profile {name} not found"}
    with open(path) as f:
        prof = json.load(f)
    for j, v in prof.get("calibration", {}).items():
        if j in JOINT_NAMES:
            state.calibration[j] = v
    state._save_calibration()
    return {"status": "ok", "calibration": prof["calibration"]}

@app.get("/api/calibration/profiles")
async def list_profiles():
    PROFILES_DIR.mkdir(parents=True, exist_ok=True)
    profiles = []
    for p in PROFILES_DIR.glob("*.json"):
        try:
            with open(p) as f:
                d = json.load(f)
                profiles.append({"name": d.get("profile_name", p.stem), "file": p.name})
        except Exception:
            pass
    return {"profiles": profiles}

@app.delete("/api/calibration/profiles/{name}")
async def delete_profile(name: str):
    path = PROFILES_DIR / f"{name}.json"
    if path.exists():
        path.unlink()
        return {"status": "ok"}
    return {"error": "not found"}

@app.post("/api/calibration/reset")
async def reset_calibration():
    for j in JOINT_NAMES:
        state.calibration[j] = 0.0
    state._save_calibration()
    return {"status": "ok"}


# WebSocket

@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(state.get_state())
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        pass


def main():
    # Start background state poller
    t = threading.Thread(target=state_poller, daemon=True)
    t.start()
    uvicorn.run(app, host="0.0.0.0", port=8080)

if __name__ == "__main__":
    main()
