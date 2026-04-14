"""
RobotDogQwen Web Server - Host version
Runs on Banana Pi host, communicates with ROS2 via docker exec
"""

import asyncio
import json
import math
import re
import socket
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

# Profile name validation: only alphanumeric, hyphens, underscores
_PROFILE_NAME_RE = re.compile(r'^[a-zA-Z0-9_-]+$')


def _validate_profile_name(name: str) -> str | None:
    """Return error message or None if name is valid."""
    if not name:
        return "profile_name required"
    if not _PROFILE_NAME_RE.match(name):
        return "Invalid profile name: only letters, digits, hyphens, underscores allowed"
    if '..' in name or '/' in name or '\\' in name:
        return "Invalid profile name: path traversal not allowed"
    return None


# --- Input validation for control commands ---

_MAX_LINEAR = 1.0    # m/s
_MAX_LATERAL = 0.5   # m/s
_MAX_ANGULAR = 3.0   # rad/s


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _validate_positions(positions) -> str | None:
    """Validate joint position array — must be exactly 12 finite numbers."""
    if not isinstance(positions, (list, tuple)):
        return "positions must be a list"
    if len(positions) != 12:
        return f"Need exactly 12 positions, got {len(positions)}"
    for i, p in enumerate(positions):
        if not isinstance(p, (int, float)):
            return f"Position {i} is not a number"
        if not math.isfinite(p):
            return f"Position {i} is not a finite number"
    return None

# --- ROS2 via docker exec ---

BRIDGE_SOCKET = Path("/tmp/robot_dog/ros_bridge.sock")

class BridgeClient:
    """Unix socket client with length-prefix framing."""

    def __init__(self, timeout=2):
        self._timeout = timeout

    def send(self, cmd: dict) -> tuple[dict, int]:
        """Send a command and receive the response. Returns (result_dict, error_code)."""
        if not BRIDGE_SOCKET.exists():
            return {"error": "bridge not available"}, 1
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(self._timeout)
            sock.connect(str(BRIDGE_SOCKET))
            payload = json.dumps(cmd).encode()
            sock.send(len(payload).to_bytes(4, 'big') + payload)
            # Read length-prefixed response
            header = b''
            while len(header) < 4:
                chunk = sock.recv(4 - len(header))
                if not chunk:
                    sock.close()
                    return {"error": "incomplete response"}, 1
                header += chunk
            msg_len = int.from_bytes(header, 'big')
            if msg_len > 65536:
                sock.close()
                return {"error": "response too large"}, 1
            data = b''
            while len(data) < msg_len:
                chunk = sock.recv(msg_len - len(data))
                if not chunk:
                    sock.close()
                    return {"error": "incomplete response"}, 1
                data += chunk
            sock.close()
            return json.loads(data.decode()), 0
        except (OSError, json.JSONDecodeError) as e:
            return {"error": str(e)}, 1


# Global bridge client
_bridge_client = BridgeClient()


def bridge_send(cmd: dict, timeout=2):
    """Send command to ROS2 bridge via Unix socket."""
    _bridge_client._timeout = timeout
    return _bridge_client.send(cmd)

# --- State ---

class RobotState:
    def __init__(self):
        self.joint_positions = [0.0] * 12
        self.emergency_stopped = False
        self.servos_enabled = True  # Track servo enable state
        self.calibration = self._load_calibration()
        self.connected = False
        self.imu_orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        self.imu_angular_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.imu_acceleration = {'x': 0.0, 'y': 0.0, 'z': 0.0}
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
        """Refresh state from bridge socket"""
        result, code = bridge_send({"type": "get_state"})
        if code == 0 and result.get("ok"):
            self.joint_positions = result.get("joint_positions", self.joint_positions)
            self.emergency_stopped = result.get("emergency_stopped", self.emergency_stopped)
            self.connected = True
        else:
            self.connected = False
            print(f"DEBUG: get_state failed: code={code}, result={result}")

        # Fetch IMU data
        imu_result, imu_code = bridge_send({"type": "get_imu"})
        if imu_code == 0 and imu_result.get("ok") and imu_result.get("imu"):
            imu = imu_result["imu"]
            self.imu_orientation = imu.get("orientation", self.imu_orientation)
            self.imu_angular_vel = imu.get("angular_velocity", self.imu_angular_vel)
            self.imu_acceleration = imu.get("linear_acceleration", self.imu_acceleration)
        else:
            print(f"DEBUG: get_imu failed: code={imu_code}, result={imu_result}")

    def get_state(self):
        return {
            "joint_positions": self.joint_positions,
            "joint_names": JOINT_NAMES,
            "emergency_stopped": self.emergency_stopped,
            "servos_enabled": self.servos_enabled,
            "calibration": self.calibration,
            "connected": self.connected,
            "imu_orientation": self.imu_orientation,
            "imu_angular_vel": self.imu_angular_vel,
            "imu_acceleration": self.imu_acceleration,
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

app = FastAPI(title="RobotDogQwen Web Interface")

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
    lx = _clamp(data.get("linear_x", 0.0), -_MAX_LINEAR, _MAX_LINEAR)
    ly = _clamp(data.get("linear_y", 0.0), -_MAX_LATERAL, _MAX_LATERAL)
    az = _clamp(data.get("angular_z", 0.0), -_MAX_ANGULAR, _MAX_ANGULAR)
    bridge_send({"type": "cmd_vel", "linear_x": lx, "linear_y": ly, "angular_z": az})
    return {"status": "ok", "clamped": {"linear_x": lx, "linear_y": ly, "angular_z": az}}

@app.post("/api/control/stop")
async def stop_robot():
    bridge_send({"type": "stop"})
    return {"status": "ok"}

@app.post("/api/control/servo_enable")
async def servo_enable(data: dict):
    enable = data.get("enable", True)
    state.servos_enabled = enable  # Track local state
    bridge_send({"type": "servo_enable", "enable": enable})
    return {"status": "ok"}

@app.post("/api/control/emergency_stop")
async def emergency_stop(data: dict):
    bridge_send({"type": "emergency_stop", "stop": data.get("stop", True)})
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
    if joint not in JOINT_NAMES:
        return {"error": "Invalid joint"}
    if not isinstance(offset, (int, float)) or not math.isfinite(offset):
        return {"error": "Offset must be a finite number"}
    state.calibration[joint] = offset
    state._save_calibration()
    return {"status": "ok"}

@app.post("/api/calibration/apply")
async def apply_calibration():
    vals = [state.calibration.get(n, 0.0) for n in JOINT_NAMES]
    bridge_send({"type": "joint_command", "positions": vals})
    return {"status": "ok"}

@app.post("/api/calibration/live")
async def live_calibration(data: dict):
    """Apply calibration positions directly without saving"""
    positions = data.get("positions", [])
    err = _validate_positions(positions)
    if err:
        return {"error": err}
    bridge_send({"type": "joint_command", "positions": positions})
    return {"status": "ok"}

@app.post("/api/calibration/save")
async def save_profile(data: dict):
    name = data.get("profile_name", "default")
    err = _validate_profile_name(name)
    if err:
        return {"error": err}
    calib = data.get("calibration", state.calibration)
    PROFILES_DIR.mkdir(parents=True, exist_ok=True)
    path = PROFILES_DIR / f"{name}.json"
    with open(path, 'w') as f:
        json.dump({"profile_name": name, "calibration": calib}, f, indent=2)
    return {"status": "ok", "profile": name}

@app.post("/api/calibration/load")
async def load_profile(data: dict):
    name = data.get("profile_name")
    err = _validate_profile_name(name)
    if err:
        return {"error": err}
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
    err = _validate_profile_name(name)
    if err:
        return {"error": err}
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
