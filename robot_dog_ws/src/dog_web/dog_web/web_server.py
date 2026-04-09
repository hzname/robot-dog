"""
Robot Dog Web Server - Full Interface
FastAPI + rclpy for ROS2 communication
Multi-page interface: Dashboard, Control, State, Calibration
"""

import asyncio
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import uvicorn

# Import API routers
from .api.control import router as control_router, set_ros_node as set_control_node
from .api.state import router as state_router, set_ros_node as set_state_node
from .api.calibration import router as calib_router, set_ros_node as set_calib_node

JOINT_NAMES = [
    "lf_hip_joint", "lf_thigh_joint", "lf_calf_joint",
    "rf_hip_joint", "rf_thigh_joint", "rf_calf_joint",
    "lr_hip_joint", "lr_thigh_joint", "lr_calf_joint",
    "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint",
]

CALIBRATION_FILE = Path("/workspace/calibration.json")
DEFAULT_CALIBRATION = {name: 0.0 for name in JOINT_NAMES}


class DogWebNode(Node):
    """ROS2 node for web interface communication"""
    
    def __init__(self):
        super().__init__('dog_web_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_enable_pub = self.create_publisher(Bool, '/servo_enable', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop_trigger', 10)
        self.position_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_position_command', 10)

        # Subscribers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, qos)
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self._on_emergency_stop, 10)

        # State
        self.joint_positions = [0.0] * 12
        self.emergency_stopped = False
        self.calibration = self._load_calibration()

    def _on_joint_states(self, msg):
        self.joint_positions = list(msg.position) if msg.position else [0.0] * 12

    def _on_emergency_stop(self, msg):
        self.emergency_stopped = msg.data

    def _load_calibration(self):
        if CALIBRATION_FILE.exists():
            try:
                with open(CALIBRATION_FILE, 'r') as f:
                    data = json.load(f)
                    cal = dict(DEFAULT_CALIBRATION)
                    cal.update(data)
                    return cal
            except Exception:
                pass
        return dict(DEFAULT_CALIBRATION)

    def _save_calibration(self):
        CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(self.calibration, f, indent=2)

    def publish_cmd_vel(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def publish_servo_enable(self, enable):
        msg = Bool()
        msg.data = enable
        self.servo_enable_pub.publish(msg)

    def publish_emergency_stop(self, stop):
        msg = Bool()
        msg.data = stop
        self.emergency_stop_pub.publish(msg)

    def publish_calibration_pose(self):
        """Send calibration offsets as joint positions"""
        msg = Float64MultiArray()
        positions = [self.calibration.get(name, 0.0) for name in JOINT_NAMES]
        msg.data = positions
        self.position_cmd_pub.publish(msg)

    def set_calibration(self, joint_name, offset):
        if joint_name in self.calibration:
            self.calibration[joint_name] = offset
            self._save_calibration()

    def get_state(self):
        return {
            'joint_positions': self.joint_positions,
            'joint_names': JOINT_NAMES,
            'emergency_stopped': self.emergency_stopped,
            'calibration': self.calibration,
        }


# Global ROS node
ros_node = None


def init_ros():
    """Initialize ROS2 node"""
    global ros_node
    rclpy.init()
    ros_node = DogWebNode()
    
    # Set ROS node in API modules
    set_control_node(ros_node)
    set_state_node(ros_node)
    set_calib_node(ros_node)


# --- FastAPI App ---
app = FastAPI(title="Robot Dog Web Interface")

# Resolve static/templates - dev path first, then installed share
def _find_dir(name):
    # Dev: dog_web/static or dog_web/templates (sibling of dog_web package)
    dev = Path(__file__).parent.parent.parent / name
    if dev.exists():
        return dev
    # Installed share
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('dog_web')) / name
        if share.exists():
            return share
    except Exception:
        pass
    return dev  # fallback, may not exist

static_dir = _find_dir("static")
templates_dir = _find_dir("templates")

if static_dir.exists():
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

templates = Jinja2Templates(directory=str(templates_dir))

# Include API routers
app.include_router(control_router)
app.include_router(state_router)
app.include_router(calib_router)


# --- Page Routes ---

@app.get("/")
async def root():
    """Redirect to dashboard"""
    from fastapi.responses import RedirectResponse
    return RedirectResponse(url="/dashboard")


@app.get("/dashboard")
async def dashboard(request: Request):
    """Dashboard page - main overview"""
    return templates.TemplateResponse("dashboard.html", {"request": request})


@app.get("/control")
async def control(request: Request):
    """Control page - movement control"""
    return templates.TemplateResponse("control.html", {"request": request})


@app.get("/state")
async def state_page(request: Request):
    """State page - detailed robot state"""
    return templates.TemplateResponse("state.html", {"request": request})


@app.get("/calibration")
async def calibration(request: Request):
    """Calibration page - joint calibration"""
    return templates.TemplateResponse("calibration.html", {"request": request})


# --- WebSocket for real-time updates ---

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """Real-time state updates"""
    await websocket.accept()
    try:
        while True:
            if ros_node:
                state = ros_node.get_state()
                await websocket.send_json(state)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass


def main():
    """Initialize and start the web server"""
    init_ros()
    
    # Start ROS spinning in background
    import threading
    def spin_ros():
        rclpy.spin(ros_node)
    
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    # Start web server
    uvicorn.run(app, host='0.0.0.0', port=8080)


if __name__ == '__main__':
    main()
