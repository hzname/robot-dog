"""
Robot Dog Web Interface Server
FastAPI + rclpy for ROS2 communication
"""

import asyncio
import json
import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn


JOINT_NAMES = [
    "lf_hip_joint", "lf_thigh_joint", "lf_calf_joint",
    "rf_hip_joint", "rf_thigh_joint", "rf_calf_joint",
    "lr_hip_joint", "lr_thigh_joint", "lr_calf_joint",
    "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint",
]

CALIBRATION_FILE = Path("/workspace/calibration.json")

# Default calibration offsets (radians)
DEFAULT_CALIBRATION = {name: 0.0 for name in JOINT_NAMES}


class DogROSNode(Node):
    def __init__(self):
        super().__init__('web_interface')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
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
                    # Merge with defaults
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
        """Send calibration offsets as joint positions (neutral + offset)"""
        msg = Float64MultiArray()
        positions = []
        for name in JOINT_NAMES:
            offset = self.calibration.get(name, 0.0)
            positions.append(offset)
        msg.data = positions
        self.position_cmd_pub.publish(msg)

    def set_calibration(self, joint_name, offset):
        if joint_name in self.calibration:
            self.calibration[joint_name] = offset
            self._save_calibration()

    def get_state(self):
        return {
            'joint_positions': self.joint_positions,
            'emergency_stopped': self.emergency_stopped,
            'calibration': self.calibration,
        }


# Global ROS node
ros_node = None
ros_spin_thread = None


def ros_thread_fn():
    global ros_node
    rclpy.init()
    ros_node = DogROSNode()
    rclpy.spin(ros_node)
    rclpy.shutdown()


# --- FastAPI App ---
app = FastAPI()


def get_html_path():
    """Find index.html in package share directory"""
    try:
        share_dir = Path(get_package_share_directory('dog_web'))
        html_path = share_dir / 'index.html'
        if html_path.exists():
            return html_path
    except Exception:
        pass
    
    # Fallback to development location
    html_path = Path(__file__).parent.parent / 'resource' / 'index.html'
    if html_path.exists():
        return html_path
    
    # Another fallback
    html_path = Path('/workspace/install/share/dog_web/index.html')
    return html_path


@app.get("/", response_class=HTMLResponse)
async def index():
    html_path = get_html_path()
    if not html_path.exists():
        return HTMLResponse(content="<h1>index.html not found</h1><p>Path: {html_path}</p>", status_code=404)
    return HTMLResponse(content=html_path.read_text(), status_code=200)


@app.get("/api/state")
async def get_state():
    if ros_node is None:
        return {"error": "ROS not ready"}
    return ros_node.get_state()


@app.post("/api/cmd_vel")
async def cmd_vel(data: dict):
    if ros_node is None:
        return {"error": "ROS not ready"}
    ros_node.publish_cmd_vel(
        linear_x=data.get('linear_x', 0.0),
        linear_y=data.get('linear_y', 0.0),
        angular_z=data.get('angular_z', 0.0),
    )
    return {"ok": True}


@app.post("/api/servo_enable")
async def servo_enable(data: dict):
    if ros_node is None:
        return {"error": "ROS not ready"}
    ros_node.publish_servo_enable(data.get('enable', True))
    return {"ok": True}


@app.post("/api/emergency_stop")
async def emergency_stop(data: dict):
    if ros_node is None:
        return {"error": "ROS not ready"}
    ros_node.publish_emergency_stop(data.get('stop', True))
    return {"ok": True}


@app.post("/api/calibrate")
async def calibrate(data: dict):
    if ros_node is None:
        return {"error": "ROS not ready"}
    joint_name = data.get('joint')
    offset = data.get('offset', 0.0)
    if joint_name and joint_name in JOINT_NAMES:
        ros_node.set_calibration(joint_name, offset)
        ros_node.publish_calibration_pose()
        return {"ok": True}
    return {"error": "Invalid joint name"}


@app.post("/api/calibrate/apply")
async def calibrate_apply():
    if ros_node is None:
        return {"error": "ROS not ready"}
    ros_node.publish_calibration_pose()
    return {"ok": True}


@app.post("/api/stop")
async def stop_robot():
    if ros_node is None:
        return {"error": "ROS not ready"}
    ros_node.publish_cmd_vel(0.0, 0.0, 0.0)
    return {"ok": True}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
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
    global ros_spin_thread
    ros_spin_thread = threading.Thread(target=ros_thread_fn, daemon=True)
    ros_spin_thread.start()

    # Wait for ROS to be ready
    for _ in range(50):
        if ros_node is not None:
            break
        time.sleep(0.1)

    uvicorn.run(app, host='0.0.0.0', port=8080)


if __name__ == '__main__':
    main()
