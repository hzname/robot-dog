"""
Simplified Web Server for Robot Dog - runs on host, communicates with ROS2 via Docker
"""

import asyncio
import json
import subprocess
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

JOINT_NAMES = [
    "lf_hip_joint", "lf_thigh_joint", "lf_calf_joint",
    "rf_hip_joint", "rf_thigh_joint", "rf_calf_joint",
    "lr_hip_joint", "lr_thigh_joint", "lr_calf_joint",
    "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint",
]

def docker_exec_ros2(cmd: str):
    """Execute ROS2 command in robot_dog container"""
    full_cmd = f"docker exec robot_dog bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && {cmd}'"
    try:
        result = subprocess.run(full_cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.stdout, result.stderr, result.returncode
    except subprocess.TimeoutExpired:
        return "", "timeout", 1
    except Exception as e:
        return "", str(e), 1

# --- FastAPI App ---
app = FastAPI()

HTML_PATH = Path("/home/sg/robot-dog/robot_dog_ws/src/dog_web/resource/index.html")

@app.get("/", response_class=HTMLResponse)
async def index():
    if not HTML_PATH.exists():
        return HTMLResponse(content="<h1>index.html not found</h1>", status_code=404)
    return HTMLResponse(content=HTML_PATH.read_text(), status_code=200)

@app.get("/api/state")
async def get_state():
    # Get joint states from ROS2
    stdout, stderr, code = docker_exec_ros2("ros2 topic echo /joint_states --once")
    if code != 0 or not stdout:
        return {"error": "Failed to get state", "stderr": stderr}
    
    try:
        # Parse joint states output
        lines = stdout.strip().split('\n')
        positions = [0.0] * 12
        # Simple parsing - would need better parsing for real data
        for i, line in enumerate(lines):
            if 'position' in line and i < len(positions):
                pass  # Parse properly
        
        return {
            "joint_positions": positions,
            "emergency_stopped": False,
            "calibration": {name: 0.0 for name in JOINT_NAMES},
        }
    except Exception as e:
        return {"error": str(e)}

@app.post("/api/cmd_vel")
async def cmd_vel(data: dict):
    linear_x = data.get('linear_x', 0.0)
    linear_y = data.get('linear_y', 0.0)
    angular_z = data.get('angular_z', 0.0)
    
    cmd = f'ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: {linear_x}, y: {linear_y}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {angular_z}}}}}" --once'
    docker_exec_ros2(cmd)
    return {"ok": True}

@app.post("/api/servo_enable")
async def servo_enable(data: dict):
    enable = data.get('enable', True)
    cmd = f'ros2 topic pub /servo_enable "std_msgs/msg/Bool" "data: {str(enable).lower()}" --once'
    docker_exec_ros2(cmd)
    return {"ok": True}

@app.post("/api/emergency_stop")
async def emergency_stop(data: dict):
    stop = data.get('stop', True)
    cmd = f'ros2 topic pub /emergency_stop_trigger "std_msgs/msg/Bool" "data: {str(stop).lower()}" --once'
    docker_exec_ros2(cmd)
    return {"ok": True}

@app.post("/api/stop")
async def stop_robot():
    cmd = 'ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once'
    docker_exec_ros2(cmd)
    return {"ok": True}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # Simple state - can be improved with proper parsing
            state = {
                "joint_positions": [0.0] * 12,
                "emergency_stopped": False,
                "calibration": {name: 0.0 for name in JOINT_NAMES},
            }
            await websocket.send_json(state)
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass

def main():
    uvicorn.run(app, host='0.0.0.0', port=8080)

if __name__ == '__main__':
    main()
