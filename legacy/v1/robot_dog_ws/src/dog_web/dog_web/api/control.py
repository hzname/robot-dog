"""
Control API - movement commands
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter(prefix="/api/control", tags=["control"])


class CmdVel(BaseModel):
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


class ServoEnable(BaseModel):
    enable: bool


class EmergencyStop(BaseModel):
    stop: bool


# Global reference to ROS node (set by web_server.py)
ros_node = None


def set_ros_node(node):
    global ros_node
    ros_node = node


@router.post("/cmd_vel")
async def cmd_vel(cmd: CmdVel):
    """Send velocity command to robot"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    ros_node.publish_cmd_vel(cmd.linear_x, cmd.linear_y, cmd.angular_z)
    return {"status": "ok", "command": cmd.dict()}


@router.post("/servo_enable")
async def servo_enable(data: ServoEnable):
    """Enable or disable servos"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    ros_node.publish_servo_enable(data.enable)
    return {"status": "ok", "servos_enabled": data.enable}


@router.post("/emergency_stop")
async def emergency_stop(data: EmergencyStop):
    """Trigger or release emergency stop"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    ros_node.publish_emergency_stop(data.stop)
    return {"status": "ok", "emergency_stopped": data.stop}


@router.post("/stop")
async def stop_robot():
    """Stop all movement"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    ros_node.publish_cmd_vel(0.0, 0.0, 0.0)
    return {"status": "ok", "message": "Robot stopped"}
