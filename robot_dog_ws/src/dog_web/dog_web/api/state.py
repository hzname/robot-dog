"""
State API - robot state information
"""

from fastapi import APIRouter, HTTPException

router = APIRouter(prefix="/api/state", tags=["state"])

# Global reference to ROS node
ros_node = None


def set_ros_node(node):
    global ros_node
    ros_node = node


@router.get("/")
async def get_state():
    """Get complete robot state"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    return ros_node.get_state()


@router.get("/joints")
async def get_joint_states():
    """Get joint positions only"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    return {"joints": ros_node.joint_positions}


@router.get("/emergency")
async def get_emergency_status():
    """Get emergency stop status"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    return {"emergency_stopped": ros_node.emergency_stopped}


@router.get("/system")
async def get_system_status():
    """Get system status"""
    if ros_node is None:
        return {
            "ros_connected": False,
            "servo_driver": "unknown",
            "gait_controller": "unknown"
        }
    
    # Could be extended to check node availability via ROS2 services
    return {
        "ros_connected": True,
        "servo_driver": "active",  # Simple assumption
        "gait_controller": "active"
    }
