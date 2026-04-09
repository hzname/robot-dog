"""
Calibration API - joint calibration management
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Dict, Optional
from pathlib import Path
import json

router = APIRouter(prefix="/api/calibration", tags=["calibration"])

# Global reference to ROS node
ros_node = None

CALIBRATION_DIR = Path("/workspace/calibration/profiles")


def set_ros_node(node):
    global ros_node
    ros_node = node
    
    # Ensure calibration directory exists
    CALIBRATION_DIR.parent.mkdir(parents=True, exist_ok=True)
    CALIBRATION_DIR.mkdir(exist_ok=True)


class SetCalibration(BaseModel):
    joint: str
    offset: float


class SaveCalibration(BaseModel):
    profile_name: str
    calibration: Dict[str, float]


class ApplyCalibration(BaseModel):
    profile_name: Optional[str] = None


@router.get("/")
async def get_calibration():
    """Get current calibration values"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    return {"calibration": ros_node.calibration}


@router.post("/set")
async def set_calibration(data: SetCalibration):
    """Set calibration offset for a single joint"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    
    if data.joint not in ros_node.calibration:
        raise HTTPException(status_code=400, detail=f"Unknown joint: {data.joint}")
    
    ros_node.set_calibration(data.joint, data.offset)
    return {"status": "ok", "joint": data.joint, "offset": data.offset}


@router.post("/apply")
async def apply_calibration():
    """Apply current calibration to robot"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    ros_node.publish_calibration_pose()
    return {"status": "ok", "message": "Calibration applied"}


@router.post("/save")
async def save_calibration(data: SaveCalibration):
    """Save calibration to a profile file"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    
    profile_path = CALIBRATION_DIR / f"{data.profile_name}.json"
    
    profile = {
        "profile_name": data.profile_name,
        "created": None,  # Could add timestamp
        "calibration": data.calibration
    }
    
    with open(profile_path, 'w') as f:
        json.dump(profile, f, indent=2)
    
    return {"status": "ok", "profile": data.profile_name, "path": str(profile_path)}


@router.post("/load")
async def load_calibration(data: ApplyCalibration):
    """Load calibration from a profile file"""
    if data.profile_name is None:
        raise HTTPException(status_code=400, detail="profile_name required")
    
    profile_path = CALIBRATION_DIR / f"{data.profile_name}.json"
    
    if not profile_path.exists():
        raise HTTPException(status_code=404, detail=f"Profile not found: {data.profile_name}")
    
    with open(profile_path, 'r') as f:
        profile = json.load(f)
    
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    
    # Apply loaded calibration to ROS node
    for joint, offset in profile["calibration"].items():
        if joint in ros_node.calibration:
            ros_node.set_calibration(joint, offset)
    
    return {"status": "ok", "profile": data.profile_name, "calibration": profile["calibration"]}


@router.get("/profiles")
async def list_profiles():
    """List all saved calibration profiles"""
    profiles = []
    for profile_file in CALIBRATION_DIR.glob("*.json"):
        try:
            with open(profile_file, 'r') as f:
                profile = json.load(f)
                profiles.append({
                    "name": profile.get("profile_name", profile_file.stem),
                    "file": profile_file.name
                })
        except Exception:
            pass
    
    return {"profiles": profiles}


@router.delete("/profiles/{profile_name}")
async def delete_profile(profile_name: str):
    """Delete a calibration profile"""
    profile_path = CALIBRATION_DIR / f"{profile_name}.json"
    
    if not profile_path.exists():
        raise HTTPException(status_code=404, detail=f"Profile not found: {profile_name}")
    
    profile_path.unlink()
    return {"status": "ok", "deleted": profile_name}


@router.post("/reset")
async def reset_calibration():
    """Reset calibration to default (zero) values"""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not available")
    
    for joint in ros_node.calibration:
        ros_node.set_calibration(joint, 0.0)
    
    return {"status": "ok", "message": "Calibration reset to defaults"}
