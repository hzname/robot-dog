#!/usr/bin/env python3
"""ServoConfigReader
Read the unified servo_config.json and provide convenient accessors.
All nodes should import this module and use the getters.
"""
import os
import json
import threading
from typing import List, Dict, Any


class ServoConfigReader:
    """Singleton style reader for servo_config.json.
    It caches the parsed JSON and monitors file modification time.
    """
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, config_path: str = "/home/sg/robot-dog/servo_config.json"):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._init(config_path)
            return cls._instance

    def _init(self, config_path: str):
        self.config_path = config_path
        self._config = {}
        self._mtime = 0.0
        self._load()

    def _load(self) -> None:
        if not os.path.isfile(self.config_path):
            raise FileNotFoundError(f"Servo config not found: {self.config_path}")
        mtime = os.path.getmtime(self.config_path)
        if mtime != self._mtime:
            with open(self.config_path, "r", encoding="utf-8") as f:
                self._config = json.load(f)
            self._mtime = mtime

    def get_servo_config_mtime(self) -> float:
        return self._mtime

    def is_config_changed(self) -> bool:
        current = os.path.getmtime(self.config_path)
        return current != self._mtime

    def reload_if_needed(self) -> None:
        if self.is_config_changed():
            self._load()

    # ---------- Accessors ----------
    def get_all_joints(self) -> List[Dict[str, Any]]:
        """Return list of 12 joint configs ordered as in config['joints']['list'].
        Each entry includes: name, channel (servo_id), inverted, min_rad, max_rad.
        """
        self.reload_if_needed()
        joint_list = self._config.get("joints", {}).get("list", [])
        return joint_list

    def get_standing_params(self) -> Dict[str, Any]:
        body = self._config.get("body", {})
        standing = body.get("standing", {})
        comp = standing.get("_computed", {})
        return {
            "standing_height": comp.get("standing_height_m"),
            "min_height": comp.get("min_height_m"),
            "max_height": comp.get("max_height_m"),
            "foot_x_scale": body.get("standing", {}).get("foot_x_scale"),
            "foot_y_scale": body.get("standing", {}).get("foot_y_scale"),
            "hip_length": body.get("link_lengths_m", {}).get("hip"),
        }

    def get_gait_params(self) -> Dict[str, Any]:
        gait = self._config.get("gait", {})
        # default gait type parameters
        default_type = gait.get("default_type", "trot")
        gait_type = gait.get("gaits", {}).get(default_type, {})
        return {
            "period": gait_type.get("period_s"),
            "step_height": gait_type.get("step_height_m"),
            "turn_gain": gait_type.get("turn_gain"),
            "cog_pitch_gain": gait_type.get("cog_pitch_gain_m_per_rad"),
            "cog_roll_gain": gait_type.get("cog_roll_gain_m_per_rad"),
            "foot_x_scale": gait_type.get("foot_x_scale"),
            "foot_y_scale": gait_type.get("foot_y_scale"),
        }

    def get_balance_params(self) -> Dict[str, Any]:
        bal = self._config.get("balance", {})
        pid = bal.get("pid", {})
        max_corr = bal.get("max_correction", {})
        max_int = bal.get("max_integral", {})
        return {
            "pid": pid,
            "max_correction": max_corr,
            "max_integral": max_int,
            "max_correction_rate": bal.get("max_correction_rate"),
            "target_pose_clamp": bal.get("target_pose_clamp_rad"),
            "height_min": bal.get("height_min_m"),
            "height_max": bal.get("height_max_m"),
        }

    def get_servo_driver_params(self) -> Dict[str, Any]:
        drv = self._config.get("servo_driver", {})
        return {
            "bus_type": drv.get("bus_type"),
            "device_port": drv.get("device_port"),
            "pca9685_address": drv.get("pca9685_address"),
            "pwm_frequency_hz": drv.get("pwm_frequency_hz"),
        }

# Convenience singleton instance
def load_servo_config(path: str = None) -> Dict[str, Any]:
    reader = ServoConfigReader(path) if path else ServoConfigReader()
    return reader._config

def get_all_joints(path: str = None) -> List[Dict[str, Any]]:
    return ServoConfigReader(path).get_all_joints()

def get_standing_params(path: str = None) -> Dict[str, Any]:
    return ServoConfigReader(path).get_standing_params()

def get_gait_params(path: str = None) -> Dict[str, Any]:
    return ServoConfigReader(path).get_gait_params()

def get_balance_params(path: str = None) -> Dict[str, Any]:
    return ServoConfigReader(path).get_balance_params()

def get_servo_driver_params(path: str = None) -> Dict[str, Any]:
    return ServoConfigReader(path).get_servo_driver_params()

def get_servo_config_mtime(path: str = None) -> float:
    return ServoConfigReader(path).get_servo_config_mtime()

def is_config_changed(path: str = None) -> bool:
    return ServoConfigReader(path).is_config_changed()
