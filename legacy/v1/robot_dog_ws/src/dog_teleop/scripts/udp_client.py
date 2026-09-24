#!/usr/bin/env python3
"""
UDP client for remote robot dog teleoperation from PC.

This script runs on a remote PC and sends control commands
to the robot dog's UDP server.

Usage:
    python3 udp_client.py --host <robo_dog_ip> --port 8888
    
Controls:
    w/s      : Move forward/backward
    a/d      : Strafe left/right  
    q/e      : Rotate left/right (yaw)
    space    : Emergency stop (zero velocity)
    
    1        : Stand pose
    2        : Sit pose
    3        : Lie down pose
    
    g        : Toggle trot gait on/off
    
    +/-      : Increase/decrease speed
    
    h        : Show help
    Ctrl+C   : Quit

Dependencies:
    pip install pynput
"""

import argparse
import json
import socket
import sys
import threading
import time
from typing import Optional

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False
    print("Warning: pynput not installed. Falling back to basic input.")
    print("Install with: pip install pynput")


class UDPClient:
    """UDP client for sending teleop commands to robot dog."""
    
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(2.0)
        
        # Current state
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        self.linear_speed = 0.5
        self.angular_speed = 0.8
        
        self.gait_active = False
        
        # Key state tracking
        self.pressed_keys = set()
        self.lock = threading.Lock()
        
        self.running = True
    
    def send_velocity(self, linear_x: float, linear_y: float, angular_z: float):
        """Send velocity command."""
        message = {
            'type': 'velocity',
            'linear_x': linear_x,
            'linear_y': linear_y,
            'angular_z': angular_z
        }
        self._send(message)
    
    def send_pose(self, pose: str):
        """Send pose command."""
        message = {
            'type': 'pose',
            'pose': pose
        }
        self._send(message)
        print(f"[POSE] Set to: {pose.upper()}")
    
    def send_gait(self, active: bool):
        """Send gait command."""
        self.gait_active = active
        message = {
            'type': 'gait',
            'active': active
        }
        self._send(message)
        status = "ON" if active else "OFF"
        print(f"[GAIT] Trot: {status}")
    
    def send_speed(self, linear: float, angular: float):
        """Send speed adjustment."""
        self.linear_speed = linear
        self.angular_speed = angular
        message = {
            'type': 'speed',
            'linear': linear,
            'angular': angular
        }
        self._send(message)
        print(f"[SPEED] Linear: {linear:.1f}, Angular: {angular:.1f}")
    
    def send_stop(self):
        """Send emergency stop."""
        message = {'type': 'stop'}
        self._send(message)
        print("[STOP] Emergency stop sent")
    
    def _send(self, message: dict):
        """Send JSON message over UDP."""
        try:
            data = json.dumps(message).encode('utf-8')
            self.socket.sendto(data, (self.host, self.port))
        except Exception as e:
            print(f"[ERROR] Failed to send: {e}")
    
    def update_velocity(self):
        """Update velocity based on pressed keys."""
        with self.lock:
            keys = self.pressed_keys.copy()
        
        vx, vy, wz = 0.0, 0.0, 0.0
        
        if 'w' in keys:
            vx = 1.0
        elif 's' in keys:
            vx = -1.0
        
        if 'a' in keys:
            vy = 1.0
        elif 'd' in keys:
            vy = -1.0
        
        if 'q' in keys:
            wz = 1.0
        elif 'e' in keys:
            wz = -1.0
        
        self.target_linear_x = vx
        self.target_linear_y = vy
        self.target_angular_z = wz
        
        # Send velocity command
        self.send_velocity(vx, vy, wz)
    
    def on_key_press(self, key):
        """Handle key press event."""
        try:
            char = key.char.lower() if hasattr(key, 'char') and key.char else None
        except:
            char = None
        
        with self.lock:
            if char:
                self.pressed_keys.add(char)
        
        # Handle special keys
        if char == '1':
            self.send_pose('stand')
        elif char == '2':
            self.send_pose('sit')
        elif char == '3':
            self.send_pose('lie')
        elif char == 'g':
            self.send_gait(not self.gait_active)
        elif char == '+' or char == '=':
            self.linear_speed = min(1.0, self.linear_speed + 0.1)
            self.angular_speed = min(1.0, self.angular_speed + 0.1)
            self.send_speed(self.linear_speed, self.angular_speed)
        elif char == '-' or char == '_':
            self.linear_speed = max(0.1, self.linear_speed - 0.1)
            self.angular_speed = max(0.1, self.angular_speed - 0.1)
            self.send_speed(self.linear_speed, self.angular_speed)
        elif char == 'h':
            print_help()
        elif char == ' ':
            self.send_stop()
            with self.lock:
                self.pressed_keys.clear()
    
    def on_key_release(self, key):
        """Handle key release event."""
        try:
            char = key.char.lower() if hasattr(key, 'char') and key.char else None
        except:
            char = None
        
        with self.lock:
            if char and char in self.pressed_keys:
                self.pressed_keys.remove(char)
    
    def run_pynput(self):
        """Run using pynput keyboard listener."""
        print("Starting keyboard listener...")
        
        listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
        listener.start()
        
        try:
            while self.running and listener.is_alive():
                self.update_velocity()
                time.sleep(0.05)  # 20 Hz
        except KeyboardInterrupt:
            pass
        finally:
            listener.stop()
    
    def run_basic(self):
        """Run using basic stdin input (fallback)."""
        import select
        import termios
        import tty
        
        print("Running in basic mode (single keypress only)...")
        
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while self.running:
                # Check for input
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == '\x03':  # Ctrl+C
                        break
                    
                    # Handle key
                    if key == 'w':
                        self.send_velocity(1.0, 0.0, 0.0)
                    elif key == 's':
                        self.send_velocity(-1.0, 0.0, 0.0)
                    elif key == 'a':
                        self.send_velocity(0.0, 1.0, 0.0)
                    elif key == 'd':
                        self.send_velocity(0.0, -1.0, 0.0)
                    elif key == 'q':
                        self.send_velocity(0.0, 0.0, 1.0)
                    elif key == 'e':
                        self.send_velocity(0.0, 0.0, -1.0)
                    elif key == ' ':
                        self.send_velocity(0.0, 0.0, 0.0)
                        self.send_stop()
                    elif key == '1':
                        self.send_pose('stand')
                    elif key == '2':
                        self.send_pose('sit')
                    elif key == '3':
                        self.send_pose('lie')
                    elif key == 'g':
                        self.send_gait(not self.gait_active)
                    elif key == '+' or key == '=':
                        self.linear_speed = min(1.0, self.linear_speed + 0.1)
                        self.send_speed(self.linear_speed, self.angular_speed)
                    elif key == '-' or key == '_':
                        self.linear_speed = max(0.1, self.linear_speed - 0.1)
                        self.send_speed(self.linear_speed, self.angular_speed)
                    elif key == 'h':
                        print_help()
                    else:
                        # Stop on unknown key
                        self.send_velocity(0.0, 0.0, 0.0)
                else:
                    # No input - send zero velocity
                    self.send_velocity(0.0, 0.0, 0.0)
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def run(self):
        """Run the client."""
        if PYNPUT_AVAILABLE:
            self.run_pynput()
        else:
            self.run_basic()
        
        # Cleanup
        print("\nStopping...")
        self.send_velocity(0.0, 0.0, 0.0)
        self.send_gait(False)
        self.running = False
        self.socket.close()


def print_help():
    """Print usage instructions."""
    msg = """
╔════════════════════════════════════════════════════════════════╗
║         Robot Dog UDP Client - Remote Control                  ║
╠════════════════════════════════════════════════════════════════╣
║ Movement:           │ Poses:           │ Gait Control:         ║
║   w/s : Forward/Back│   1 : Stand      │   g : Toggle trot     ║
║   a/d : Strafe L/R  │   2 : Sit        │                       ║
║   q/e : Rotate L/R  │   3 : Lie down   │ Speed Control:        ║
║                     │                  │   +/= : Increase      ║
║   space : Stop      │                  │   -/_ : Decrease      ║
║                     │                  │                       ║
║   h : Show help     │   Ctrl+C : Quit  │                       ║
╚════════════════════════════════════════════════════════════════╝
"""
    print(msg)


def main():
    parser = argparse.ArgumentParser(description='Robot Dog UDP Teleop Client')
    parser.add_argument('--host', type=str, default='192.168.1.100',
                        help='Robot dog IP address (default: 192.168.1.100)')
    parser.add_argument('--port', type=int, default=8888,
                        help='UDP port (default: 8888)')
    
    args = parser.parse_args()
    
    print(f"Connecting to robot dog at {args.host}:{args.port}")
    print_help()
    
    client = UDPClient(args.host, args.port)
    client.run()


if __name__ == '__main__':
    main()
