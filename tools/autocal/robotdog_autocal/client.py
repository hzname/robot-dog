"""WebSocket client for the robot's calibration channel (standard library only).

Talks to web_teleop on the robot (http://<robot>:8080), so the laptop needs no
ROS installation - just Python, NumPy and OpenCV.
"""

import base64
import json
import os
import queue
import socket
import struct
import threading
import time


class RobotClient:
    def __init__(self, host, port=8080, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            f'GET /ws HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n'
            f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n'
        ).encode())
        head = b''
        while b'\r\n\r\n' not in head:
            chunk = self.sock.recv(1)
            if not chunk:
                raise ConnectionError('robot closed the connection during the handshake')
            head += chunk
        if b' 101 ' not in head.split(b'\r\n', 1)[0]:
            raise ConnectionError('robot web page refused the WebSocket: ' + head.split(b'\r\n')[0].decode())
        self.sock.settimeout(None)
        self._send_lock = threading.Lock()
        self.replies = queue.Queue()
        self.latest_status = None
        self.state = None
        self._alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    # ------------------------------------------------------------ transport
    def _send(self, obj):
        data = json.dumps(obj).encode()
        mask = os.urandom(4)
        n = len(data)
        head = bytes([0x81])
        head += bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack('!H', n)
        with self._send_lock:
            self.sock.sendall(head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def _exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('robot closed the connection')
            buf += chunk
        return buf

    def _reader(self):
        try:
            while self._alive:
                b1, b2 = self._exact(2)
                n = b2 & 0x7F
                if n == 126:
                    (n,) = struct.unpack('!H', self._exact(2))
                elif n == 127:
                    (n,) = struct.unpack('!Q', self._exact(8))
                payload = self._exact(n)
                if b1 & 0x0F != 1:
                    continue
                msg = json.loads(payload.decode())
                kind = msg.get('type')
                if kind == 'cal_status':
                    self.latest_status = msg
                elif kind == 'state':
                    self.state = msg.get('mode')
                elif kind in ('cal_info', 'cal_set_result', 'cal_pose_ok', 'error'):
                    self.replies.put(msg)
        except (OSError, ConnectionError, ValueError):
            self.replies.put({'type': 'error', 'message': 'connection to the robot lost'})

    def _call(self, obj, timeout=5.0):
        self._send(obj)
        try:
            reply = self.replies.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError(f'no answer to {obj["type"]}') from None
        if reply.get('type') == 'error':
            raise RuntimeError(reply.get('message'))
        return reply

    # ------------------------------------------------------------ calibration API
    def info(self):
        return self._call({'type': 'cal_hello'})

    def pose(self, joints_deg):
        self._call({'type': 'cal_pose', 'joints': joints_deg})

    def status(self, wait=1.0):
        end = time.time() + wait
        while self.latest_status is None and time.time() < end:
            time.sleep(0.02)
        if self.latest_status is None:
            raise TimeoutError('no servo status from the robot')
        return self.latest_status

    def set_params(self, params):
        r = self._call({'type': 'cal_set', 'params': params})
        return r.get('ok', False), r.get('reason', '')

    def estop(self, active=True):
        self._send({'type': 'estop', 'active': bool(active)})

    def close(self):
        self._alive = False
        try:
            self.sock.close()
        except OSError:
            pass
