"""web_teleop: phone / laptop control page for the robot dog.

Serves a page on http://<robot>:8080 with a virtual joystick (touch), keyboard
driving (WASD / arrows, hold-to-move) and browser Gamepad API support, and
forwards everything to the same ROS topics as the other teleop nodes.
"""

import asyncio
import json
import os
import signal
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from dog_web import protocol
from dog_web.calibration import ALLOWED_MODES, CalibrationBridge
from dog_web.wsserver import Server, WebSocketClosed


def _is_calibration(text: str) -> bool:
    try:
        msg = json.loads(text)
    except ValueError:
        return False
    return isinstance(msg, dict) and str(msg.get('type', '')).startswith('cal_')


class WebTeleop(Node):
    def __init__(self):
        super().__init__('web_teleop')
        self.host = self.declare_parameter('host', '0.0.0.0').value
        self.port = int(self.declare_parameter('port', 8080).value)
        self.drive_timeout = float(self.declare_parameter('drive_timeout', 0.4).value)
        lim = protocol.Limits()
        for name in lim.__dict__:
            setattr(lim, name, float(self.declare_parameter(name, getattr(lim, name)).value))
        self.limits = lim
        static = self.declare_parameter('static_dir', '').value
        self.static_dir = static or os.path.join(get_package_share_directory('dog_web'), 'static')

        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.command_pub = self.create_publisher(String, 'command', 10)
        self.estop_pub = self.create_publisher(
            Bool, 'estop', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.pose_pub = self.create_publisher(Vector3, 'body_pose', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'state', self._on_state, latched)
        self.create_subscription(BatteryState, 'power', self._on_power, 10)
        self._last_power = 0.0
        # hazard guard (dog_perception): why the robot slows down or will not go forward
        self.create_subscription(String, 'perception/guard', self._on_guard, 10)
        self._guard = None

        self.mode = 'unknown'
        self.web_clients = set()
        self._aio_loop = None
        self.allow_calibration = bool(self.declare_parameter('allow_calibration', True).value)
        self.cal = CalibrationBridge(self) if self.allow_calibration else None

    # ------------------------------------------------------------ ROS side
    def _on_state(self, msg: String):
        self.mode = msg.data
        if self._aio_loop:
            self._aio_loop.call_soon_threadsafe(lambda: asyncio.ensure_future(self._broadcast_state()))

    def _on_power(self, msg: BatteryState):
        now = time.monotonic()
        if now - self._last_power < 0.5 or not self._aio_loop:
            return
        self._last_power = now
        text = protocol.power(msg.voltage, -msg.current)
        self._aio_loop.call_soon_threadsafe(lambda: asyncio.ensure_future(self._broadcast(text)))

    def _on_guard(self, msg: String):
        try:
            text = protocol.guard(msg.data)
        except (ValueError, TypeError):
            return
        state = json.loads(text)['state']
        if state == self._guard or not self._aio_loop:  # send changes only (10 Hz source)
            return
        self._guard = state
        self._aio_loop.call_soon_threadsafe(lambda: asyncio.ensure_future(self._broadcast(text)))

    async def _broadcast(self, text):
        for ws in list(self.web_clients):
            try:
                await ws.send(text)
            except WebSocketClosed:
                self.web_clients.discard(ws)

    def send_actions(self, actions: protocol.Actions):
        if actions.estop is not None:
            self.estop_pub.publish(Bool(data=actions.estop))
            self.get_logger().warning('web: E-STOP %s' % ('ENGAGED' if actions.estop else 'released'))
        if actions.command:
            self.command_pub.publish(String(data=actions.command))
            self.get_logger().info('web: %s' % actions.command)
        if actions.twist is not None:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = actions.twist
            self.twist_pub.publish(t)
        if actions.pose is not None:
            v = Vector3()
            v.y, v.z = actions.pose
            self.pose_pub.publish(v)

    # ------------------------------------------------------------ web side
    def _state_json(self):
        return protocol.state(self.mode, self.mode == 'estop', len(self.web_clients))

    async def _broadcast_state(self):
        text = self._state_json()
        for ws in list(self.web_clients):
            try:
                await ws.send(text)
            except WebSocketClosed:
                self.web_clients.discard(ws)

    async def handle_client(self, ws):
        self.web_clients.add(ws)
        self.get_logger().info('web client connected: %s (%d total)' % (ws.peer, len(self.web_clients)))
        watchdog = protocol.DriveWatchdog(self.drive_timeout)
        stop_task = asyncio.ensure_future(self._watch(ws, watchdog))
        cal_task = None
        try:
            await ws.send(protocol.hello(self.limits))
            await self._broadcast_state()
            while True:
                text = await ws.recv()
                if _is_calibration(text):
                    reply = await self._calibration(text)
                    if reply.get('type') == 'cal_info' and cal_task is None:
                        cal_task = asyncio.ensure_future(self._stream_status(ws))
                    await ws.send(json.dumps(reply))
                    continue
                actions = protocol.handle_message(text, self.limits)
                if actions.errors:
                    await ws.send(json.dumps({'type': 'error', 'message': actions.errors[0]}))
                    continue
                if actions.twist is not None:
                    watchdog.feed(actions.twist, time.monotonic())
                try:
                    self.send_actions(actions)
                except Exception as exc:  # noqa: BLE001 - keep the session alive
                    self.get_logger().error('web: failed to publish: %r' % exc)
                    await ws.send(json.dumps({'type': 'error', 'message': 'internal error'}))
        except WebSocketClosed:
            pass
        finally:
            stop_task.cancel()
            if cal_task:
                cal_task.cancel()
            self.web_clients.discard(ws)
            if watchdog.moving:
                self.send_actions(protocol.Actions(twist=(0.0, 0.0, 0.0)))
            self.get_logger().info('web client left: %s' % ws.peer)
            await self._broadcast_state()

    async def _calibration(self, text):
        """Handles one cal_* message; always returns a reply dict."""
        if not self.cal:
            return {'type': 'error', 'message': 'calibration is disabled (allow_calibration:=false)'}
        try:
            msg = json.loads(text)
            kind = msg.get('type')
            if kind == 'cal_hello':
                return await self.cal.info()
            if self.mode not in ALLOWED_MODES:
                return {'type': 'error', 'message':
                        f'calibration needs the robot passive (now: {self.mode}); press E-STOP, then release'}
            if kind == 'cal_pose':
                self.cal.pose(dict(msg.get('joints') or {}))
                return {'type': 'cal_pose_ok'}
            if kind == 'cal_set':
                return await self.cal.set_params(dict(msg.get('params') or {}))
            return {'type': 'error', 'message': f'unknown calibration message {kind!r}'}
        except (ValueError, RuntimeError, TimeoutError) as exc:
            return {'type': 'error', 'message': str(exc)}

    async def _stream_status(self, ws):
        while True:
            await asyncio.sleep(0.1)
            try:
                await ws.send(json.dumps(self.cal.status()))
            except WebSocketClosed:
                return

    async def _watch(self, ws, watchdog):
        while True:
            await asyncio.sleep(0.1)
            if watchdog.expired(time.monotonic()):
                self.get_logger().warning('web: drive stream from %s stalled - stopping' % ws.peer)
                self.send_actions(protocol.Actions(twist=(0.0, 0.0, 0.0)))

    async def serve(self):
        self._aio_loop = asyncio.get_running_loop()
        stop = asyncio.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            self._aio_loop.add_signal_handler(sig, stop.set)
        server = Server(self.static_dir, self.handle_client)
        port = await server.start(self.host, self.port)
        self.get_logger().info('web teleop on http://%s:%d' % (self.host, port))
        try:
            await stop.wait()
        finally:
            await server.stop()


def _spin(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(args=None):
    # Signals are handled in serve() so a final stop can still be published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WebTeleop()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=_spin, args=(executor,), daemon=True)
    spinner.start()
    try:
        asyncio.run(node.serve())
    finally:
        node.send_actions(protocol.Actions(twist=(0.0, 0.0, 0.0)))
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
