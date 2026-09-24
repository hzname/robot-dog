"""Calibration channel of the web teleop WebSocket.

Lets an external tool (tools/autocal on a laptop with a camera) drive single
joints and tune the servo calibration live, without ROS on the laptop.

Client -> server:
  {"type": "cal_hello"}
      -> {"type": "cal_info", "joints": [...], "calibration": {joint: {field: value}},
          "geometry": {...}}  and cal_status starts streaming at 10 Hz
  {"type": "cal_pose", "joints": {"lf_thigh_joint": 30.0, ...}}   degrees, URDF convention
  {"type": "cal_set", "params": {"lf_thigh_joint.offset_deg": 47.5, ...}}
      -> {"type": "cal_set_result", "ok": bool, "reason": "..."}
Server -> client:
  {"type": "cal_status", "pulses": {joint: us}, "positions": {joint: deg},
   "power": {"voltage": V, "current": A} | null}

Only accepted while locomotion is passive (robot not standing), so a
calibration tool can never fight the gait controller.
"""

import asyncio
import math

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import BatteryState, JointState

CAL_FIELDS = ('channel', 'direction', 'offset_deg', 'pulse_min_us', 'pulse_max_us', 'range_deg',
              'min_deg', 'max_deg', 'servo_arm_mm', 'joint_arm_mm', 'rod_mm', 'axis_distance_mm',
              'coupled_to', 'coupling')
GEOMETRY = ('geometry.hip_offset', 'geometry.thigh', 'geometry.calf', 'geometry.hip_x',
            'geometry.hip_y', 'geometry.knee_direction')
ALLOWED_MODES = ('passive', 'unknown')


def _value(v: ParameterValue):
    return {ParameterType.PARAMETER_BOOL: v.bool_value,
            ParameterType.PARAMETER_INTEGER: v.integer_value,
            ParameterType.PARAMETER_DOUBLE: v.double_value,
            ParameterType.PARAMETER_STRING: v.string_value,
            ParameterType.PARAMETER_STRING_ARRAY: list(v.string_array_value)}.get(v.type)


def _param(name, value) -> Parameter:
    pv = ParameterValue()
    if isinstance(value, bool):
        raise ValueError(f'{name}: booleans are not calibration values')
    if isinstance(value, str):
        pv.type, pv.string_value = ParameterType.PARAMETER_STRING, value
    elif isinstance(value, (int, float)) and math.isfinite(value):
        pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(value)
    else:
        raise ValueError(f'{name}: expected a number or a joint name')
    return Parameter(name=name, value=pv)


async def _await_future(fut, timeout):
    end = asyncio.get_running_loop().time() + timeout
    while not fut.done():
        if asyncio.get_running_loop().time() > end:
            raise TimeoutError('no answer from the robot')
        await asyncio.sleep(0.02)
    return fut.result()


class CalibrationBridge:
    def __init__(self, node, driver='servo_driver', locomotion='locomotion'):
        self.node = node
        ns = node.get_namespace().rstrip('/')
        self.joint_pub = node.create_publisher(JointState, 'joint_commands', 10)
        self.pulses, self.positions, self.power = {}, {}, None
        node.create_subscription(JointState, 'servo_pulses', self._on_pulses, 10)
        node.create_subscription(JointState, 'joint_states', self._on_positions, 10)
        node.create_subscription(BatteryState, 'power', self._on_power, 10)
        self.get_drv = node.create_client(GetParameters, f'{ns}/{driver}/get_parameters')
        self.set_drv = node.create_client(SetParameters, f'{ns}/{driver}/set_parameters')
        self.get_loco = node.create_client(GetParameters, f'{ns}/{locomotion}/get_parameters')

    def _on_pulses(self, msg):
        self.pulses = dict(zip(msg.name, msg.position))

    def _on_positions(self, msg):
        self.positions = {n: math.degrees(p) for n, p in zip(msg.name, msg.position)}

    def _on_power(self, msg):
        self.power = {'voltage': round(msg.voltage, 3), 'current': round(-msg.current, 3)}

    def status(self):
        return {'type': 'cal_status', 'pulses': self.pulses, 'positions': self.positions,
                'power': self.power}

    async def _get(self, client, names):
        if not client.wait_for_service(timeout_sec=0.0):
            await asyncio.sleep(0.5)
            if not client.wait_for_service(timeout_sec=0.0):
                raise RuntimeError(f'{client.srv_name} is not available')
        res = await _await_future(client.call_async(GetParameters.Request(names=list(names))), 3.0)
        return {n: _value(v) for n, v in zip(names, res.values)}

    async def info(self):
        joints = (await self._get(self.get_drv, ['joint_names']))['joint_names']
        names = [f'{j}.{f}' for j in joints for f in CAL_FIELDS]
        values = await self._get(self.get_drv, names)
        cal = {j: {f: values[f'{j}.{f}'] for f in CAL_FIELDS} for j in joints}
        try:
            geo = await self._get(self.get_loco, GEOMETRY)
            geometry = {k.split('.', 1)[1]: v for k, v in geo.items() if v is not None}
        except (RuntimeError, TimeoutError):
            geometry = {}
        return {'type': 'cal_info', 'joints': joints, 'calibration': cal, 'geometry': geometry}

    async def set_params(self, params: dict):
        req = SetParameters.Request(parameters=[_param(k, v) for k, v in params.items()])
        if not self.set_drv.wait_for_service(timeout_sec=0.5):
            raise RuntimeError('servo driver is not available')
        res = await _await_future(self.set_drv.call_async(req), 3.0)
        failed = [(k, r.reason) for k, r in zip(params, res.results) if not r.successful]
        if failed:
            return {'type': 'cal_set_result', 'ok': False,
                    'reason': '; '.join(f'{k}: {why}' for k, why in failed)}
        return {'type': 'cal_set_result', 'ok': True, 'reason': ''}

    def pose(self, joints: dict):
        msg = JointState()
        for name, deg in joints.items():
            if not isinstance(deg, (int, float)) or isinstance(deg, bool) \
                    or not math.isfinite(deg) or abs(deg) > 180:
                raise ValueError(f'{name}: angle must be a number of degrees within +-180')
            msg.name.append(str(name))
            msg.position.append(math.radians(deg))
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.joint_pub.publish(msg)
