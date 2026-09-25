"""URDF generator for the robot dog, driven by dog_bringup/config/robot.yaml.

No xacro: launch files call build_urdf() directly, so geometry lives in one
YAML file shared with the locomotion controller.

Joint convention matches dog_control/kinematics.hpp:
  <leg>_hip_joint   axis +x, zero = leg vertical
  <leg>_thigh_joint axis +y, zero = thigh straight down, + swings foot back
  <leg>_calf_joint  axis +y, zero = knee straight
"""

import math

import yaml

LEGS = (('lf', 1, 1), ('rf', 1, -1), ('lr', -1, 1), ('rr', -1, -1))  # name, front, side

DEFAULT_DESCRIPTION = {
    'body_length': 0.23, 'body_width': 0.10, 'body_height': 0.05,
    'body_mass': 0.80, 'hip_mass': 0.06, 'thigh_mass': 0.08, 'calf_mass': 0.03,
    'foot_radius': 0.012, 'servo_effort': 1.1, 'servo_velocity': 6.0, 'sim_p_gain': 25.0,
    'hip_limits_deg': [-40.0, 40.0], 'thigh_limits_deg': [-45.0, 135.0],
    'calf_limits_deg': [-165.0, -15.0],
}


def load_config(path):
    """Returns (geometry, description) dicts from a robot.yaml file.
    The description also carries the perception sensors under 'sensors'."""
    with open(path) as f:
        data = yaml.safe_load(f)
    params = data['/**']['ros__parameters']
    desc = dict(DEFAULT_DESCRIPTION)
    desc.update(params.get('description', {}))
    desc['sensors'] = dict(params.get('sensors', {}))
    return params['geometry'], desc


def sensor_frames(sensors):
    """[(frame, kind, xyz, rpy)] of the perception sensors; rpy makes the
    sensor x axis point along the beam (lidar: the dip direction of its scan
    plane)."""
    s = sensors or {}
    out = []
    if s.get('x_lidar'):
        tilt, yaw = math.radians(s['x_lidar_tilt_deg']), math.radians(s['x_lidar_yaw_deg'])
        for name, side in (('lidar_left', 1), ('lidar_right', -1)):
            # the left lidar dips towards the right (-yaw) and vice versa
            out.append((name, 'lidar', (s['x_lidar_x'], side * s['x_lidar_y'], s['x_lidar_z']),
                        (0.0, tilt, -side * yaw)))
    if s.get('gs2'):
        out.append(('gs2', 'gs2', (s['gs2_x'], s['gs2_y'], s['gs2_z']),
                    (0.0, math.radians(s['gs2_pitch_deg']), 0.0)))
    if s.get('tof'):
        for k, name in enumerate(s['tof_names']):
            out.append((f'tof_{name}', 'tof', (s['tof_x'][k], s['tof_y'][k], s['tof_z'][k]),
                        (0.0, math.radians(s['tof_pitch_deg'][k]), math.radians(s['tof_yaw_deg'][k]))))
    return out


def _box_inertia(m, x, y, z):
    return (m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12)


def _cyl_inertia(m, r, h):
    """Cylinder along z."""
    ixx = m * (3 * r * r + h * h) / 12
    return (ixx, ixx, m * r * r / 2)


def _inertial(m, ixyz, xyz=(0, 0, 0)):
    ixx, iyy, izz = ixyz
    return (
        f'<inertial><origin xyz="{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}"/>'
        f'<mass value="{m:.4f}"/>'
        f'<inertia ixx="{ixx:.3e}" ixy="0" ixz="0" iyy="{iyy:.3e}" iyz="0" izz="{izz:.3e}"/>'
        '</inertial>')


def _limit(deg_range, effort, velocity):
    lo, hi = (math.radians(v) for v in deg_range)
    return f'<limit lower="{lo:.4f}" upper="{hi:.4f}" effort="{effort}" velocity="{velocity}"/>'


def stand_angles(geometry, height):
    """(hip, thigh, calf) [rad] with the foot straight under the thigh axis at
    `height` below it (same IK as dog_control, knee bent backwards)."""
    L2, L3 = geometry['thigh'], geometry['calf']
    r = min(max(height, abs(L2 - L3) + 1e-3), L2 + L3 - 1e-3)
    calf = -math.acos(max(-1.0, min(1.0, (r * r - L2 * L2 - L3 * L3) / (2 * L2 * L3))))
    thigh = math.atan2(-L3 * math.sin(calf), L2 + L3 * math.cos(calf))
    return 0.0, thigh, calf


def build_urdf(geometry, description=None, gazebo=False, namespace='dog', initial=None):
    g = geometry
    d = dict(DEFAULT_DESCRIPTION)
    d.update(description or {})
    L1, L2, L3 = g['hip_offset'], g['thigh'], g['calf']
    bl, bw, bh = d['body_length'], d['body_width'], d['body_height']
    r_leg = 0.012
    out = ['<?xml version="1.0"?>', '<robot name="robot_dog">']
    out.append(
        '<material name="body"><color rgba="0.20 0.22 0.25 1"/></material>'
        '<material name="leg"><color rgba="0.85 0.55 0.15 1"/></material>'
        '<material name="foot"><color rgba="0.1 0.1 0.1 1"/></material>')

    out.append('<link name="base_link"/>')
    out.append(
        '<link name="trunk">'
        f'<visual><geometry><box size="{bl} {bw} {bh}"/></geometry><material name="body"/></visual>'
        f'<collision><geometry><box size="{bl} {bw} {bh}"/></geometry></collision>'
        + _inertial(d['body_mass'], _box_inertia(d['body_mass'], bl, bw, bh)) + '</link>')
    out.append('<joint name="base_to_trunk" type="fixed"><parent link="base_link"/>'
               '<child link="trunk"/></joint>')
    out.append('<link name="imu_link"/>'
               '<joint name="imu_joint" type="fixed"><parent link="trunk"/>'
               '<child link="imu_link"/><origin xyz="0 0 0.02"/></joint>')
    for frame, kind, xyz, rpy in sensor_frames(d.get('sensors')):
        vis = ('<visual><geometry><cylinder radius="0.019" length="0.03"/></geometry>'
               '<material name="foot"/></visual>') if kind == 'lidar' else \
            ('<visual><geometry><box size="0.011 0.026 0.024"/></geometry>'
             '<material name="foot"/></visual>') if kind == 'gs2' else \
            ('<visual><geometry><box size="0.006 0.018 0.012"/></geometry>'
             '<material name="foot"/></visual>')
        out.append(f'<link name="{frame}">{vis}</link>'
                   f'<joint name="{frame}_joint" type="fixed"><parent link="trunk"/>'
                   f'<child link="{frame}"/><origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" '
                   f'rpy="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"/></joint>')

    eff, vel = d['servo_effort'], d['servo_velocity']
    for name, front, side in LEGS:
        hx, hy = front * g['hip_x'], side * g['hip_y']
        # hip: abduction joint, link spans the lateral offset
        out.append(
            f'<joint name="{name}_hip_joint" type="revolute"><parent link="trunk"/>'
            f'<child link="{name}_hip"/><origin xyz="{hx} {hy} 0"/><axis xyz="1 0 0"/>'
            + _limit(d['hip_limits_deg'], eff, vel) + '</joint>')
        out.append(
            f'<link name="{name}_hip"><visual><origin xyz="0 {side * L1 / 2:.4f} 0" rpy="1.5708 0 0"/>'
            f'<geometry><cylinder radius="0.018" length="{L1}"/></geometry><material name="body"/></visual>'
            + _inertial(d['hip_mass'], _cyl_inertia(d['hip_mass'], 0.018, L1), (0, side * L1 / 2, 0))
            + '</link>')
        # thigh
        out.append(
            f'<joint name="{name}_thigh_joint" type="revolute"><parent link="{name}_hip"/>'
            f'<child link="{name}_thigh"/><origin xyz="0 {side * L1} 0"/><axis xyz="0 1 0"/>'
            + _limit(d['thigh_limits_deg'], eff, vel) + '</joint>')
        out.append(
            f'<link name="{name}_thigh"><visual><origin xyz="0 0 {-L2 / 2}"/>'
            f'<geometry><cylinder radius="{r_leg}" length="{L2}"/></geometry><material name="leg"/></visual>'
            + _inertial(d['thigh_mass'], _cyl_inertia(d['thigh_mass'], r_leg, L2), (0, 0, -L2 / 2))
            + '</link>')
        # calf
        out.append(
            f'<joint name="{name}_calf_joint" type="revolute"><parent link="{name}_thigh"/>'
            f'<child link="{name}_calf"/><origin xyz="0 0 {-L2}"/><axis xyz="0 1 0"/>'
            + _limit(d['calf_limits_deg'], eff, vel) + '</joint>')
        out.append(
            f'<link name="{name}_calf"><visual><origin xyz="0 0 {-L3 / 2}"/>'
            f'<geometry><cylinder radius="{r_leg * 0.8:.4f}" length="{L3}"/></geometry>'
            '<material name="leg"/></visual>'
            + _inertial(d['calf_mass'], _cyl_inertia(d['calf_mass'], r_leg * 0.8, L3), (0, 0, -L3 / 2))
            + '</link>')
        # foot (contact point)
        fr = d['foot_radius']
        out.append(
            f'<joint name="{name}_foot_joint" type="fixed"><parent link="{name}_calf"/>'
            f'<child link="{name}_foot"/><origin xyz="0 0 {-L3}"/></joint>')
        out.append(
            f'<link name="{name}_foot"><visual><geometry><sphere radius="{fr}"/></geometry>'
            '<material name="foot"/></visual>'
            f'<collision><geometry><sphere radius="{fr}"/></geometry></collision>'
            + _inertial(0.005, (1e-6, 1e-6, 1e-6)) + '</link>')

    if gazebo:
        out.append(_gazebo_extras(namespace, d['sim_p_gain'], d['servo_velocity'], initial))
        out.append(_gazebo_sensors(namespace, d.get('sensors')))
    out.append('</robot>')
    return '\n'.join(out)


def joint_names():
    return [f'{leg}_{j}_joint' for leg, _, _ in LEGS for j in ('hip', 'thigh', 'calf')]


def sim_command_topic(namespace, joint):
    return f'/{namespace}/sim/{joint}/cmd_pos'


def _gazebo_extras(ns, p_gain, vmax, initial=None):
    parts = []
    for k, joint in enumerate(joint_names()):
        init = f'<initial_position>{initial[k % 3]:.4f}</initial_position>' if initial else ''
        # Velocity-command mode: the joint behaves like a hobby servo, a
        # position loop whose speed and torque are capped by the URDF limits.
        parts.append(
            '<gazebo><plugin filename="gz-sim-joint-position-controller-system" '
            'name="gz::sim::systems::JointPositionController">'
            f'<joint_name>{joint}</joint_name><topic>{sim_command_topic(ns, joint)}</topic>'
            '<use_velocity_commands>true</use_velocity_commands>'
            f'<p_gain>{p_gain}</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>{init}'
            f'<cmd_max>{vmax}</cmd_max><cmd_min>{-vmax}</cmd_min></plugin></gazebo>')
    parts.append(
        '<gazebo><plugin filename="gz-sim-joint-state-publisher-system" '
        f'name="gz::sim::systems::JointStatePublisher"><topic>/{ns}/sim/joint_states</topic>'
        '</plugin></gazebo>')
    parts.append(
        '<gazebo><plugin filename="gz-sim-odometry-publisher-system" '
        'name="gz::sim::systems::OdometryPublisher">'
        f'<odom_topic>/{ns}/sim/odom</odom_topic><odom_frame>odom</odom_frame>'
        '<robot_base_frame>base_link</robot_base_frame><dimensions>3</dimensions>'
        '<odom_publish_frequency>50</odom_publish_frequency></plugin></gazebo>')
    parts.append(
        '<gazebo reference="trunk"><sensor name="imu" type="imu"><always_on>1</always_on>'
        f'<update_rate>100</update_rate><topic>/{ns}/sim/imu</topic></sensor></gazebo>')
    for leg, _, _ in LEGS:
        parts.append(f'<gazebo reference="{leg}_foot"><mu1>1.2</mu1><mu2>1.2</mu2></gazebo>')
    return '\n'.join(parts)


def sim_sensor_topic(namespace, frame):
    return f'/{namespace}/sim/{frame}/scan'


def _gazebo_sensors(ns, sensors):
    """gpu_lidar sensors: the X lidars as 360 deg single-beam scanners, each
    VL53L1X as a small 5 x 5 ray cone (dog_gazebo/tof_bridge turns it into a
    sensor_msgs/Range like the real sensor)."""
    s = sensors or {}
    parts = []
    for frame, kind, _, _ in sensor_frames(s):
        if kind == 'gs2':  # a fan of rays in the sensor's x-y plane (tilted down with it)
            half = math.radians(s['gs2_fov_deg']) / 2
            n, rate, noise = int(s['gs2_samples']), s['gs2_rate'], s['gs2_noise']
            scan = (f'<horizontal><samples>{n}</samples><min_angle>{-half:.4f}</min_angle>'
                    f'<max_angle>{half:.4f}</max_angle></horizontal>')
            rng = f'<min>{s["gs2_range_min"]}</min><max>{s["gs2_range_max"]}</max><resolution>0.001</resolution>'
        elif kind == 'lidar':
            n, rate, noise = int(s['x_lidar_samples']), s['x_lidar_rate'], s['x_lidar_noise']
            scan = (f'<horizontal><samples>{n}</samples><min_angle>{-math.pi:.5f}</min_angle>'
                    f'<max_angle>{math.pi * (1 - 2.0 / n):.5f}</max_angle></horizontal>')
            rng = '<min>0.03</min><max>12.0</max><resolution>0.001</resolution>'
        else:
            half = math.radians(s['tof_fov_deg']) / 2
            n, rate, noise = 5, s['tof_rate'], s['tof_noise']
            scan = (f'<horizontal><samples>5</samples><min_angle>{-half:.4f}</min_angle>'
                    f'<max_angle>{half:.4f}</max_angle></horizontal>'
                    f'<vertical><samples>5</samples><min_angle>{-half:.4f}</min_angle>'
                    f'<max_angle>{half:.4f}</max_angle></vertical>')
            rng = '<min>0.04</min><max>1.3</max><resolution>0.001</resolution>'
        parts.append(
            f'<gazebo reference="{frame}"><sensor name="{frame}" type="gpu_lidar">'
            f'<always_on>1</always_on><update_rate>{rate}</update_rate>'
            f'<topic>{sim_sensor_topic(ns, frame)}</topic><gz_frame_id>{frame}</gz_frame_id>'
            f'<lidar><scan>{scan}</scan><range>{rng}</range>'
            f'<noise><type>gaussian</type><mean>0</mean><stddev>{noise}</stddev></noise>'
            '</lidar></sensor></gazebo>')
    return '\n'.join(parts)


def main():
    """CLI: generate_urdf <robot.yaml> [--gazebo] > robot.urdf"""
    import argparse
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('config')
    ap.add_argument('--gazebo', action='store_true')
    args = ap.parse_args()
    geometry, description = load_config(args.config)
    print(build_urdf(geometry, description, gazebo=args.gazebo))


if __name__ == '__main__':
    main()
