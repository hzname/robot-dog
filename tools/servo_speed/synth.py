"""Deterministic synthetic current-trace generator for the servo speed probe.

The generator drives the bench ramp the robot will run (plan 01-09): a servo
sweeps +-amp_deg around center_us at each commanded speed (rad/s of the
command angle, 50 Hz PWM staircase of 20 ms steps) while the INA219 records
shunt and bus registers. Rows are (t_s, stroke_id, direction, cmd_us,
shunt_raw, bus_raw) tuples -- the six CSV fields of
tools/servo_speed/README.md; stroke_id is -1 outside strokes and 0.. without
gaps. meta carries the fields of <csv>.meta.json.

Servo model: a P controller on the command error with speed cap ``v_max`` and
acceleration cap ``a_max``; the current is
``i_hold + c*[|vel| > 0.05] + a_coef*|vel| + b_coef*|acc|`` low-passed with
``tau_i`` and measured with Gaussian noise ``noise_a`` A (shunt_raw is the
signed shunt register, LSB 10 uV; bus_raw carries voltage in bits 15:3, LSB
4 mV, only every 8th row). kp, a_max and tau_i keep the prototype values and
b_coef was halved from the prototype 0.0002 to 0.0001, so that analyze.py
sees a sharp kink (v_sat is the first grid speed not below v_max; v_dur 2-4 %
low). Reason for the change: at 0.0002 the 1 ms front-phase jitter of the
current edges inflated the distances between saturated speeds in the
ramp_timing run (6.0 rad/s, noise 8 mA, up direction: 1.33 x eps, the kink
was lost); at 0.0001 the worst saturated pair stays at 0.62 x eps over a
seed sweep with the kink still sharp. If the analyzer tests fail, adjust
those physical constants within reason and record the reason here -- never
loosen the analysis thresholds or the test tolerances.

ramp_timing=True repeats the phase sequence of the bench ramp: APPROACH
(hold at the center, then a linear ramp to -amp_deg at speeds[0]) and then,
per speed, strokes back to back (even stroke -amp -> +amp, odd the other
way), each followed by a hold_s hold at the stroke number and direction; a
rest_s rest at -amp (stroke_id -1, direction 0) sits between speed groups,
except after the last one. Time is one continuous 1 ms grid, the servo state
carries across strokes and the command the servo sees is delayed by
U(0, jitter_max_s) per stroke (and one delay for the whole APPROACH).
"""

import math

import numpy as np

TICK = 0.001    # [s] sample step
STEP_S = 0.020  # [s] PWM staircase step (50 Hz)

DEFAULT_SPEEDS = (1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0,
                  8.0, 9.0, 10.0)
EXTENDED_SPEEDS = DEFAULT_SPEEDS + (11.0, 12.0)


def _stroke_fn(v, t_cmd, start, end, span):
    """Command angle (rad) of one stroke: 20 ms staircase, start .. end."""
    sign = 1.0 if end > start else -1.0

    def f(u):
        if u < 0.0:
            return start
        if u >= t_cmd:
            return end
        moved = v * math.floor(u / STEP_S + 1e-9) * STEP_S
        if moved > span:
            moved = span
        return start + sign * moved

    return f


def _approach_fn(v0, t_app, amp_rad):
    """Command angle (rad) of the APPROACH linear part: center -> -amp."""

    def f(u):
        if u < 0.0:
            return 0.0
        if u >= t_app:
            return -amp_rad
        moved = v0 * math.floor(u / STEP_S + 1e-9) * STEP_S
        if moved > amp_rad:
            moved = amp_rad
        return -moved

    return f


def _plain_segments(rng, speeds, strokes_per_speed, amp_rad, hold_s, pre_s,
                    jitter_max_s, friction_a):
    """Strokes back to back, each with a pre_s pre-roll: the model without ramp_timing."""
    segments = []
    span = 2.0 * amp_rad
    n_pre = int(round(pre_s / TICK))
    n_hold = int(round(hold_s / TICK))
    for s_idx, v in enumerate(speeds):
        v = float(v)
        t_cmd = span / v
        n_cmd = int(math.ceil(t_cmd / TICK - 1e-9))
        for j in range(strokes_per_speed):
            sid = s_idx * strokes_per_speed + j
            dirn = 1 if j % 2 == 0 else -1
            start = -amp_rad if dirn == 1 else amp_rad
            end = -start
            segments.append({'n': n_pre, 'sid': -1, 'dirn': 0,
                             'f': (lambda u, s=start: s), 'tau': 0.0,
                             'c': friction_a[0]})
            tau = rng.uniform(0.0, jitter_max_s)
            segments.append({'n': n_cmd + n_hold, 'sid': sid, 'dirn': dirn,
                             'f': _stroke_fn(v, t_cmd, start, end, span),
                             'tau': tau,
                             'c': friction_a[0] if dirn == 1 else friction_a[1]})
    return segments


def _ramp_segments(rng, speeds, strokes_per_speed, amp_rad, hold_s, rest_s,
                   jitter_max_s, friction_a):
    """The 01-09 ramp timing: APPROACH, strokes with holds, REST between groups."""
    segments = []
    span = 2.0 * amp_rad
    n_hold = int(round(hold_s / TICK))
    segments.append({'n': n_hold, 'sid': -1, 'dirn': 0,
                     'f': (lambda u: 0.0), 'tau': 0.0, 'c': friction_a[0]})
    v0 = float(speeds[0])
    t_app = amp_rad / v0
    n_app = int(math.ceil(t_app / TICK - 1e-9))
    segments.append({'n': n_app, 'sid': -1, 'dirn': 0,
                     'f': _approach_fn(v0, t_app, amp_rad),
                     'tau': rng.uniform(0.0, jitter_max_s), 'c': friction_a[0]})
    n_rest = int(round(rest_s / TICK))
    for g, v in enumerate(speeds):
        v = float(v)
        t_cmd = span / v
        n_cmd = int(math.ceil(t_cmd / TICK - 1e-9))
        for j in range(strokes_per_speed):
            sid = g * strokes_per_speed + j
            dirn = 1 if j % 2 == 0 else -1
            start = -amp_rad if dirn == 1 else amp_rad
            end = -start
            segments.append({'n': n_cmd + n_hold, 'sid': sid, 'dirn': dirn,
                             'f': _stroke_fn(v, t_cmd, start, end, span),
                             'tau': rng.uniform(0.0, jitter_max_s),
                             'c': friction_a[0] if dirn == 1 else friction_a[1]})
        if g < len(speeds) - 1:
            segments.append({'n': n_rest, 'sid': -1, 'dirn': 0,
                             'f': (lambda u, a=-amp_rad: a), 'tau': 0.0,
                             'c': friction_a[0]})
    return segments


def synth_run(v_max, seed=0, noise_a=0.008, shunt_scale=1.0,
              speeds=DEFAULT_SPEEDS, strokes_per_speed=10, amp_deg=25.0,
              us_per_deg=9.444, center_us=1400.0, shunt_ohm=0.1, hold_s=0.4,
              pre_s=0.3, kp=800.0, a_max=8000.0, jitter_max_s=0.020,
              friction_a=(0.14, 0.06), a_coef=0.06, b_coef=0.0001, i_hold=0.15,
              tau_i=0.002, drop_fraction=0.0, channel=1,
              stop_reason='completed', rest_s=3.0, ramp_timing=False):
    """Generate one synthetic run; returns (rows, meta); see the module docstring."""
    rng = np.random.default_rng(seed)
    amp_rad = math.radians(amp_deg)
    if ramp_timing:
        segments = _ramp_segments(rng, speeds, strokes_per_speed, amp_rad,
                                  hold_s, rest_s, jitter_max_s, friction_a)
    else:
        segments = _plain_segments(rng, speeds, strokes_per_speed, amp_rad,
                                   hold_s, pre_s, jitter_max_s, friction_a)

    rows = []
    ang = 0.0
    vel = 0.0
    i_f = float(i_hold)
    t = 0.0
    n_row = 0
    for seg in segments:
        f = seg['f']
        tau = seg['tau']
        c = seg['c']
        for k in range(seg['n']):
            u = k * TICK
            issued_rad = f(u)
            target_rad = f(u - tau)
            vel_des = kp * (target_rad - ang)
            if vel_des > v_max:
                vel_des = v_max
            elif vel_des < -v_max:
                vel_des = -v_max
            dv = vel_des - vel
            lim = a_max * TICK
            if dv > lim:
                dv = lim
            elif dv < -lim:
                dv = -lim
            vel += dv
            ang += vel * TICK
            i_raw = (i_hold + (c if abs(vel) > 0.05 else 0.0)
                     + a_coef * abs(vel) + b_coef * abs(dv) / TICK)
            i_f += (i_raw - i_f) * TICK / tau_i
            noise = rng.normal(0.0, noise_a)
            roll = rng.random()
            if roll < drop_fraction:
                t += TICK
                continue
            measured = i_f + noise
            shunt_raw = int(round(measured * shunt_ohm / 1e-5 * shunt_scale))
            bus_raw = None
            if n_row % 8 == 0:
                bus_raw = int(round((6.0 - 0.25 * measured) / 0.004)) << 3 | 0x2
            cmd_us = center_us + issued_rad * us_per_deg * 180.0 / math.pi
            rows.append((t, seg['sid'], seg['dirn'], cmd_us, shunt_raw, bus_raw))
            n_row += 1
            t += TICK

    meta = {
        'shunt_ohm': shunt_ohm,
        'us_per_deg': us_per_deg,
        'amp_deg': amp_deg,
        'center_us': center_us,
        'channel': channel,
        'speeds_rad_s': [float(s) for s in speeds],
        'strokes_per_speed': strokes_per_speed,
        'hold_s': hold_s,
        'rest_s': rest_s,
        'ina_config': 0x199F,
        'stop_reason': stop_reason,
    }
    return rows, meta
