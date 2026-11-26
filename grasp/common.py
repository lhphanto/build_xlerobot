"""Shared helpers for the grasp and basket planners (run with the lerobot env: placo + scipy). No motion.

Model calls go through xlerobot_mcp/controllers/left_reach.py, so plans match what preview_left_reach will accept.
"""
import json
import math
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MCP = os.path.join(ROOT, 'xlerobot_mcp')
sys.path.insert(0, os.path.join(MCP, 'controllers'))
import joint_control as jc  # noqa: E402
import left_reach as lr  # noqa: E402

OUT_DIR = os.path.join(ROOT, 'samples', 'grasp')
MAX_STEP_TRAVEL = 380  # ticks; preview_left_reach refuses > 400, keep margin for a few ticks of drift
TILTS = (15, 20, 25, 30, 35, 40, 45)
# Ready pose: gripper 14 cm up, ~0.22 m out, 30 deg lean, roll free (15 deg put the wrist flex at its 3147 limit and
# blocked onward chains). November24 model search: 3 steps from the folded rest
# pose (2255/1002/2785/2856), and 2-3 steps from there to the November23 grasp hovers.
READY = ((0.20, -0.08, 0.14), 30)


def group():
    return json.loads(jc.CONFIG.read_text())['groups']['left-arm']


def limits(g=None):
    g = g or group()
    return {m: g['joints'][m]['safe_raw_limits'] for m in ('1', '2', '3', '4', '5')}


def live_joints():
    """Fresh left-arm positions from the running server (read-only)."""
    out = {}
    for i in range(1, 7):
        r = subprocess.run([os.path.join(MCP, '.venv', 'bin', 'python'), os.path.join(MCP, 'client.py'), 'call', 'joint_status',
                            '--arguments', json.dumps({'group': 'left-arm', 'servo_id': i})], capture_output=True, text=True, timeout=60)
        if r.returncode:
            sys.exit(f'joint_status ID{i} failed; is the server running? {r.stderr[-300:]}')
        st = json.loads(json.loads(r.stdout)['results'][0]['stdout'])['state']
        if st['torque'] != 1 or st['moving']:
            sys.exit(f'ID{i} is not holding still (torque {st["torque"]}, moving {st["moving"]}); hold the arm first')
        out[str(i)] = st['position']
    return out


class Arm:
    def __init__(self, g=None):
        self.g = g or group()
        self.cal = self.g['kinematic_calibration']
        self.lim = limits(self.g)
        self.model, self.to_deg, _ = lr.kinematics(self.cal)

    def fk(self, raw):
        T = self.model.forward_kinematics(self.to_deg(raw))
        return T[:3, 3].copy(), T[:3, 2].copy()

    def frame(self, raw):
        return self.model.forward_kinematics(self.to_deg(raw))

    def jaw_yaw(self, raw):
        """Current jaw-closing line as top-down mode's jaw_yaw_deg (0-180). left_reach.jaw_line takes the horizontal
        direction at that yaw and projects it perpendicular to the approach, so invert that: the horizontal d with
        d = x + k * approach, d_z = 0."""
        T = self.frame(raw); x, a = T[:3, 0], T[:3, 2]
        d = x - (x[2] / a[2]) * a if abs(a[2]) > 1e-3 else x
        return round(math.degrees(math.atan2(d[1], d[0])) % 180, 1)

    def lean_deg(self, raw):
        """Angle of the approach from straight down (0 = down, 90 = horizontal), capped at 90."""
        _, a = self.fk(raw)
        return min(90.0, math.degrees(math.acos(float(np.clip(-a[2], -1, 1)))))

    def solve(self, raw, target, tilt, yaw):
        """IDs1-5 targets or None; same solver and tolerances as the server's top-down mode."""
        try:
            t, *_ = lr.solve_pointing(self.cal, raw, np.asarray(target, float), self.lim, float(tilt), yaw)
        except ValueError:
            return None
        return t

    @staticmethod
    def travel(raw, t):
        return max(abs(t[m] - raw[m]) for m in t)

    def greedy_chain(self, raw, target, tilt, yaw, max_steps=8):
        """Steps as long as the step limit allows along the straight path (position and lean together). Joint travel
        per cm is far from uniform: near an outstretched pose 1 cm can cost ~270 ticks (November24 grasp hover), so
        equal steps fail where short-then-long steps work."""
        target = np.asarray(target, float); p0, _ = self.fk(raw); t0 = self.lean_deg(raw)
        at = lambda f: (p0 + (target - p0) * f, tilt if f >= 1 else min(max(round(t0 + (tilt - t0) * f), 0), lr.MAX_APPROACH_TILT_DEG))
        cur, f0, steps = dict(raw), 0., []
        while f0 < 1 and len(steps) < max_steps:
            best = None; lo, hi = f0, 1.
            for f in [1.] + [None] * 7:
                f = f if f is not None else (lo + hi) / 2
                pk, tk = at(f); t = self.solve(cur, pk, tk, yaw)
                if t is not None and self.travel(cur, t) <= MAX_STEP_TRAVEL:
                    best = (f, pk, tk, t)
                    if f >= 1:
                        break
                    lo = f
                else:
                    hi = f
            if best is None or best[0] - f0 < 0.02:
                raise ValueError('greedy chain stuck')
            f0, pk, tk, t = best
            steps.append({'target': [round(float(v), 3) for v in pk], 'tilt': float(tk), 'travel': self.travel(cur, t), 'joints': t})
            cur = dict(cur, **t)
        if f0 < 1:
            raise ValueError('greedy chain needs more steps')
        return steps, cur

    def chain(self, raw, target, tilt, yaw, max_steps=8, greedy=False):
        """Steps [(target, tilt)] from raw to the target pose, each within MAX_STEP_TRAVEL. Tries interpolating position
        and lean together, then lean first, then position first; greedy=True then tries greedy_chain. Returns (steps,
        end_raw) or raises ValueError."""
        target = np.asarray(target, float)
        p0, _ = self.fk(raw)
        t0 = self.lean_deg(raw)
        schedules = (lambda f: (f, f), lambda f: (max(0., 2 * f - 1), min(1., 2 * f)), lambda f: (min(1., 2 * f), max(0., 2 * f - 1)))
        for n in range(1, max_steps + 1):
            for sched in (schedules if n > 1 else schedules[:1]):
                cur, steps = dict(raw), []
                for k in range(1, n + 1):
                    fp, ft = sched(k / n)
                    pk = p0 + (target - p0) * fp
                    tk = tilt if k == n else round(t0 + (tilt - t0) * ft)
                    tk = min(max(tk, 0), lr.MAX_APPROACH_TILT_DEG)
                    t = self.solve(cur, pk, tk, yaw)
                    if t is None or self.travel(cur, t) > MAX_STEP_TRAVEL:
                        break
                    steps.append({'target': [round(float(v), 3) for v in pk], 'tilt': float(tk), 'travel': self.travel(cur, t), 'joints': t})
                    cur = dict(cur, **t)
                else:
                    return steps, cur
        if greedy:  # slower; worth it when there is one target, not inside a candidate search
            try:
                return self.greedy_chain(raw, target, tilt, yaw, max_steps)
            except ValueError:
                pass
        raise ValueError(f'No chain of up to {max_steps} steps reaches {target.round(3).tolist()} at {tilt} deg within {MAX_STEP_TRAVEL} ticks per step')


def footprint_centre(obj):
    """Middle of the object's measured x/y extent (centre_xy is a point median, biased toward the faces the camera sees)."""
    return [round((obj['x'][0] + obj['x'][1]) / 2, 4), round((obj['y'][0] + obj['y'][1]) / 2, 4)]


def expect_args(expect=None, bottom_z=None):
    out = ''
    if expect is not None:
        out += ' --expect ' + ' '.join(f'{float(v):.3f}' for v in expect)
    if bottom_z is not None:
        out += f' --expect-bottom-z {float(bottom_z):.3f}'
    return out


def reach_cmd(step, yaw, note, expect=None, bottom_z=None):
    """A reach step; expect/bottom_z make step.py's camera check compare the object's measured position."""
    y = 'free' if yaw is None else f'{yaw:g}'
    x, yy, z = step['target']
    return {'kind': 'reach', 'target': step['target'], 'tilt': step['tilt'], 'jaw_yaw_deg': yaw, 'note': note,
            'expect': None if expect is None else [round(float(v), 3) for v in expect], 'expect_bottom_z': bottom_z,
            'command': f'python grasp/step.py reach {x:.3f} {yy:.3f} {z:.3f} {step["tilt"]:g} {y}' + expect_args(expect, bottom_z)}


DOWN = np.array([0., 0., -1.])


def held_object(arm, raw, hang):
    """(point on the held object's body, its bottom) from the arm model. A box gripped near its top pivots in the
    jaws and hangs straight down `hang` m below the gripper point, whatever the lean (November24 camera depth on three
    lifts at 35-40 deg: bottom 0.111 m below the gripper point for a 0.115 m hang, horizontally under it; the old
    along-the-approach model put it ~6 cm further out). The body point (60% down) is where the camera can see it."""
    p, _ = arm.fk(raw)
    return p + 0.6 * hang * DOWN, float(p[2] - hang)


def print_plan(title, steps, path):
    print(f'\n{title}: {len(steps)} steps (each needs the operator\'s go; step.py re-previews before moving)')
    for i, s in enumerate(steps, 1):
        print(f'  {i}. {s["note"]}\n     {s["command"]}')
        for r in s.get('if_fails') or []:
            print(f'       if not: {r["note"]}\n         {r["command"]}')
    print(f'plan saved: {path}')
