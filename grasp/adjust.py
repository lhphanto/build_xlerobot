#!/usr/bin/env python3
"""Plan an operator adjustment of the left gripper from its current pose (lerobot env). No motion.

    python grasp/adjust.py --right 1.5                 # cm; operator's view from behind the arm: right = -y
    python grasp/adjust.py --retract 3                 # cm toward the arm base (horizontal); --extend is the opposite
    python grasp/adjust.py --swing-left 10             # degrees about the base (servo 1 down); --swing-right (servo 1 up)
    python grasp/adjust.py --down 2 --forward 1        # combine freely; --up/--down/--forward/--back/--left/--right in cm
    [--pose JSON] [--free-roll] [--json]

The lean (angle from straight down) is kept, and so is the jaw line unless --free-roll (use it while carrying, where
the wrist roll does not matter). Moves over the 380-tick step limit are split into up to 3 steps.
`step.py adjust ...` plans with this and runs the steps on one go. November24 the operator asked for: right 15 mm at the
hover; a swing and 2 + 3 cm retracts over the basket.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as cm  # noqa: E402


def target_of(p, a):
    q = np.array(p, float)
    q += [0.01 * (a.forward - a.back), 0.01 * (a.left - a.right), 0.01 * (a.up - a.down)]
    r = np.hypot(q[0], q[1]); u = q[:2] / r
    q[:2] += 0.01 * (a.extend - a.retract) * u
    th = math.radians(a.swing_left - a.swing_right)  # left = +y side = counter-clockwise from above = servo 1 down
    q[:2] = [q[0] * math.cos(th) - q[1] * math.sin(th), q[0] * math.sin(th) + q[1] * math.cos(th)]
    return q


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for n in ('right', 'left', 'forward', 'back', 'up', 'down', 'retract', 'extend'):
        ap.add_argument('--' + n, type=float, default=0., metavar='CM')
    ap.add_argument('--swing-left', type=float, default=0., metavar='DEG'); ap.add_argument('--swing-right', type=float, default=0., metavar='DEG')
    ap.add_argument('--pose'); ap.add_argument('--free-roll', action='store_true'); ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    pose = json.loads(a.pose) if a.pose else cm.live_joints()
    arm = cm.Arm(); cur = {m: pose[m] for m in ('1', '2', '3', '4', '5')}
    p, _ = arm.fk(cur); lean = round(arm.lean_deg(cur)); yaw = None if a.free_roll else arm.jaw_yaw(cur)
    q = target_of(p, a)
    if np.linalg.norm(q - p) < 0.002:
        sys.exit('No adjustment given (or under 2 mm)')
    try:
        chain, end = arm.chain(cur, q, lean, yaw, max_steps=3, greedy=True)
    except ValueError as e:
        sys.exit(f'Not reachable from here: {e}')
    move = (q - p).round(3).tolist()
    steps = [cm.reach_cmd(s, yaw, f'adjust {i}/{len(chain)}: gripper {move} m in total, lean {lean} deg, '
                          + ('roll free' if yaw is None else f'jaw line {yaw:g} deg') + f' (travel {s["travel"]})') for i, s in enumerate(chain, 1)]
    if a.json:
        print(json.dumps(steps)); return
    print(f'from {p.round(3).tolist()} to {q.round(3).tolist()}; joints {cur} -> {end}')
    cm.print_plan('Adjustment', steps, '(not saved)')


if __name__ == '__main__':
    main()
