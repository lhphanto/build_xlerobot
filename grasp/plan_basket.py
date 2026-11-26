#!/usr/bin/env python3
"""Basket task planner (lerobot env): from the closed grasp, lift, carry over the basket, lower and release. No motion.

    python grasp/plan_basket.py samples/grasp/<frame>_grasp_plan.json [--drop X Y] [--rim-z Z]   # live joints; defaults: Nov24 basket
    python grasp/plan_basket.py <plan> --drop X Y --rim-z Z --pose '{"1":...,"6":...}'

--drop: where the object's bottom should be when released (arm frame, m), inside the basket rim. --rim-z: rim height.
Get both with reach_point.py on a frame (it prints the arm-frame point of a pixel):
    python reach_point.py --pixel U V --image FRAME.png
The object hangs (height - grip depth) straight down below the gripper point (it pivots in the jaws; November24 camera
depth), so the carry puts the gripper point over --drop whatever the lean. Steps:
  1. test lift 4 cm; the camera check must see the object's bottom rise (else lower back and use the grasp plan's retry)
  2. lifts of up to 6 cm, raising the lean only when the step limit needs it, until the bottom is --clearance (3.5 cm) above the rim
  3. carry: gripper pose (wrist roll free) putting the bottom within --drop-radius of --drop at that height
  4. lower until the bottom is at rim + --release (default 1 cm below the rim: Nov23 released ~2-3 cm above, Nov24 1 cm
     above; the operator wanted lower both times)
  5. open, then lift the empty gripper 5 cm clear of the object
  6. camera view of the basket
"""
import argparse
import itertools
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as cm  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('grasp_plan')
    ap.add_argument('--drop', type=float, nargs=2, default=[0.18, -0.20], metavar=('X', 'Y'),
                    help='object centre at release (arm frame, m), inside the rim. Default: the gripper point where the box went in on '
                         'November24 after the operator swung servo 1 back and retracted 5 cm; re-measure if the basket moved')
    ap.add_argument('--rim-z', type=float, default=-0.01, help='basket rim height (m; November23/24: -0.01)')
    ap.add_argument('--pose', help='six joint positions as JSON (default: live from the server, gripper closed on the object)')
    ap.add_argument('--clearance', type=float, default=0.035,
                    help='object bottom above the rim while carrying (m); Nov24 carried ~3.7 cm above it without touching')
    ap.add_argument('--release', type=float, default=-0.01,
                    help='object bottom height relative to the rim when opening (m; negative = below the rim). Nov24 released at +1 cm; operator wanted 2 cm lower')
    ap.add_argument('--drop-radius', type=float, default=0.02, help='allowed miss of the bottom from --drop (m)')
    a = ap.parse_args()
    gp = json.load(open(a.grasp_plan)); obj = gp['object']
    hang = obj['height'] - gp['grip_depth']; yaw = gp['jaw_yaw_deg']; tilt = gp['tilt']
    pose = json.loads(a.pose) if a.pose else cm.live_joints()
    arm = cm.Arm(); cur = {m: pose[m] for m in ('1', '2', '3', '4', '5')}
    bottom = lambda raw: arm.fk(raw)[0] + hang * cm.DOWN
    carry_z = a.rim_z + a.clearance
    steps = []

    def accept(target, t, tl, y, note):
        """Add a reach; its camera check expects the held object where the arm model puts it after the move."""
        nonlocal cur, tilt
        pt, bz = cm.held_object(arm, dict(cur, **t), hang)
        steps.append(cm.reach_cmd({'target': [round(float(v), 3) for v in target], 'tilt': float(tl)}, y, note + '; camera: object moved with the jaws', pt, bz))
        cur = dict(cur, **t); tilt = tl

    # 1-2. test lift, then lifts until the bottom clears the rim
    p, _ = arm.fk(cur)
    for k in range(8):
        if k and bottom(cur)[2] >= carry_z:
            break
        z = p[2] + (0.04 if k == 0 else min(0.06, carry_z - bottom(cur)[2] + 0.005))  # last lift only as far as the rim clearance needs
        for tl in [t for t in range(int(tilt), 61, 5)]:
            t = arm.solve(cur, [p[0], p[1], z], tl, yaw)
            if t is not None and arm.travel(cur, t) <= cm.MAX_STEP_TRAVEL:
                accept([p[0], p[1], z], t, tl, yaw, ('test lift 4 cm' if k == 0 else f'lift to z {z:.2f}')
                       + f' ({tl} deg; object bottom ~z {bottom(dict(cur, **t))[2]:.2f})')
                if k == 0:
                    g0 = [round(float(v), 3) for v in p]  # where the jaws closed (may differ from the plan after operator shifts)
                    steps[-1]['if_fails'] = [{'note': 'object did not rise (camera CHECK) or is sliding: lower back to the grasp point, then the grasp plan\'s retry (squeeze more / regrasp deeper)',
                                              'command': cm.reach_cmd({'target': g0, 'tilt': float(gp['tilt'])}, yaw, '')['command']}]
                p = np.array([p[0], p[1], z]); break
        else:
            sys.exit(f'Cannot lift above z {p[2]:.2f} within the step limit; plan so far: {[s["command"] for s in steps]}')
    else:
        sys.exit('Object bottom still below the rim after 8 lifts')

    # 3. carry: search gripper poses whose object bottom lands on --drop
    drop = np.array(a.drop); best = None
    # With the object hanging straight down the gripper point goes over the drop point. Candidates in order of
    # preference (on target first; Nov24 carries used 20-30 deg); the first that chains within 3 steps wins.
    for (dx, dy), tl in itertools.product(((0, 0), (-0.01, 0), (0, 0.01), (0, -0.01), (0.01, 0)), (30, 20, 40, 10)):
        target = np.array([drop[0] + dx, drop[1] + dy, max(p[2], carry_z + hang + 0.002)])
        t = arm.solve(cur, target, tl, None)
        if t is None:
            continue
        b = bottom(dict(cur, **t)); miss = float(np.linalg.norm(b[:2] - drop))
        if miss > a.drop_radius or b[2] < carry_z - 0.005:
            continue
        if arm.travel(cur, t) <= cm.MAX_STEP_TRAVEL:
            best = [(target, t, tl)]; break
        try:
            ch, end = arm.chain(cur, target, tl, None, max_steps=3)
        except ValueError:
            continue
        best = [(np.array(c['target']), c['joints'], c['tilt']) for c in ch]; break
    if best is None:
        sys.exit(f'No reachable carry puts the object bottom within {a.drop_radius * 100:.0f} cm of {a.drop}; pick a drop point nearer the arm')
    for i, (tg, t, tl) in enumerate(best, 1):
        accept(tg, t, tl, None, f'carry {i}/{len(best)} ({tl:g} deg, roll free; yaw may lag with a loaded arm, re-run from the held pose if it stops)'
               + (f'; object bottom ~{bottom(dict(cur, **t)).round(2).tolist()}' if i == len(best) else ''))
    # The hanging object is in the camera's view here (November24 replay: measured under the gripper at the carry end), so
    # this check needs OK: within --drop-radius of the drop point. CHECK or UNSEEN stops a run for the operator.
    body, bz = cm.held_object(arm, cur, hang)
    steps.append({'kind': 'look', 'expect': body.round(3).tolist(), 'expect_bottom_z': round(bz, 3),
                  'note': f'camera: object over the drop point (within {max(a.drop_radius, 0.03) * 100:.0f} cm) and above the rim; stops the run unless OK',
                  'command': 'python grasp/step.py look' + cm.expect_args(body, bz)})

    # 4. lower to the release height
    p, _ = arm.fk(cur); dz = bottom(cur)[2] - (a.rim_z + a.release)
    if dz > 0.005:
        target = [p[0], p[1], p[2] - dz]
        for tl in (tilt, tilt + 5, tilt - 5):
            try:
                ch, _ = arm.chain(cur, target, tl, None, max_steps=3)
            except ValueError:
                continue
            for i, c in enumerate(ch, 1):
                accept(c['target'], c['joints'], c['tilt'], None, f'lower {i}/{len(ch)}' + (f': object bottom ~{abs(a.release) * 100:.0f} cm {"below" if a.release < 0 else "above"} the rim' if i == len(ch) else ''))
            break
        else:
            print(f'note: could not lower {dz * 100:.0f} cm within 3 steps; releasing from the carry height')
    # The basket interior is below the camera's view (Nov24: drop point at rim height projects to y 1059 of 1080, 4 cm
    # lower is outside), so after the drop the camera only records a snapshot; the operator confirms the landing.
    steps.append({'kind': 'open', 'target': 2162, 'note': 'open to drop; operator confirms it landed (basket interior is below the camera view; snapshot only)',
                  'command': 'python grasp/step.py open --target 2162'})
    # lift the empty, open gripper straight up so the jaws do not drag the object and the camera can see past them
    p, _ = arm.fk(cur); up = [p[0], p[1], p[2] + 0.05]
    for tl in (tilt, tilt + 5, tilt - 5, tilt + 10):
        t = arm.solve(cur, up, tl, None)
        if t is not None and arm.travel(cur, t) <= cm.MAX_STEP_TRAVEL:
            steps.append(cm.reach_cmd({'target': [round(float(v), 3) for v in up], 'tilt': float(tl)}, None, 'lift the empty gripper 5 cm clear of the object')); break
    else:
        print('note: no single-step 5 cm lift after the release; lift by hand-planned step (step.py adjust --up 5)')
    steps.append({'kind': 'view', 'note': 'tilt the camera down to the basket (2350), snapshot, return to the calibration pose (neck only); operator checks the landing',
                  'command': 'python grasp/step.py view --tilt-to 2350'})
    plan = {'task': 'basket', 'grasp_plan': os.path.abspath(a.grasp_plan), 'drop': a.drop, 'rim_z': a.rim_z, 'hang_m': round(hang, 3),
            'start_pose': pose, 'expected_end_pose': cur, 'steps': steps}
    path = a.grasp_plan.replace('_grasp_plan.json', '') + '_basket_plan.json'
    json.dump(plan, open(path, 'w'), indent=1)
    cm.print_plan('Basket task', steps, path)


if __name__ == '__main__':
    main()
