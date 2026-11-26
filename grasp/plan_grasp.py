#!/usr/bin/env python3
"""Grasp task planner (lerobot env): Contact-GraspNet grasps -> a step list ending with the object in the closed jaws. No motion.

    python grasp/plan_grasp.py samples/grasp/<frame>_grasps.json            # live joints from the server
    python grasp/plan_grasp.py <grasps.json> --pose '{"1":2186,...,"6":1808}' # plan from a given pose

Choice of grasp: among grasps our arm can use (cgn_grasps 'reachable'), top-down (<20 deg) first, then within 1 cm of
the object's footprint centre, then the best score (--index overrides).
Target: its centre moved by --offset (default 0, -0.015 m in y: the operator corrected camera grasps toward -y by
20 and 20 mm on November23 and 15 mm on November24, cause not yet known), at --grip-depth below the object's top (default 3.5 cm: the gripper point is at the fingertips, so this is how
much of the object the jaws overlap. Nov23: 1 cm slipped, 2 cm held; Nov25: 2 cm carried but the 13.6 cm box twisted
and slipped in the jaws later, so deeper). The close is followed by a second squeeze (cardboard gives under the first).
Jaw line: the grasp's. Tilt: the smallest lean in 15-45 deg that reaches both the grasp and a hover --hover above it
with a descent under the step limit. Approach: a chain from the current pose, each step within 380 ticks.

If no top-down grasp is reachable but the object can be reached from the side, the plan is a side pull instead:
gripper nearly horizontal (60-90 deg lean, jaws closing across the line from the arm base) at the object's mid-height,
advance around it, close lightly, slide it back toward the arm (to ~0.35 m), open and back away. Re-run perception
and plan_grasp afterwards (the object has moved). --pull forces a pull plan.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as cm  # noqa: E402


def choose(rows, obj):
    """Nearest-to-top-down first, then grasps within 1 cm of the footprint centre (1 cm bins), then score. November24:
    the top-scoring grasp sat on the box's near corner; the one 0.5 cm from the centre was picked by hand and held."""
    usable = [r for r in rows if r['reachable']]
    if not usable:
        sys.exit('No grasp our arm can use; move the object closer or re-run perception')
    c = cm.footprint_centre(obj)
    dist = lambda r: math.hypot(r['centre'][0] - c[0], r['centre'][1] - c[1])
    return sorted(usable, key=lambda r: (r['down_deg'] > 20, int(dist(r) / 0.01), -r['score']))[0]


PULL_TO_M = 0.35      # bring the object to about this distance from the arm base
PRE_M = 0.07          # pre-grasp: this far in front of the object along the line from the base


def pull_plan(arm, pose, obj, offset, args):
    """Side pull steps, or None. Object centre from the perception footprint (+ offset), gripped at mid-height."""
    c = np.array([np.mean(obj['x']) + offset[0], np.mean(obj['y']) + offset[1]])
    r = float(np.linalg.norm(c)); u = c / r
    d = float(np.clip(r - PULL_TO_M, 0.03, 0.08))
    z = obj['desk_z'] + float(np.clip(0.5 * obj['height'], 0.04, obj['height'] - 0.02))
    yaw = (math.degrees(math.atan2(u[1], u[0])) + 90) % 180       # jaw line across the line from the base
    pts = {'pre': np.r_[c - PRE_M * u, z], 'grab': np.r_[c + 0.01 * u, z], 'pulled': np.r_[c + 0.01 * u - d * u, z]}
    pts['retreat'] = np.r_[c - (d + PRE_M) * u, z + 0.04]
    for tilt in (60, 70, 80, 90):  # 60 needs the shortest approach chain (Nov24 model: 5 steps from ready vs 8 at 90)
        try:
            chain, cur = arm.chain(pose, pts['pre'], tilt, yaw, max_steps=8)
            segs = {}
            for name in ('grab', 'pulled', 'retreat'):  # each segment may split into up to 3 steps (an 8 cm advance is ~550 ticks)
                segs[name], cur = arm.chain(cur, pts[name], tilt, yaw, max_steps=3)
        except ValueError:
            continue
        return tilt, yaw, d, chain, pts, segs
    return None


def pull_steps(obj, pts, tilt, yaw, d, chain, segs):
    body = lambda xy: [xy[0], xy[1], obj['desk_z'] + 0.35 * obj['height']]
    here = body([np.mean(obj['x']), np.mean(obj['y'])]); moved = body(pts['pulled'][:2]); desk = obj['desk_z']
    steps = [cm.reach_cmd(s_, yaw, f'side approach {i}/{len(chain)}: {s_["tilt"]:g} deg lean' + (' (pre-grasp, 7 cm in front of the object)' if i == len(chain) else ''),
                          *((here, desk) if i == len(chain) else (None, None))) for i, s_ in enumerate(chain, 1)]
    steps.insert(len(steps) - 1, {'kind': 'open', 'target': 2162, 'note': 'open before the side approach', 'command': 'python grasp/step.py open --target 2162 --no-look'})
    def seg(name, note, e):
        n = len(segs[name])
        return [cm.reach_cmd(s_, yaw, f'{note} ({i}/{n})' if n > 1 else note, *((e, desk) if i == n and e else (None, None))) for i, s_ in enumerate(segs[name], 1)]
    steps += seg('grab', 'advance around the object (1 cm past its centre); camera: not pushed', here)
    steps.append({'kind': 'close', 'squeeze': 12, 'note': 'close lightly to slide it (12-tick squeeze)', 'command': 'python grasp/step.py close --squeeze 12' + cm.expect_args(here)})
    steps += seg('pulled', f'pull {d * 100:.0f} cm toward the arm, sliding it on the desk; camera: object moved with the jaws', moved)
    steps.append({'kind': 'open', 'target': 2162, 'note': 'release', 'command': 'python grasp/step.py open --target 2162' + cm.expect_args(moved, desk)})
    steps += seg('retreat', 'back away and up 4 cm', None)
    steps.append({'kind': 'replan', 'note': 'object moved: take a new frame, segment, cgn_grasps, plan_grasp again', 'command': '(perception -> plan_grasp)'})
    return steps


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('grasps')
    ap.add_argument('--pose', help='six joint positions as JSON (default: live from the server)')
    ap.add_argument('--offset', type=float, nargs=2, default=[0.0, -0.015], metavar=('DX', 'DY'),
                    help='added to the grasp centre (m); operator corrections: Nov23 -20, -20 mm, Nov24 -15 mm in y')
    ap.add_argument('--grip-depth', type=float, default=0.035,
                    help='gripper point (fingertips) below the object top (m); Nov25 2 cm slipped during the carry')
    ap.add_argument('--hover', type=float, default=0.05, help='hover height above the grasp point (m)')
    ap.add_argument('--index', type=int, help='use this row of the grasps file instead of the automatic choice')
    ap.add_argument('--pull', action='store_true', help='plan a side pull even if a top-down grasp is reachable')
    a = ap.parse_args()
    data = json.load(open(a.grasps)); obj = data['object']
    g = data['grasps'][a.index] if a.index is not None else choose(data['grasps'], obj)
    pose = json.loads(a.pose) if a.pose else cm.live_joints()
    arm = cm.Arm()
    gx, gy = g['centre'][0] + a.offset[0], g['centre'][1] + a.offset[1]
    grasp = np.array([gx, gy, obj['top_z'] - a.grip_depth]); hover = grasp + [0, 0, a.hover]
    yaw = float(g['jaw_yaw_deg'])
    print(f"grasp: score {g['score']:.2f}, CGN centre {g['centre']}, jaw line {yaw:g} deg, width {g['width_mm']:.0f} mm, {g['down_deg']:.0f} deg from down")
    print(f"target {grasp.round(3).tolist()} (offset {a.offset}, {a.grip_depth * 100:.1f} cm below top z {obj['top_z']}), hover {hover.round(3).tolist()}, "
          f"{math.hypot(gx, gy):.3f} m from the arm base")

    choice = None
    for tilt in (() if a.pull else cm.TILTS):
        th = arm.solve(pose, hover, tilt, yaw)
        if th is None:
            continue
        tg = arm.solve(dict(pose, **th), grasp, tilt, yaw)
        if tg is None or arm.travel(dict(pose, **th), tg) > cm.MAX_STEP_TRAVEL:
            continue
        try:  # the whole approach must chain from here too
            chain, at_hover = arm.chain(pose, hover, tilt, yaw)
        except ValueError:
            continue
        descent = arm.solve(at_hover, grasp, tilt, yaw)
        if descent is None or arm.travel(at_hover, descent) > cm.MAX_STEP_TRAVEL:
            continue
        choice = tilt; break
    if choice is None:
        print(f'no top-down grasp reachable ({math.hypot(gx, gy):.2f} m out); trying a side pull')
        pp = pull_plan(arm, pose, obj, a.offset, a)
        if pp is None:
            sys.exit('Neither a top-down grasp nor a side pull is reachable; move the object closer to the arm by hand')
        tilt, yaw_p, d, chain, pts, segs = pp
        steps = pull_steps(obj, pts, tilt, yaw_p, d, chain, segs)
        plan = {'task': 'pull', 'grasps_file': os.path.abspath(a.grasps), 'object': obj, 'offset': a.offset, 'tilt': tilt, 'jaw_yaw_deg': yaw_p,
                'pull_m': d, 'points': {k: v.round(4).tolist() for k, v in pts.items()}, 'start_pose': pose, 'steps': steps}
        path = os.path.splitext(a.grasps)[0].replace('_grasps', '') + '_pull_plan.json'
        json.dump(plan, open(path, 'w'), indent=1)
        cm.print_plan(f'Side pull ({tilt} deg lean, pull {d * 100:.0f} cm)', steps, path)
        return
    print(f'tilt {choice} deg (smallest that reaches hover and grasp)')
    # Camera checks: the object should stay standing where perception found it until the jaws close.
    obj_pt = [obj['centre_xy'][0], obj['centre_xy'][1], obj['desk_z'] + 0.35 * obj['height']]; desk = obj['desk_z']
    steps = []
    for i, s_ in enumerate(chain, 1):
        last = i == len(chain)
        steps.append(cm.reach_cmd(s_, yaw, f'approach {i}/{len(chain)}: {s_["tilt"]:g} deg lean' + (
            f' (hover, fingertips ~{(a.hover - a.grip_depth) * 100:.0f} cm above the top); camera: object still standing where planned' if last else ''),
            *((obj_pt, desk) if last else (None, None))))
    steps[-1]['if_fails'] = [{'note': 'camera CHECK or operator sees it off-centre: re-run perception from a fresh frame and re-plan from here', 'command': '(sam_click -> cgn_grasps -> plan_grasp)'}]
    steps.append({'kind': 'open', 'target': 2162, 'note': 'open to ~88 mm', 'command': 'python grasp/step.py open --target 2162' + cm.expect_args(obj_pt, desk)})
    steps.append(cm.reach_cmd({'target': [round(float(v), 3) for v in grasp], 'tilt': float(choice)}, yaw,
                              f'descend {a.hover * 100:.0f} cm to the grasp point; camera: object not pushed', obj_pt, desk))
    # Retry if the grip is weak (push < 7 ticks) or the test lift leaves the object behind: squeeze more, else regrasp deeper.
    deeper = grasp - [0, 0, 0.01]; end_pose = dict(at_hover, **descent); retry = []
    retry.append({'note': f'grip push < {7} ticks: squeeze more (close again: it re-contacts at once and adds the squeeze)',
                  'command': 'python grasp/step.py close --squeeze 24' + cm.expect_args(obj_pt)})
    td = arm.solve(end_pose, deeper, choice, yaw)
    if td is not None and arm.travel(end_pose, td) <= cm.MAX_STEP_TRAVEL and a.grip_depth + 0.01 < obj['height'] - 0.02:
        retry += [{'note': 'still weak or slipped: regrasp 1 cm deeper (a) open', 'command': 'python grasp/step.py open --target 2162' + cm.expect_args(obj_pt)},
                  {'note': '(b) descend 1 cm deeper', 'command': cm.reach_cmd({'target': [round(float(v), 3) for v in deeper], 'tilt': float(choice)}, yaw, '', obj_pt, desk)['command']},
                  {'note': '(c) close again', 'command': 'python grasp/step.py close --squeeze 24' + cm.expect_args(obj_pt)}]
    else:
        retry.append({'note': 'a 1 cm deeper regrasp is not reachable from here; open, lift back to the hover and re-plan', 'command': '(re-plan)'})
    steps.append({'kind': 'close', 'squeeze': 24, 'note': 'close to contact, squeeze 24; step.py reports the grip push (>= 7 OK; Nov23: 4 dropped, 9 carried)',
                  'command': 'python grasp/step.py close --squeeze 24' + cm.expect_args(obj_pt)})
    # Second squeeze: cardboard gives under the first, so closing again re-contacts at once and squeezes 24 ticks further
    # (Nov24 it raised the holding load 44 -> 80; Nov25 one squeeze, load 84, and the box slipped later in the carry).
    steps.append({'kind': 'close', 'squeeze': 24, 'note': 're-squeeze: close again (re-contacts and squeezes 24 further; the grip push after it should be >= 7)',
                  'command': 'python grasp/step.py close --squeeze 24' + cm.expect_args(obj_pt), 'if_fails': retry})
    plan = {'task': 'grasp', 'grasps_file': os.path.abspath(a.grasps), 'object': obj, 'grasp': g, 'offset': a.offset, 'grip_depth': a.grip_depth,
            'grasp_point': grasp.round(4).tolist(), 'hover_point': hover.round(4).tolist(), 'tilt': choice, 'jaw_yaw_deg': yaw,
            'start_pose': pose, 'expected_end_pose': dict(at_hover, **descent), 'object_point': obj_pt, 'steps': steps}
    path = os.path.splitext(a.grasps)[0].replace('_grasps', '') + '_grasp_plan.json'
    json.dump(plan, open(path, 'w'), indent=1)
    cm.print_plan('Grasp task', steps, path)


if __name__ == '__main__':
    main()
