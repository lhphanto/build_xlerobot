#!/usr/bin/env python3
"""Plan the chain from the current left-arm pose to the ready pose (lerobot env). No motion.

    python grasp/plan_ready.py [--pose JSON] [--json]
--json prints only the step list (used by `step.py ready`, which runs the steps one after another).
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as cm  # noqa: E402


def plan(arm, pose, target, tilt):
    """[(step, yaw)] to the ready pose. With the roll free each step's solution may flip the wrist roll and blow the
    step limit (November24: no chain from the grasp hover); then keep the current jaw line, and if that fails too,
    lift 5 cm first (same lean and jaw line)."""
    try:
        return [(s, None) for s in arm.chain(pose, target, tilt, None, max_steps=8, greedy=True)[0]]
    except ValueError:
        pass
    yaw = arm.jaw_yaw(pose)
    try:
        return [(s, yaw) for s in arm.chain(pose, target, tilt, yaw, max_steps=8, greedy=True)[0]]
    except ValueError:
        pass
    p, _ = arm.fk(pose); lean = round(arm.lean_deg(pose))
    up, cur = arm.chain(pose, p + [0, 0, 0.05], lean, yaw, max_steps=2, greedy=True)
    return [(s, yaw) for s in up] + [(s, yaw) for s in arm.chain(cur, target, tilt, yaw, max_steps=8, greedy=True)[0]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pose'); ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    pose = json.loads(a.pose) if a.pose else cm.live_joints()
    arm = cm.Arm(); target, tilt = cm.READY
    p, _ = arm.fk(pose)
    if max(abs(p - target)) < 0.01 and abs(arm.lean_deg(pose) - tilt) < 5:
        steps = []
    else:
        chain = plan(arm, pose, target, tilt)
        steps = [cm.reach_cmd(s, y, f'ready {i}/{len(chain)}: {s["tilt"]:g} deg lean' + ('' if y is None else f', jaw line {y:g} deg'))
                 for i, (s, y) in enumerate(chain, 1)]
    if a.json:
        print(json.dumps(steps)); return
    if not steps:
        print('already at the ready pose'); return
    cm.print_plan('Ready pose', steps, '(not saved)')


if __name__ == '__main__':
    main()
