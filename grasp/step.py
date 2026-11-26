#!/usr/bin/env python3
"""One supervised left-arm step per operator go. Uses the running MCP server. The only automatic retry: a reach that
stops on a yaw stall (servo 1 falls behind with a loaded arm), or finishes just outside the server's 8-tick final
tolerance with no error (loaded shoulder sag: Nov25 test lift ended 9 ticks off), is re-previewed and re-run once
from where it holds; a sag finish within SAG_TOLERANCE after that is accepted (the camera check still decides).
hold and view turn the neck torque on in place (neck_hold, no travel) when it is off (Nov25 it was off at the view).

    python grasp/step.py status
    python grasp/step.py camera                                   # start the camera stream and wait for its warm-up (hold/ready do this first)
    python grasp/step.py hold                                     # torque on in place (recover-hold if the elbow rests just past its limit)
    python grasp/step.py ready                                    # plan and run the chain to the ready pose (stops at the first failure)
    python grasp/step.py run PLAN.json --steps 1-8                # run a range of plan steps on one go; stops on a failure or camera CHECK
    python grasp/step.py reach X Y Z TILT YAW|free [--preview]   # top-down move_left_reach after a fresh preview
    python grasp/step.py open [--target 2162]                     # left_gripper_open
    python grasp/step.py close [--squeeze 24]                     # left_gripper_close (close to contact, then squeeze)
    python grasp/step.py adjust --right 1.5 | --retract 3 | --swing-left 10 | --up 2 ... [--free-roll]  # operator correction
    python grasp/step.py look [--expect X Y Z] [--expect-bottom-z Z]  # camera check only
    python grasp/step.py view [--tilt-to 2350] [--pan 0]           # tilt the camera down to the basket, snapshot, return to calibration pose

After every step a camera check runs (grasp/look.py in the contact_graspnet env; --no-look skips it): a snapshot, and
with --expect the object is segmented around that arm-frame point and its measured position compared (OK / CHECK /
UNSEEN = hidden behind the arm). `run` stops on CHECK; UNSEEN stops only plan steps of kind 'look' (which need to see it).
close reports the grip push (holding minus goal ticks); under 7 suggests the retry in the plan (re-close: run close
again, it re-contacts at once and squeezes further; or regrasp deeper).

reach: reads fresh joints, previews with top_down/approach_tilt_deg/jaw_yaw_deg and executes only if the preview
passes (--preview stops there). A failed move holds where it stopped; re-run the same command to re-preview from there.
Every result is appended to samples/grasp/steps.jsonl. Any Python works (only the server client is used).
"""
import argparse
import json
import os
import subprocess
import sys
import time
import uuid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MCP = os.path.join(ROOT, 'xlerobot_mcp')
LOG = os.path.join(ROOT, 'samples', 'grasp', 'steps.jsonl')
LEROBOT_PYTHON = os.environ.get('LEROBOT_PYTHON', os.path.expanduser('~/miniforge3/envs/lerobot/bin/python'))
CGN_PYTHON = os.environ.get('CGN_PYTHON', os.path.expanduser('~/miniforge3/envs/contact_graspnet/bin/python'))
WEAK_PUSH = 7  # ticks; Nov23 a push of 4 dropped the box, 9 carried it
SAG_TOLERANCE = 16  # ticks; a reach with no error whose joints all finish within this (server: 8) counts after one retry


def call(name, args):
    return subprocess.run([os.path.join(MCP, '.venv', 'bin', 'python'), os.path.join(MCP, 'client.py'), 'call', name, '--arguments', json.dumps(args)],
                          capture_output=True, text=True, timeout=200)


def output(r):
    """(last JSON line of the tool's stdout or None, stderr/error text)."""
    if r.returncode == 0:
        res = json.loads(r.stdout)['results'][0]
    else:
        try:
            rep = json.loads(r.stderr.strip().splitlines()[-1])
        except (ValueError, IndexError):
            return None, r.stderr[-600:]
        res = rep.get('tool_result')
        if res is None:
            return None, rep.get('error', '')
    lines = [l for l in res.get('stdout', '').splitlines() if l.startswith('{')]
    return (json.loads(lines[-1]) if lines else None), res.get('stderr', '')


def states():
    out = {}
    for i in range(1, 7):
        r = call('joint_status', {'group': 'left-arm', 'servo_id': i})
        if r.returncode:
            sys.exit(f'joint_status ID{i} failed; is the server running? {r.stderr[-300:]}')
        out[str(i)] = json.loads(json.loads(r.stdout)['results'][0]['stdout'])['state']
    return out


def log(entry):
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    with open(LOG, 'a') as f:
        f.write(json.dumps(dict(entry, time=time.strftime('%Y-%m-%dT%H:%M:%S'))) + '\n')


def show_states(st):
    for m, s in st.items():
        print(f"  ID{m} pos {s['position']} goal {s['goal']} torque {s['torque']} moving {s['moving']} temp {s.get('temperature_raw')} C")
    hot = [m for m, s in st.items() if (s.get('temperature_raw') or 0) >= 50]
    if hot:
        print(f'  WARNING: ID{",".join(hot)} at 50 C or more; rest the arm before continuing (stop is 60 C)')


def look(a, label):
    """Camera check after a step; never fails the step."""
    if getattr(a, 'no_look', False):
        return None
    cmd = [CGN_PYTHON, os.path.join(ROOT, 'grasp', 'look.py'), '--label', label]
    if a.expect:
        cmd += ['--object', *map(str, a.expect)]
    if a.expect_bottom_z is not None:
        cmd += ['--expect-bottom-z', str(a.expect_bottom_z)]
    g = getattr(a, 'gripper_point', None) or last_log('reach').get('target')  # the arm is above this: not the object
    if g and a.expect:
        cmd += ['--gripper', *map(str, g)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    lines = [l for l in r.stdout.splitlines() if l.strip()]
    if r.returncode or not lines:
        print('camera check failed (step result unaffected):', (r.stderr or r.stdout)[-300:]); return None
    res = json.loads(lines[-1])
    print('camera:', lines[-2] if len(lines) > 1 else '', '| image', res.get('preview') or res.get('image'))
    return res


def warm_camera():
    """Start the camera stream if needed and wait for its warm-up (exposure and white balance settle ~25-30 s after the
    camera opens; frames before that are yellow). A snapshot does both; once warm it returns in ~3 s."""
    t = time.time()
    r = subprocess.run([os.path.join(MCP, '.venv', 'bin', 'python'), os.path.join(MCP, 'client.py'), 'call', 'camera_snapshot', '--arguments', '{}',
                        '--output-dir', os.path.join(ROOT, 'samples', 'grasp')], capture_output=True, text=True, timeout=150)
    if r.returncode:
        sys.exit('camera warm-up failed (nothing moved): ' + r.stderr.strip()[-300:])
    res = json.loads(r.stdout)['results']
    path = res[1].get('saved_image') if len(res) > 1 else None
    print(f'camera warm ({time.time() - t:.0f} s); snapshot {path}')
    log({'step': 'camera', 'seconds': round(time.time() - t, 1), 'image': path})
    return 0


def neck_positions():
    out = {}
    for m in (7, 8):
        r = call('joint_status', {'group': 'neck', 'servo_id': m})
        st = json.loads(json.loads(r.stdout)['results'][0]['stdout'])['state']
        out[m] = st
    return out


def neck_to(targets):
    """Move the neck to {id: raw} with nudge_joint steps of at most 32 ticks (pan then tilt). Returns final positions."""
    for m in (7, 8):
        for _ in range(12):
            pos = neck_positions()[m]['position']; d = targets[m] - pos
            if abs(d) <= 2:
                break
            step = max(-32, min(32, d))
            out, err = output(call('nudge_joint', {'group': 'neck', 'servo_id': m, 'delta_raw': step, 'request_id': str(uuid.uuid4())}))
            if out is None and err:
                raise RuntimeError(f'neck ID{m} nudge {step} failed: {err.strip()[-300:]}')
    return {m: s['position'] for m, s in neck_positions().items()}


def ensure_neck_holding():
    """neck_hold at the current pose if either neck servo has torque off (no commanded travel; the server checks both
    are stationary and inside limits). The camera-to-arm calibration needs the neck to stay put, and view needs it on."""
    st = neck_positions()
    if all(s['torque'] == 1 for s in st.values()):
        return 0
    exp = {str(m): s['position'] for m, s in st.items()}; rid = str(uuid.uuid4())
    print(f'neck torque off; neck_hold in place at {exp} (no travel), request_id {rid}')
    out, err = output(call('neck_hold', {'expected_positions': exp, 'request_id': rid}))
    log({'step': 'neck_hold', 'request_id': rid, 'positions': exp, 'completed': bool(out and out.get('completed')), 'error': None if out else err[-300:]})
    if not out or not out.get('completed'):
        sys.exit('neck_hold failed or uncertain (nothing else sent; run status): ' + err[-300:])
    return 0


def view(a):
    """Look somewhere else with the neck (visual only), then return to the camera-to-arm calibration pose."""
    cfg = json.load(open(os.path.join(MCP, 'robot_config.json')))['groups']['neck']['joints']
    lim = {m: cfg[str(m)]['safe_raw_limits'] for m in (7, 8)}
    ref = json.load(open(os.path.join(ROOT, 'samples', 'hand_eye', 'handeye_result.json')))['neck_raw']; home = {7: ref['7'], 8: ref['8']}
    ensure_neck_holding()
    st = neck_positions()
    start = {m: s['position'] for m, s in st.items()}
    goal = {7: min(max(start[7] + a.pan, lim[7][0]), lim[7][1]), 8: min(max(a.tilt_to, lim[8][0]), lim[8][1])}
    if goal != {7: start[7] + a.pan, 8: a.tilt_to}:
        print(f'note: view pose clipped to the neck limits {lim}: {goal}')
    at = path = None
    try:
        at = neck_to(goal); print('view pose', at)
        r = subprocess.run([os.path.join(MCP, '.venv', 'bin', 'python'), os.path.join(MCP, 'client.py'), 'call', 'camera_snapshot', '--arguments', '{}',
                            '--output-dir', os.path.join(ROOT, 'samples', 'grasp')], capture_output=True, text=True, timeout=120)
        path = json.loads(r.stdout)['results'][1]['saved_image'] if r.returncode == 0 else None
        print('view image', path)
    finally:
        back = neck_to(home)
        ok = all(abs(back[m] - home[m]) <= 6 for m in (7, 8))
        print('returned to calibration pose' if ok else f'WARNING: neck at {back}, not within 6 ticks of {home}; depth checks will refuse', back)
    log({'step': 'view', 'pan': a.pan, 'tilt_to': a.tilt_to, 'view_pose': at, 'image': path, 'returned': back, 'home_ok': ok})
    return 0 if ok else 1


def hold(a):
    """left_arm_hold: plain hold, or recover-hold when only the elbow (ID3) rests up to 32 ticks past its limit.
    Starts and warms up the camera first, so every later step has settled frames."""
    warm_camera(); ensure_neck_holding()
    st = states(); joints = json.load(open(os.path.join(MCP, 'robot_config.json')))['groups']['left-arm']['joints']
    if all(s['torque'] == 1 for s in st.values()):
        print('already holding'); show_states(st); return 0
    out = {m: (s['position'] - joints[m]['safe_raw_limits'][1] if s['position'] > joints[m]['safe_raw_limits'][1] else
               s['position'] - joints[m]['safe_raw_limits'][0] if s['position'] < joints[m]['safe_raw_limits'][0] else 0) for m, s in st.items()}
    if any(v for m, v in out.items() if m != '3') or abs(out['3']) > 32:
        sys.exit(f'Joints outside their limits by {out}; reposition by hand with support (nothing sent). '
                 'November24: the limp elbow rested 34 ticks past its limit; folding it in a few degrees by hand fixed it.')
    action = 'recover-hold' if out['3'] else 'hold'
    exp = {m: s['position'] for m, s in st.items()}; rid = str(uuid.uuid4())
    print(f'{action} from {exp}; request_id {rid}')
    res, err = output(call('left_arm_hold', {'action': action, 'request_id': rid, 'expected_positions': exp}))
    log({'step': 'hold', 'action': action, 'request_id': rid, 'completed': bool(res and res.get('completed')), 'error': err[-300:] if not res else None})
    if not res or not res.get('completed'):
        sys.exit(f'{action} failed or uncertain (do not retry blindly; run status): {err[-400:]}')
    show_states(states()); return 0


def ready(a):
    """Chain to the ready pose (common.READY): each step re-previewed and executed; stops at the first failure."""
    warm_camera()
    r = subprocess.run([LEROBOT_PYTHON, os.path.join(ROOT, 'grasp', 'plan_ready.py'), '--json'], capture_output=True, text=True, timeout=600)
    lines = [l for l in r.stdout.splitlines() if l.startswith('[')]
    if r.returncode or not lines:
        sys.exit('ready plan failed (nothing moved): ' + (r.stderr or r.stdout).strip().splitlines()[-1][-300:])
    steps = json.loads(lines[-1])
    if not steps:
        print('already at the ready pose'); return 0
    for i, st in enumerate(steps, 1):
        print(f'--- {st["note"]}')
        x, y, z = st['target']
        yaw = 'free' if st.get('jaw_yaw_deg') is None else str(st['jaw_yaw_deg'])
        sa = argparse.Namespace(x=x, y=y, z=z, tilt=st['tilt'], yaw=yaw, preview=False, expect=None, expect_bottom_z=None, no_look=i < len(steps))
        if reach_retry(sa):
            print(f'stopped at step {i}/{len(steps)}; the arm holds where it stopped. Re-run `step.py ready` to re-plan from there.')
            return 1
    print('at the ready pose'); return 0


def adjust(a):
    """Operator correction from the current pose (grasp/adjust.py plans it: lean kept, jaw line kept unless --free-roll),
    then its steps are run on this one go."""
    args = [f'--{k.replace("_", "-")}={v:g}' for k, v in vars(a).items() if k in ADJUST_KEYS and v]
    if not args:
        sys.exit('give at least one of --right/--left/--forward/--back/--up/--down/--retract/--extend (cm) or --swing-left/--swing-right (deg)')
    r = subprocess.run([LEROBOT_PYTHON, os.path.join(ROOT, 'grasp', 'adjust.py'), *args, *(['--free-roll'] if a.free_roll else []), '--json'],
                       capture_output=True, text=True, timeout=600)
    lines = [l for l in r.stdout.splitlines() if l.startswith('[')]
    if r.returncode or not lines:
        sys.exit('adjust plan failed (nothing moved): ' + (r.stderr or r.stdout).strip().splitlines()[-1][-300:])
    steps = json.loads(lines[-1])
    for i, st in enumerate(steps, 1):
        print(f'--- {st["note"]}')
        x, y, z = st['target']
        yaw = 'free' if st.get('jaw_yaw_deg') is None else str(st['jaw_yaw_deg'])
        sa = argparse.Namespace(x=x, y=y, z=z, tilt=st['tilt'], yaw=yaw, preview=False, expect=a.expect, expect_bottom_z=a.expect_bottom_z,
                                no_look=a.no_look or i < len(steps))
        if reach_retry(sa):
            print(f'stopped at adjust step {i}/{len(steps)}; the arm holds where it stopped'); return 1
    return 0


ADJUST_KEYS = ('right', 'left', 'forward', 'back', 'up', 'down', 'retract', 'extend', 'swing_left', 'swing_right')


def sag_finish(e):
    """A reach that ended without error, all joints within SAG_TOLERANCE of target but not within the server's 8."""
    fe = e.get('final_errors') or {}
    return e.get('completed') is False and not e.get('error') and e.get('result') != 'uncertain' and fe and max(abs(v) for v in fe.values()) <= SAG_TOLERANCE


def reach_retry(a):
    """reach, re-run once from the held pose if it stopped on a stall (a waypoint not reached in time) or finished
    just outside the final tolerance (sag_finish); a sag finish after the re-run is accepted with a note."""
    rc = reach(a)
    if rc:
        last = last_log('reach')  # the camera check may have logged after it
        stall = 'stopped' in str(last.get('error') or '') and last.get('result') != 'uncertain'
        if stall or sag_finish(last):
            print('reach ' + ('stopped on a stall' if stall else f'finished {last["final_errors"]} (outside 8 ticks, no error)')
                  + '; re-previewing and re-running once from where the arm holds')
            rc = reach(a)
            if rc and sag_finish(last_log('reach')):
                print(f'accepted: every joint within {SAG_TOLERANCE} ticks of target, no error (loaded sag); the camera check decides')
                rc = 0
    return rc


def last_log(step=None):
    if not os.path.exists(LOG):
        return {}
    for line in reversed(open(LOG).read().splitlines()):
        e = json.loads(line)
        if step is None or e.get('step') == step:
            return e
    return {}


def run_plan(a):
    """Run plan steps lo..hi (1-based) in order. Stops at the first failed step or camera CHECK; checks need eyes."""
    plan = json.load(open(a.plan)); steps = plan['steps']
    lo, hi = (int(v) for v in (a.steps.split('-') if '-' in a.steps else (a.steps, a.steps)))
    for i in range(lo, hi + 1):
        st = steps[i - 1]; print(f'--- step {i}: {st["note"]}')
        if st['kind'] not in ('reach', 'open', 'close', 'view', 'look'):
            print(f'step {i} is a {st["kind"]} step ({st["command"]}); stopping here for the operator'); return 1
        argv = st['command'].split()[2:]  # drop "python grasp/step.py"
        sa = parser().parse_args(argv)
        if hasattr(sa, 'no_look') and i < hi and not sa.expect:
            sa.no_look = True  # intermediate approach steps: no camera check unless the plan asks for one
        rc = reach_retry(sa) if sa.cmd == 'reach' else dispatch(sa)
        last = last_log()
        verdict = last.get('verdict') if last.get('step') == 'look' else None
        if verdict == 'UNSEEN' and st['kind'] != 'look':
            print('camera: object hidden behind the arm (UNSEEN); continuing')
        if rc or verdict == 'CHECK' or (verdict in ('UNSEEN', None) and st['kind'] == 'look'):
            why = 'step failed' if rc and st['kind'] != 'look' else f'camera {verdict or "check failed"}'
            print(f'stopped after step {i} ({why}); look before continuing'); return 1
    return 0


def reach(a):
    target = [a.x, a.y, a.z]
    opts = {'top_down': True, 'approach_tilt_deg': a.tilt}
    if a.yaw != 'free':
        opts['jaw_yaw_deg'] = float(a.yaw)
    exp = {m: s['position'] for m, s in states().items()}
    print('start', exp)
    s, err = output(call('preview_left_reach', {'expected_positions': exp, 'target_m': target, **opts}))
    if not s or 'plan' not in s:
        log({'step': 'reach', 'target': target, **opts, 'preview': 'rejected', 'error': err.strip()[-300:]})
        sys.exit('PREVIEW REJECTED, nothing sent: ' + err.strip().splitlines()[-1] if err.strip() else 'PREVIEW REJECTED, nothing sent')
    p = s['plan']; pt = p.get('pointing') or {}
    travel = max(abs(p['targets'][m] - exp[m]) for m in p['targets'])
    print(f"preview: {len(p['waypoints'])} stages, joint targets {p['targets']} (largest travel {travel}), gripper move cm "
          f"{[round(v * 100, 1) for v in p['move_m']]}, model error {p['ik_error_mm']:.1f} mm, approach error {pt.get('approach_error_deg', 0):.1f} deg, "
          f"path worst {pt.get('path_max_approach_error_deg', 0):.1f} deg")
    if a.preview:
        return 0
    rid = str(uuid.uuid4()); print('request_id', rid)
    out, err = output(call('move_left_reach', {'expected_positions': exp, 'target_m': target, 'plan_hash': p['plan_hash'], 'request_id': rid, **opts}))
    if not out:
        log({'step': 'reach', 'target': target, **opts, 'request_id': rid, 'result': 'uncertain', 'error': err[-300:]})
        sys.exit('NO RESULT (treat as uncertain, do not retry blindly; run status): ' + err[-600:])
    leads = out.get('leads') or []
    print(f"completed {out.get('completed')} error {out.get('error')} final errors {out.get('final_errors_ticks')} leads {len(leads)} "
          f"(max {max([abs(l['lead']) for l in leads], default=0)}) glitches {out.get('glitches')}")
    if out.get('cleanup'):
        print('cleanup (holding where it stopped):', json.dumps(out['cleanup'])[:400])
    log({'step': 'reach', 'target': target, **opts, 'request_id': rid, 'completed': out.get('completed'), 'error': out.get('error'),
         'final_errors': out.get('final_errors_ticks'), 'leads': len(leads), 'glitches': out.get('glitches')})
    show_states(states())
    a.gripper_point = target
    look(a, f'after reach to {target}')
    return 0 if out.get('completed') else 1


def gripper(a):
    exp = {m: s['position'] for m, s in states().items()}
    if a.action == 'open' and abs(exp['6'] - a.target) <= 10:
        print(f'gripper already open ({exp["6"]}, target {a.target}); nothing sent')
        log({'step': 'open', 'skipped': 'already open', 'position': exp['6']}); look(a, 'after open'); return 0
    rid = str(uuid.uuid4()); print('start', exp, 'request_id', rid)
    if a.action == 'open':
        out, err = output(call('left_gripper_open', {'expected_positions': exp, 'target': a.target, 'request_id': rid}))
    else:
        out, err = output(call('left_gripper_close', {'expected_positions': exp, 'squeeze': a.squeeze, 'request_id': rid}))
    if not out:
        log({'step': a.action, 'request_id': rid, 'result': 'uncertain', 'error': err[-300:]})
        sys.exit('NO RESULT (treat as uncertain; run status): ' + err[-600:])
    brief = {k: v for k, v in out.items() if k not in ('trace',)}
    print(json.dumps(brief))
    if a.action == 'close':
        if out.get('contact'):
            push = out['holding_position'] - out['hold_goal']; brief['push_ticks'] = push
            print(f"grip push (holding - goal) {push} ticks: " + ('OK' if push >= WEAK_PUSH else
                  f'WEAK (< {WEAK_PUSH}); retry: run close again (re-contacts and squeezes further) or the plan\'s regrasp-deeper steps'))
        else:
            print('NO CONTACT: closed to the floor, nothing between the jaws; open and re-check the object with look')
    log({'step': a.action, 'request_id': rid, **brief})
    show_states(states())
    look(a, f'after {a.action}')
    return 0 if out.get('completed') else 1


def parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('status'); sub.add_parser('hold'); sub.add_parser('ready'); sub.add_parser('camera')
    ad = sub.add_parser('adjust', help='operator correction from the current pose (cm / deg); lean and jaw line kept')
    for n in ADJUST_KEYS:
        ad.add_argument('--' + n.replace('_', '-'), type=float, default=0., metavar='DEG' if n.startswith('swing') else 'CM')
    ad.add_argument('--free-roll', action='store_true', help='let the wrist roll change (carrying; jaw line does not matter)')
    rn = sub.add_parser('run'); rn.add_argument('plan'); rn.add_argument('--steps', required=True, help='e.g. 1-8 or 9')
    r = sub.add_parser('reach'); [r.add_argument(n, type=float) for n in ('x', 'y', 'z', 'tilt')]; r.add_argument('yaw'); r.add_argument('--preview', action='store_true')
    o = sub.add_parser('open'); o.add_argument('--target', type=int, default=2162)
    c = sub.add_parser('close'); c.add_argument('--squeeze', type=int, default=24)
    v = sub.add_parser('view'); v.add_argument('--pan', type=int, default=0, help='ticks from the current pan, + turns right')
    v.add_argument('--tilt-to', type=int, default=2350, help='absolute tilt; higher looks down. 2350 shows the basket (Nov24 test, limit 2356)')
    lk = sub.add_parser('look')
    for p_ in (r, o, c, lk, ad):
        p_.add_argument('--expect', type=float, nargs=3, metavar=('X', 'Y', 'Z'), help='where the object should be (arm frame)')
        p_.add_argument('--expect-bottom-z', type=float)
        if p_ is not lk:
            p_.add_argument('--no-look', action='store_true', help='skip the camera check after the step')
    return ap


def dispatch(a):
    if a.cmd == 'run':
        return run_plan(a)
    if a.cmd == 'status':
        show_states(states()); return 0
    if a.cmd == 'hold':
        return hold(a)
    if a.cmd == 'camera':
        return warm_camera()
    if a.cmd == 'adjust':
        return adjust(a)
    if a.cmd == 'ready':
        return ready(a)
    if a.cmd == 'look':
        return 0 if look(a, 'look') else 1
    if a.cmd == 'view':
        return view(a)
    if a.cmd == 'reach':
        return reach(a)
    a.action = a.cmd
    return gripper(a)




def main():
    return dispatch(parser().parse_args())


if __name__ == '__main__':
    sys.exit(main())
