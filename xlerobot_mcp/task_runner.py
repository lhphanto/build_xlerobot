#!/usr/bin/env python3
"""Offline left-arm route planning and crash-safe execution journal; NEVER opens hardware.

Agent executes fresh previews through existing MCP tools. Numerical plans are not live
permission, collision checking, physical progress, or proof of grip. No automatic retries.
"""
import argparse
import contextlib
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import uuid

ROOT = Path(__file__).resolve().parent
CHECKS = {'grip_secure', 'landing_clear', 'placed', 'arm_supported'}


def digest(root=ROOT):
    files = [root/'robot_config.json', root/'server.py', root/'task_runner.py',
             root.parent/'SO101/so101_new_calib.urdf', *sorted((root/'controllers').glob('*.py'))]
    return hashlib.sha256(b''.join(p.read_bytes() for p in files)).hexdigest()


def vector(value):
    if not isinstance(value, list) or len(value) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError('Provide three finite numeric coordinates')
    return value


def positions(value):
    if not isinstance(value, dict) or set(value) != set('123456') or any(type(v) is not int or not 0 <= v <= 4095 for v in value.values()):
        raise ValueError('Provide six integer positions keyed 1 through 6')
    return dict(value)


class Model:
    def __init__(self, root=ROOT):
        sys.path.insert(0, str(root/'controllers'))
        import left_reach as lr
        import right_gripper_up as reach
        self.lr, self.reach = lr, reach
        self.group = json.loads((root/'robot_config.json').read_text())['groups']['left-arm']
        self.cal = self.group['kinematic_calibration']
        self.model, self.to_deg, _ = lr.kinematics(self.cal)

    def point(self, raw):
        return self.model.forward_kinematics(self.to_deg(raw))[:3, 3].tolist()

    def plan(self, raw, target):
        # Optimistic stationary holding states: this is feasibility, never fresh hardware.
        states = {}
        for key, value in raw.items():
            lo, hi = self.group['joints'][key]['observed_raw_limits']
            states[key] = dict(model=777, mode=0, torque=1, moving=0, min=lo, max=hi,
                               position=value, goal=value, voltage_present_raw=120, temperature_raw=25)
        observer = self.reach.make_observer(raw, self.cal)
        p, _ = self.lr.make_plan(self.group, states, raw, target, observer)
        return p


def compile_route(route, initial, model, identity, max_step_mm=30):
    initial = positions(initial)
    if type(max_step_mm) not in (int, float) or not math.isfinite(max_step_mm) or not 2 <= max_step_mm <= 50:
        raise ValueError('Segment length must be 2–50 mm')
    if not isinstance(route, dict) or set(route) != {'phases'} or not isinstance(route['phases'], list) or not 1 <= len(route['phases']) <= 32:
        raise ValueError('Route requires 1–32 phases')
    raw, actions = dict(initial), []

    def segment(target, phase, depth=0):
        nonlocal raw
        here = model.point(raw)
        distance = math.dist(here, target)
        if distance * 1000 > max_step_mm + .5:
            count = math.ceil(distance * 1000 / max_step_mm)
            for n in range(1, count+1):
                segment([a+(b-a)*n/count for a, b in zip(here, target)], phase, depth)
            return
        try:
            p = model.plan(raw, target)
        except ValueError as exc:
            # Split travel/path staging rejections only. Unreachable geometry, limits,
            # faults and calibration failures are NOT cured by taking smaller steps.
            reason = str(exc)
            splittable = ('ticks from the target; above 400' in reason or 'below its lower end' in reason)
            if not splittable or depth >= 5 or distance < .004:
                raise
            middle = [(a+b)/2 for a, b in zip(here, target)]
            segment(middle, phase, depth+1)
            segment(target, phase, depth+1)
            return
        previous = dict(raw)
        raw.update(p['targets'])
        actions.append(dict(kind='move', phase=phase, target_m=list(target), nominal_start=previous,
                            nominal_targets=p['targets'], model_end_m=model.point(raw),
                            live_preview_required=True))
        if len(actions) > 128:
            raise ValueError('Route exceeds 128 actions')

    for number, phase in enumerate(route['phases']):
        if not isinstance(phase, dict) or not isinstance(phase.get('name'), str) or not phase['name'].strip():
            raise ValueError('Each phase requires a nonempty name')
        label = phase['name']
        if 'checkpoint' in phase:
            if set(phase) != {'name', 'checkpoint'} or phase['checkpoint'] not in CHECKS:
                raise ValueError('Unknown checkpoint or unexpected fields')
            actions.append(dict(kind='checkpoint', phase=label, check=phase['checkpoint']))
            continue
        keys = set(phase)
        if keys not in ({'name', 'delta_mm'}, {'name', 'target_m'}):
            raise ValueError('Move requires exactly one delta_mm or target_m; other actions are not inferred')
        if 'delta_mm' in phase:
            delta = vector(phase['delta_mm'])
            if math.dist([0, 0, 0], delta) > 300:
                raise ValueError('A phase may not exceed 300 mm')
            target = [a+b/1000 for a, b in zip(model.point(raw), delta)]
        else:
            target = vector(phase['target_m'])
        if math.dist(model.point(raw), target) < .002:
            raise ValueError('Move is too small; minimum 2 mm')
        segment(target, label)
    if len(actions) > 128:
        raise ValueError('Route exceeds 128 actions')
    payload = dict(version=1, offline_only=True, identity=identity, initial_positions=initial,
                   max_step_mm=max_step_mm, actions=actions,
                   warnings=['Nominal model only; every move needs a fresh MCP preview and camera review.',
                             'No collision avoidance, force control, automatic gripper action or parking pose.'])
    payload['plan_id'] = hashlib.sha256(json.dumps(payload, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return payload


def atomic_write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.'+path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as out:
            json.dump(value, out, indent=2, allow_nan=False)
            out.flush()
            os.fsync(out.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


@contextlib.contextmanager
def locked(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix(path.suffix+'.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def validate_plan(plan, identity):
    candidate = dict(plan)
    supplied = candidate.pop('plan_id', None)
    actual = hashlib.sha256(json.dumps(candidate, sort_keys=True, allow_nan=False).encode()).hexdigest()
    if supplied != actual or plan.get('version') != 1 or plan.get('identity') != identity:
        raise ValueError('Plan changed or controller/configuration identity changed; plan again')


def new_run(plan, identity):
    validate_plan(plan, identity)
    return dict(version=1, run_id=str(uuid.uuid4()), plan=plan, cursor=0, status='ready', history=[], pending=None)


def reserve(run, identity):
    validate_plan(run['plan'], identity)
    if run['status'] != 'ready' or run['pending'] is not None:
        raise ValueError('Run is not ready; pending/failed moves must never be retried automatically')
    action = run['plan']['actions'][run['cursor']]
    if action['kind'] != 'move':
        raise ValueError('Observation checkpoint must be resolved first')
    ticket = dict(request_id=str(uuid.uuid4()), index=run['cursor'], target_m=action['target_m'], phase=action['phase'])
    run.update(status='reserved', pending=ticket)
    return ticket


def advance(run):
    run['cursor'] += 1
    run['pending'] = None
    run['status'] = 'complete' if run['cursor'] == len(run['plan']['actions']) else 'ready'


def decode_result(value):
    """Normalize existing native MCP or supported client output, retaining real/mock identity."""
    if 'results' in value:
        value = value['results'][0]
    if 'content' in value:
        block = next(c['text'] for c in value['content'] if c['type'] == 'text')
        value = json.loads(block[block.index('{'):])
    if 'stdout' not in value:
        raise ValueError('Record the original MCP/client envelope, including mock and exit_code')
    return value, json.loads(value['stdout'])


def record(run, request_id, envelope):
    if run['status'] != 'reserved' or not run['pending'] or run['pending']['request_id'] != request_id:
        raise ValueError('No matching reserved request')
    ticket = copy.deepcopy(run['pending'])
    try:
        outer, result = decode_result(envelope)
        plan = result['plan']
        target_matches = math.dist(vector(plan['target_m']), ticket['target_m']) < 1e-8
        state = result['states']
        valid_state = set(state) == set('123456') and all(s['moving'] == 0 and s['torque'] == 1 and s['model'] == 777 and s['mode'] == 0
                         and s['min'] <= s['position'] <= s['max'] and 110 <= s['voltage_present_raw'] <= 130 and s['temperature_raw'] < 60
                         for s in state.values())
        errors = result['final_errors_ticks']
        good = (outer.get('mock') is False and outer.get('exit_code') == 0 and result.get('completed') is True
                and target_matches and valid_state and set(errors) >= set('1234')
                and all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 8 for v in errors.values()))
    except (KeyError, ValueError, TypeError, StopIteration):
        good, result = False, envelope
    run['history'].append(dict(ticket=ticket, completed=good, feedback=result))
    if good:
        advance(run)
    else:
        # Preserve the request forever, including malformed/uncertain responses.
        run['status'] = 'stopped'
    return good


def observe(run, check, source, evidence):
    if run['status'] != 'ready' or run['pending'] is not None:
        raise ValueError('Cannot observe past an outstanding or failed move')
    action = run['plan']['actions'][run['cursor']]
    if action.get('check') != check or action['kind'] != 'checkpoint':
        raise ValueError('Observation does not match the current checkpoint')
    if source not in ('operator', 'camera') or not isinstance(evidence, str) or len(evidence.strip()) < 8:
        raise ValueError('Record actual operator or camera evidence')
    if check == 'arm_supported' and source != 'operator':
        raise ValueError('Torque release requires explicit operator support confirmation')
    run['history'].append(dict(check=check, source=source, evidence=evidence.strip()))
    advance(run)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    plan = sub.add_parser('plan')
    plan.add_argument('--route', type=Path, required=True)
    plan.add_argument('--positions', required=True, help='JSON object with fresh left-arm positions')
    plan.add_argument('--max-step-mm', type=float, default=30)
    plan.add_argument('--output', type=Path, required=True)
    for name in ('start', 'status', 'reserve', 'record', 'observe'):
        p = sub.add_parser(name)
        p.add_argument('--journal', type=Path, required=True)
        if name == 'start': p.add_argument('--plan', type=Path, required=True)
        if name == 'record':
            p.add_argument('--request-id', required=True)
            p.add_argument('--result', type=Path, required=True)
        if name == 'observe':
            p.add_argument('--check', choices=sorted(CHECKS), required=True)
            p.add_argument('--source', choices=['operator', 'camera'], required=True)
            p.add_argument('--evidence', required=True)
    args = parser.parse_args()
    if args.command == 'plan':
        result = compile_route(json.loads(args.route.read_text()), json.loads(args.positions), Model(), digest(), args.max_step_mm)
        atomic_write(args.output, result)
        print(json.dumps(dict(plan_id=result['plan_id'], actions=len(result['actions']), offline_only=True, output=str(args.output))))
        return 0
    with locked(args.journal):
        if args.command == 'start':
            if args.journal.exists(): raise ValueError('Journal already exists; never overwrite a run')
            run = new_run(json.loads(args.plan.read_text()), digest())
        else:
            run = json.loads(args.journal.read_text())
        result = None
        if args.command == 'reserve': result = reserve(run, digest())
        elif args.command == 'record': result = {'accepted': record(run, args.request_id, json.loads(args.result.read_text()))}
        elif args.command == 'observe': observe(run, args.check, args.source, args.evidence)
        if args.command != 'status': atomic_write(args.journal, run)
        # Reservation is durable BEFORE the UUID is returned to the caller.
        print(json.dumps(dict(status=run['status'], cursor=run['cursor'], pending=run['pending'], result=result,
                              next=run['plan']['actions'][run['cursor']] if run['cursor'] < len(run['plan']['actions']) else None)))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (Exception, KeyboardInterrupt) as exc:
        print(json.dumps({'error': str(exc) or type(exc).__name__, 'hardware_commands_sent': 0}), file=sys.stderr)
        sys.exit(2)
