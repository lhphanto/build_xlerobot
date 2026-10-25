#!/usr/bin/env python3
"""Fixed, supervised right-arm commissioning trials; one selected joint only."""
import argparse
import copy
import fcntl
import hashlib
import json
import os
import signal
import sys
import tempfile
import time
import joint_control as jc
import right_arm_hold as ah
from settle_timeout import StationarySettlingTimeout, stationary_timeout_evidence


def prepare(group, expected, motor):
    if motor not in ah.IDS or set(expected) != set(map(str, ah.IDS)) or any(type(v) is not int or not 0 <= v <= 4095 for v in expected.values()):
        raise ValueError('One arm ID1-6 and six fresh integer positions required')
    group = copy.deepcopy(group)
    if group['joints']['5']['safe_raw_limits'] is None:
        if motor == 5:
            raise ValueError('ID5 travel blocked until an operator-approved software range is saved')
        if group['joints']['5']['observed_raw_limits'] != [0, 4095] or not 8 <= expected['5'] <= 4087:
            raise ValueError('ID5 calibration or held pose unsuitable')
        group['joints']['5']['safe_raw_limits'] = [expected['5']-8, expected['5']+8]
    return group


def validate(group, states, expected, motor, offsets):
    if group.get('port_verified') is not True:
        raise ValueError('Verified right bus required')
    for mid, state in states.items():
        joint = group['joints'][mid]
        lo, hi = joint['safe_raw_limits']
        if [state['min'], state['max']] != joint['observed_raw_limits'] or not state['min'] <= lo < hi <= state['max']:
            raise ValueError('Calibration changed')
        if state['model'] != 777 or state['mode'] != 0 or state['torque'] != 1 or state['moving']:
            raise ValueError('All six joints must already hold, stationary in position mode')
        if not lo <= state['position'] <= hi or not lo <= state['goal'] <= hi or abs(state['goal']-state['position']) > 8 or abs(state['position']-expected[mid]) > 4:
            raise ValueError('Pose changed, outside limits, or outstanding target')
        if not 110 <= state['voltage_present_raw'] <= 130 or state['temperature_raw'] >= 60:
            raise ValueError('Power or temperature unsuitable')
    start = states[str(motor)]['position']
    lo, hi = group['joints'][str(motor)]['safe_raw_limits']
    if any(not lo <= start+d <= hi for d in offsets):
        raise ValueError('Trial target outside saved limits')
    return start


def packet(sdk, ph, pk, motor, target, speed_raw=100):
    data = [target & 255, target >> 8, 0, 0, speed_raw, 0]
    comm, error = pk.writeTxRx(ph, motor, 42, 6, data)
    if comm != sdk.COMM_SUCCESS or error:
        raise RuntimeError('Target acknowledgment failed; no retry')
    if jc.read(sdk, ph, pk, motor, 'goal') != target:
        raise RuntimeError('Target readback mismatch')


def control(sdk, ph, pk, group, expected, motor, clearance=False, direction='positive', return_to_start=True, speed_raw=100):
    if type(clearance) is not bool or type(return_to_start) is not bool or speed_raw not in (30,100) or (clearance and not return_to_start) or direction not in ('positive', 'negative'):
        raise ValueError('Invalid trial mode or direction')
    group = prepare(group, expected, motor)
    tolerance = 6 if motor == 5 else 4
    delta = 80 if clearance or motor == 1 else 32
    delta *= 1 if direction == 'positive' else -1
    legs = [('positive', 80), ('center-after-positive', 0), ('negative', -80), ('center-after-negative', 0)] if clearance else [('out', delta), ('return', 0)]
    if not return_to_start:legs=legs[:1]
    offsets = [d for _, d in legs]
    initial = ah.snapshot(sdk, ph, pk)
    start = validate(group, initial, expected, motor, offsets)
    low, high = start+min(0, *offsets)-8, start+max(0, *offsets)+8
    attempted = False
    readings, trace, recent = [], [], []
    began = time.monotonic()
    stage, target = 'preflight', start
    cleanup = {'attempted': False, 'hold_acknowledged': False}

    def monitor(goal):
        states = ah.snapshot(sdk, ph, pk)
        for mid, state in states.items():
            baseline = initial[mid]
            lo, hi = group['joints'][mid]['safe_raw_limits']
            if state['model'] != 777 or state['mode'] != 0 or state['torque'] != 1 or [state['min'], state['max']] != [baseline['min'], baseline['max']]:
                raise RuntimeError('Joint holding or calibration changed')
            if not lo <= state['position'] <= hi or not 110 <= state['voltage_present_raw'] <= 130 or state['temperature_raw'] >= 60:
                raise RuntimeError('Pose, power or temperature unsuitable')
            if mid == str(motor):
                if not low <= state['position'] <= high or state['goal'] != goal:
                    raise RuntimeError('Selected joint escaped trial window or target changed')
            elif abs(state['position']-baseline['position']) > 8 or state['goal'] != baseline['goal'] or state['moving']:
                raise RuntimeError('Another joint moved or changed target')
        active = states[str(motor)]
        row = {'elapsed_s': round(time.monotonic()-began, 4), 'stage': stage,
               'target_raw': goal, 'goal_raw': active['goal'], 'position_raw': active['position'],
               'error_ticks': goal-active['position'], 'moving': active['moving'], 'torque': active['torque'],
               'voltage_raw': active['voltage_present_raw'], 'temperature_c': active['temperature_raw']}
        for name in ('present_speed_raw', 'present_load_raw', 'servo_status_raw', 'motor_phase_current_raw'):
            addr, size = jc.STATUS_REGS[name]
            fn = pk.read2ByteTxRx if size == 2 else pk.read1ByteTxRx
            value, comm, error = fn(ph, motor, addr)
            if comm != sdk.COMM_SUCCESS or error:
                raise RuntimeError('Diagnostic read failed: '+name)
            row[name] = value
        if row['servo_status_raw']:
            raise RuntimeError('Servo status fault')
        if len(trace) < 256:
            trace.append(row)
        recent.append(row)
        del recent[:-3]
        return states

    try:
        for stage, offset in legs:
            target = start+offset
            attempted = True
            packet(sdk, ph, pk, motor, target, speed_raw)
            deadline, stable = time.monotonic()+3, 0
            while time.monotonic() < deadline:
                states = monitor(target)
                active = states[str(motor)]
                stable = stable+1 if abs(active['position']-target) <= tolerance and not active['moving'] else 0
                if stable >= 3:
                    break
                time.sleep(.05)
            else:
                evidence = stationary_timeout_evidence(recent, target, stage)
                if evidence is not None:
                    raise StationarySettlingTimeout('Stationary positioning failure; existing target retained', evidence)
                raise RuntimeError('Joint did not settle in 3 seconds; no next leg attempted')
            readings.append({'stage': stage, 'target_raw': target, 'tolerance_raw': tolerance, 'states': states})
            if clearance or stage == 'out':
                until = time.monotonic()+2
                while time.monotonic() < until:
                    time.sleep(.05)
                    monitor(target)
        return {'completed': True, 'servo_id': motor, 'clearance': clearance, 'delta_raw': abs(delta),
                'direction': direction, 'return_to_start':return_to_start, 'speed_raw':speed_raw, 'start_raw': start, 'stages': readings, 'trace': trace,
                'tolerance_raw': tolerance, 'torque': 'left on',
                'note': 'Right-arm joint mapping/direction unverified. Supported empty-arm test only; no range approval saved.'}
    except BaseException as failure:
        if isinstance(failure, StationarySettlingTimeout):
            cleanup.update({'action': 'preserve_existing_target', 'target_write_sent': False,
                            'goal_raw': target, 'verified_stationary': failure.evidence})
        elif attempted:
            cleanup['attempted'] = True
            old = {sig: signal.signal(sig, signal.SIG_IGN) for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                if jc.read(sdk, ph, pk, motor, 'torque') != 1:
                    raise RuntimeError('Torque not on; not re-enabling')
                pos = jc.read(sdk, ph, pk, motor, 'position')
                lo, hi = group['joints'][str(motor)]['safe_raw_limits']
                if not lo <= pos <= hi or not low <= pos <= high:
                    raise RuntimeError('Hold feedback outside trial limits')
                packet(sdk, ph, pk, motor, pos, speed_raw)
                cleanup.update({'hold_acknowledged': True, 'hold_goal_raw': pos})
            except Exception as exc:
                cleanup['error'] = str(exc)
            finally:
                for sig, handler in old.items():
                    signal.signal(sig, handler)
        print(json.dumps({'completed': False, 'servo_id': motor, 'failed_stage': stage,
                          'error': str(failure), 'trace': trace, 'cleanup': cleanup}), flush=True)
        print('Trial stopped. Keep arm supported; no automatic return or torque release.', file=sys.stderr)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--id', type=int, choices=ah.IDS, required=True)
    parser.add_argument('--expected-positions', required=True)
    parser.add_argument('--clearance', action='store_true')
    parser.add_argument('--direction', choices=['positive', 'negative'], default='positive')
    args = parser.parse_args()
    group = json.loads(jc.CONFIG.read_text())['groups']['right-arm']
    import scservo_sdk as sdk
    port = group['port']
    name = hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(), 'xlerobot-'+name+'.lock'), 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ph, pk = sdk.PortHandler(port), sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):
                raise RuntimeError('Port setup failed')
            def timeout(length):
                ph.packet_start_time = ph.getCurrentTime()
                ph.packet_timeout = ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout = timeout
            time.sleep(.1)
            print(json.dumps(control(sdk, ph, pk, group, json.loads(args.expected_positions), args.id, args.clearance, args.direction)), flush=True)
        finally:
            ph.closePort()


if __name__ == '__main__':
    def interrupt(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt)
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
