#!/usr/bin/env python3
"""One supervised right-arm ID1 +80-tick yaw trial and return; no other joint writes."""
import copy
import argparse, fcntl, hashlib, json, os, signal, sys, tempfile, time
import joint_control as jc
import right_arm_hold as ah


def validate(group, states, expected):
    if not group.get('port_verified') or set(expected) != set(map(str, ah.IDS)):
        raise ValueError('Verified right bus and six expected positions required')
    for mid, state in states.items():
        joint = group['joints'][mid]
        lo, hi = joint['safe_raw_limits']
        if [state['min'], state['max']] != joint['observed_raw_limits'] or not state['min'] <= lo < hi <= state['max']:
            raise ValueError('Calibration changed')
        if state['model'] != 777 or state['mode'] != 0 or state['torque'] != 1 or state['moving']:
            raise ValueError('All six joints must already be holding and stationary')
        if not lo <= state['position'] <= hi or abs(state['goal']-state['position']) > 8 or abs(state['position']-expected[mid]) > 4:
            raise ValueError('Pose changed, outside limits, or outstanding target')
        if not 110 <= state['voltage_present_raw'] <= 130 or state['temperature_raw'] >= 60:
            raise ValueError('Power or temperature unsuitable')
    start = states['1']['position']
    if start + 80 > group['joints']['1']['safe_raw_limits'][1]:
        raise ValueError('Yaw target outside limits')
    return start


def packet(sdk, ph, pk, target):
    data = [target & 255, target >> 8, 0, 0, 100, 0]
    comm, error = pk.writeTxRx(ph, 1, 42, 6, data)
    if comm != sdk.COMM_SUCCESS or error:
        raise RuntimeError('Yaw target acknowledgment failed; no retry')
    if jc.read(sdk, ph, pk, 1, 'goal') != target:
        raise RuntimeError('Yaw target readback mismatch')


def control(sdk, ph, pk, group, expected):
    # ID5 remains unqualified for travel; only monitor its held pose locally.
    group = copy.deepcopy(group)
    if group['joints']['5']['safe_raw_limits'] is None:
        if set(expected)!=set(map(str,ah.IDS)) or any(type(v) is not int or not 0<=v<=4095 for v in expected.values()):
            raise ValueError('Six integer observed positions required')
        if group['joints']['5']['observed_raw_limits'] != [0,4095] or not 8<=expected['5']<=4087:
            raise ValueError('ID5 unqualified limits or hold pose unsuitable')
        group['joints']['5']['safe_raw_limits']=[expected['5']-8,expected['5']+8]
    initial = ah.snapshot(sdk, ph, pk)
    start = validate(group, initial, expected)
    attempted = False
    readings = []
    def monitor(target):
        states = ah.snapshot(sdk, ph, pk)
        for mid, state in states.items():
            baseline = initial[mid]
            lo, hi = group['joints'][mid]['safe_raw_limits']
            if state['model'] != 777 or state['mode'] != 0 or state['torque'] != 1 or [state['min'], state['max']] != [baseline['min'], baseline['max']]:
                raise RuntimeError('Joint holding/configuration changed')
            if not lo <= state['position'] <= hi or not 110 <= state['voltage_present_raw'] <= 130 or state['temperature_raw'] >= 60:
                raise RuntimeError('Pose/power/temperature unsuitable during trial')
            if mid == '1':
                if not start-8 <= state['position'] <= start+88 or state['goal'] != target:
                    raise RuntimeError('Yaw escaped trial window or target changed')
            elif abs(state['position']-baseline['position']) > 8 or state['goal'] != baseline['goal'] or state['moving']:
                raise RuntimeError('Another arm joint moved or changed target')
        return states
    try:
        for label, target in [('out', start+80), ('return', start)]:
            attempted = True  # Includes acknowledgment lost after application.
            packet(sdk, ph, pk, target)
            deadline = time.monotonic()+3
            stable = 0
            while time.monotonic() < deadline:
                states = monitor(target)
                stable = stable+1 if abs(states['1']['position']-target) <= 4 and not states['1']['moving'] else 0
                if stable >= 3:
                    break
                time.sleep(.05)
            else:
                raise RuntimeError('Yaw did not settle in 3 seconds; no return attempted')
            readings.append({'stage':label, 'target_raw':target, 'states':states})
            if label == 'out':
                # Visible pause, continuously rechecking all six joints.
                pause_deadline = time.monotonic()+2
                while time.monotonic() < pause_deadline:
                    time.sleep(.05)
                    monitor(target)
        return {'completed':True, 'servo_id':1, 'delta_raw':80, 'servo_degrees':7.03125, 'pause_seconds':2,
                'start_raw':start, 'stages':readings, 'torque':'left on',
                'note':'Supported yaw commissioning only; no other joint writes; loaded power remains unqualified.'}
    except BaseException:
        if attempted:
            old = {sig:signal.signal(sig, signal.SIG_IGN) for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                if jc.read(sdk, ph, pk, 1, 'torque') != 1:
                    raise RuntimeError('Yaw torque no longer on; not re-enabling')
                pos = jc.read(sdk, ph, pk, 1, 'position')
                lo, hi = group['joints']['1']['safe_raw_limits']
                if not lo <= pos <= hi or not start-8 <= pos <= start+88:
                    raise RuntimeError('Yaw feedback outside trial limits')
                packet(sdk, ph, pk, pos)
                print('Requested yaw hold at current feedback; no automatic return or torque release.', file=sys.stderr)
            except Exception as exc:
                print(f'HOLD FAILED: {exc}; keep supporting arm and use physical cutoff if needed.', file=sys.stderr)
            finally:
                for sig, handler in old.items():
                    signal.signal(sig, handler)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-positions', required=True)
    args = parser.parse_args()
    expected = json.loads(args.expected_positions)
    if set(expected) != set(map(str, ah.IDS)) or any(type(v) is not int or not 0 <= v <= 4095 for v in expected.values()):
        raise ValueError('Six integer observed positions required')
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
            print(json.dumps(control(sdk, ph, pk, group, expected)), flush=True)
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
