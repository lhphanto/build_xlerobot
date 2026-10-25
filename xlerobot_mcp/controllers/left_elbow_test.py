#!/usr/bin/env python3
"""One supervised left-arm ID3 bounded2-7degree elbow trial and return; no other joint writes."""
import argparse, fcntl, hashlib, json, math, os, signal, sys, tempfile, time
import joint_control as jc
import left_arm_hold as ah
from settle_timeout import StationarySettlingTimeout, stationary_timeout_evidence


def validate(group, states, expected, delta=80):
    if not group.get('port_verified') or set(expected) != set(map(str, ah.IDS)):
        raise ValueError('Verified left bus and six expected positions required')
    for mid, state in states.items():
        joint = group['joints'][mid]
        lo, hi = joint['safe_raw_limits']
        if [state['min'], state['max']] != joint['observed_raw_limits'] or not state['min'] <= lo < hi <= state['max']:
            raise ValueError('Calibration changed')
        if state['model'] != 777 or state['mode'] != 0 or state['torque'] != 1 or state['moving']:
            raise ValueError('All six joints must already be holding and stationary')
        if not lo <= state['position'] <= hi or abs(state['goal']-state['position']) > (16 if mid == '3' else 8) or abs(state['position']-expected[mid]) > 4:
            raise ValueError('Pose changed, outside limits, or outstanding target')
        if mid == '3' and not lo <= state['goal'] <= hi:
            raise ValueError('ID3 existing goal outside saved limits')
        if not 110 <= state['voltage_present_raw'] <= 130 or state['temperature_raw'] >= 60:
            raise ValueError('Power or temperature unsuitable')
    start = states['3']['position']
    lo, hi = group['joints']['3']['safe_raw_limits']
    if not lo <= start+delta <= hi:
        raise ValueError('Elbow target outside limits')
    return start


def packet(sdk, ph, pk, target, speed_raw=100):
    if speed_raw not in (30,100):raise ValueError('Fixed supported speed required')
    data = [target & 255, target >> 8, 0, 0, speed_raw, 0]
    comm, error = pk.writeTxRx(ph, 3, 42, 6, data)
    if comm != sdk.COMM_SUCCESS or error:
        raise RuntimeError('Elbow target acknowledgment failed; no retry')
    if jc.read(sdk, ph, pk, 3, 'goal') != target:
        raise RuntimeError('Elbow target readback mismatch')


def control(sdk, ph, pk, group, expected, direction='positive', amplitude_deg=7.03125, slow=False):
    if direction not in ('positive','negative'):
        raise ValueError('Direction must be positive or negative')
    if type(amplitude_deg) not in (int,float) or not math.isfinite(amplitude_deg) or not 2<=amplitude_deg<=7.03125 or type(slow) is not bool:raise ValueError('Amplitude2-7.03125degrees and boolean slow flag required')
    ticks=int(round(amplitude_deg*4096/360));speed_raw=30 if slow else 100
    delta = ticks if direction == 'positive' else -ticks
    initial = ah.snapshot(sdk, ph, pk)
    start = validate(group, initial, expected, delta)
    attempted = False
    readings = []
    trace = []
    recent = []
    began = time.monotonic()
    stage = 'preflight'
    cleanup = {'attempted':False, 'hold_acknowledged':False}
    def in_trial_window(position):
        margin = 16 if stage == 'return' else 8
        return start+min(delta-8,-margin) <= position <= start+max(delta+8,margin)

    def sample(target, states):
        state = states['3']
        row = {'elapsed_s':round(time.monotonic()-began, 4), 'stage':stage,
               'target_raw':target, 'position_raw':state['position'],
               'goal_raw':state['goal'], 'error_ticks':target-state['position'],
               'moving':state['moving'], 'torque':state['torque'],
               'voltage_raw':state['voltage_present_raw'], 'temperature_c':state['temperature_raw']}
        for name in ['present_speed_raw','present_load_raw','servo_status_raw','motor_phase_current_raw']:
            addr, size = jc.STATUS_REGS[name]
            fn = pk.read2ByteTxRx if size == 2 else pk.read1ByteTxRx
            value, comm, error = fn(ph, 3, addr)
            if comm != sdk.COMM_SUCCESS or error:
                raise RuntimeError(f'ID3 diagnostic read failed: {name}, comm={comm}, error={error}')
            row[name] = value
        if len(trace) < 128:
            trace.append(row)
        if row['servo_status_raw']:
            raise RuntimeError('ID3 servo status fault during diagnostic trial')
        return row
    def monitor(target):
        states = ah.snapshot(sdk, ph, pk)
        row = sample(target, states)
        for mid, state in states.items():
            baseline = initial[mid]
            lo, hi = group['joints'][mid]['safe_raw_limits']
            if state['model'] != 777 or state['mode'] != 0 or state['torque'] != 1 or [state['min'], state['max']] != [baseline['min'], baseline['max']]:
                raise RuntimeError('Joint holding/configuration changed')
            if not lo <= state['position'] <= hi or not 110 <= state['voltage_present_raw'] <= 130 or state['temperature_raw'] >= 60:
                raise RuntimeError('Pose/power/temperature unsuitable during trial')
            if mid == '3':
                if not in_trial_window(state['position']) or state['goal'] != target:
                    raise RuntimeError('Elbow escaped trial window or target changed')
            elif abs(state['position']-baseline['position']) > 8 or state['goal'] != baseline['goal'] or state['moving']:
                raise RuntimeError('Another arm joint moved or changed target')
        recent.append(row)
        del recent[:-3]
        return states
    try:
        for label, target in [('out', start+delta), ('return', start)]:
            stage = label
            tolerance = 16 if label == 'return' else 4
            attempted = True  # Includes acknowledgment lost after application.
            packet(sdk, ph, pk, target, speed_raw)
            deadline = time.monotonic()+3
            stable = 0
            while time.monotonic() < deadline:
                states = monitor(target)
                stable = stable+1 if abs(states['3']['position']-target) <= tolerance and not states['3']['moving'] else 0
                if stable >= 3:
                    break
                time.sleep(.05)
            else:
                evidence = stationary_timeout_evidence(recent, target, stage)
                if evidence is not None:
                    raise StationarySettlingTimeout(
                        f'Elbow {stage} missed tolerance while verified stationary; existing target preserved', evidence)
                raise RuntimeError(f'Elbow {stage} did not settle in 3 seconds; ending trial and requesting hold')
            readings.append({'stage':label, 'target_raw':target, 'tolerance_raw':tolerance, 'states':states})
            if label == 'out':
                # Visible pause, continuously rechecking all six joints.
                pause_deadline = time.monotonic()+2
                while time.monotonic() < pause_deadline:
                    time.sleep(.05)
                    monitor(target)
        return {'completed':True, 'servo_id':3, 'delta_raw':delta, 'servo_degrees':abs(delta)*360/4096, 'speed_raw':speed_raw, 'direction':direction, 'pause_seconds':2, 'settling_tolerances_raw':{'out':4, 'return':16},
                'start_raw':start, 'stages':readings, 'trace':trace, 'torque':'left on',
                'note':'Supported elbow commissioning only; no other joint writes; loaded power remains unqualified.'}
    except BaseException as failure:
        if isinstance(failure, StationarySettlingTimeout):
            cleanup.update({'action':'preserve_existing_target', 'goal_raw':target,
                            'verified_stationary':failure.evidence, 'target_write_sent':False})
            print('Stationary positioning failure: existing target and holding torque retained; no new target sent.', file=sys.stderr)
        else:
            if attempted:
                cleanup['attempted'] = True
                old = {sig:signal.signal(sig, signal.SIG_IGN) for sig in (signal.SIGINT, signal.SIGTERM)}
                try:
                    if jc.read(sdk, ph, pk, 3, 'torque') != 1:
                        raise RuntimeError('Elbow torque no longer on; not re-enabling')
                    pos = jc.read(sdk, ph, pk, 3, 'position')
                    lo, hi = group['joints']['3']['safe_raw_limits']
                    if not lo <= pos <= hi or not in_trial_window(pos):
                        raise RuntimeError('Elbow feedback outside trial limits')
                    packet(sdk, ph, pk, pos, speed_raw)
                    cleanup.update({'hold_goal_raw':pos, 'hold_acknowledged':True})
                    print('Requested elbow hold at current feedback; no automatic return or torque release.', file=sys.stderr)
                except Exception as exc:
                    cleanup['error'] = str(exc)
                    print(f'HOLD FAILED: {exc}; keep supporting arm and use physical cutoff if needed.', file=sys.stderr)
                finally:
                    for sig, handler in old.items():
                        signal.signal(sig, handler)
        print(json.dumps({'completed':False, 'servo_id':3, 'delta_raw':delta, 'start_raw':start,
                          'failed_stage':stage, 'failure_kind':('stationary_settling_timeout' if isinstance(failure, StationarySettlingTimeout) else 'other'), 'error':str(failure), 'trace':trace, 'cleanup':cleanup,
                          'return_attempted':stage=='return'}), flush=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-positions', required=True)
    parser.add_argument('--direction', choices=['positive','negative'], default='positive')
    parser.add_argument('--amplitude-deg',type=float,default=7.03125)
    parser.add_argument('--slow',action='store_true')
    args = parser.parse_args()
    expected = json.loads(args.expected_positions)
    if set(expected) != set(map(str, ah.IDS)) or any(type(v) is not int or not 0 <= v <= 4095 for v in expected.values()):
        raise ValueError('Six integer observed positions required')
    group = json.loads(jc.CONFIG.read_text())['groups']['left-arm']
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
            print(json.dumps(control(sdk, ph, pk, group, expected, args.direction, args.amplitude_deg, args.slow)), flush=True)
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
