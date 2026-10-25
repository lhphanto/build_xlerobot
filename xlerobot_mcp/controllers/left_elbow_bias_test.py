#!/usr/bin/env python3
"""One supervised left-ID3 gravity-bias trial: overcommand by the measured sag, then hold; no return."""
import argparse, fcntl, hashlib, json, os, signal, sys, tempfile, time
import joint_control as jc
import left_arm_hold as ah
import left_elbow_test as et

# Bias equals the measured residual. Small residuals are noise; >16 is outside
# what the elbow test preflight accepts and suggests something other than sag.
MIN_SAG, MAX_SAG = 5, 16
TOLERANCE = 4


def plan(group, states, expected):
    """Pure check; returns the overcommanded target for the existing elbow goal."""
    start = et.validate(group, states, expected, 0)
    desired = states['3']['goal']
    sag = start - desired
    if not MIN_SAG <= abs(sag) <= MAX_SAG:
        raise ValueError(f'ID3 residual {sag} ticks outside {MIN_SAG}-{MAX_SAG}; nothing to compensate')
    target = desired - sag
    et.validate(group, states, expected, target-start)
    return {'start_raw':start, 'desired_raw':desired, 'sag_ticks':sag, 'bias_ticks':-sag, 'target_raw':target}


def control(sdk, ph, pk, group, expected):
    initial = ah.snapshot(sdk, ph, pk)
    p = plan(group, initial, expected)
    start, desired, target = p['start_raw'], p['desired_raw'], p['target_raw']
    trace = []
    began = time.monotonic()
    attempted = False
    cleanup = {'attempted':False, 'hold_acknowledged':False}
    def in_window(position):
        # Between the old position and the biased target, with an 8-tick margin.
        return min(start, target)-8 <= position <= max(start, target)+8

    def monitor():
        states = ah.snapshot(sdk, ph, pk)
        s = states['3']
        row = {'elapsed_s':round(time.monotonic()-began, 4), 'target_raw':target,
               'position_raw':s['position'], 'goal_raw':s['goal'],
               'error_to_desired':s['position']-desired, 'moving':s['moving']}
        for name in ['present_speed_raw','present_load_raw','servo_status_raw']:
            addr, size = jc.STATUS_REGS[name]
            fn = pk.read2ByteTxRx if size == 2 else pk.read1ByteTxRx
            value, comm, error = fn(ph, 3, addr)
            if comm != sdk.COMM_SUCCESS or error:
                raise RuntimeError(f'ID3 diagnostic read failed: {name}')
            row[name] = value
        if len(trace) < 128:
            trace.append(row)
        if row['servo_status_raw']:
            raise RuntimeError('ID3 servo status fault during bias trial')
        for mid, state in states.items():
            baseline = initial[mid]
            lo, hi = group['joints'][mid]['safe_raw_limits']
            if state['model'] != 777 or state['mode'] != 0 or state['torque'] != 1 or [state['min'], state['max']] != [baseline['min'], baseline['max']]:
                raise RuntimeError('Joint holding/configuration changed')
            if not lo <= state['position'] <= hi or not 110 <= state['voltage_present_raw'] <= 130 or state['temperature_raw'] >= 60:
                raise RuntimeError('Pose/power/temperature unsuitable during trial')
            if mid == '3':
                if not in_window(state['position']) or state['goal'] != target:
                    raise RuntimeError('Elbow escaped bias window or target changed')
            elif abs(state['position']-baseline['position']) > 8 or state['goal'] != baseline['goal'] or state['moving']:
                raise RuntimeError('Another arm joint moved or changed target')
        return states

    try:
        attempted = True  # Includes acknowledgment lost after application.
        et.packet(sdk, ph, pk, target, 30)
        deadline = time.monotonic()+3
        stable, last = 0, None
        while time.monotonic() < deadline:
            states = monitor()
            pos = states['3']['position']
            stable = stable+1 if not states['3']['moving'] and last is not None and abs(pos-last) <= 2 else 0
            last = pos
            if stable >= 3:
                break
            time.sleep(.05)
        else:
            raise RuntimeError('Elbow did not become stationary in 3 seconds')
        final = states['3']['position']
        # Stationary either way; a miss is reported, not corrected or retried.
        return {'completed':True, 'reached_desired':abs(final-desired) <= TOLERANCE, **p,
                'final_raw':final, 'final_error_to_desired':final-desired,
                'final_error_to_goal':final-target, 'tolerance_raw':TOLERANCE,
                'trace':trace, 'torque':'left on',
                'note':'Biased goal left in place so the elbow holds at the corrected pose. Goal-vs-position preflights on other tools still see the offset.'}
    except BaseException as failure:
        if attempted:
            cleanup['attempted'] = True
            old = {sig:signal.signal(sig, signal.SIG_IGN) for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                if jc.read(sdk, ph, pk, 3, 'torque') != 1:
                    raise RuntimeError('Elbow torque no longer on; not re-enabling')
                pos = jc.read(sdk, ph, pk, 3, 'position')
                lo, hi = group['joints']['3']['safe_raw_limits']
                if not lo <= pos <= hi or not in_window(pos):
                    raise RuntimeError('Elbow feedback outside trial limits')
                et.packet(sdk, ph, pk, pos, 30)
                cleanup.update({'hold_goal_raw':pos, 'hold_acknowledged':True})
                print('Requested elbow hold at current feedback; no automatic return or torque release.', file=sys.stderr)
            except Exception as exc:
                cleanup['error'] = str(exc)
                print(f'HOLD FAILED: {exc}; keep supporting arm and use physical cutoff if needed.', file=sys.stderr)
            finally:
                for sig, handler in old.items():
                    signal.signal(sig, handler)
        print(json.dumps({'completed':False, **p, 'error':str(failure), 'trace':trace, 'cleanup':cleanup}), flush=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-positions', required=True)
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
