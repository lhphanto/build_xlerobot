#!/usr/bin/env python3
"""Supervised first test: enable neck ID7 at its current pose, then +16 ticks."""
import argparse
import fcntl
import hashlib
import json
import os
import signal
import sys
import tempfile
import time
import joint_control as jc


def test(sdk, ph, pk, group):
    motor = 7
    state = {k: jc.read(sdk, ph, pk, motor, k) for k in jc.REGS}
    limits = group['joints']['7']['safe_raw_limits']
    if not group['port_verified'] or not limits or [state['min'], state['max']] != limits:
        raise ValueError('Port/limits differ from verified calibration')
    start = state['position']; target = start + 16
    if state['model'] != 777 or state['mode'] != 0 or state['torque'] != 0 or state['moving']:
        raise ValueError('Requires stationary STS3215 position-mode neck with torque OFF')
    if not limits[0] <= start < target <= limits[1]:
        raise ValueError('Start/target outside calibrated limits')
    if not 110 <= state['voltage_present_raw'] <= 130:
        raise ValueError('Expected approximately 12V; refusing test')
    print(json.dumps({'id': motor, 'start': start, 'target': target, 'state': state}), flush=True)
    jc.write(sdk, ph, pk, motor, 44, 0)
    jc.write(sdk, ph, pk, motor, 46, 100)
    jc.write(sdk, ph, pk, motor, 42, start)
    if jc.read(sdk, ph, pk, motor, 'goal') != start or abs(jc.read(sdk, ph, pk, motor, 'position') - start) > 4:
        raise RuntimeError('Preload verification failed or neck moved during setup')
    try:
        comm, error = pk.write1ByteTxRx(ph, motor, 40, 1)
        if comm != sdk.COMM_SUCCESS or error:
            raise RuntimeError('Torque-enable acknowledgment failed')
        if jc.read(sdk, ph, pk, motor, 'torque') != 1:
            raise RuntimeError('Torque enable not confirmed')
        time.sleep(.3)
        if abs(jc.read(sdk, ph, pk, motor, 'position') - start) > 8:
            raise RuntimeError('Unexpected movement while holding initial pose')
        reached = jc.nudge(sdk, ph, pk, motor, target)
        print(json.dumps({'completed': True, 'reported_position': reached, 'torque': 'left enabled to hold neck'}))
    except BaseException:
        try:
            pos = jc.read(sdk, ph, pk, motor, 'position')
            if not limits[0] <= pos <= limits[1]:
                raise RuntimeError('Hold position outside limits')
            jc.write(sdk, ph, pk, motor, 42, pos)
        except Exception as exc:
            print(f'HOLD FAILED: {exc}; support neck and use physical cutoff if necessary.', file=sys.stderr)
        raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true'); a=p.parse_args()
    if not a.execute:
        print('Preview: neck ID7 only, preload current pose, enable torque, +16 raw ticks; hold on exit.'); return
    import scservo_sdk as sdk
    group=json.loads(jc.CONFIG.read_text())['groups']['neck']; port=group['port']
    name=hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(),'xlerobot-'+name+'.lock'),'a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        ph,pk=sdk.PortHandler(port),sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000): raise RuntimeError('Port setup failed')
            def timeout(length):
                ph.packet_start_time=ph.getCurrentTime(); ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout; time.sleep(.1)
            test(sdk,ph,pk,group)
        finally: ph.closePort()

if __name__=='__main__':
    def interrupt(signum, frame): raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try: main()
    except (Exception,KeyboardInterrupt) as exc:
        print(f'Test stopped: {exc}',file=sys.stderr); sys.exit(1)
