#!/usr/bin/env python3
"""Bounded base commands derived from teleop_keyboard_base.py; preview by default."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import tempfile
import time

IDS = (8, 9, 7, 10)  # FL, FR, RL, RR; never address arm IDs 1-6
CONFIG = Path(__file__).resolve().parents[1] / 'robot_config.json'
PORT = json.loads(CONFIG.read_text())['base']['port']
ACTIONS = ('status', 'forward', 'backward', 'left', 'right', 'rotate-left', 'rotate-right', 'stop')


def wheel_raw(action, speed, turn):
    x, y, t = {
        'forward': (speed, 0, 0), 'backward': (-speed, 0, 0),
        'left': (0, speed, 0), 'right': (0, -speed, 0),
        'rotate-left': (0, 0, math.radians(turn)),
        'rotate-right': (0, 0, -math.radians(turn)), 'stop': (0, 0, 0),
    }[action]
    k = 0.119 + 0.2675
    values = [v / 0.05 * 4096 / (2 * math.pi)
              for v in (-x+y+k*t, x+y+k*t, -x-y+k*t, x-y+k*t)]
    scale = min(1, 3000 / max(max(map(abs, values)), 1))
    signed = [round(v * scale) for v in values]
    return [(abs(v) | 0x8000) if v < 0 else v for v in signed]


REGS = {'model':(3,2),'mode':(33,1),'torque':(40,1),'goal_velocity':(46,2),
        'present_velocity':(58,2),'voltage':(62,1),'temperature':(63,1),'moving':(66,1)}

def read(sdk,ph,pk,motor,field):
    addr,size=REGS[field]
    fn=pk.read2ByteTxRx if size==2 else pk.read1ByteTxRx
    value,comm,error=fn(ph,motor,addr)
    if comm!=sdk.COMM_SUCCESS or error:
        raise RuntimeError(f'Wheel read failed: id={motor}, field={field}, comm={comm}, fault={error}')
    return value

def states(sdk,ph,pk):
    return {str(m):{f:read(sdk,ph,pk,m,f) for f in REGS} for m in IDS}

def stationary_tolerance(motor):
    values=json.loads(CONFIG.read_text())["base"].get("stationary_velocity_tolerance_raw",{})
    value=values.get(str(motor),10)
    if type(value) is not int or not 0<=value<=60:raise ValueError("Invalid measured stationary tolerance")
    return value

def validate_wheels(snapshot):
    for motor,state in snapshot.items():
        if state['model']!=777 or state['mode']!=1:
            raise ValueError(f'Wheel {motor} requires STS3215 velocity mode; refusing automatic mode changes')
        if not 110<=state['voltage']<=130 or state['temperature']>=60:
            raise ValueError(f'Wheel {motor} voltage/temperature outside commissioning conditions')
        if state['goal_velocity'] or (state['present_velocity'] & 0x7fff)>stationary_tolerance(motor):
            raise ValueError(f'Wheel {motor} has nonzero velocity; do not take over another controller')

def verify_stop(sdk,ph,pk):
    deadline=time.monotonic()+2
    stable=0
    while time.monotonic()<deadline:
        snapshot=states(sdk,ph,pk)
        stopped=all(v['goal_velocity']==0 and (v['present_velocity'] & 0x7fff)<=stationary_tolerance(m) for m,v in snapshot.items())
        stable=stable+1 if stopped else 0
        if stable>=3:return snapshot
        time.sleep(.05)
    raise RuntimeError('Wheel zero/stationary feedback not confirmed within2 seconds')

def write(sdk, ph, pk, motor, addr, value, size=1):
    fn = pk.write2ByteTxRx if size == 2 else pk.write1ByteTxRx
    comm, error = fn(ph, motor, addr, value)
    if comm != sdk.COMM_SUCCESS or error:
        raise RuntimeError(f'write failed: motor={motor} register={addr} comm={comm} error={error}')


def stop_all(sdk, ph, pk):
    failures = []
    for motor in IDS:
        try:
            write(sdk, ph, pk, motor, 46, 0, 2)
        except Exception as exc:
            failures.append(str(exc))
    if failures:
        raise RuntimeError('Stop not confirmed: ' + '; '.join(failures))


def phase(args,name):
    request_id=getattr(args,'request_id',None)
    if not request_id:return
    tmp=None
    try:
        path=CONFIG.parent/'.private/base-phase.json'
        fd,tmp=tempfile.mkstemp(dir=path.parent,prefix='.base-phase-')
        with os.fdopen(fd,'w') as f:json.dump({'request_id':request_id,'phase':name,'unix':time.time()},f)
        os.replace(tmp,path)
    except Exception as exc:
        # Diagnostic file failures must never prevent wheel-zero cleanup.
        print(f'Phase diagnostic unavailable: {exc}',file=sys.stderr)
    finally:
        if tmp and os.path.exists(tmp):
            try:os.unlink(tmp)
            except OSError:pass


def run_live(args, sdk):
    # Lock coordinates this wrapper only; close keyboard/phone teleop first.
    name = hashlib.sha256(os.path.realpath(args.port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(), 'xlerobot-' + name + '.lock'), 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ph, pk = sdk.PortHandler(args.port), sdk.PacketHandler(0)
        try:
            if not ph.openPort():
                raise RuntimeError('Could not open serial port')
            if not ph.setBaudRate(1_000_000):
                raise RuntimeError('Could not set baud rate')
            def timeout(length):
                ph.packet_start_time=ph.getCurrentTime();ph.packet_timeout=ph.tx_time_per_byte*(length+3)+50
            ph.setPacketTimeout=timeout
            time.sleep(0.1)
            if args.action == 'status':
                print(json.dumps({'wheel_states':states(sdk,ph,pk),'hardware_writes':0}),flush=True)
                return
            if args.action != 'stop':
                initial=states(sdk,ph,pk)
                print(json.dumps({'initial_wheel_states':initial}),flush=True)
                validate_wheels(initial)
            stop_all(sdk,ph,pk)
            if args.action != 'stop':
                for motor in IDS:
                    write(sdk,ph,pk,motor,40,1)
                verify_stop(sdk,ph,pk)
                deadline = time.monotonic() + args.seconds
                raws = wheel_raw(args.action, args.speed, args.turn_speed)
                driving_reported=False
                while time.monotonic() < deadline:
                    for motor, raw in zip(IDS, raws):
                        if time.monotonic() >= deadline:
                            break
                        write(sdk, ph, pk, motor, 46, raw, 2)
                    if not driving_reported:
                        phase(args,'driving');driving_reported=True
                    time.sleep(min(1/30, max(0, deadline - time.monotonic())))
        finally:
            # Ignore repeated interrupts only during best-effort cleanup.
            old = {s: signal.signal(s, signal.SIG_IGN) for s in (signal.SIGINT, signal.SIGTERM)}
            try:
                if args.action != 'status':
                    phase(args,'stopping')
                    stop_all(sdk, ph, pk)
                    feedback=verify_stop(sdk,ph,pk)
                    print(json.dumps({'wheel_stop_feedback':feedback,'zero_velocity_confirmed':True}),flush=True)
                    phase(args,'stopped')
            finally:
                ph.closePort()
                for s, handler in old.items():
                    signal.signal(s, handler)
    if args.action != 'status':
        print('Zero velocity acknowledged and near-zero wheel feedback verified; chassis stop requires operator observation.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=ACTIONS)
    p.add_argument('--port', default=PORT)
    p.add_argument('--seconds', type=float, default=0.5)
    p.add_argument('--speed', type=float, default=0.1)
    p.add_argument('--turn-speed', type=float, default=20)
    p.add_argument('--request-id', default=None)
    p.add_argument('--execute', action='store_true', help='Send commands to hardware')
    args = p.parse_args()
    base = json.loads(CONFIG.read_text())['base']
    if base.get('port_verified') is not True or base['wheel_ids_FL_FR_RL_RR'] != list(IDS):
        p.error('Base topology requires verification')
    if args.port != base['port']:
        p.error('Port must match authoritative robot_config.json')
    if args.action == 'status':
        import scservo_sdk as sdk
        run_live(args, sdk)
        return
    for label, value, cap in [('seconds', args.seconds, 3), ('speed', args.speed, 0.2), ('turn-speed', args.turn_speed, 45)]:
        if not math.isfinite(value) or not 0 < value <= cap:
            p.error(f'{label} must be finite and in (0, {cap}]')
    print(json.dumps({'execute': args.execute, 'port': args.port, 'action': args.action,
                      'seconds': args.seconds, 'wheel_ids_FL_FR_RL_RR': IDS,
                      'raw_velocities': wheel_raw(args.action, args.speed, args.turn_speed)}), flush=True)
    if not args.execute:
        return
    import scservo_sdk as sdk
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Signal {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    run_live(args, sdk)


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print(f'ERROR: {exc}. If hardware was active, verify it stopped; use physical power cutoff if needed.', file=sys.stderr)
        sys.exit(1)
