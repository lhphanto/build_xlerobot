#!/usr/bin/env python3
"""Read arm/neck state or make one commissioned, bounded joint nudge."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import time

CONFIG = Path(__file__).resolve().parents[1] / 'robot_config.json'
REGS = {'model': (3, 2), 'min': (9, 2), 'max': (11, 2), 'mode': (33, 1),
        'voltage_min_raw': (15, 1), 'voltage_max_raw': (14, 1),
        'voltage_present_raw': (62, 1), 'temperature_raw': (63, 1),
        'torque': (40, 1), 'goal': (42, 2), 'position': (56, 2), 'moving': (66, 1)}


# Extra registers are read only by status, not by motion polling.
# Addresses: official WaveShare ST Servo Memory Map Explanation.
STATUS_REGS = {**REGS, 'max_torque_raw': (16, 2), 'phase_raw': (18, 1),
               'position_p_raw': (21, 1), 'position_d_raw': (22, 1),
               'position_i_raw': (23, 1), 'starting_torque_raw': (24, 1),
               'dead_zone_positive_raw': (26, 1), 'dead_zone_negative_raw': (27, 1),
               'homing_offset_raw': (31, 2), 'angle_resolution_raw': (30, 1), 'acceleration_raw': (41, 1),
               'running_speed_raw': (46, 2), 'torque_limit_raw': (48, 2),
               'present_speed_raw': (58, 2), 'present_load_raw': (60, 2),
               'servo_status_raw': (65, 1), 'motor_phase_current_raw': (69, 2)}


def diagnostic_status(sdk, ph, pk, motor):
    state, faults = {}, {}
    for name, (addr, size) in STATUS_REGS.items():
        fn = pk.read2ByteTxRx if size == 2 else pk.read1ByteTxRx
        value, comm, error = fn(ph, motor, addr)
        if comm != sdk.COMM_SUCCESS:
            raise RuntimeError(f'Communication failed reading {name}: {comm}')
        state[name] = value
        if error:
            faults[name] = {'bits': error, 'description': pk.getRxPacketError(error)}
    return state, faults


def read(sdk, ph, pk, motor, name):
    addr, size = REGS[name]
    fn = pk.read2ByteTxRx if size == 2 else pk.read1ByteTxRx
    value, comm, error = fn(ph, motor, addr)
    if comm != sdk.COMM_SUCCESS or error:
        raise RuntimeError(f'Read failed: id={motor}, {name}, comm={comm}, error={error}; ' + pk.getRxPacketError(error))
    return value


def write(sdk, ph, pk, motor, addr, value):
    comm, error = pk.write2ByteTxRx(ph, motor, addr, value)
    if comm != sdk.COMM_SUCCESS or error:
        raise RuntimeError(f'Write failed: id={motor}, register={addr}, comm={comm}, error={error}')


def validate_target(group, joint, state, delta):
    if group.get('port_verified') is not True or joint.get('commissioned') is not True:
        raise ValueError('Live movement requires verified port mapping and commissioned safe joint limits')
    limits = joint.get('safe_raw_limits')
    if not isinstance(limits, list) or len(limits) != 2 or any(type(v) is not int for v in limits):
        raise ValueError('Provide two calibrated integer safe_raw_limits')
    lo, hi = limits
    if not 0 <= lo < hi <= 4095 or not state['min'] <= lo < hi <= state['max']:
        raise ValueError('Software limits must lie inside non-wrapping servo limits')
    if state['model'] != 777 or state['mode'] != 0 or state['torque'] != 1:
        raise ValueError('Requires STS3215, position mode, and torque already enabled')
    if state['moving'] or abs(state['goal'] - state['position']) > 8:
        raise ValueError('Joint is moving or has an outstanding goal; do not take over')
    target = state['position'] + delta
    if not lo <= state['position'] <= hi or not lo <= target <= hi:
        raise ValueError('Current or target position is outside calibrated safe limits')
    return target


def nudge(sdk, ph, pk, motor, target):
    attempted = False
    try:
        # Finite, conservative speed; never change EEPROM, mode, offsets, or torque.
        write(sdk, ph, pk, motor, 44, 0)
        write(sdk, ph, pk, motor, 46, 100)
        attempted = True
        write(sdk, ph, pk, motor, 42, target)
        deadline = time.monotonic() + 3
        stable = 0
        while time.monotonic() < deadline:
            pos = read(sdk, ph, pk, motor, 'position')
            moving = read(sdk, ph, pk, motor, 'moving')
            stable = stable + 1 if abs(pos - target) <= 8 and moving == 0 else 0
            if stable >= 3:
                return pos
            time.sleep(0.05)
        raise RuntimeError('Target was not reached within 3 seconds')
    except BaseException:
        if attempted:
            # Preserve gravity support. Holding current position is best effort,
            # not a guaranteed emergency stop if communications are unavailable.
            try:
                pos = read(sdk, ph, pk, motor, 'position')
                if not 0 <= pos <= 4095:
                    raise RuntimeError('Invalid feedback while holding')
                write(sdk, ph, pk, motor, 42, pos)
                print('Requested hold at current feedback position.', file=sys.stderr)
            except Exception as exc:
                print(f'HOLD FAILED: {exc}; support the robot and use physical cutoff if needed.', file=sys.stderr)
        raise


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['status', 'nudge'])
    p.add_argument('--group', required=True, choices=['left-arm', 'right-arm', 'neck'])
    p.add_argument('--id', type=int, required=True)
    p.add_argument('--delta-raw', type=int, default=0, help='Relative ticks, at most +/-32 (~2.8 servo degrees)')
    p.add_argument('--execute', action='store_true')
    p.add_argument('--config', type=Path, default=CONFIG)
    a = p.parse_args()
    if a.action == 'nudge' and not 0 < abs(a.delta_raw) <= 32:
        p.error('nudge requires a nonzero --delta-raw between -32 and 32')
    if a.action == 'status' and (a.execute or a.delta_raw):
        p.error('status accepts neither --execute nor --delta-raw')
    config = json.loads(a.config.read_text())
    group = config['groups'][a.group]
    allowed = range(7, 9) if a.group == 'neck' else range(1, 7)
    if a.id not in allowed:
        p.error('ID is not allowed for the selected group')
    joint = group['joints'][str(a.id)]
    if a.action == 'nudge' and not a.execute:
        print(json.dumps({'preview': True, 'port': group['port'], 'group': a.group,
                          'id': a.id, 'delta_raw': a.delta_raw,
                          'commissioned': joint['commissioned'],
                          'note': 'No hardware accessed; target and limits require live feedback.'}))
        return
    if a.action == 'nudge' and (not group['port_verified'] or not joint['commissioned']):
        raise ValueError('Movement disabled pending port verification and safe joint commissioning')
    import scservo_sdk as sdk
    port = group['port']
    name = hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20]
    with open(os.path.join(tempfile.gettempdir(), 'xlerobot-' + name + '.lock'), 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ph, pk = sdk.PortHandler(port), sdk.PacketHandler(0)
        try:
            if not ph.openPort() or not ph.setBaudRate(1000000):
                raise RuntimeError('Could not configure serial bus')
            # Match the timeout workaround in the user's servo_regs.py.
            def packet_timeout(length):
                ph.packet_start_time = ph.getCurrentTime()
                ph.packet_timeout = ph.tx_time_per_byte * (length + 3) + 50
            ph.setPacketTimeout = packet_timeout
            time.sleep(0.1)
            if a.action == 'status':
                # Report diagnostic values plus faults without allowing motion.
                state, faults = diagnostic_status(sdk, ph, pk, a.id)
                print(json.dumps({'port': port, 'id': a.id, 'state': state, 'servo_faults': faults}), flush=True)
                return
            state = {name: read(sdk, ph, pk, a.id, name) for name in REGS}
            print(json.dumps({'port': port, 'id': a.id, 'state': state}), flush=True)
            if a.action == 'nudge':
                target = validate_target(group, joint, state, a.delta_raw)
                pos = nudge(sdk, ph, pk, a.id, target)
                print(json.dumps({'target_raw': target, 'reported_position_raw': pos, 'torque': 'left enabled'}))
        finally:
            ph.closePort()


if __name__ == '__main__':
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Signal {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
