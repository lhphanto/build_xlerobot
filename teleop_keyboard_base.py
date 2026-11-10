#!/usr/bin/env python
"""Drive the XLeRobot mecanum base from the keyboard.

Standalone: talks to the four wheel servos over the Feetech bus directly, so
nothing has to be copied into lerobot/robots/ and lerobot need not be installed.

Kinematics, motor IDs and key bindings are taken from XLeRobot's own
software/src/robots/xlerobot_mecanum/{xlerobot.py,config_xlerobot.py}:

    motors            wiring-dependent, see --wheel-ids (this robot: 7=RL 8=FL 9=FR 10=RR)
    mecanum X matrix  [[-1, 1, k], [1, 1, k], [-1, -1, k], [1, -1, k]]
    k                 half_wheelbase + half_track
    keys              i/k forward/back, j/l strafe, u/o rotate, n/m speed, b quit

The wheels share the right arm's bus (IDs 1-6 are that arm), so point --port at
the right arm's controller board.

Usage:
    python teleop_keyboard_base.py --port /dev/cu.usbmodemXXXX
    python teleop_keyboard_base.py --port ... --dry-run    # print commands, don't move
"""

import argparse
import math
import select
import sys
import termios
import time
import tty

import numpy as np
import scservo_sdk as scs

# The kinematics matrix rows are in this order; which servo id is on each wheel
# is wiring-dependent and set with --wheel-ids.
WHEEL_ORDER = ("front_left", "front_right", "rear_left", "rear_right")

# This robot's wiring: 7=rear-left, 8=front-left, 9=front-right, 10=rear-right.
# XLeRobot's stock order is 7,8,9,10 = FL,FR,RL,RR -> pass --wheel-ids 7,8,9,10.
DEFAULT_WHEEL_IDS = "8,9,7,10"


def parse_wheel_ids(spec):
    """Return [(name, id), ...] in FL, FR, RL, RR order."""
    parts = [t.strip() for t in spec.split(",") if t.strip()]
    if len(parts) != 4:
        raise SystemExit("--wheel-ids needs 4 ids in FL,FR,RL,RR order; got " + str(len(parts)))
    try:
        ids = [int(i) for i in parts]
    except ValueError:
        raise SystemExit("--wheel-ids must be integers; got " + repr(spec)) from None
    if len(set(ids)) != 4:
        raise SystemExit("--wheel-ids must be distinct; got " + str(ids))
    return list(zip(WHEEL_ORDER, ids, strict=True))

# Register map (STS/SMS), same addresses servo_regs.py uses.
ADDR_OPERATING_MODE = (33, 1)
ADDR_TORQUE_ENABLE = (40, 1)
ADDR_GOAL_VELOCITY = (46, 2)
ADDR_LOCK = (55, 1)
MODE_VELOCITY = 1
STEPS_PER_DEG = 4096.0 / 360.0
PROTOCOL_END = 0
PORT_SETTLE_S = 0.1
NUM_RETRY = 2

KEYS = {
    "i": "forward", "k": "backward", "j": "left", "l": "right",
    "u": "rotate_left", "o": "rotate_right",
    "n": "speed_up", "m": "speed_down", "b": "quit", " ": "stop",
}


def open_port(port, baud, timeout_ms=50):
    ph = scs.PortHandler(port)
    if not ph.openPort():
        raise SystemExit(f"could not open {port}")
    ph.setBaudRate(baud)
    ph.setPacketTimeoutMillis(timeout_ms)
    time.sleep(PORT_SETTLE_S)
    return ph, scs.PacketHandler(PROTOCOL_END)


def write_reg(ph, pk, motor_id, addr_len, value, retries=NUM_RETRY):
    addr, length = addr_len
    for _ in range(1 + retries):
        if length == 2:
            comm, _ = pk.write2ByteTxRx(ph, motor_id, addr, value)
        else:
            comm, _ = pk.write1ByteTxRx(ph, motor_id, addr, value)
        if comm == scs.COMM_SUCCESS:
            return True
    return False


def write_protected(ph, pk, motor_id, addr_len, value):
    """Addresses below 40 live in the lock-protected EPROM area."""
    write_reg(ph, pk, motor_id, ADDR_LOCK, 0)
    try:
        return write_reg(ph, pk, motor_id, addr_len, value)
    finally:
        write_reg(ph, pk, motor_id, ADDR_LOCK, 1)


def degps_to_raw(degps):
    """Signed speed -> Goal_Velocity, which is sign-magnitude with the sign in bit 15."""
    raw = int(round(degps * STEPS_PER_DEG))
    raw = max(-0x7FFF, min(0x7FFF, raw))
    return (abs(raw) | 0x8000) if raw < 0 else raw


class MecanumBase:
    def __init__(self, ph, pk, wheels, wheel_radius, half_wheelbase, half_track, max_raw, dry_run=False):
        self.ph, self.pk, self.dry_run = ph, pk, dry_run
        self.wheels = wheels
        self.wheel_radius = wheel_radius
        self.max_raw = max_raw
        k = half_wheelbase + half_track
        # XLeRobot's matrix, X roller layout; rows are FL, FR, RL, RR.
        self.M = np.array(
            [[-1.0, 1.0, k], [1.0, 1.0, k], [-1.0, -1.0, k], [1.0, -1.0, k]], dtype=float
        )

    def body_to_wheel_raw(self, x, y, theta_degps):
        """x forward m/s, y left m/s, theta deg/s CCW -> one raw command per wheel."""
        q = np.array([x, y, math.radians(theta_degps)], dtype=float)
        wheel_linear = self.M @ q                       # m/s at the wheel rim
        wheel_degps = (wheel_linear / self.wheel_radius) * (180.0 / math.pi)
        # Scale the whole vector down if any wheel exceeds the cap, preserving direction.
        peak = max((abs(d) * STEPS_PER_DEG for d in wheel_degps), default=0.0)
        if peak > self.max_raw:
            wheel_degps = wheel_degps * (self.max_raw / peak)
        return [degps_to_raw(float(d)) for d in wheel_degps]

    def configure(self):
        for name, motor_id in self.wheels:
            if self.dry_run:
                continue
            if not write_protected(self.ph, self.pk, motor_id, ADDR_OPERATING_MODE, MODE_VELOCITY):
                raise SystemExit(f"could not set velocity mode on {name} (id {motor_id}); run a scan")
            write_reg(self.ph, self.pk, motor_id, ADDR_TORQUE_ENABLE, 1)

    def send(self, x, y, theta):
        raws = self.body_to_wheel_raw(x, y, theta)
        if self.dry_run:
            pretty = ", ".join(f"{n}(id{i})={r if r < 0x8000 else -(r & 0x7FFF)}"
                               for (n, i), r in zip(self.wheels, raws, strict=True))
            print(f"  x={x:+.2f} y={y:+.2f} th={theta:+6.1f} -> {pretty}")
            return
        for (_, motor_id), raw in zip(self.wheels, raws, strict=True):
            write_reg(self.ph, self.pk, motor_id, ADDR_GOAL_VELOCITY, raw, retries=0)

    def stop(self):
        for _, motor_id in self.wheels:
            if not self.dry_run:
                write_reg(self.ph, self.pk, motor_id, ADDR_GOAL_VELOCITY, 0)

    def release(self):
        self.stop()
        for _, motor_id in self.wheels:
            if not self.dry_run:
                write_reg(self.ph, self.pk, motor_id, ADDR_TORQUE_ENABLE, 0)


def read_key(timeout=0.0):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else None


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="serial port of the right arm / base board")
    p.add_argument("--baud", type=int, default=1_000_000)
    p.add_argument("--wheel-ids", default=DEFAULT_WHEEL_IDS, metavar="FL,FR,RL,RR",
                   help="servo ids for front-left, front-right, rear-left, rear-right. "
                        "Default is this robot's wiring (7=RL 8=FL 9=FR 10=RR); "
                        "XLeRobot stock is --wheel-ids 7,8,9,10")
    p.add_argument("--speed", type=float, default=0.15, help="starting linear speed, m/s")
    p.add_argument("--turn-speed", type=float, default=45.0, help="rotation speed, deg/s")
    p.add_argument("--wheel-radius", type=float, default=0.05)
    p.add_argument("--half-wheelbase", type=float, default=0.119)
    p.add_argument("--half-track", type=float, default=0.2675)
    p.add_argument("--max-raw", type=int, default=3000, help="per-wheel raw speed cap")
    p.add_argument("--hold", type=float, default=0.35,
                   help="seconds a keypress keeps the base moving; it stops if no key repeats")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--dry-run", action="store_true", help="print wheel commands instead of moving")
    return p.parse_args()


def main():
    args = parse_args()
    print("Settings:")
    for name, value in sorted(vars(args).items()):
        print(f"  --{name.replace('_', '-'):<16} {value!r}")

    wheels = parse_wheel_ids(args.wheel_ids)
    print("  wheels           " + ", ".join(f"{n}=id{i}" for n, i in wheels))

    ph, pk = open_port(args.port, args.baud)
    base = MecanumBase(ph, pk, wheels, args.wheel_radius, args.half_wheelbase,
                       args.half_track, args.max_raw, dry_run=args.dry_run)
    base.configure()

    print(
        "\n  i/k  forward / backward      j/l  strafe left / right\n"
        "  u/o  rotate left / right     n/m  speed up / down\n"
        "  space  stop                  b or Ctrl+C  quit\n"
        f"\nspeed {args.speed:.2f} m/s, turn {args.turn_speed:.0f} deg/s. "
        "The base stops on its own if you stop pressing keys.\n"
    )

    speed, turn = args.speed, args.turn_speed
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    last_cmd = (0.0, 0.0, 0.0)
    last_key_time = 0.0
    try:
        tty.setcbreak(fd)
        while True:
            t0 = time.perf_counter()
            key = read_key(0.0)
            if key:
                action = KEYS.get(key.lower())
                if action == "quit":
                    break
                if action == "speed_up":
                    speed, turn = speed * 1.25, turn * 1.25
                    print(f"\rspeed {speed:.2f} m/s, turn {turn:.0f} deg/s      ", end="", flush=True)
                elif action == "speed_down":
                    speed, turn = speed / 1.25, turn / 1.25
                    print(f"\rspeed {speed:.2f} m/s, turn {turn:.0f} deg/s      ", end="", flush=True)
                elif action == "stop":
                    last_cmd, last_key_time = (0.0, 0.0, 0.0), 0.0
                elif action:
                    last_cmd = {
                        "forward": (speed, 0.0, 0.0),
                        "backward": (-speed, 0.0, 0.0),
                        "left": (0.0, speed, 0.0),
                        "right": (0.0, -speed, 0.0),
                        "rotate_left": (0.0, 0.0, turn),
                        "rotate_right": (0.0, 0.0, -turn),
                    }[action]
                    last_key_time = time.perf_counter()

            # Dead-man: without a fresh keypress the base coasts to a stop.
            if last_key_time and (time.perf_counter() - last_key_time) > args.hold:
                last_cmd, last_key_time = (0.0, 0.0, 0.0), 0.0

            base.send(*last_cmd)
            time.sleep(max(1.0 / args.fps - (time.perf_counter() - t0), 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        base.release()
        ph.closePort()
        print("\nStopped, wheel torque released.")


if __name__ == "__main__":
    main()
