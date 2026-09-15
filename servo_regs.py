#!/usr/bin/env python
"""Recover / inspect / edit Feetech STS-SMS servo registers, including the
lock-protected factory ("DEFAULT") block at addresses 80-86.

Depends only on pyserial + feetech-servo-sdk (no lerobot, no torch).

The register map below is transcribed from lerobot,
    lerobot/src/lerobot/motors/feetech/tables.py :: STS_SMS_SERIES_CONTROL_TABLE
which in turn follows Feetech's STS/SMS manual:
    http://doc.feetech.cn/#/prodinfodownload?srcType=FT-SMS-STS-emanual-229f4476422d4059abfb1cb0

Lock semantics (address 55): 0 = unlocked, 1 = locked. Writes to the factory
area are silently ignored while the servo is locked.

Usage:
    python servo_regs.py scan                                  # find port, baud, IDs
    python servo_regs.py dump --port P --baud B --id N
    python servo_regs.py set  --port P --baud B --id N --reg Maximum_Acceleration --value 254
"""

import argparse
import select
import sys
import time

import scservo_sdk as scs
import serial.tools.list_ports

# name: (address, size_bytes)
TABLE = {
    # --- EPROM ---
    "Firmware_Major_Version": (0, 1),
    "Firmware_Minor_Version": (1, 1),
    "Model_Number": (3, 2),
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay_Time": (7, 1),
    "Response_Status_Level": (8, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Max_Temperature_Limit": (13, 1),
    "Max_Voltage_Limit": (14, 1),
    "Min_Voltage_Limit": (15, 1),
    "Max_Torque_Limit": (16, 2),
    "Phase": (18, 1),
    "Unloading_Condition": (19, 1),
    "LED_Alarm_Condition": (20, 1),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Minimum_Startup_Force": (24, 2),
    "CW_Dead_Zone": (26, 1),
    "CCW_Dead_Zone": (27, 1),
    "Protection_Current": (28, 2),
    "Angular_Resolution": (30, 1),
    "Homing_Offset": (31, 2),
    "Operating_Mode": (33, 1),
    "Protective_Torque": (34, 1),
    "Protection_Time": (35, 1),
    "Overload_Torque": (36, 1),
    "Velocity_closed_loop_P_proportional_coefficient": (37, 1),
    "Over_Current_Protection_Time": (38, 1),
    "Velocity_closed_loop_I_integral_coefficient": (39, 1),
    # --- SRAM ---
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Time": (44, 2),
    "Goal_Velocity": (46, 2),
    "Torque_Limit": (48, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Status": (65, 1),
    "Moving": (66, 1),
    "Present_Current": (69, 2),
    # --- Factory / "DEFAULT" area, lock protected ---
    "Moving_Velocity_Threshold": (80, 1),
    "DTs": (81, 1),                      # ms
    "Velocity_Unit_factor": (82, 1),
    "Hts": (83, 1),                      # ns, firmware >= 2.54 only, else 0
    "Maximum_Velocity_Limit": (84, 1),
    "Maximum_Acceleration": (85, 1),
    "Acceleration_Multiplier": (86, 1),  # applies when Acceleration is 0
}

LOCK_ADDR, LOCK_LEN = TABLE["Lock"]
SRAM_START = 40
FACTORY_START = 80

# Model number (address 3) -> human name. From lerobot MODEL_NUMBER_TABLE.
MODEL_NUMBERS = {777: "sts3215", 2825: "sts3250", 3215: "sm8512bl", 5013: "scs0009"}

# Writing this value to Torque_Enable is not a torque state: it tells the servo to
# treat its current position as the centre (2048) by rewriting Homing_Offset.
TORQUE_CENTER_CMD = 128
CENTER_POSITION = 2048
CENTER_TOLERANCE = 10  # steps; the encoder reading jitters by a few counts

# Status register (65) error bits.
ERROR_BITS = [(0x01, "input voltage out of range"), (0x02, "angle limit exceeded"),
              (0x04, "overheating"), (0x08, "overcurrent"), (0x20, "overload")]
STS_PROTOCOL_END = 0  # STS/SMS are little-endian; SCS series would use 1
PORT_SETTLE_S = 0.1     # let a freshly opened USB-serial port settle before talking
DEFAULT_TIMEOUT_MS = 50  # matches lerobot's patched packet timeout
NUM_RETRY = 2         # retries per packet; USB-serial drops the odd frame

SCAN_BAUDRATES = [1_000_000, 500_000, 250_000, 128_000, 115_200,
                  57_600, 38_400, 19_200, 14_400, 9_600, 4_800]

DIAGNOSTIC = [
    "Firmware_Major_Version", "Firmware_Minor_Version", "Model_Number",
    "ID", "Baud_Rate", "Response_Status_Level",
    "Min_Position_Limit", "Max_Position_Limit", "Homing_Offset", "Operating_Mode",
    "Lock", "Torque_Enable", "Acceleration", "Present_Position", "Present_Voltage",
    "Moving_Velocity_Threshold", "DTs", "Velocity_Unit_factor", "Hts",
    "Maximum_Velocity_Limit", "Maximum_Acceleration", "Acceleration_Multiplier",
]


def candidate_ports():
    return [p for p in serial.tools.list_ports.comports()
            if "Bluetooth" not in p.device and "debug-console" not in p.device]


def _patched_set_packet_timeout(self, packet_length, _ms):
    """Replacement for PortHandler.setPacketTimeout.

    The PyPI `feetech-servo-sdk` computes a far too short packet timeout and
    recomputes it on every transaction, which silently overrides whatever
    setPacketTimeoutMillis() was given. See
    https://gitee.com/ftservo/SCServoSDK/issues/IBY2S6 - fixed upstream in
    FTServo_Python but never published to PyPI. lerobot patches this the same
    way (lerobot/motors/feetech/feetech.py::patch_setPacketTimeout).
    """
    self.packet_start_time = self.getCurrentTime()
    self.packet_timeout = (self.tx_time_per_byte * packet_length) + (self.tx_time_per_byte * 3.0) + _ms


def open_port(port, baud, timeout_ms=DEFAULT_TIMEOUT_MS):
    ph = scs.PortHandler(port)
    if not ph.openPort():
        raise OSError(f"could not open {port}")
    ph.setBaudRate(baud)
    ph.setPacketTimeout = (
        lambda packet_length, _s=ph, _ms=timeout_ms:
        _patched_set_packet_timeout(_s, packet_length, _ms)
    )
    ph.setPacketTimeoutMillis(timeout_ms)
    # USB-serial bridges (CH340 et al.) drop the first frames sent immediately
    # after the port is opened or re-opened; let the line settle first.
    time.sleep(PORT_SETTLE_S)
    # feetech-servo-sdk is Dynamixel-style: PacketHandler(end) takes no port,
    # and every method receives the port handler as its first argument.
    return ph, scs.PacketHandler(STS_PROTOCOL_END)


def read_reg(ph, pk, motor_id, addr, length, retries=NUM_RETRY):
    """Return (value, ok). A single dropped frame should not read as a dead servo."""
    for _ in range(1 + retries):
        if length == 2:
            val, comm, _err = pk.read2ByteTxRx(ph, motor_id, addr)
        else:
            val, comm, _err = pk.read1ByteTxRx(ph, motor_id, addr)
        if comm == scs.COMM_SUCCESS:
            return val, True
    return val, False


def ping(ph, pk, motor_id, retries=NUM_RETRY):
    """Return (model_number, ok)."""
    for _ in range(1 + retries):
        model, comm, _err = pk.ping(ph, motor_id)
        if comm == scs.COMM_SUCCESS:
            return model, True
    return 0, False


def write_reg(ph, pk, motor_id, addr, length, value):
    if length == 2:
        comm, _err = pk.write2ByteTxRx(ph, motor_id, addr, value)
    else:
        comm, _err = pk.write1ByteTxRx(ph, motor_id, addr, value)
    return comm == scs.COMM_SUCCESS


def cmd_scan(args):
    all_ports = list(serial.tools.list_ports.comports())
    print("Serial ports present:")
    for p in all_ports:
        print(f"  {p.device}  |  {p.description}  |  {p.hwid}")
    if not all_ports:
        print("  (none)")

    targets = [args.port] if args.port else [p.device for p in candidate_ports()]
    if not targets:
        print("\nNo USB serial adapter is enumerating. This is a cable/adapter/driver")
        print("problem, not a servo EEPROM problem - no servo register can affect this.")
        return 1

    ids = range(0, 254) if args.full else range(0, 21)
    print(f"\nPinging IDs {ids.start}-{ids.stop - 1} across {len(SCAN_BAUDRATES)} baud rates...")
    found_any = False
    for port in targets:
        for baud in SCAN_BAUDRATES:
            try:
                ph, pk = open_port(port, baud, timeout_ms=args.timeout)
            except OSError as e:
                print(f"  {port}: {e}")
                break
            hits = []
            for motor_id in ids:
                model, ok = ping(ph, pk, motor_id)
                if ok:
                    hits.append((motor_id, model))
            if hits:
                found_any = True
                for motor_id, model in hits:
                    name = MODEL_NUMBERS.get(model, "unknown model")
                    print(f"  FOUND  port={port}  baud={baud}  id={motor_id}  "
                          f"model={model} ({name})")
                    if not args.no_dump:
                        print(f"{'':<4}{'addr':>5} {'len':>3}  {'register':<48} value")
                        dump_servo(ph, pk, motor_id, indent="    ")
                        print()
            else:
                print(f"  ...  port={port}  baud={baud}: nothing")
            ph.closePort()

    if not found_any:
        print("\nNo servo responded. Next checks: servo power (it needs its own supply,")
        print("USB alone is not enough), the data line wiring, and -- if you only scanned")
        print("IDs 0-20 -- rerun with --full.")
        return 1
    return 0


def decode_sign_magnitude(value, sign_bit):
    """Homing_Offset is sign-magnitude with the sign in bit 11 (as in lerobot)."""
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if (value >> sign_bit) & 1 else magnitude


def area_of(addr):
    if addr < SRAM_START:
        return "EPROM"
    if addr < FACTORY_START:
        return "SRAM"
    return "DEFAULT (factory, lock protected)"


def dump_servo(ph, pk, motor_id, names=None, indent=""):
    """Print every register in `names` (default: the whole table) grouped by area."""
    names = names if names is not None else list(TABLE)
    ordered = sorted(names, key=lambda n: TABLE[n][0])
    current_area = None
    for name in ordered:
        addr, length = TABLE[name]
        area = area_of(addr)
        if area != current_area:
            current_area = area
            print(f"{indent}--- {area} ---")
        val, ok = read_reg(ph, pk, motor_id, addr, length)
        print(f"{indent}{addr:>5} {length:>3}  {name:<48} {val if ok else '<no response>'}")


def cmd_dump(args):
    ph, pk = open_port(args.port, args.baud, timeout_ms=args.timeout)
    names = DIAGNOSTIC if args.diagnostic else None
    print(f"{'addr':>5} {'len':>3}  {'register':<48} value")
    dump_servo(ph, pk, args.id, names)
    ph.closePort()
    return 0


def cmd_test(args):
    """Health-check one servo: link quality, supply voltage, temperature, faults."""
    ph, pk = open_port(args.port, args.baud, timeout_ms=args.timeout)
    failures = []

    def check(label, ok, detail):
        print(f"  [{'PASS' if ok else 'FAIL'}]  {label:<22} {detail}")
        if not ok:
            failures.append(label)

    # 1. link reliability - one dropped frame in 20 is worth knowing about
    attempts = 20
    good = sum(1 for _ in range(attempts) if ping(ph, pk, args.id, retries=0)[1])
    check("link", good == attempts, f"{good}/{attempts} pings answered")
    if good == 0:
        print("\n  Servo is not responding at all. Run `scan` to find its real id/baud.")
        ph.closePort()
        return 1

    model, _ = read_reg(ph, pk, args.id, *TABLE["Model_Number"])
    print(f"  {'':<7}{'model':<22} {model} ({MODEL_NUMBERS.get(model, 'unknown')})")

    # 2. supply voltage against the servo's own configured limits
    volts, _ = read_reg(ph, pk, args.id, *TABLE["Present_Voltage"])
    vmin, _ = read_reg(ph, pk, args.id, *TABLE["Min_Voltage_Limit"])
    vmax, _ = read_reg(ph, pk, args.id, *TABLE["Max_Voltage_Limit"])
    in_range = vmin <= volts <= vmax
    note = "" if in_range else "  <-- OUTSIDE THE SERVO'S OWN LIMITS"
    check("supply voltage", in_range,
          f"{volts / 10:.1f} V (limits {vmin / 10:.1f}-{vmax / 10:.1f} V){note}")
    # A 12 V servo that boots on USB's 5 V rail answers pings but cannot drive.
    if in_range and volts < 100:
        print(f"  [WARN]  {'':<22} {volts / 10:.1f} V is low for a 12 V servo; "
              "expect little or no torque")

    # 3. temperature
    temp, _ = read_reg(ph, pk, args.id, *TABLE["Present_Temperature"])
    tmax, _ = read_reg(ph, pk, args.id, *TABLE["Max_Temperature_Limit"])
    check("temperature", temp < tmax, f"{temp} C (limit {tmax} C)")

    # 4. latched fault flags
    status, _ = read_reg(ph, pk, args.id, *TABLE["Status"])
    faults = [name for bit, name in ERROR_BITS if status & bit]
    check("fault flags", not faults, f"status=0x{status:02x} " + (", ".join(faults) or "clear"))

    # 5. position sanity
    pos, _ = read_reg(ph, pk, args.id, *TABLE["Present_Position"])
    lo, _ = read_reg(ph, pk, args.id, *TABLE["Min_Position_Limit"])
    hi, _ = read_reg(ph, pk, args.id, *TABLE["Max_Position_Limit"])
    check("position", lo <= pos <= hi, f"{pos} (limits {lo}-{hi})")

    if args.move:
        print(f"\n  Motion test: moving {args.move:+d} steps and back...")
        start = pos
        target = max(lo, min(hi, start + args.move))
        write_reg(ph, pk, args.id, *TABLE["Torque_Enable"], 1)
        try:
            write_reg(ph, pk, args.id, *TABLE["Goal_Position"], target)
            time.sleep(args.settle)
            reached, _ = read_reg(ph, pk, args.id, *TABLE["Present_Position"])
            moved = abs(reached - start)
            check("motion", moved > abs(args.move) // 4,
                  f"commanded {start} -> {target}, reached {reached} (moved {moved} steps)")
            write_reg(ph, pk, args.id, *TABLE["Goal_Position"], start)
            time.sleep(args.settle)
        finally:
            write_reg(ph, pk, args.id, *TABLE["Torque_Enable"], 0)
            print(f"  {'':<7}{'torque':<22} disabled again")

    print(f"\n  {'ALL CHECKS PASSED' if not failures else 'FAILED: ' + ', '.join(failures)}")
    ph.closePort()
    return 0 if not failures else 1


def enter_pressed():
    """Non-blocking check for Enter on stdin (POSIX)."""
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        sys.stdin.readline()
        return True
    return False


def write_verified(ph, pk, motor_id, name, value):
    """Write one register (unlocking protected areas) and confirm by reading it back."""
    addr, length = TABLE[name]
    protected = area_of(addr) != "SRAM"
    if protected:
        write_reg(ph, pk, motor_id, LOCK_ADDR, LOCK_LEN, 0)
    try:
        write_reg(ph, pk, motor_id, addr, length, value)
    finally:
        if protected:
            write_reg(ph, pk, motor_id, LOCK_ADDR, LOCK_LEN, 1)
    after, ok = read_reg(ph, pk, motor_id, addr, length)
    return ok and after == value, after


def record_range(ph, pk, motor_id, stop, interval=0.05):
    """Stream Present_Position until stop() is true.

    Returns (min, max, crossed_wrap). crossed_wrap means consecutive samples jumped
    by more than half a turn, i.e. the joint travelled through the 4095 <-> 0 seam,
    so min/max no longer describe a contiguous range.
    """
    addr, length = TABLE["Present_Position"]
    lo = hi = last = None
    crossed_wrap = False
    while not stop():
        raw, ok = read_reg(ph, pk, motor_id, addr, length)
        if ok:
            pos = decode_sign_magnitude(raw, 15)
            if last is not None and abs(pos - last) > CENTER_POSITION:
                crossed_wrap = True
            last = pos
            lo = pos if lo is None else min(lo, pos)
            hi = pos if hi is None else max(hi, pos)
            flag = "  !! crossed the 4095/0 seam" if crossed_wrap else ""
            print(f"\r  position {pos:>5}   min {lo:>5}   max {hi:>5}{flag}   ", end="", flush=True)
        time.sleep(interval)
    print()
    return lo, hi, crossed_wrap


def cmd_range(args):
    """Record a joint's range of motion by hand and write Min/Max_Position_Limit."""
    ph, pk = open_port(args.port, args.baud, timeout_ms=args.timeout)
    if not ping(ph, pk, args.id)[1]:
        print(f"No response from id={args.id} at baud={args.baud}. Run `scan` first.")
        ph.closePort()
        return 1

    old_min, _ = read_reg(ph, pk, args.id, *TABLE["Min_Position_Limit"])
    old_max, _ = read_reg(ph, pk, args.id, *TABLE["Max_Position_Limit"])
    print(f"Current limits: Min_Position_Limit={old_min}  Max_Position_Limit={old_max}")
    print("Torque will be DISABLED so you can move the joint by hand - support any load first.")
    input("Press Enter to start recording...")

    write_reg(ph, pk, args.id, *TABLE["Torque_Enable"], 0)
    print("Move the joint slowly to BOTH mechanical ends. Press Enter when done.")
    lo, hi, crossed_wrap = record_range(ph, pk, args.id, stop=enter_pressed)

    if lo is None:
        print("No position readings were received.")
        ph.closePort()
        return 1
    print(f"Recorded range: {lo} .. {hi}")

    if crossed_wrap or lo < 0 or hi > 4095:
        print("The joint's travel crosses the 4095/0 seam, so it cannot be expressed as")
        print("Min < Max. Put the joint in the middle of its range, re-centre with")
        print("`set --reg Torque_Enable --value 128`, then record the range again.")
        ph.closePort()
        return 1

    new_min, new_max = lo + args.offset, hi - args.offset
    if new_min >= new_max:
        print(f"Range {lo}..{hi} is too small for --offset {args.offset} "
              f"(would give {new_min}..{new_max}). Use a smaller --offset.")
        ph.closePort()
        return 1
    print(f"Proposed limits (offset {args.offset}): "
          f"Min_Position_Limit={new_min}  Max_Position_Limit={new_max}")

    if not args.yes and input("Write these limits? [y/N] ").strip().lower() != "y":
        print("Nothing written.")
        ph.closePort()
        return 0

    results = [(name, *write_verified(ph, pk, args.id, name, value), value)
               for name, value in (("Min_Position_Limit", new_min), ("Max_Position_Limit", new_max))]
    for name, ok, after, value in results:
        print(f"  {name}: wrote {value}, read back {after}  {'OK' if ok else 'MISMATCH'}")
    print("Torque is left disabled.")
    ph.closePort()
    return 0 if all(ok for _, ok, _, _ in results) else 1


def center_here(ph, pk, motor_id):
    """Send Torque_Enable=128 and verify via position/offset, not via address 40."""
    pos_addr, pos_len = TABLE["Present_Position"]
    ofs_addr, ofs_len = TABLE["Homing_Offset"]
    te_addr, te_len = TABLE["Torque_Enable"]

    pos_before, ok1 = read_reg(ph, pk, motor_id, pos_addr, pos_len)
    ofs_before, ok2 = read_reg(ph, pk, motor_id, ofs_addr, ofs_len)
    if not (ok1 and ok2):
        print("could not read position/offset before centring")
        return False
    print(f"Torque_Enable (addr {te_addr}) <- {TORQUE_CENTER_CMD}: set current position as centre")
    print(f"  before: Present_Position={pos_before}  "
          f"Homing_Offset={decode_sign_magnitude(ofs_before, 11)} (raw {ofs_before})")

    # The servo rewrites Homing_Offset, which lives in the lock-protected EPROM,
    # so unlock around the command to make sure the new offset is kept.
    write_reg(ph, pk, motor_id, LOCK_ADDR, LOCK_LEN, 0)
    try:
        write_reg(ph, pk, motor_id, te_addr, te_len, TORQUE_CENTER_CMD)
        time.sleep(0.2)  # give the servo time to store the offset
    finally:
        write_reg(ph, pk, motor_id, LOCK_ADDR, LOCK_LEN, 1)

    pos_after, ok1 = read_reg(ph, pk, motor_id, pos_addr, pos_len)
    ofs_after, ok2 = read_reg(ph, pk, motor_id, ofs_addr, ofs_len)
    torque, _ = read_reg(ph, pk, motor_id, te_addr, te_len)
    if not (ok1 and ok2):
        print("  after: <no response>")
        return False
    print(f"  after:  Present_Position={pos_after}  "
          f"Homing_Offset={decode_sign_magnitude(ofs_after, 11)} (raw {ofs_after})  "
          f"Torque_Enable={torque}")

    centred = abs(pos_after - CENTER_POSITION) <= CENTER_TOLERANCE
    if centred:
        print(f"  OK - position now reads {pos_after} (target {CENTER_POSITION} +/- {CENTER_TOLERANCE})")
        print("  Power-cycle the servo and run `dump` to confirm Homing_Offset was kept.")
        print("  Any existing lerobot calibration for this motor is now stale; recalibrate.")
    else:
        print(f"  FAILED - position reads {pos_after}, expected about {CENTER_POSITION}. "
              "The servo did not accept the centre command.")
    return centred


def cmd_set(args):
    if args.reg not in TABLE:
        print(f"Unknown register {args.reg!r}. Known names:")
        for name in TABLE:
            print("   ", name)
        return 1

    addr, length = TABLE[args.reg]
    ph, pk = open_port(args.port, args.baud, timeout_ms=args.timeout)

    if args.reg == "Torque_Enable" and args.value == TORQUE_CENTER_CMD:
        good = center_here(ph, pk, args.id)
        ph.closePort()
        return 0 if good else 1

    before, ok = read_reg(ph, pk, args.id, addr, length)
    if not ok:
        print(f"No response from id={args.id} at baud={args.baud}. Run `scan` first.")
        ph.closePort()
        return 1
    print(f"{args.reg} (addr {addr}, {length}B): {before} -> {args.value}")

    # Only the plain SRAM window (40-79) is freely writable. Both the EPROM
    # block below it and the factory block above it are gated by Lock.
    protected = area_of(addr) != "SRAM"
    # Changing the ID moves the servo mid-sequence: the relock and the read-back
    # have to be addressed to its new ID, not the one we opened with.
    target_id = args.value if args.reg == "ID" else args.id

    if protected:
        write_reg(ph, pk, args.id, LOCK_ADDR, LOCK_LEN, 0)  # unlock
    try:
        write_reg(ph, pk, args.id, addr, length, args.value)
    finally:
        if protected:
            write_reg(ph, pk, target_id, LOCK_ADDR, LOCK_LEN, 1)  # relock

    after, ok = read_reg(ph, pk, target_id, addr, length)
    good = ok and after == args.value
    print(f"read back: {after if ok else '<no response>'}  "
          f"{'OK' if good else 'MISMATCH - the write did not take'}")
    if good and args.reg == "ID":
        print(f"servo now answers on id={args.value}; use --id {args.value} from here on")
    ph.closePort()
    return 0 if good else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT_MS,
                    help=f"per-packet timeout in ms (default {DEFAULT_TIMEOUT_MS})")

    # Accept --timeout on either side of the subcommand. SUPPRESS keeps the
    # subparser from overwriting a value given before the subcommand.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--timeout", type=int, default=argparse.SUPPRESS,
                        help=f"per-packet timeout in ms (default {DEFAULT_TIMEOUT_MS})")

    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("scan", parents=[common], help="find servos and dump the registers of each one found")
    p.add_argument("--port", help="only scan this port")
    p.add_argument("--full", action="store_true", help="scan IDs 0-253 instead of 0-20")
    p.add_argument("--no-dump", action="store_true",
                   help="only report which IDs answered, do not dump their registers")
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser("dump", parents=[common], help="print every register of one servo")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, required=True)
    p.add_argument("--id", type=int, required=True)
    p.add_argument("--diagnostic", action="store_true",
                   help="show only the key registers instead of the full table")
    p.set_defaults(func=cmd_dump)

    p = sub.add_parser("test", parents=[common],
                       help="health-check a servo: link, voltage, temperature, faults")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, required=True)
    p.add_argument("--id", type=int, required=True)
    p.add_argument("--move", type=int, default=0, metavar="STEPS",
                   help="also command a relative move of STEPS and verify it tracked "
                        "(omit for a read-only check)")
    p.add_argument("--settle", type=float, default=1.0,
                   help="seconds to wait for the move to complete (default 1.0)")
    p.set_defaults(func=cmd_test)

    p = sub.add_parser("range", parents=[common],
                       help="record a joint's range by hand and write Min/Max_Position_Limit")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, required=True)
    p.add_argument("--id", type=int, required=True)
    p.add_argument("--offset", type=int, default=20, metavar="STEPS",
                   help="steps kept inside each recorded end: Min = min + STEPS, "
                        "Max = max - STEPS (default 20). Unrelated to Homing_Offset.")
    p.add_argument("--yes", action="store_true", help="write the limits without asking")
    p.set_defaults(func=cmd_range)

    p = sub.add_parser("set", parents=[common], help="write one register, unlocking the factory area when needed")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, required=True)
    p.add_argument("--id", type=int, required=True)
    p.add_argument("--reg", required=True)
    p.add_argument("--value", type=int, required=True)
    p.set_defaults(func=cmd_set)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
