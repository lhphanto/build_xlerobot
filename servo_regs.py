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
STS_PROTOCOL_END = 0  # STS/SMS are little-endian; SCS series would use 1
PORT_SETTLE_S = 0.1   # let a freshly opened USB-serial port settle before talking
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


def open_port(port, baud, timeout_ms=20):
    ph = scs.PortHandler(port)
    if not ph.openPort():
        raise OSError(f"could not open {port}")
    ph.setBaudRate(baud)
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


def cmd_set(args):
    if args.reg not in TABLE:
        print(f"Unknown register {args.reg!r}. Known names:")
        for name in TABLE:
            print("   ", name)
        return 1

    addr, length = TABLE[args.reg]
    ph, pk = open_port(args.port, args.baud, timeout_ms=args.timeout)

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
    ap.add_argument("--timeout", type=int, default=20, help="per-packet timeout in ms (default 20)")

    # Accept --timeout on either side of the subcommand. SUPPRESS keeps the
    # subparser from overwriting a value given before the subcommand.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--timeout", type=int, default=argparse.SUPPRESS,
                        help="per-packet timeout in ms (default 20)")

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
