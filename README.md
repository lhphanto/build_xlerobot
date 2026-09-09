# build_xlerobot

Tools for building and servicing an XLeRobot.

## `servo_regs.py`

A command-line utility to scan for, inspect, and edit the registers of Feetech
STS/SMS bus servos (STS3215 and relatives) — including the lock-protected
factory area at addresses 80–86 that most tools hide behind a modifier key.

Use it to recover a servo you can no longer find on the bus, to read back its
full configuration, or to change a single register with verification that the
write actually landed.

### Requirements

The script itself imports only two packages:

- [`pyserial`](https://pypi.org/project/pyserial/)
- [`feetech-servo-sdk`](https://pypi.org/project/feetech-servo-sdk/) (imported as `scservo_sdk`)

It does **not** import `lerobot`. It is nonetheless meant to be run from a
lerobot environment, because that is where those two dependencies already live
and where the rest of the XLeRobot workflow runs.

#### Recommended: run it in your lerobot environment

Follow the official guide at
<https://huggingface.co/docs/lerobot/installation>. In short (conda, Python 3.12):

```bash
conda create -y -n lerobot python=3.12
conda activate lerobot

git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e ".[feetech]"     # Feetech motor support - installs feetech-servo-sdk
```

The `feetech` extra is the one that matters here; `pyserial` arrives with the
`hardware` extra (included in `core_scripts`). If you installed lerobot for
XLeRobot you almost certainly already have both, and can just run the script.

#### Alternative: a standalone environment

If you only want the servo tooling and not lerobot's ML stack (which pulls in
PyTorch):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial feetech-servo-sdk
```

### Usage

```bash
# 1. Find every servo on the bus and dump its registers
python servo_regs.py scan
python servo_regs.py scan --full          # sweep IDs 0-253 instead of 0-20
python servo_regs.py scan --no-dump       # just report which IDs answered

# 2. Read one servo whose address you already know
python servo_regs.py dump --port /dev/cu.usbmodemXXXX --baud 1000000 --id 1
python servo_regs.py dump ... --diagnostic   # key registers only

# 3. Write a single register, with read-back verification
python servo_regs.py set --port /dev/cu.usbmodemXXXX --baud 1000000 --id 1 \
    --reg Maximum_Acceleration --value 254
```

`scan` walks all 11 supported baud rates (4 800 → 1 000 000) and pings each ID,
so it still finds a servo whose baud rate or ID has drifted. For each servo it
finds it prints the model name and then the full register table, grouped by area
(EPROM / SRAM / DEFAULT) — one command tells you everything about the bus.
Pass `--no-dump` for just the summary line.

Every packet is retried (`NUM_RETRY`) and a freshly opened port is given
`PORT_SETTLE_S` to settle first. USB-serial bridges routinely drop the first
frames after open; without this a healthy servo intermittently reads as absent.

`set` prints the value before and after, then reports `OK` or
`MISMATCH - the write did not take`, so a silently ignored write is visible
rather than assumed to have worked.

### The factory ("DEFAULT") area and the Lock register

Addresses 80–86 hold motion-profile constants. They are protected by the **Lock**
register at address 55:

| Lock value | Meaning                      |
| ---------- | ---------------------------- |
| `0`        | unlocked — writes take effect |
| `1`        | locked — writes are silently ignored |

Feetech's own debug software gates this area behind <kbd>Ctrl</kbd>+<kbd>D</kbd>.

The Lock register guards **both** protected blocks — the EPROM window at
addresses 0–39 and the factory window at 80–86. Only the plain SRAM window
(40–79) is freely writable. `servo_regs.py set` works this out from the address:
for a protected register it writes `Lock = 0`, performs the write, then restores
`Lock = 1`; for an SRAM register it writes directly and never touches the lock.

Changing `ID` (address 5) is a special case, handled automatically: the servo
starts answering on its new address the moment the write lands, so the relock and
the read-back are addressed to the new ID rather than the old one.

```bash
# give a freshly plugged-in servo (factory default id=1) the id 3
python servo_regs.py set --port /dev/cu.usbmodemXXXX --baud 1000000 --id 1 \
    --reg ID --value 3
```

| Addr | Register                    | Notes                                        |
| ---- | --------------------------- | -------------------------------------------- |
| 80   | `Moving_Velocity_Threshold` |                                              |
| 81   | `DTs`                       | ms                                           |
| 82   | `Velocity_Unit_factor`      |                                              |
| 83   | `Hts`                       | ns; firmware ≥ 2.54 only, reads `0` otherwise |
| 84   | `Maximum_Velocity_Limit`    |                                              |
| 85   | `Maximum_Acceleration`      |                                              |
| 86   | `Acceleration_Multiplier`   | applies when `Acceleration` (addr 41) is 0   |

> **Warning**
> These are calibration constants, not runtime settings, and they persist across
> power cycles. Run `dump` and record the current values before changing
> anything. Note that ID and baud rate live at addresses 5 and 6 — nothing in
> the factory area can affect how the servo enumerates on the bus.

### Troubleshooting

**`scan` lists no USB serial port.** The adapter is not enumerating — a cable,
adapter, or driver problem. No servo register can cause this. On macOS a CH340
adapter appears as `/dev/cu.usbmodem*` or `/dev/cu.usbserial-*`.

**A port exists but nothing answers.** Check that the servo has its own power
supply; USB alone will not drive the bus, and an unpowered servo is
indistinguishable from a dead one. Then retry with `--full` in case the ID moved.

### Provenance

The register map is transcribed from lerobot's
`src/lerobot/motors/feetech/tables.py` (`STS_SMS_SERIES_CONTROL_TABLE`), which
follows [Feetech's STS/SMS manual](http://doc.feetech.cn/#/prodinfodownload?srcType=FT-SMS-STS-emanual-229f4476422d4059abfb1cb0).
Every address and size in this script was diffed against that table.
