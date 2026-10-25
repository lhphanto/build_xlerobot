# Base commissioning results

Completed October12,2026. ToDo3 passed for supervised bounded base commands on clear level floor.

| Test | Result |
|---|---|
| Forward and backward | Two pulses per direction; user confirmed correct directions and prompt stopping |
| Left and right strafe | Two pulses per direction; user confirmed correct directions and prompt stopping |
| Left and right turn | Two pulses per direction; user confirmed correct directions and prompt stopping |
| Active interruption | Controller driving phase observed, stop_active returned interrupted true, wheel-zero cleanup completed, user confirmed prompt stop |
| Final state | All4 wheel goals and speed feedback zero, torque on, fault-free;11.9–12.0V,29–35C |

Each ordinary pulse was at most0.5 seconds, requested linear speed0.1m/s or turn rate20deg/s. Thirteen pulses completed: twelve ordinary pulses and one interrupted pulse. The user explicitly skipped the third round. No arms or neck were commanded during base commissioning.

One initial forward request was rejected by preflight before any nonzero velocity write; cleanup confirmed zero wheel goals and speeds. Two later backward preflight checks stopped locally before dispatch. These were retained in the log and were not counted as completed pulses. The first rejected UUID remains consumed.

Idle speed feedback sometimes reads0/±50 even while the operator confirms a physical stop. A narrowly bounded60-unit stationary tolerance is recorded for each wheel. Zero goals, successful fault-free reads and physical supervision are still required; feedback within that baseline is not standalone proof of a chassis stop.

Software changes: read-only base_status, pulse-phase diagnostics for matching an active request, preflight wheel model/mode/voltage checks, no automatic EEPROM mode changes, and post-stop velocity verification. Seven controller regression tests and HTTP MCP tests passed. Loaded power qualification, odometry and a hardware watchdog remain outside this commissioning result.

Authoritative project: /Users/huiliu/Documents/build_xlerobot/xlerobot_mcp. Raw test log preserved alongside this report. Next plan item: resolve arm limits and power checks.
