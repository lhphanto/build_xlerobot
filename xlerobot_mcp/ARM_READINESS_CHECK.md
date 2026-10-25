# Arm limits and power checks

October 16, 2026. ToDo 4 is in progress. These checks made no hardware writes.

All 12 arm servos respond in position mode, torque off, fault-free at12.0–12.1 V and31–33 C. Four resting positions remain outside saved travel limits. Right ID5 still has full0–4095 limits and no approved software range. Existing calibration and commissioning flags were preserved.

| Arm | ID | EEPROM limits | Position | Inside limits | Voltage |
|---|---:|---|---:|---|---|
| left-arm | 1 | 776–3132 | 1833 | Yes | 12.1 V |
| left-arm | 2 | 822–3132 | 789 | No | 12.0 V |
| left-arm | 3 | 650–2804 | 2818 | No | 12.0 V |
| left-arm | 4 | 930–3147 | 2287 | Yes | 12.0 V |
| left-arm | 5 | 488–3774 | 2170 | Yes | 12.0 V |
| left-arm | 6 | 1453–2808 | 1532 | Yes | 12.0 V |
| right-arm | 1 | 1686–2327 | 1752 | Yes | 12.1 V |
| right-arm | 2 | 903–3219 | 897 | No | 12.0 V |
| right-arm | 3 | 950–3049 | 3062 | No | 12.0 V |
| right-arm | 4 | 925–3207 | 2211 | Yes | 12.1 V |
| right-arm | 5 | 0–4095 | 1969 | Yes | 12.0 V |
| right-arm | 6 | 1272–2703 | 2313 | Yes | 12.1 V |

Remaining: select the first arm, establish a supported pose inside all its saved limits, confirm the current supply and cable ratings, and verify voltage/current under the intended load. Idle voltage and prior base pulses do not qualify loaded arm power. If left is selected first, right ID5 range confirmation remains a requirement before right-arm use.

## Left arm supported pose verified

User manually positioned and supported the left arm. Fresh reads: ID1=1903, ID2=1303, ID3=1762, ID4=2167, ID5=2169, ID6=2017. All6 are inside their existing saved limits, position mode, torqueOFF, no faults,12.0–12.1V. No calibration writes or powered arm movement were performed.

The selected-arm position issue is resolved for this supported pose. Remaining ToDo4 requirement: confirm current power/cable ratings and verify intended loaded arm operation. Right-arm IDs2/3 and ID5 range remain deferred until right-arm preparation.

## Provisional power assumption

User confirmed separate12V cables and requested assuming3A for the left connection because its exact rating is unknown. The resulting planning budget is36W. Actual cable/supply capacity and loaded current remain unverified; this assumption does not implement a current limit or complete loaded-power qualification.
