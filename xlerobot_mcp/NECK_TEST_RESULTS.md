# Neck commissioning completed

October9,2026. ToDo2 complete for supervised neck commands with existing holding torque.

| Item | Saved result |
|---|---|
| Pan ID7 | Positive ticks turn right; default1995; operating limits1977–2139 |
| Tilt ID8 | Positive ticks tilt down; default2040; operating limits1959–2120 |
| Current state | Pan1995, tilt2042; both stationary, torqueON, fault-free at12.1V |

Three original out-and-back trials per axis produced stable encoder endpoints. The user later witnessed a pan trial, manually aligned forward pan, and confirmed a larger tilt-down-and-return trial was smooth and the returned tilt angle suitable as default. Original trials were not all witnessed; preserve that distinction in the raw log. A camera timeout interrupted the original sequence; the remaining trial and subsequent camera captures succeeded.

Ordinary commissioned-joint controls are now enabled within the conservative software limits. In-range and out-of-range target checks passed against fresh states without additional movement. Arms and base remain uncommissioned. Test sessions are disabled. No change to EEPROM calibration or homing offsets was made.
