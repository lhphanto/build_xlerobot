# XLeRobot local MCP

A local HTTP MCP service wraps the robot scripts as named tools. It binds only to `127.0.0.1:8765`, requires a random bearer token, and rejects browser Origin headers. The operator launches it in Terminal and explicitly connects the client. This is a separate permissioned integration: it does not grant the chat shell camera/serial access. Hardware permissions still need to work for the service process. No public tunnel is needed.

## Current capabilities (November 25, 2026): read this first

The sections after this one are a dated commissioning log; where they disagree with this section, this section is
current. Agents operate the robot through this server (`client.py call <tool> --arguments JSON`), never by opening the
serial bus or camera themselves.

**Operating rules (from the operator, standing):** no robot motion without the operator's explicit go for that motion
or a clearly bounded sequence; never bypass a safety gate (e.g. `nudge_joint` refuses the commissioned left arm: use
the reach tools); never print `.private/token`; the operator starts/restarts the server (code changes need a restart);
a failed or uncertain motion is not retried blindly: read `joint_status` first. Release torque only with the arm
supported by the operator.

**What works now**

| Area | Tools / scripts | State |
|---|---|---|
| Camera | `camera_snapshot` | Continuous ffmpeg stream (`controllers/camera_stream.py`); the first snapshot waits ~30 s for exposure/white balance (earlier frames are yellow), later ones ~3 s; stream stops after 30 idle min |
| Stereo depth, camera-to-arm calibration | `reach_point.py --pixel U V --image F` (repo root) | Valid only with the neck at the calibration pose (pan 2012/tilt ~2287, within 6 ticks; `samples/hand_eye/handeye_result.json`) |
| Neck | `neck_hold`, `nudge_joint` (neck, <=32 ticks) | Torque can be off after a restart/idle: `neck_hold` with fresh positions (request_id must be a UUID) holds in place, no travel (`step.py hold`/`view` do it). Tilt ID8 limits [1959, 2356] (higher = down; 2350 shows the basket); pan ID7 [1977, 2139] (+ = right) |
| Left arm holding | `left_arm_hold` hold / recover-hold / release | recover-hold accepts only the elbow up to 32 ticks past its limit |
| Left arm motion | `preview_left_reach` then `move_left_reach` (same args + plan_hash) | Gripper point to [x, y, z] in the left arm base frame (x forward, y left, z up; operator's "right" = -y). `top_down` with `approach_tilt_deg` 0-90 and optional `jaw_yaw_deg` (jaw line, mod 180) or free roll; <=400 ticks per joint per move |
| Left gripper | `left_gripper_open`, `left_gripper_close` (squeeze 0-24), `left_gripper_tighten` | Open target 2162 (~88 mm). Grip push = holding - goal ticks: >=7 holds a juice box, 4 dropped it. Grip depth matters more for a tall box: 2 cm let it twist (Nov25); plans now grip 3.5 cm deep and squeeze twice |
| Base | `move_base`, `base_status`, `stop_base` | Short pulses only (<=0.5 s); base power fault open |
| Right arm | holding/trial tools | Commissioning only; no task motion yet |

**Pick-and-place skill (left arm):** `../grasp/` — SlimSAM segmentation + Contact-GraspNet grasps + model-checked
plans + `grasp/step.py`, one supervised step per go with a camera check after each. Runbook and lessons:
`../grasp/README.md`. Succeeded with a juice box into the black basket on November23, November24 and November25
(the last with no operator corrections).

**Known limits:** grasp position has a camera/model bias (operator corrects ~15-20 mm toward -y; default offset
-15 mm); the camera cannot see the object while the arm is between it and the lens (checks report UNSEEN); no
collision checking against the scene; shoulder ID2 warms (~40 C after a carry; warn 50, stop 60); a loaded yaw can
stall, and the loaded shoulder can end a few ticks past the 8-tick final tolerance (`move_left_reach` returns
completed false, no error); step.py re-runs either once and accepts a finish within 16 ticks.

## One-time setup

```sh
cd /Users/huiliu/Documents/build_xlerobot/xlerobot_mcp
/Users/huiliu/miniforge3/envs/lerobot/bin/python setup.py
```

This creates `.venv` for the official MCP SDK and `.private` for a local token and Codex config snippet. Robot subprocesses use the existing `lerobot` Python. It does not modify that environment, start hardware, or edit Codex settings. FFmpeg must be on PATH for camera capture.

## Connect first in mock mode

```sh
cd /Users/huiliu/Documents/build_xlerobot/xlerobot_mcp
./start.sh --mock
```

Keep Terminal open. Add the block in `.private/codex-config.toml` to `~/.codex/config.toml` (or use equivalent MCP settings in your client). Preserve existing config; do not add a second `[mcp_servers.xlerobot]` if it exists. The block contains a secret: do not paste it into chat or commit it. Restart/reload the client's MCP connection as needed, then ask for `robot_info`. Confirm that it returns `mock: true` and `motion_enabled: false`. Registration in this chat is not verified until the tools appear and a call succeeds.

Official configuration reference: https://learn.chatgpt.com/docs/extend/mcp?surface=cli

## Real camera and read-only diagnostics

Stop the mock server with Ctrl+C, close Photo Booth and other teleop programs, then:

```sh
./start.sh
```

Now ask for `joint_status` or `camera_snapshot`. Starting the service alone does not open hardware. Read tools open hardware only when called. Camera capture uses a continuous stream with a ~30 s first-use warm-up and 180-degree rotation (November24; originally a 5-second warm-up per capture). It returns a fresh side-by-side image directly through MCP. macOS may request camera permission for the host process; approve it only if expected. Hardware access must be verified here before claiming direct control works.

## Supervised movement

Only after hardware readback works and you are ready to supervise, stop the server and restart:

```sh
./start.sh --allow-motion
```

Enabling the service flag is not authorization for arbitrary movements: request each action or a clearly bounded sequence in chat. Tool approvals stay set to prompt in the supplied config.

Tools:

| Tool | Behavior |
|---|---|
| `robot_info` | Server mode/configuration; saved observations are not fresh hardware status |
| `joint_status` | Read one servo, including fault bits |
| `camera_snapshot` | Fresh, upright image while stationary |
| `move_base` | Up to 0.5 s at 0.1 m/s or 20 deg/s, then zero velocity |
| `nudge_joint` | Up to 32 ticks, requiring commissioned limits and enabled holding torque |
| `test_neck_pan` | Explicit first-test workflow: neck ID7 only, preload current position, enable torque, +16 ticks, hold |
| `stop_active` | Interrupt a running MCP movement; controller attempts wheel stop or joint hold |
| `stop_base` | Interrupt active movement then send wheel zeros; never disables arm torque |

Each intentional movement needs a fresh UUID `request_id`. Reusing an ID is rejected, including after a failed/uncertain command. IDs persist across server restarts in the private request ledger. Do not retry an uncertain motion with a new UUID. Commands are serialized and competing calls are rejected rather than queued. Close other robot controllers: this service cannot prevent unrelated applications from writing to the bus.

Arm/neck `commissioned` flags remain false. A completed neck test does not automatically mark a joint commissioned; observe its axis/direction first. No tool edits calibration, enables arbitrary arm torque, runs a shell command, or accepts an arbitrary serial port. `test_neck_pan` is deliberately separate from ordinary nudges and can move physical hardware only in motion-enabled mode.

`robot_config.json` is the authoritative configuration shared with the installed skill. Do not overwrite it with old observations. Actual commands always read fresh state. Recorded joint travel limits are not collision-free envelopes. Right-arm ID5's full 0–4095 range still requires review; four arm joints were outside stored limits at the latest user readback. (November24: supervised IK reaches, calibrated stereo depth and a step-by-step grasp skill now exist, see the capabilities section; there is still no autonomous unsupervised grasping or continuous collision avoidance.)

## Stop limitations

Software cleanup is best effort. A USB disconnection, host crash, killed process, blocked SDK call, or power fault can prevent stopping. There is no verified firmware watchdog or hardware emergency-stop integration. Keep physical cutoff accessible. Do not unplug the service as a substitute for a stop command. After joint movement, torque remains enabled to support the load. If a controller is unresponsive, the service does not start a competing serial writer; use physical intervention as needed.

The wrapper's base/joint/neck subprocesses contain finite movement logic. The MCP request timeout also requests interruption, but it cannot guarantee cleanup if hardware/SDK is unresponsive. Stopping the server does not guarantee a physical stop. Since November24 the camera runs as a persistent stream worker (`controllers/camera_stream.py`) rather than opening per capture.

## Tests

```sh
.venv/bin/python test_server.py
```

Tests use mock hardware only: real HTTP MCP handshake, tool discovery, auth rejection, Origin rejection, motion-disable gate, bounded inputs, duplicate requests, and interruption coordination. Live camera and neck status have been verified through the local client. Native tool discovery in this chat remains unverified. The existing controller mocks were also run during development. Do not infer hardware safety or successful physical movement from these tests.

## Protocol 2 connection workflow

`client.py` is the supported route when the chat's native MCP tool catalog does not expose xlerobot. It uses the same authenticated local service and safeguards; it does not access serial or camera directly. No token is printed, no call is automatically retried, and motion calls require an explicit UUID and a current protocol-2 server.

```sh
.venv/bin/python client.py call robot_info
.venv/bin/python client.py verify --output-dir /tmp/xlerobot-frames
.venv/bin/python client.py call joint_status --arguments '{"group":"neck","servo_id":7}'
.venv/bin/python client.py call camera_snapshot --output-dir /tmp/xlerobot-frames
```

`verify` reads a servo on each bus, both neck joints and a fresh camera image. It sends zero motion commands. Verify the printed `boot_id` changes after an operator restart and inspect the saved image. Mode/configuration responses are not fresh servo state. If the code digest changes while running, motion is rejected until restart.

Motion IDs now persist in `.private/motion-requests.sqlite3` across restarts. A failed or interrupted attempt still consumes its ID. The mock ledger is separate. Do not delete the ledger to retry an uncertain command. Ordinary joint nudges retain commissioning gates; the temporary 32/91-tick configuration overrides have been removed. `test_neck_pan` always means the documented +16 first test.

The project's `robot_config.json` is authoritative; the installed skill symlinks to it and delegates its direct scripts to `controllers/`. Edit only the project configuration. Do not treat exported snapshots as another active installation.

Additional no-hardware regression suite:

```sh
.venv/bin/python test_connection_controls.py
```

It checks durable replay protection, canonical UUIDs, consumption before failed execution, busy rejection, stop during process creation, motion-disabled startup, changed-code rejection, and ordinary joint commissioning gates. Physical stopping and movement still require separate supervised commissioning.

## Bounded neck commissioning

`test_neck_joint(servo_id, target_raw, expected_position, request_id, enable_if_off=False)` is a separate commissioning tool for neck IDs7/8 only. It requires a configured, unexpired `neck_test_session` with a fixed center, radius no greater than91 ticks, and a remaining move budget per axis. The current and target position must remain in that window; it rejects cross-window sweeps, changed calibration, stale positions, faults, motion, outstanding goals and unsuitable voltage/temperature. It never marks a joint commissioned.

If torque is off, explicit `enable_if_off` is required and the first step is limited to32 ticks. It preloads and verifies current position before enabling torque. Every attempt consumes its budget before writes. Motion and best-effort failure hold leave torque enabled. Disable the session after the approved sequence, a failed test, or a supervision stop. Before saving ordinary operating limits and camera-forward pose, review feedback, images and operator observation.

Run `.venv/bin/python test_neck_commission.py` for the no-hardware commissioning checks. The tool is for explicit supervised tests, not an alternate route around ordinary joint limits.

## Base commissioned October12 2026

All six base directions were confirmed by the operator with two trials per direction; the user skipped the originally planned third round. One active pulse was interrupted through stop_active, with wheel-zero cleanup and physical stopping confirmed. Ordinary caps remain0.5 seconds,0.1m/s and20deg/s. Arms remain uncommissioned.

base_status is read-only. The wrapper checks all4 wheels before driving, requires existing velocity mode and zero goals, and verifies stopped feedback afterward. Idle raw speed can alternate0/±50 on each wheel while the robot is physically still; measured tolerance60 is recorded in robot_config.json. Physical supervision remains required. Diagnostic pulse-phase file writes are best-effort so failure cannot block stop cleanup.

Use .venv/bin/python test_base_feedback.py for the seven wheel-controller regression cases. See BASE_TEST_RESULTS.md and the preserved raw log for commissioning evidence.

## Supported left-arm holding test

`left_arm_hold(action, request_id, expected_positions)` is a separate supported commissioning workflow. `hold` requires fresh expected positions for IDs1–6, torqueOFF, position mode, unchanged calibrated limits, in-range positions and fault-free approximately12V readings. Target writes may activate torque on this hardware. It handles one joint at a time using one atomic current-position/time/speed packet, verifies the goal, enables torque explicitly only if still off, and checks pose and voltage before the next joint. Cleanup only sends hold targets to joints whose torque is read back ON; it never activates an untouched joint. It changes no EEPROM, homing offsets, wheels or neck. Ordinary arm nudges remain gated by commissioning.

The operator must support the empty arm and keep fingers clear throughout. A failure can leave holding torque partially enabled; cleanup attempts a feedback hold and never automatically drops the arm. `release` requires explicit operator support and changes only left-arm torque. This workflow does not establish a3A current limit or verify supply/cable capacity. Loaded-power qualification remains pending actual ratings or measurements.

Tests: `.venv/bin/python test_left_arm_hold.py`.

The original all-targets-before-torque workflow was rejected by live verification: IDs1/2 became torqueON at their current positions during preloading. The operator supported the arm and explicitly authorized release; all6 were then verified torqueOFF and fault-free. The workflow was revised before another attempt. Vendor register reference: https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation


## Supported left-arm yaw trial

`test_left_yaw(expected_positions, request_id)` is a dedicated commissioning tool for the operator-identified left/right yaw joint, left-arm ID1. One call commands +80 raw ticks (7.03125 servo degrees), pauses at least2s after settling, and returns to the freshly read starting position. Positive anatomical direction still needs operator observation. No generic arm commissioning flags are changed.

Requires all six joints already holding, stationary, fault-free, in unchanged calibrated limits, within4 ticks of fresh supplied positions, with no outstanding goals,11–13V and below60°C. Only ID1 receives atomic target/time/speed packets at raw speed100. All six joints are monitored through both legs and the pause; unexpected drift, faults, target change, voltage or temperature stops the sequence. Each leg has a3s settle timeout. On failure or interruption it attempts to hold ID1 at feedback, never automatically returns or releases torque. Uncertain motion must not be retried. Empty arm must remain supported under operator supervision. This does not qualify loaded supply current or collision-free arm travel.

Tests: `.venv/bin/python test_left_yaw.py` (11 controller cases), and `test_server.py` (14 tools, HTTP schema/replay/auth and mock dispatch).


## Supported left-arm ID2 trial

`test_left_shoulder(expected_positions, request_id)` commands only left-arm ID2: +32 ticks (2.8125 servo degrees), two-second monitored pause, then return to its fresh starting pose. The operator identified it as probable shoulder lift; physical mapping and positive direction remain unverified. All six joints must already hold the supported empty arm; the same calibration, pose, faults, temperature, voltage, drift and failure-hold guards as the yaw trial apply. No EEPROM, torque-enable or other-joint writes. On interruption/failure it requests a current-feedback hold, never an automatic return or torque release. New UUID for one intentional trial; never retry an uncertain outcome.

`test_left_shoulder.py` covers11 controller cases; `test_server.py` covers the15-tool MCP interface, auth, bounds and replay. Ordinary arm commissioning and loaded current qualification remain pending.


## Supported left-arm ID3 trial

`test_left_elbow(expected_positions, request_id)` commands only left-arm ID3: +32 ticks (2.8125 servo degrees), a two-second monitored pause, then return to the fresh starting pose. User mapping is probable elbow bend until physically observed. Empty arm must be supported, all six joints already holding, stationary, in unchanged calibration, fault-free,11–13V and below60°C. It uses the shoulder controller's pose, target, drift, timeout and interruption checks, with writes restricted to ID3. Failure requests a current-feedback hold, not an automatic return or torque release. Ordinary arm commissioning flags remain false; loaded current remains unqualified.

Tests: `test_left_elbow.py` (11 controller cases), `test_server.py` (16 tools including ID3 mock dispatch/replay/auth/bounds).


ID3 tracking diagnostics now record elapsed time, stage, target/goal/position, error, moving/torque, voltage, temperature, speed, load, status and motor phase current. Up to128 samples are returned on success or failure, with failure-stage and hold-cleanup evidence. MCP retains up to64k of ID3 stdout so the bounded trace survives tool errors. These extra reads never change control settings. Mock suite now14 cases including timeout trace, diagnostic communication failure and status fault. Motor phase current is not a measurement of USB-C supply current.


ID3 larger visibility trial (operator approved October16): the current `test_left_elbow` is fixed to +80 ticks (7.03125 servo degrees), two-second monitored pause, then return if it settles. Speed100,3s per-leg timeout,4-tick settle tolerance and other-joint8-tick drift checks remain unchanged; only the active ID3 test window expands to start−8 through start+88. Current/target positions must remain inside stored limits. Tracking and failure holds are retained. Mock suite14 cases passes for this bound. This replaces the previous32-tick test amplitude; do not run it without the corresponding explicit scope. Physical ID3 mapping/direction still unconfirmed because operator could hardly see the smaller trial. Timeout text now accurately names outward or return stage.


## Supported left-arm ID4 wrist trial

`test_left_wrist(expected_positions, request_id)` commands only left-arm ID4: +32 ticks (2.8125 servo degrees), a two-second monitored pause, then return if settled. Mapping is probable wrist flex until physically verified. All six joints already holding the supported empty arm, unchanged EEPROM/software limits, fresh pose, no faults,11–13V and below60°C are required. Other joint drift >8ticks stops the trial; active ID4 stays within start−8 through start+40. Speed100,3s per-leg timeout and4-tick settling tolerance remain. Trace/cleanup evidence is retained on failures. Interruption/failure requests current-feedback hold, never automatic return or torque release. No EEPROM or other-joint writes; loaded power and ordinary arm commissioning remain unqualified.

`test_left_wrist.py`:14 controller cases. `test_server.py`:17-tool authenticated MCP integration including ID4 dispatch, bounds and replay.


## Supported left-arm ID5 roll trial

`test_left_roll(expected_positions, request_id)` commands only left-arm ID5: +32ticks (2.8125 servo degrees), two-second monitored pause, then return if settled. Operator identifies wrist roll; positive physical direction awaits observation. All six joints must already hold the supported empty arm, stationary and in unchanged calibrated limits, no outstanding targets/faults,11–13V and below60°C. Speed100,3s per-leg timeout,4-tick settling tolerance, other-joint8-tick drift guard, tracking trace and current-feedback failure hold remain. ID5 travel stays within start−8 through start+40. No EEPROM, torque-enable, wheels, neck or other-joint writes. Ordinary arm commissioning and loaded current remain unqualified.

`test_left_roll.py`:14 controller mock cases; `test_server.py`:18-tool MCP integration including ID5 dispatch, bounds and replay.


## Supported left-arm ID6 empty-gripper trial

`test_left_gripper(expected_positions, request_id)` commands only ID6: +32ticks (2.8125 servo degrees), two-second monitored pause, then return if settled. User confirms gripper; opening/closing direction awaits observation. Gripper empty, fingers clear, empty arm supported and all six joints already holding are required. Same unchanged calibrated limits, fresh pose, voltage/temperature/fault guards, speed100,3s leg timeout,4-tick settle tolerance, other-joint8-tick drift checks and tracking evidence apply. ID6 stays within start−8 through start+40, all inside stored limits. Failure/interruption requests current-feedback hold without automatic return or release. No EEPROM, torque-enable, other-joint or force-setting changes. This is a small empty-gripper direction test, not grasp-force or object-handling qualification.

`test_left_gripper.py`:14 mock cases. `test_server.py`:19-tool MCP integration including gripper bounds, dispatch and replay.


ID3 return acceptance update (operator authorized October16): only this supported commissioning tool now accepts ±8ticks (~0.70°) for the return leg. Outward acceptance stays ±4ticks, with three consecutive stationary samples required on each leg. Fixed +80tick trial,2s pause,3s timeout, speed100, unchanged calibrated limits and all drift/fault/power checks remain. This accepts the observed6–7tick residual; it does not improve accuracy, qualify a payload, or commission generic arm control. `test_left_elbow.py` now19 cases, explicitly testing ±8 acceptance,9 rejection, outward5 rejection and continued movement rejection. Live verification pending restart and one supervised trial.


## Supported left-arm ID1 clearance check

`test_left_yaw_range(expected_positions, request_id)` is one explicitly approved four-leg sequence around a fresh fixed center: +80ticks (left), center,−80ticks (right), center. Every point has a two-second monitored pause; each leg must settle within4ticks for three readings inside3s. Both endpoints must lie within the existing saved EEPROM/software limits. All six joints must already hold the supported empty arm; existing pose, model, mode, torque, voltage, temperature and fault checks apply. Nonactive joints may not drift more than8ticks. Only ID1 target/time/speed packets are written, speed100, active window center±88ticks. Failure/interruption attempts current-feedback hold without automatically returning or releasing torque. The MCP timeout is30s and trace capped256samples/128k output.15 controller mock tests and20-tool authenticated MCP integration pass.

This checks clearance of the assembled arm; it does not recalibrate or overwrite EEPROM limits. Do not save a collision-free operating window without operator observations, and do not infer a full-arm collision envelope from testing one joint while other joints hold one pose.


## Supported ID2 shoulder clearance

`test_left_shoulder_range(expected_positions, request_id)` commands one four-leg sequence: +80ticks(up), fresh fixed center,−80ticks(down), center;2s monitored pause at each point. Existing EEPROM/software limits must contain both endpoints. All six joints must already hold the supported empty arm; model/mode/torque/pose, voltage11–13V, temperature<60°C, faults, other-joint8tick drift checks remain. ID2-only atomic target/time/speed100 writes; active window center±88,4tick settling band for three stationary samples,3s leg timeout. Failure requests current-feedback hold without an automatic return or torque release. MCP timeout30s, trace cap256/128k.15 controller mock cases and21-tool authenticated integration pass. This checks only the current assembled-arm pose, not a multi-joint collision envelope. No EEPROM calibration changes or ordinary commissioning enable.


## Supported ID3 elbow clearance

`test_left_elbow_range(expected_positions, request_id)` commands one explicitly approved sequence: +80ticks, fresh fixed center,−80ticks, center;2s monitored pause at each point. Endpoint settling band4ticks, center8ticks, three stationary readings; all other model/mode/torque, fresh-pose, unchanged calibrated-limit, voltage11–13V, temperature<60°C, fault and other-joint8tick drift checks remain. Both endpoints must fit saved limits. ID3-only atomic packets at speed100; active window center±88,3s leg timeout; failure/interruption requests current-feedback hold without automatic return/release. MCP timeout30s, trace cap256/128k.20 controller mock tests plus22-tool MCP integration cover negative endpoint rejection, both return tolerances, failure hold and replay. Operator must support empty arm and verify clearance both ways; never infer a multi-joint collision envelope from one fixed-pose test. No EEPROM calibration writes or ordinary arm enable.


## Supported ID4 wrist-flex clearance

`test_left_wrist_range(expected_positions, request_id)` runs one supervised +80ticks(down), center,−80ticks(up), center sequence with2s pauses. Both endpoints within existing calibration; all six already holding the supported empty arm, unchanged limits, fresh pose, no faults,11–13V and<60°C required. Only ID4 receives atomic target/time/speed100 packets; active window center±88,4tick settling tolerance for three stationary readings,3s leg timeout and other-joint8tick drift checks. Failure/interruption requests current-feedback hold without automatic return/release. MCP timeout30s, trace cap256/128k.15 mock cases and23-tool MCP integration pass. Verify wrist/gripper/cable clearance physically before any pose-specific window is saved. ID3 clearance remains incomplete; this test never commands ID3 or changes its tolerance/settings. No EEPROM calibration or ordinary motion enable.


ID4 return-tolerance update (operator authorized October23): only `test_left_wrist_range` center returns now accept±6ticks (~0.53°); outward endpoints remain±4. Three stationary samples, speed100,±80tick excursion,2s pauses,3s timeout and all calibration/fault/power/drift guards remain. No servo tuning/settings changed.20 mock cases explicitly cover both±6 boundaries,7tick rejection, outward5 rejection and moving-return rejection.23-tool MCP checks pass. One supervised verification sequence pending restart; does not enable ordinary control or establish grasp/payload capability.


## Supported ID5 roll clearance

`test_left_roll_range(expected_positions, request_id)` commands one explicitly supervised +80tick clockwise, center,−80tick counterclockwise, center sequence with2s pauses, viewed from gripper tip toward arm. All six joints must already hold the supported empty arm; both targets within existing saved limits, fresh pose, model/mode/torque,11–13V,<60°C, strict fault checks and other-joint8tick drift guard apply. Only ID5 atomic target/time/speed100 packets; active window center±88,4tick tolerance for three stationary samples,3s leg timeout. Failure requests current-feedback hold without auto-return/release.30s MCP timeout and bounded256sample/128k trace.15 mock cases plus24-tool authenticated MCP integration. ID4's6tick return and ID3's8tick return exceptions do not apply to ID5. Calibration/ordinary arm flags unchanged; verify actual cable/obstacle clearance at this pose.


## Supported ID6 empty-gripper clearance

`test_left_gripper_range(expected_positions, request_id)` commands one supervised +80tick farther-open, center,−80tick toward-closed, center sequence with2s pauses. These are relative offsets around the current opening, not commands to fully open/close. Gripper empty, fingers clear, supported empty arm and all six joints holding required. Both targets within existing saved calibration; fresh pose, mode/model/torque, voltage11–13V,<60°C, strict faults and other-joint8tick drift checks apply. Only ID6 atomic target/time/speed100 packets; active window center±88,4tick tolerance for three stationary samples,3s leg timeout. On failure requests current-feedback hold without automatic return/release.30s MCP timeout, trace256/128k.15 mocks and25-tool integration pass. No EEPROM/torque-limit/force settings changed. This is empty-gripper clearance only, not grasp-force or object-handling qualification; ordinary arm flags remain false.


## ID3 stationary-timeout target-reset fix — October23

Both ID3 controllers now distinguish a narrowly verified stationary settling timeout from other failures. The test still fails and the sequence ends, but no replacement position target is sent and existing holding torque remains on. This prevents the generic cleanup from rebasing the goal after the motor has already stopped.

Eligibility: three validated samples within0.1–0.5s, same stage and unchanged target, position spread≤2ticks and error≤16ticks, moving=0/speed=0, torqueON/status0,11–13V/<60°C, motor-output-duty magnitude≤150raw and motor phase current≤16raw. All existing all-joint pose/calibration/fault checks must have passed. These narrow diagnostic bands are not a force or supply-current qualification, and16ticks is not the acceptance tolerance. Endpoints remain±4 and center returns±8; failure stays failure. No automatic next leg, retry or target reset in this branch. Other exceptions, interruptions, continued motion, excessive output or missing/stale diagnostics retain the original current-feedback stop attempt. Scope is ID3 only.

Failure JSON identifies `failure_kind=stationary_settling_timeout` and `cleanup.action=preserve_existing_target`, with `target_write_sent=false` and sample evidence. No fresh hold acknowledgment is claimed because nothing was written. Existing goal may remain outstanding; ordinary preflight can reject further commands and must not be bypassed. Holding a target sustains effort, so operator support and physical cutoff remain required.

Validation:23 two-leg tests,24 four-leg tests,6 classifier tests (53 total), plus25-tool HTTP MCP auth/bounds/replay/mock integration pass. Live verification remains pending operator restart and separate explicit supervised trial. Servo settings, PID, acceleration, torque limits, calibration and acceptance tolerances unchanged. This fixes an unintended post-timeout target command; it does not prove or eliminate a mechanical cause of the observed twitch.


## ID3 coarse center-return acceptance — October23

Operator explicitly authorized ±16ticks (~1.41°) for ID3 returns to center in both commissioning tools. Outward endpoints remain±4. Three stationary samples, speed100,3s timeout, saved calibration and all fault/power/other-joint checks remain. Only ID3 commissioning preflight accepts an existing stationary residual≤16 (with old goal inside saved limits), so a preserved target does not require a reset to enter the test; other joints remain≤8, expected pose freshness remains≤4. Two-leg return monitoring allows at most16ticks below center to implement the negative acceptance boundary; outward monitoring retains its old−8 margin. Ordinary arm gates remain disabled. The target-reset fix and its narrow stationary-timeout evidence gate are unchanged; faults/uncertainty/larger residuals still take legacy stop handling.

This accepts coarse measured error; it does not improve tracking, qualify load or permit precise grasping.27 two-leg tests,28 range tests and6 classifier tests (61 total) plus25-tool HTTP MCP integration pass. Boundary tests cover±16 acceptance,17 rejection, outward5 rejection, no extra target for accepted13tick returns, other-joint9 preflight rejection, and preservation behavior on stationary endpoint timeout. Live verification pending restart and explicit supervision. No EEPROM, PID, torque, speed or acceleration settings changed.


ID3 opposite-direction-only commissioning: `test_left_elbow` accepts `direction=positive|negative` (default positive). One call commands exactly one80tick excursion in that direction,2s pause, then return. Negative permits the remaining opposite check without repeating the positive excursion. Both target and direction-specific monitored window must fit saved limits: endpoint margin8ticks, return-center margin16. Endpoint±4/return±16 acceptance and all existing guards/reset fix remain.33 two-leg controller tests pass, including negative-only command sequence, both16tick return boundaries, lower-limit rejection, invalid direction and interruption;25-tool MCP integration covers negative dispatch, bounds and replay. Code reload required before movement. Full-body clearance not inferred.


### Right-arm supported holding tool — October 23, 2026
`right_arm_hold` supports only right-bus arm IDs 1–6, using the left-arm atomic current-position/speed activation workflow. Target writes may enable torque. All six fresh expected positions are required; unchanged calibration, stationary torque-off position-mode state, pose, faults, voltage and temperature are checked. Failure can leave partial holding torque; no automatic release. Release requires operator support.
ID5 has no approved travel limits. For holding only, a temporary expected-position ±8-tick window is used within unchanged EEPROM 0–4095; no config limits or commissioning flags are changed. This does not qualify ID5 motion.
Validation: 26 controller/connection tests passed, plus mock HTTP MCP auth, 26-tool discovery, right-arm dispatch, input bounds and replay checks. No live right-arm writes performed. Operator restart and supported hold approval required next.


### Right-arm ID1 trial prepared
`test_right_yaw`: fixed +80 ticks (~7°), monitored two-second pause, then original position. All six joints must already hold, with fresh positions, unchanged calibration and no outstanding targets. Only ID1 receives writes; return/outward settle tolerance4ticks, trial window start−8..start+88. Failure holds current ID1 feedback when valid, no automatic return/release. ID5 remains unqualified, checked in a local ±8-tick held-position window without config edits. 43 controller/connection tests and 27-tool mock HTTP integration passed. Live ID1 trial pending operator restart; no live yaw sent.


## Complete right-arm test software — October 23, 2026
All six small out-and-back tests and all six clearance sequences are now exposed through MCP and the supported client. Anatomical names are provisional; right-arm physical directions remain unverified.

| ID | Provisional joint | Initial trial | Clearance trial | Live status |
|---|---|---|---|---|
| 1 | Yaw | +80 ticks (~7°), return | +80, center, −80, center | Pending |
| 2 | Shoulder | +32 ticks (~2.8°), return | +80, center, −80, center | Pending |
| 3 | Elbow | +32 ticks (~2.8°), return | +80, center, −80, center | Pending |
| 4 | Wrist flex | +32 ticks (~2.8°), return | +80, center, −80, center | Pending |
| 5 | Wrist roll | +32 ticks (~2.8°), return | +80, center, −80, center | Blocked pending approved software limits |
| 6 | Empty gripper | +32 ticks (~2.8°), return | +80, center, −80, center | Pending |

Each tool executes only one joint's finite sequence, with two-second monitored pauses. All six joints must already be holding, within unchanged saved calibration, with fresh expected positions and no outstanding targets. Target/return settling tolerance is four ticks throughout; left elbow16/wrist6 allowances are not inherited. ID5 null limits are accommodated only as an untouched held-pose ±8-tick monitor window during other-joint tests. ID5 travel is rejected until an operator-approved software range is saved; no full-range approval was inferred.
The shared right-arm controller checks voltage, temperature, fault feedback, target readback and other-joint drift. Verified stationary settling failures retain the existing target and fail the test, avoiding a new settling target; interruption/uncertain or moving failures attempt a current-feedback hold. No automatic torque release or next leg after failure. Stop remains best effort.
Validation: 283 unittest tests passed (including parameterized cases for each right joint, both trial types, strict return accuracy and failure paths). Mock HTTP MCP passed:38tools, right-ID/mode dispatch, malformed inputs, replay rejection, authentication/origin checks, motion gate and stop coordination. No live right-arm travel sent. One operator restart is needed before physical tests, conducted separately one joint at a time. Ordinary arm commissioning flags remain false.

## Right ID5 coarse commissioning tolerance
User authorized six-tick (~0.53°) settling tolerance for right ID5 only. All other right joints remain4ticks; no preflight drift, outstanding-goal, trial-window, power/fault or stationary-timeout classifier bounds changed. ID3 remains unqualified. `test_right_roll` now accepts positive/negative encoder direction, fixed32tick excursion and return, no free target. Last ID5 position1995 is too high for another +32 within approved1928–2008; next proposed trial is −32 and return, nominal1963→1995, with fresh preflight required. 53 targeted tests passed including5/6tick acceptance forID5 only,7tick rejection, positive target rejection and negative movement within actualwindow; 38tool HTTPintegration/auth/replay/bounds checks passed. Live trial pending operator restart and readiness. No live writes sent for this change.

## Right ID3 P comparison tool prepared
`test_right_elbow_p`: one fixed +32tick/return trial at P16 or P24, selected per explicitly supervised call. No automated gain sweep or automatic next comparison. Requires all six joints holding, fresh expected positions, unchanged calibration, right ID3 originalP16/D32/I0/lock1, current goal within4ticks; ordinary right-arm4tick settle criterion stays unchanged. Generic trial power/fault/pose windows and one-ID-only target writes apply.
P21 is protected configuration. P24 application uses ID3 lock55=0, P21=24, lock55=1, each acknowledged/read back. Before any protected write, a mode0600 fsynced recovery record is saved at `.private/right-elbow-p-recovery.json`. Finally restoresP16 and lock1, verifies readback, removes record only upon verification. Catchable interruption and failed movement also enter restoration. Hard process/USB/power loss can prevent restoration; record then blocks new MCP motion, except explicit fixed restore and supported right-arm release. `robot_info` exposes recovery-pending state. Restore action has no target/torque writes and accepts no arbitrary settings. No D/I/EEPROM limit changes. Physical response can change when gain changes/restores.
Comparisons must be performed as separate calls with operator-supported pose reestablished to match the baseline; do not infer a gain effect from different poses or wrist support. Arm last torqueOFF after mechanical inspection; user reported smoothly heavier upward motion, no obvious catch. No live P writes or motion were sent during software development.
Validation:303 unittest tests passed, including17 tuning-specific tests covering original-setting/torque/pose gates, journal-before-unlock, onlyID3 protected writes, successful restoration, lost acknowledgment, interruption, timeout, faults, restore failures/recovery, and pending-record motion blocking.39tool mock HTTP MCP auth, bounds, fixedgain dispatch and replay passed. Operator restarted mid-build; a final restart is required after completed server/controller edits. Next: fresh state, supported in-range hold approval, then oneP16 baseline; inspect result and physical observation beforeP24.
Official register reference: https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation

## Coordinated small upward reach prepared
User changed first practical reach from forward to slow/small upward. Added read-only `preview_right_gripper_up` and fixed-step `move_right_gripper_up`. This is an explicit supervised commissioning action, not generic Cartesian manipulation. Existing arm commissioned flags stayfalse. Fixed model+Z2mm, IDs2/3/4 only, no pan/roll/gripper writes, no automatic return. URDF +Z is a model assumption pending operator observation on mounted robot; numerical displacement is not metric physical proof. No camera/depth/collision avoidance.
Both preview and execution check live model777/mode0, unchanged saved limits, software range, fresh expected pose, power/temp, resolution1 and live homing offsets against `lxh_right_arm.json`. Execution additionally requires all6 torqueON and goal errors<=4ticks. No automatic torque enable. Plan identity hashes expected pose, authoritative right-group config, saved calibration and URDF. Recomputed hash must match preview; live state/offsets rechecked after IK before first write. Installed LeRobot degree normalization uses4095. IK retains pan, roll and gripper; rounded trajectory must stay within URDF/software limits, <=32ticks per active joint total, <=8tick waypoint increments, at most4waypoints, <=1mm sideways and <=3° orientation deviation, model endpoint2mm within0.5mm.
Each waypoint writes atomic current-target/time0/speed30 (~2.6°/s) SRAM packets toIDs2–4 then requires3 stationary samples within4ticks, at most1.5s per waypoint. All6 holding/goal/pose/power readbacks monitored and active-joint diagnostic faults checked. On failure no next waypoint; verified stationary timeout preserves targets, otherwise bounded current-feedback hold on attempted joints only. No automatic torque release, PID, EEPROM, or base/neck writes; no automatic motion retry. Shared MCP lock, persistent request UUID and pendingP-recovery gates apply.
Validation:317 unittest tests passed plus41tool HTTP MCP discovery/auth/origin/bounds/replay/stop checks.14 dedicated reach tests cover coordinated writes, read-only torqueOFF preview versus torqueON execution, homing/calibration drift, stale/out-of-range starts, target increments, untouched-joint motion, timeout/interruption/lostacks/fault/voltage and stationary no-retarget cleanup. Actual installed placo model dry-run at last pose proposes ID2+2,ID3−10,ID4+8ticks in2waypoints; predicted model displacement1.953mm up with0.056mm residual. Native solver notices routed tostderr; stdoutJSON verified. This is offline numerical evidence, not physical qualification. No live reach or homing-offset check sent yet; right arm last observed torqueOFF. Operator restart and supported hold/up-step readiness needed next.

## Startup recovery replaces recurring manual reposition requirement
User rejected having to hand-position the elbow before every practical command. Ordinary movement already leaves holding torqueON; right torqueOFF was from explicitly requested inspection/reset. Added explicit `right_arm_hold(action="recover-hold")` for supervised startup only. It preserves the normal hold gate and all EEPROM/software limits. Starting all6 must be torqueOFF, stationary, fault-free model777/mode0, fresh pose and unchanged calibration, suitable power/temp. OnlyID3 may begin outside saved limits, by at most16ticks (~1.4°); every other joint must be inside.
All joint targets are inside saved ranges. ID3 receives only an inward target16ticks inside the nearer boundary, at speed30, maximum32ticks (~2.8°) total. Target write may enable torque; verify it before explicit enable if needed. Monitor the bounded inward corridor and other-joint state; require3 stationary in-range samples within4ticks in2.5s before enabling remaining joints. Normal hold remains current-position-only. Startup recovery that is already in-range just holds; no software/EEPROM range, P/D/I, mode or offset change. Failure halts later activation, leaves possibly partial holding support and the existing bounded inward goal, reports failure, no automatic retries or torque release. Operator support/cutoff remains necessary for this first unverified startup action. Known right-ID3 weak reverse response may still cause recovery to fail; this is not a promise that mechanical/controller tracking has been fixed. Larger overshoot remains blocked. Routine commands do not release/reposition the arm.
Validation:327 unittest tests passed, including10 dedicated recovery cases and unchanged ordinary hold tests;41tool HTTP integration includes recover-hold dispatch, required-position bounds and replay. No live recovery or up-step sent. A service restart is required for the new action before a supervised recovery/hold, then live calibrated up preview.

## Left-arm slow up command prepared
User switched from right coordinated reach to left arm: move up slow. Fresh left IDs1–6 torqueON/stationary/fault-free, positions1908,1149,2558,2179,2169,2017, all within saved limits. Added explicit `move_left_shoulder_up` commissioning action: fixedID2 +32ticks (~2.8°), speed30 (~2.6°/s), then hold; no automatic return. Shoulder positive-up direction already operator verified. This is a joint-space shoulder lift, not straight-line gripper control. Left gripper Cartesian calibration remains absent/unverified. No right-arm, base, neck or other left-joint writes.
Reuses shared finite-trial core with fixed wrapper selecting ID2, no-return and speed30. Existing right tests retain their default return=true/speed100; no public arbitrary-motion options added. Existing all-six holding, freshness, saved-limit/calibration, power/temp/fault, other-joint drift,4tick endpoint and stationary-timeout/no-twitch checks retained. Catchable interruptions/failures attempt bounded holding of current ID2 feedback; no automatic torque release. No PID/EEPROM or commissioning flag changes.
Validation:334 unittest tests passed plus42tool HTTP MCP authentication, bounds, exact left-script dispatch and replay checks.7 new left-up tests include one-way30speed packet, no return, original loaded-elbow preflight allowance, torque/limit/power/fault gates, interruption/lostack and stationary preserved-target failure. No live left-up movement sent. Operator restart and fresh left supervision/clearance confirmation required before new one-way action. Right arm remains at last holding state; its weak ID3 reverse tracking and attempted-up direction remain unresolved.

## Left gripper Cartesian-up IK path prepared
User clarified gripper up, not shoulder rotation, and pointed to existing `teleop_phone_so101.py`. Reuses its installed `lerobot.model.kinematics.RobotKinematics` FK/IK and SO101 URDF; replaces phone target with fixed +modelZ5mm and guarded MCP execution. Added `preview_left_gripper_up` and `move_left_gripper_up`, separate from shoulder-only action. IDs2/3/4 coordinated, pan/roll/gripper untouched, no automatic return; no PID/torque/EEPROM writes or ordinary commissioning bypass.
Left calibration was missing from LeRobot cache. Read live homing offsets (status now includes address31), ranges, model/mode and resolution1 from all6, checked saved range observations and known anatomical mappings, saved the calibration snapshot in authoritative `robot_config.json` left-group `kinematic_calibration`. Uses installed LeRobot DEGREES midpoint convention with4095 scale. Physical joint-zero/model-to-body geometry and metric Cartesian accuracy remain unverified; this is an approximate supervised model-based reach. Fresh offsets/ranges/resolution must match snapshot before preview/execution.
Shared planner retains <=32ticks/joint total, <=8tick waypoint, <=4waypoints, speed30, model side/rotation/URDF/software bounds, preview hash and after-IK fresh preflight. Left startup holding error<=8 uses existing left-joint preflight bound, accommodating already-known staticID3 error5 without release/re-hold; right startup bound remains4. Endpoint settling remains4ticks. Shared executed FK observer now monitors actual encoder positions each poll: rejects model-derived downward drift>0.5mm, sideways>1.5mm, excessive height; final settled joint endpoints also require model-derived rise within1mm of requested height. This is a model-space guard, not calibrated Cartesian sensing or collision avoidance. Failure stops later waypoints, bounded hold/preserved stationary target cleanup, no automatic return/retry/release.
Actual left offline model result at recent pose: shoulder1211→1209, elbow2558→2534, wrist2179→2205;4smallwaypoints, max26ticks(~2.3°); predicted+Z5.095mm,0.095mm numerical residual. Model FK of previous shoulder-only moves predicts forward/slightly sideways and down, qualitatively matching user's observation; no metric validation inferred.
Validation:344 unittest tests passed, plus44tool HTTP auth/origin/bounds/replay and exact left-IK-script dispatch.26 focused calibration/reach/model-progress tests include stale homing rejection, missing calibration, separate left8/right4 startup policies, model wrong-way/sideways stop and insufficient task-height rejection despite settled joints. No live left IK movement sent. Service restart and operator readiness for a single slow model-estimated5mm gripper lift are needed next. Both arms last observed holding.

## Offline investigation: orientation-coupled lag failure and revised left IK plan
User accepted investigation and preparation of a concrete test, no live motion. Recorded first waypoint lowered wrist while elbow had not followed. Existing teleop/model orientation_weight0.01 produced required joint changes shoulder−2/elbow−24/wrist+26ticks for5mm up, with perfect final pitch. If elbow lags and wrist responds first, sampled first-waypoint FK predicts up to1.596mm downward, matching the physical tiny dip qualitatively. This demonstrates a planning dependence on synchronized response; it does not prove the root cause of elbow delay, hardware fault, or that timing/PID has been fixed.
Compared fixed orientation weights offline at the exact recorded starting pose. Any positive weight tried preserved orientation and gave the same coupled plan. Position-only weight0 gives shoulder−13/elbow−4/wrist+3ticks, two waypoints, predicted4.859mm up/0.141mm residual and1.231deg pitch change. User requested gripper translation, not fixed pitch; existing3deg pitch bound remains enforced. Shared solver now supports fixed weight0 or0.01 internally; left wrapper selects0, right keeps0.01. Left planner checks27 sampled independent lag combinations per waypoint, including4tick prior arrival allowance toward baseline. Rejects model downward>0.5mm or sideways>1.5mm in sampled cases. Old left plan rejected; new plan worst sampled dip0.228mm and stays within same bounds. These are sampled model checks, not physical or continuous-time guarantees; real-time FK observer/fault/pose guards remain active.
Preview hash now additionally binds actual integer targets and waypoints, so an old preview cannot authorize a changed physical plan. No step/joint-window, goal-error, endpoint-tolerance, speed, power, PID, torque or EEPROM limits changed. No new live action, gain experiment or calibration write sent.
Validation:345 controller/connection tests passed (348 discovery cases included3model tests skipped in SDKvenv); all3 model regressions separately passed in unchanged lerobot environment. They verify rejection of the recorded unsafe plan, acceptance/bounds of the position-focused plan and recomputation at the post-failure pose.44tool mockHTTP/auth/bounds/replay tests passed. Next proposed live action after operator restart: one supervised slow model5mm left-gripper up step with small allowed pitch change, then hold.

### Left-gripper upward distance parameter

`preview_left_gripper_up` and `move_left_gripper_up` accept `distance_mm` (2–10 mm, default5). Pass the same distance and fresh six-joint positions to preview and execute; the preview hash binds the distance and actual plan. Example: `{"expected_positions": {"1": ..., "2": ..., "3": ..., "4": ..., "5": ..., "6": ...}, "distance_mm": 10}` for preview, then add its `plan_hash` and a new `request_id` for the explicitly supervised move.

Distance is model +Z, not a measured physical lift. The left planner uses a shoulder-leading first stage (at most16ticks), then follows with elbow/wrist stages. All targets remain within32ticks of the measured starting pose. Intermediate advancement requires positive encoder-model lift, stationary feedback and at most8ticks of lag within the sampled path envelope. Its minimum lift is computed over that same integer lag envelope (minimum0.2mm), rather than assuming the elbow has followed its target; final settling remains4ticks and modeled height must be within1mm of the request. The hash binds stored starting goals as well as the pose, so an old unchanged target cannot silently become a new trial. Too-small requests that cannot advance the shoulder reject before writing. Pitch, sampled lag, actual-progress and interruption guards remain active. These stages are a new commissioning workflow and require supervised physical validation. No automatic return or repeat.

### Bounded elbow amplitude comparison

`test_left_elbow` accepts `amplitude_deg` (2–7.03125, default7.03125/80ticks), `slow` (defaultfalse; true selects30ticks/s), and existing positive/negative direction. A5degree slow trial uses57ticks, a monitored2second pause, and return if outward settling succeeds. Existing outward4tick and return16tick tolerances remain unchanged. All six joints must already hold, the empty arm must be supported, and clearance is operator verified. OnlyID3 receives target writes. No PID, torque-enable or EEPROM changes. Failures stop the sequence; no automatic retries.

### Explicit coarse left-gripper lift commissioning

Both left-gripper preview/move tools now accept `coarse: true` with `distance_mm`10–20 (defaultfalse retains strict2–10mm). This is a separately bounded supervised trial mode, not precise placement or a commissioned general arm API. OnlyIDs2/3/4 move, at30ticks/s, up to64ticks total per joint (~5.6degrees),16ticks per stage and6stages; pitch/orientation change is limited to6degrees. Existing saved joint limits, model down/side corridor0.5/1.5mm, fresh calibration/goal identity, torque-on/power/fault/other-joint and stop checks remain active. No automatic retries, return, PID, torque-enable or EEPROM changes.

Tracking allowances are shoulder8ticks, elbow16ticks and wrist4ticks. Intermediate/final positions must lie between measured baseline and stage target; plans sample those integer arrival lags and independent response delays. Planning rejects a coarse request if its modeled worst-case endpoint under the allowed lag is below half the nominal requested lift. Execution also requires actual encoder-FK upward progress at each stage and at least half the requested lift at completion. For20mm nominal this means at least10mm model lift; it NEVER means exactly20mm physical displacement. Results identify `completion_kind: coarse_model_lift`, requested nominal lift, minimum accepted model lift and actual encoder-model displacement. The preview contains the sampled final model lift interval; it is not a statistical confidence interval or calibrated real-world accuracy guarantee.

At the recorded post-elbow-test pose,20mm nominal predicts10.4–20.1mm model rise with~5.1degree pitch change, while15mm fails the uncertainty check. Fresh state must be re-read/replanned; this historical preview must never authorize motion. All six joints must already hold; coarse startup allows elbow residual16ticks, others8. Empty arm supported without pushing, links/cables clear and physical cutoff available. Larger range/tilt requires an explicitly approved watched physical trial before routine use.

#### Coarse continuous tracking (v2)

Coarse lift v2 advances an intermediate target as soon as one fresh, fault-free sample meets the existing joint-lag envelope, baseline-to-target interval and modeled upward-progress floor. It does not wait for the joints to become stationary between stages. This removes the deliberate stagewise stops while preserving target step/speed/total limits and the precomputed independent-response checks. The final stage still requires3stationary readings, fixed joint error bounds and the minimum accepted lift. Strict mode continues to use settled waypoints. The execution-policy version is hash-bound; old coarse previews cannot execute. Physical smoothness remains unverified until a watched trial.

#### LeRobot FK alternatives and planning margin

Coarse lift target selection now explores a bounded neighborhood of the existing LeRobot IK result, using LeRobot FK for every alternative. It holds yaw/roll/gripper fixed, samples shoulder choices within64ticks, solves elbow/wrist Cartesian residuals offline, rounds to integer targets and ranks a maximum12 endpoints under the fixed8/16/4tick tracking-lag envelope. At most4 candidates receive the full stage/path check. A candidate must predict at least1mm more model lift than the existing half-request completion floor; a20mm request still requires10mm actual encoder-model lift at runtime, but planning requires at least11mm under allowed lag. Saved limits,0.5/1.5mm down/side bounds,6degree tilt,16tick stages,64tick total, finite request/stop and continuous coarse execution remain unchanged. A different IK allocation does not establish physical accuracy or guarantee smoothness.

The2026-10-23 blocked pose originally predicted9.818mm minimum for20mm nominal. An alternative within the same limits predicts12.412–19.690mm model lift, using33/9/26ticks of shoulder/elbow/wrist motion and~5.98degree tilt. No larger commanded distance or changed servo gains/calibration is needed. It remains a watched commissioning trial until physically verified.

LeRobot reuse already includes RobotKinematics FK/IK and its calibration angle convention. The phone script also demonstrates continuous observation/action processing, EE step limiting, and grouped bus writes. Its follower connect/configure path disables torque and writes motor configuration; it is not invoked by this MCP controller. Porting grouped writes or a faster loop is separate from this target-selection change and has not been claimed complete.

### Explicit left startup recovery (no limit bypass)

`left_arm_hold(action="recover-hold", expected_positions=<six fresh raw positions>, request_id=<new UUID>)` handles the measured slight power-up elbow offset. All six must initially be stationary, fault-free, torque-off, position mode, with verified saved limits and suitable power/temperature. OnlyID3 may start outside its saved range, at most32ticks (~2.8degrees). It receives an inward target32ticks inside the range at30ticks/s, up to64ticks (~5.6degrees) travel. All other joints must already be inside their limits. Normal hold/release and right-arm recovery limits remain unchanged.

Activation is staged: holdIDs1/2, recoverID3, then enableIDs4/5/6 only after verified stable inward recovery. Elbow must be at least8ticks inside, stationary for3samples and within24ticks of the inward target. If that residual exceeds8ticks, the explicitly planned holding handoff may write the verified in-range current position once, then verifies stable holding within8ticks and at least8ticks of inner clearance. No target outside saved limits is written. Saved limits, offsets, PID/mode configuration and EEPROM are never changed. Persistent handoff drift/faults stop further activation.

This is a new supervised startup action, not a force/ignore-limits option or autonomous recovery. The operator supports the entire empty arm, keeps fingers/cables clear and has cutoff available. Failure/interruption can leave partial holding torque and the existing inward/current-hold target; no automatic release or retry. Software interruption cannot guarantee an immediate stationary hold while feedback is outside saved range; use physical cutoff with full arm support for unexpected movement. After commissioning, an explicitly requested arm preparation can use this bounded routine instead of exact manual pose matching. Larger offsets, other joints outside limits or faults still require intervention.

#### Pose-aware coarse sequencing from recovered rest

Coarse mode now selects the first joint from FK at the measured pose; it no longer assumes shoulder-first raises the gripper. It can lead with shoulder/elbow/wrist, establish a positive modeled buffer in at most32ticks (16tick steps), then coordinate the remaining joints. Nominal path, independent lag,3D corridor, final lift and stationary-state requirements remain unchanged. Intermediate measured model progress must remain positive even if the lead joint's permitted lag includes zero motion; a stalled first stage still stops rather than skipping progress.

Candidate search checks up to128 endpoint arrival envelopes, retains at most12, and fully checks at most4 paths. It filters both lower lift margin and possible upward overshoot from lag; endpoints whose lag can exceed nominal+1.5mm reject. Preview reports the min/max arrival-lag interval, including cases above the nominal target. Candidates with a positively predicted initial motion despite allowed lag are preferred when available.

At the recorded recovered pose856/2789/2191, the old shoulder-first start predicted~3mm DOWN and was correctly blocked. An elbow-led20mm plan predicts~12.9–21.3mm arrival model lift within existing limits. No manual matching to a middle pose is part of this planner. This is offline evidence only; recovered-start physical smoothness/lift remains unverified until a watched trial.


## ID3 gravity-sag bias trial — October24

`test_left_elbow_bias(expected_positions, request_id)` tests whether the left elbow's repeatable stationary shortfall (12–14 ticks after the coarse lift and the −80 elbow test) is gravity sag that a goal offset can cancel. It reads the existing ID3 goal and stationary residual, requires 5–16 ticks, and commands goal minus residual at speed30 (≤32 ticks travel, inside saved limits). It waits for three stationary samples (3 s timeout) and reports `reached_desired` (within 4 ticks of the original goal) plus the final errors. A miss is reported, not corrected or retried. The biased goal stays in place, so other tools' goal-vs-position preflights still see the offset. On any fault it requests a hold at current feedback. Same preflight as `test_left_elbow`; no other joint writes; no PID/EEPROM/torque changes. 7 unit tests and the mock MCP integration pass. Live verification needs operator restart and explicit supervision.


## Left lift elbow sag bias — October29

`preview_left_gripper_up` and `move_left_gripper_up` accept `elbow_bias_ticks` (integer 0–16, coarse mode only, default 0 = previous behavior). With a bias, the elbow is planned from its measured position, and each elbow command is sent that many ticks ahead of its waypoint in the elbow's travel direction, cancelling gravity sag (measured ~13–14 ticks near elbow 2670–2790 on October24). Arrival is still judged against the unbiased waypoint. The biased elbow may land up to 4 ticks past a waypoint; the final stage also requires sampled elbow position to stop changing (≤1 tick between samples), since a biased elbow can coast. The live upward-corridor and reach-window guards are unchanged and still abort with a hold. Preview reports `measured_elbow_sag_ticks` and the signed `elbow_bias_ticks`; the bias is bound into the plan hash, so preview and move must use the same value. Biased commands must stay inside saved limits and the first elbow command within 32 ticks of the stored goal. Planner lag tolerances (shoulder 8 / elbow 16 / wrist 4) are unchanged, so model lift intervals stay conservative. Right-arm execution rejects any bias.

Offline real-model check: from the October24 pose (elbow 2791) a 20 mm coarse plan is accepted with or without bias (targets 841/2755/2191, 11.1–20.3 mm). From the October24 end pose (elbow 2672) no 10–20 mm coarse plan is accepted, with or without bias; reposition the arm first. The `test_ik_planning_model` case `test_coarse20mm_predicts_useful_lift_with_measured_tracking_uncertainty` already fails on main before this change.

October29 live result: from elbow 2781 a 20 mm coarse lift with `elbow_bias_ticks=9` ended with the elbow 3 ticks from target (≈12 without bias) and 17.9 mm model lift (≈14 mm on October24 without bias); operator saw a noticeable rise. It was marked failed only because wrist ID4 ended 5 ticks short against its 4-tick final allowance.

`wrist_bias_ticks` (integer 0–8, coarse only, default 0) applies the same command offset to wrist ID4, with the same 4-tick overshoot allowance and sampled-stillness check. Wrist sag appears only after load changes (0 at rest before that lift, 5 after), so choose it from an observed final wrist miss. Elbow-only previews keep the same plan hash as before.

`client.py` now unwraps MCP exception groups: a failed tool call reports the server's message, and a failed controller run reports its output as `tool_result`, instead of "unhandled errors in a TaskGroup".

October29 second live result: from elbow 2742 / wrist 2189, a 20 mm coarse lift with `elbow_bias_ticks=12` and `wrist_bias_ticks=5` completed (request 6ae7ff2b): 19.1 mm model lift, 0.0 mm sideways, final residuals shoulder 2 / elbow 3 / wrist 2 ticks. Operator confirmed a clean rise. Sag against the biased commands was elbow 15 and wrist 7 ticks, slightly above the biases used. First completed smooth coordinated left lift; loaded power and repeatability remain unqualified.

Repeatability run 2 (October29, after supported release, hand lowering and recover-hold 2816→2784): start shoulder 850 / elbow 2787 / wrist 2146, same biases (12 / 5). Completed (request ba825a18): 20.3 mm model lift, 0.1 mm sideways, final residuals shoulder 2 / elbow 2 / wrist −1 ticks. Operator confirmed a clean rise. At-rest preview sag was elbow 3 / wrist 1, but sag under load was about 12 / 4, matching the biases; size biases from loaded behavior, not the at-rest reading.

Round 3 (October29): after recover-hold 2815→2784, the wrist stayed at 2118 and a 20 mm plan was rejected. A 15 mm plan existed but moved the elbow only +1 tick; because bias sign follows planned travel, bias 12 would have commanded the elbow 12 ticks down (2798, near its 2804 limit). Guard added: a bias is refused when that joint's planned travel is smaller than the bias. Not executed.

Repeatability run 3 (October29): start shoulder 845 / elbow 2787 / wrist 2146, biases 12 / 5. Completed (request f1891763): 21.3 mm model lift, 0.3 mm sideways, final residuals shoulder 3 / elbow 0 / wrist 0. Summary of the three compensated 20 mm lifts: 19.1, 20.3, 21.3 mm, worst final error 3 ticks, all completed. 21.3 mm is near the 21.5 mm corridor ceiling; if a run aborts high, reduce elbow bias to about 10.

Start-pose window (offline sweep, real model, elbow 2787, biases 12/5): 20 mm plans for shoulder 840–870 with wrist 2140–2160 (also 840/2180 and 860–870/2120). Target shoulder ~850 / wrist ~2150; the wrist window is only about ±10 ticks, so set the pose by hand with fresh joint readings rather than by eye. Some higher-wrist poses fail only the bias-travel guard and may plan with wrist_bias_ticks=0.


## Left lift start pose — November2

`preview_left_start_pose(expected_positions)` and `move_left_start_pose(expected_positions, plan_hash, request_id)` bring the supported, holding left arm to the fixed pose shoulder 850 / elbow 2787 / wrist 2150, so lifts no longer need hand positioning. There is no target parameter. IDs 2–4 only, speed 30, at most 128 ticks per joint and 16 per stage; farther poses are refused (use supported release). IDs 1, 5 and 6 receive no writes and may not drift more than 8 ticks. After settling (sampled stillness and moving flag), one feedback correction per joint cancels sag, keeping goal offsets inside the lift preflight bound (shoulder/wrist 8, elbow 16); final error above 8 ticks is reported as failed, never retried. All joints must already be holding. Preview reports modeled gripper height change (negative = down, about −24 mm after a lift); modeled travel above 70 mm or side travel above 20 mm is refused, and a live check stops the move 10 mm beyond that. Failure holds at feedback, or preserves targets when verified stationary. No PID/EEPROM/limit changes.

16 unit tests and the mock MCP integration pass. Offline real-model check: a 20 mm coarse lift (biases 12/5) plans from the target and from tested landings shoulder 846–854, elbow 2783–2795, wrist 2146–2154. Live runs, November2 (operator confirmed smooth motion): from shoulder 845 / elbow 2786 / wrist 2112, completed with final errors −1 / +2 / −1, no trim, modeled −11.9 mm (request f53efd51). A 20 mm lift (biases 12/5) from there then completed at 19.6 mm, 0.0 mm sideways, residuals 1 / 2 / 1 (request 22b846d8). Return from the lifted pose completed with final errors +1 / +4 / 0, modeled −20.6 mm (request 89c4e472). Elbow landed at 2791 and the 4-tick trim to goal 2783 did not move it (static hold within tolerance), so the trim can leave a small residual.


## Neck hold — November2

`neck_hold(expected_positions, request_id)` enables holding on neck IDs 7 and 8 at their current pose, tilt first, by preloading each goal to current feedback before torque. It commands no travel. Item 6 found the neck torque-off with no supported way to enable it: `nudge_joint` needs torque on, `pan_torque` needs tilt holding, `test_neck_pan` moves pan and leaves tilt off, and `test_neck_joint` needs an unexpired commissioning session. Any joint failing the checks (commissioned, stationary, inside operating limits, within 8 ticks of expected, power, no outstanding target) blocks all writes. No limits or commissioning state changed. Live verification pending.


## Right elbow baseline and P trial — November2

P16 live (`test_right_elbow`, hands off, shoulder about 1106): self-supported sag 7 ticks at load 40. Out leg 2996 to 3031 (target 3028, 3 ticks). Return stalled at 3028 for the full 3 s, load 140, 32 ticks short of 2996, so the lift back failed; request d5b70f83. Sag-versus-tolerance does not explain this, and a bias up to 16 ticks would command less than that 32-tick error. Not yet known: whether higher P, pose or mechanics is responsible.

P24 first attempts: one refused by the 4-tick goal check with no writes (9ab4cd5d). One ran (4768bcf4): P24 applied and P16/lock1 restored and verified live, but the out leg stopped 6 ticks short with reversed loads on IDs 1, 3 and 4, consistent with the operator's supporting hand, so it says nothing about the return. The gain-change goal check is now 8 ticks (operator-approved), matching the general preflight, so the trial can start hands-off. The post-update 4-tick pose check, fixed P16/P24 choice, recovery record and acceptance tolerances are unchanged.

After the operator tightened the loose right shoulder (ID2) screws, P16 repeated (defba612): out 3032 (target 3030), return did not move from 3032 toward 2998, load 148; the stall was not caused by the shoulder. Hands-off P24 (23fb579c, after one attempt stopped by a single power/temperature check before any travel, 402ffdcc): out 3031 (target 3028), return moved to 3020 and stalled, 24 ticks short of 2996, load 154. P16/lock1 restored and verified each time.

Upward-stall pattern: P×error is about constant at 512–576 with load 140–154 (P16×32–34, P24×24). The elbow appears to lift only while load is above about 150, against about 40 to hold sagging downward. Prediction, untested: P32 stalls about 17–18 ticks short, close to the left elbow's 12–14. Two values of P, so a pattern rather than a qualification.

`test_right_elbow_p` now also accepts fixed P32 (operator-approved), same recovery record and restore; other values rejected. P32 hands-off (6a0cc215): out 3030 (target 3027), return moved to 3011 and stalled 16 ticks short of 2995, load 136; P16/lock1 restored and verified. Confirms the prediction of 17–18 ticks; P×error 512–576 across P16, P24 and P32. Static sag at P16 was 6–12 ticks (load 36–60), varying with settling and time.

Operator approved permanent right elbow P32. `set_right_elbow_p(p, request_id)` writes fixed P16 or P32 with ID3 torque off, relocks and verifies; no motion, and it can undo to 16. Once P32 is saved, `test_right_elbow_p` refuses (it requires original P16). LeRobot's `so_follower.py` writes `P_Coefficient` when configuring an arm, and `teleop_phone_so101.py` connects through it, so teleop may put P back; read `position_p_raw` before relying on it. Not yet applied live.

## Right start pose — November2

Right elbow P32 applied live and read back (ac62a260); hands-off sag fell to 3 ticks at load 32 (elbow 2973, goal 2970), from 6–12 at P16. `preview_right_start_pose` and `move_right_start_pose` share the left tool's controller with a settings block: fixed pose shoulder 1105 / elbow 2990 / wrist 2230, the hand-set pose the elbow trials ran from, not yet shown to plan a lift. Elbow tracking allowance is 24 ticks because the elbow stalls about 16 short while lifting and the one correction (bounded to 16 ahead) closes that. A stall above 24 stops the move. Shoulder and wrist remain P16, and they have not been tested lifting. Not yet run live; needs restart and explicit supervision.

Live right start pose (encoder results; operator confirmation pending): downward 2973 to 2993, final errors +3 / +3 / +2 (087845e2). After a P32 `test_right_elbow` lifted 3029 to 3009, 16 short, and stopped on the power/temperature check with no value reported (c848b9be), a lift 3013 to 2991 completed, final errors +3 / +1 / +2, modeled 9.6 mm; the elbow stalled about 3005–3013, then the correction to goal 2975 carried it (1e70221b). Lowest logged voltage 119. Those checks now report joint and values. Two trips of that check on the right bus remain unexplained; shoulder and wrist were barely exercised.

## Item 6 complete — November10

Rejection prompts: pickup request refused, "go over there" sent back for direction/distance/clearance, "move the right arm up" declined (right arm not qualified); no motion in any. Scripted left-arm stop (48a9899a): stop_active sent 8.4 s after a 20 mm lift started (1.5 s after measured preview planning time), landing mid-motion (wrist 2150 to 2130, model 4.9 mm). Lift returned interrupted 0.2 s later; holds acknowledged at shoulder 849 / elbow 2787 / wrist 2130; all six holding and stationary, elbow 3 ticks off. A typed "stop" in chat arrives too late for these short moves; the physical cutoff remains the emergency stop.

Base, same day: an on-floor 0.5 s backward pulse (f6be43c3) drove only the right wheels, then two stop_base attempts did not reach wheels 9 and 10 (comm -6); the operator cut power. Idle bus 2,000 reads with no failures; lifted pulses (36418d5d, 47f14aab) and a 0.3 s floor pulse (d271ba7e) were all normal. Wheel 10 sometimes returns implausible values (41 °C dropping to 32 °C). Battery was about 10%. Leading hypothesis: supply sag under floor load, possibly with a marginal wheel-10 connection.

## Left reach to a point — November16

`preview_left_reach(expected_positions, target_m)` and `move_left_reach(expected_positions, target_m, plan_hash, request_id)` move the left gripper frame to `target_m = [x, y, z]` meters in the left arm base frame (SO101 model: x forward, y left, z up). Position-only IK on IDs 1–4 (yaw, shoulder, elbow, wrist flex) with the left kinematic calibration; wrist roll and gripper are held. A small orientation weight left targets 5–17 mm short because a 5-joint arm cannot keep its attitude while translating, so orientation is free. Refused when: target outside x 0.08–0.45, y −0.30–0.30, z −0.10–0.45; IK misses by more than 5 mm; any joint would travel more than 400 ticks; any target is outside saved limits; or the modeled path sags more than 2 cm below its lower end. Execution reuses the start-pose controller (`left_start_pose.execute`) with IDs 1–4, speed 30, 16-tick stages, a live modeled-gripper envelope of the straight start-to-target box plus 3 cm, sampled stillness, one bounded end correction, final within 8 ticks, and hold on failure without retry. The start-pose controller now takes per-arm travel, envelope and packet settings; left and right start poses keep their old values.

`../reach_point.py` (read-only) turns a raw left-half pixel into a target: fisheye-rectified SGBM (256-px search, about 14 cm minimum depth; matches at the search limit are refused), camera-to-arm calibration v1, a hover offset above the point (default 5 cm), and optionally `--preview`. It refuses if the neck has moved more than 6 ticks from the calibration pose. Validation on the 8 hand-eye images: the marker centre's stereo position agreed with its ArUco position within a median 2.9 mm (1–4 mm at 17–27 cm, 8–16 mm at 38–46 cm). With the earlier 128-px search, close points either failed or matched wrongly (121 mm error).

6 reach tests (real SO101 model), the start-pose suites and the mock MCP integration pass. Not yet run live; needs a server restart and explicit supervision. Positions inherit about 1 cm uncertainty from the arm model and calibration, so targets keep a hover offset.

First live reach (November16, request 28ea6db3): target from `reach_point.py` on a soda can (pixel 986,935 in the left half; stereo depth 0.444 m with 100% valid disparity nearby; arm-base point 0.313, −0.171, −0.014 m), hover 15 cm above the label point (planned about 9–10 cm above the can top, can height estimated). 21 stages, gripper sweep 14.4 cm sideways; completed with final errors of 4 ticks or less, voltage 11.9–12.0 V. Operator: gripper mostly centred over the can, about 8 cm above its top, so the height was about 1–2 cm lower than planned, consistent with the ~1 cm chain estimate. Earlier attempts were correctly refused for shoulder travel above 400 ticks when the can sat 36–44 cm out, at the edge of reach.

Lower approach (request 1ede1ac2): 4.8 cm straight down to z 0.086 m, 7 stages, final errors of 3 ticks or less. Target was 30 mm above the can top using the measured gap; operator measured about 35 mm, centred, so about 5 mm repeatability near this spot.

## Left gripper open / close — November16

`left_gripper_open(expected_positions, target, request_id)` and `left_gripper_close(expected_positions, request_id, squeeze=12)` (`controllers/left_gripper_grip.py`) write only ID6 at speed 30, with all six joints already holding. Positive ticks open. Open: one command to a raw target above the current position, at most 700 ticks away and within saved limits; if the jaw stops short it is reported as blocked and held where it stopped. Close: 16-tick steps until the jaw stalls (stops short of the step) or present load (reg 60, masked to magnitude) reaches 300; then it holds `squeeze` ticks (0–24) past the contact point, so grip force is bounded by P × that offset. With nothing in the jaws it stops 8 ticks before the closed limit and reports no contact. Every sample checks voltage, temperature, mode and torque on all six, ID6 goal and limits, and that no other joint moves more than 8 ticks or has its goal changed. On any failure it holds ID6 at its present position. 8 simulated tests and the mock MCP integration pass; not yet run live. First live open (request 866ba018, empty, +250 target): opened smoothly at about 50 ticks/s with free-motion load 28–52, but the fixed 3 s settle timeout ended it at +148 and cleanup held ID6 at 1912. The settle timeout now scales with distance (2 s + 1.25 × distance / 50 ticks/s); a 700-tick simulated open at the measured rate passes. Load threshold 300 has a wide margin over free motion.

First live empty close (request 483d8cb0): closed from 2061 to the floor goal 1461 (8 ticks above the 1453 limit), ending at 1463 with load 24 and no contact reported; the operator saw the jaws stop where they meet, so the floor sits at the closed point. It took about 105 s for 600 ticks, longer than the client's 65 s HTTP timeout, so the reply was lost while the server finished normally. The client timeout is now 135 s, above the longest tool timeout. Gap at 2061 measured about 70 mm.

Centering on the can (November16): operator saw the can 20 mm off-centre toward the jaw away from the laptop. A lift–shift–lower (request 6175a4cc) stopped at stage 1: the shoulder (ID2) stayed 14 ticks behind a 16-tick upward step in this extended, lowered pose (gravity), so the controller held without a next waypoint. ID2's goal was then set to its present position with a one-off write (goal 2163, settled 2167), and a direct 2.2 cm shift away from the laptop at the same height completed (request de3c0ff0, final errors of 5 ticks or less; the near jaw moves away from the can and the far jaw ends about 11 mm from it). Holding afterwards, the shoulder sits about 10 ticks below its goal, which the shared 8-tick preflight refused. Reach and gripper tools now allow the shoulder the elbow's 16-tick holding sag, stage tolerance and correction limit (`goal_tolerance` per arm in `left_elbow_test.validate`; other tools keep 8). Lifting under load still needs a shoulder gravity bias before carrying anything.

5 mm centering shift (request 55c7adb6, 2 stages, final errors of 4 ticks or less) left the yaw 10 ticks from its trimmed goal, and the next preview was refused. Design fix: a completed move may leave goal = target ± max_goal_offset with position within FINAL_TOLERANCE (8) of target, so the reach and gripper preflight now accepts exactly that holding offset: yaw and wrist 16, shoulder and elbow 24 (`left_reach.GOAL_TOLERANCE`). Joints must still be stationary with torque on and within limits; wrist roll and gripper keep 8, and all other tools keep their limits. Tested at each boundary.

Second grasp attempt (November16): the first close (request ad3c8b39) stopped on a single corrupt ID1 temperature sample (77 C; 31 C on re-read); the gripper had closed to 1769 (~35 mm) with no contact load because the jaws were about 2.8 cm behind the can's centre and the fixed (right) jaw slid around it. The gripper tool now re-reads a suspicious voltage/temperature before stopping (another ID1 sample read 63/32 on the next open) and keeps the first and latest 80 trace samples. The operator also saw the jaws tilted; the model gave −25° (movable jaw lower). `level_jaws` (preview/move_left_reach) picks the wrist roll nearest the current one that makes the jaw line horizontal (within 2°), re-solves IDs1–4 with it, and moves ID5 in the same staged move (roll then holds up to 16 ticks like yaw and wrist). Here: roll 3542 → 3186. A 2 cm back-off (request 41c662ef) stopped at stage 2 with the shoulder 20 ticks behind (P32 everywhere, load only 5–12%): a joint holding the extended arm uses most of a 16-tick step just to hold. Reach moves now give lagging joints a bounded lead: after 0.6 s outside tolerance, the goal moves 8 ticks per 0.3 s further toward the waypoint, up to 32 (shoulder) or 16 (elbow); 4 s stage limit; any lead beyond the end-correction bound is withdrawn before the final settle. Start-pose tools are unchanged. Simulated with a shoulder that only lifts once its goal is 18 ticks ahead: fails without the lead, completes with it.
Back-off retry (request f97eabdd): the shoulder now kept up (lifted 34 ticks, 12 behind), but the yaw stopped 11 behind at stage 3 (friction with the arm extended; it had also finished 10 off on an earlier shift). The lead now covers every reach joint: yaw, wrist and roll 16 ticks, shoulder 32, elbow 16. Failure results include the leads tried.
Back-off third try (request cdc512a4): leads worked (4 used, max 8) but the move stopped on an ID1 temperature sample of 110 C. At rest, 200 quick reads gave 29–30 C, 120–121 voltage, no comm errors and a clear status register, so the bad samples (77, 63, 110) appear only during motion. The staged controller (reach and start-pose tools) now confirms an out-of-range voltage/temperature with a fresh read before stopping and reports `glitches`; a sustained bad value still stops the move. Tested both ways.
Shift retry (request 9a63b30c): yaw stopped 9 behind at stage 2 with its 16-tick lead used, nothing touching (operator); yaw lead cap raised to 32.

Grasp attempt end of day (November16): after the yaw lead (32), a 15 mm right shift with 1.4 cm up completed its stages (request 20677d62; ID1 temperature glitch 110 → 33 re-read and continued; shoulder ended 9 ticks low, a reported miss by 1 tick), then forward 4.5 cm (dc192bbb), down (b25cf507, 7 mm modeled), tip 13 mm lower at the same angle (5f54a61c) and 5 mm more forward (cc0de16d), all within 6 ticks with jaws level (−0.1°). The close (request e851a78b) ran to the floor (1463, load 24) with no contact: the can slipped out again. The client got an empty error, most likely the server's 120 s tool timeout, since a full close (~700 ticks) takes about that long.

### Open items for the next session
- Grasp geometry: twice now the jaws closed and slid around the can. The pinch point is probably too far back relative to the can's centre (and/or at the jaw tips); the operator suggests going further forward so the object sits deeper between the jaws. Try an easier object first (box-shaped, light, textured).
- Gripper close speed: 16-tick steps take ~105–120 s for a full close, at the server's 120 s limit. Use larger steps while the load stays at free-motion levels (24–52), or start the close from just outside the object.
- Shoulder end correction: capped at 16 ticks but it needs about 22 to hold the extended arm, so moves end 9–10 ticks low and report a miss. Raise the shoulder `max_goal_offset` for reach (and `GOAL_TOLERANCE` to match).
- Optional wrist-only step in the reach tool, for a steeper gripper angle (position-only IK keeps the angle).
- `nudge_joint` refuses the left arm ("pending port verification and safe joint commissioning"); deliberately not bypassed.
- Lifting with a load: not attempted yet; bounded leads now make an empty arm lift.


## Top-down reach mode November23

`preview_left_reach` / `move_left_reach` take `top_down=true`, optional `jaw_yaw_deg` and `approach_tilt_deg` (0–90, default 0; 90 = pointing horizontally outward). Instead of position-only IK on IDs1–4, a bounded least-squares solve over IDs1–5 (scipy, inside the saved safe limits, seeded from the current pose and ±180° of roll, least-travel valid solution kept) puts the gripper frame at the target within 5 mm with:
- the approach (gripper frame z, out past the fingertips) within 3° of straight down, or leaned `approach_tilt_deg` away from the arm base along the target's bearing (the only lean a 5-joint arm can make);
- if given, the jaw-closing line (gripper frame x) within 3° of `jaw_yaw_deg` in the arm frame (about z); either jaw order counts, so 62 and 242 are the same request and the same plan hash.

The staged controller then moves IDs1–5 as for `level_jaws` (which cannot be combined with `top_down`). Checks added: any joint needing over 400 ticks is refused with "step there through a larger approach_tilt_deg first"; when the move starts already pointing, every stage must stay within 10° of the worse end (a descent stays pointing). The plan reports `pointing` (requested tilt, approach and jaw-line errors, path maximum) and its hash binds top_down, tilt and the jaw line. LeRobot's placo IK was not usable for this: with an orientation weight it landed 2–30 cm off and ignored joint limits.

Model limits on the November23 standing box (Contact-GraspNet grasp at (0.286, −0.127, 0.023), jaw line 62°, ~0.31 m from the base): straight down is refused 3 cm above it (wrist flex would pass 3147); 5–25° lean works at both hover and grasp height, 15° keeps the wrist well inside its range. From the November16 approach pose (2302/2022/2088/1940/3542) the 15° hover needs ~900 ticks of wrist flex, so step 45° → 30° → 15° (each ≤400 ticks; tested). Then descend at the same tilt and jaw line.
From the rest pose after recover-hold (2186/1050/2789/1820/3188, gripper pointing ~horizontally forward, 97° from down) even 45° needs 863 shoulder ticks, so the cap is 90° and the tested chain is (0.25, −0.10, 0.14) at 85° → (0.26, −0.11, 0.12) at 65° → (0.27, −0.115, 0.10) at 45° → (0.28, −0.122, 0.075) at 30° → hover (0.286, −0.127, 0.053) at 15°, jaw line 62°, largest step 356 ticks.

Grasp workflow (perception in the separate `contact_graspnet` conda env, scripts kept in the session scratchpad): snapshot → SlimSAM mask from one or two clicks → Contact-GraspNet on rectified SGBM depth → arm-frame grasps filtered for top-down/in-plane lean and width → top-down reach to hover, descend, `left_gripper_close`. Contact-GraspNet's grasp centre is for the Panda gripper's pads (10.3 cm from its base); here the target is the SO101 gripper frame point near the fingertips, so aim it about 1–2 cm below the box top.


## Direct small grip adjustment November17

`left_gripper_tighten(expected_positions, request_id, ticks=2)` is a separate supervised contact-hold adjustment, not another close-to-contact search. Operator must first visually confirm both gripping faces touch the object. It commands ID6 once to current feedback minus1–4ticks (default2, about0.18servo degrees). It never performs the old16tick discovery step. This is an angular target offset, not calibrated millimeters or force, and does not guarantee against crushing or establish a secure grip.

All six joints must already hold stationary with fresh expected positions, saved limits, normal power/temperature and unchanged other-joint goals. Gripper existing goal error must be at most2ticks and the new target must actually be farther closed than that goal. Target stays8ticks above the closed limit. Refuses initial raw load>=150 and stops/holds feedback if monitored load reaches150, gripper leaves target−2 through start+2, another joint drifts>8ticks, power/state checks fail or execution is interrupted. Requires three stationary samples within2seconds. No gain, torque-mode or EEPROM changes; no automatic retry, release, lift or additional squeeze. Repeating with a new position can accumulate pressure; each new adjustment requires separate explicit scope and visual inspection.

November17 physical checkpoint: an empty35mm box was approached and lowered around; operator confirmed both faces touching, centered and unbent after zero-extra-squeeze close. Contact was a5tick stall at1739 with load48; handoff goal1739 reduced holding load to0, so grip security remains unverified. No tightening or loaded lift has been sent.


## Reach support offset correction November17

The first5mm loaded box test failed (request86ab013e): shoulder target2405, position2414, goal2390; the9tick miss exceeds8. Model net rise was0.02mm, with a2.93mm transient dip. Operator confirmed box stayed resting, centered and unbent; no basket transfer/release/retry was sent. Normal voltage and temperature do not prove loaded power qualification.

`left_reach_v2` retains each active joint's measured initial goal-minus-position offset through its staged targets, capped by the existing end-correction profile. Shoulder end-offset cap is now32ticks (was16); yaw/wrist/roll8 and elbow16 remain unchanged. Goal-error preflight for reach and gripper tools matches endpoint offset plus8ticks: shoulder40, elbow24, yaw/wrist/roll16. Other commissioning tools keep their original checks. If a small upward move starts with support offset beyond its cap, preview rejects before writes rather than dropping that support. Start-pose tools do not preserve offsets and retain their old correction limits.

Near-vertical upward reaches of4–21mm (lateral<=2mm) additionally require final encoder-model rise of at least half the modeled requested rise, alongside the unchanged8tick per-joint completion criterion. Below-floor progress reports failure without automatic retry/return/release; no tolerance widening to call zero rise successful. Preview hash includes offsets, correction bounds and final progress floor; policy is versionedv2. One bounded end trim,16tick waypoints,400tick per-request travel, speed30, saved position limits and runtime power/fault/position envelopes remain. No gripper writes during reach, no PID/EEPROM/calibration changes. This does not prove object retention or collision-free transfer; operator visual lift/placement checks remain necessary.

### First pick-and-carry November23
Standing juice box (~5.5 × 3.5 × 13.6 cm), all moves top-down mode with operator go per step:
- Reorientation from the rest pose: five steps 85° → 15° (requests e78a487c … bf7b0775), all within 2 ticks, no leads. Operator corrected the camera-derived grasp by 20 mm toward −y (e.g. Contact-GraspNet (0.286, −0.127) → (0.286, −0.147)); the same −20 mm was applied to the second grasp and the operator found it centred.
- Attempt 1: fingertips ~1 cm below the top, squeeze 12 (contact 1748 by stall, held 1740, push 4 ticks, load 40). Lifted 3 cm and 6.8 cm, then the box dropped during the next lift.
- Attempt 2: (0.322, −0.184, 0.01) at 30°, jaw line 75°, ~2 cm below the top, squeeze 24 (contact 1760, held 1745, push 9 ticks, load 60). Carried: lifts to z 0.04/0.10/0.15 (30°→35°→40°); shoulder holding offset 22 empty → 28–30 with the box. Carry back 15 cm / right 7 cm to (0.17, −0.25, 0.15) at 30° with free roll: first try stopped at stage 6 (request 81a5cde4, yaw 15 behind with its 32-tick lead; one bogus ID1 150 °C sample re-read 31); retry from the held pose completed (918dc49e, 20 leads). Lowered 3 cm and opened over the basket (c97bf533, 4899adc3): the box landed in the basket (operator confirmed). Release was ~2–3 cm above rim height in the model; the operator would go a little lower next time.
- Shoulder temperature climbed 31 → 49 °C while holding the extended loaded arm (worst at z 0.15, elbow folded to 884); eased to 47 °C after carrying back. Rest the arm between attempts.
- Next: yaw lead above 32 (or slower yaw stages) for loaded carries; jaw centre offset for the SO101 gripper instead of the operator's 20 mm correction (check whether it is camera calibration or grasp geometry); a basket/drop-zone detector instead of two depth points on its rim.
- Scripted from this run: `grasp/` (runbook `grasp/README.md`): perception (SlimSAM + Contact-GraspNet), grasp and basket planners, one-step executor with camera checks after every step, and grasp retry. Supporting tool change: reach and gripper preflights now accept the gripper holding up to its squeeze (goal up to 24 ticks past position; was 8), because a 24-tick squeeze leaves ~9 and the lift, open or re-close must still start. Tests: `test_left_reach` (21), `test_left_gripper_grip` (16), `grasp/test_planners.py` (2). Restart the server before the next run.

## Neck tilt and camera stream November24
- Tilt (ID8) upper limit raised 2295 → 2356 after a supervised tilt-only test session (`neck_test_session` in robot_config.json, now closed): 2289 → 2318 → 2348 with `test_neck_joint`, no faults, 31 °C; the operator saw the basket rim from 2318 and stopped at 2348, then it returned to 2291. 2350 is the basket view pose (visual only); depth and arm-frame positions still need the 2287 calibration pose. `grasp/step.py view` (default `--tilt-to 2350`) tilts down, snapshots and returns within 6 ticks.
- Camera: `camera_snapshot` now reads a continuous stream (`controllers/camera_stream.py`; ffmpeg keeps the camera open and writes the latest upright frame twice a second with an atomic rename). Cold 5 s captures were very yellow: red/blue 7–26 at 8–9 s after opening, settling with brightness at about 23 s (red/blue 1.64). The first snapshot after the stream starts waits 30 s plus a colour/brightness stability check (~32 s); later snapshots take ~3 s. The stream stops after 30 idle minutes; `python controllers/camera_stream.py status|stop`. `camera_capture.py --no-stream` keeps the old cold capture. Restart the server for the new tool timeout (100 s) and description.

## Grasp skill lessons November24

Second supervised pick-and-place (juice box into the basket), and the changes made in `../grasp/` from it:
- The held box pivots in the jaws and hangs straight down; the planners had modelled it along the 30-40 deg lean, ~6 cm
  too far out, which is why the operator swung servo 1 back and retracted 5 cm over the basket. Model fixed (camera
  depth confirmed on three lifts); the basket default drop is now the point that worked, gripper over (0.18, -0.20).
- Release lower: object bottom 1 cm below the rim (was 1 cm above); the empty gripper lifts 5 cm before the camera view.
- Grasp choice prefers grasps within 1 cm of the object's footprint centre (the top score sat on a corner); offset
  default -15 mm (operator's correction this time).
- Camera checks try several seeds and ignore the arm; UNSEEN (hidden) no longer stops a run, CHECK still does.
- `step.py adjust` for operator corrections (right/left/forward/back/up/down/retract/extend cm, swing-left/right deg),
  keeping lean and jaw line; `ready` works from a grasp hover (step-growing chain); `hold`/`ready` warm the camera
  first; a stalled loaded reach is re-run once; `open` skips when already open.

## Grasp skill lessons November25

Third supervised pick-and-place (juice box into the basket), the first with no operator corrections: the November24
vertical-hang carry, drop point (0.18, -0.20) and 1-cm-below-rim release worked as planned. Changes in `../grasp/`:
- The box twisted and slipped in the jaws during the carry with a 2 cm grip (push 11, holding load 84), so the grasp
  plan now grips 3.5 cm below the top (`--grip-depth`; the gripper point is at the fingertips) and adds a second
  squeeze step after the close.
- The test lift returned completed false with the loaded shoulder 9 ticks off (tolerance 8, no error); `step.py` now
  re-runs such a reach once and accepts a finish within 16 ticks (the camera check still decides). The stall retry
  also looked at the camera-check log line instead of the reach's and could not fire after a checked step; fixed.
- The neck torque was off at the basket view; `step.py hold` and `view` now call `neck_hold` in place when it is off.
- One false camera CHECK in the carry (mask on the box top only); over the basket the camera sees only a sliver of
  the box, so the operator's eyes decide before the drop.
