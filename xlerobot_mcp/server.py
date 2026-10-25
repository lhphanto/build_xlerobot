"""Local, authenticated XLeRobot MCP. Hardware is accessed only by explicit tools."""
import argparse
import asyncio
import base64
import json
import hashlib
import sqlite3
import time
import math
import os
from pathlib import Path
import secrets
import signal
import sys
import uuid
from typing import Literal
from mcp.server.fastmcp import FastMCP
from mcp.types import ImageContent, TextContent, ToolAnnotations
import uvicorn

ROOT = Path(__file__).resolve().parent
PROTOCOL_VERSION = 2

def code_digest():
    files = [ROOT/'server.py', *sorted((ROOT/'controllers').glob('*.py'))]
    return hashlib.sha256(b''.join(p.read_bytes() for p in files)).hexdigest()

ROBOT_PYTHON = '/Users/huiliu/miniforge3/envs/lerobot/bin/python'
READ = ToolAnnotations(readOnlyHint=True, openWorldHint=False)
WRITE = ToolAnnotations(readOnlyHint=False, destructiveHint=True, idempotentHint=False, openWorldHint=False)


class Controller:
    def __init__(self, mock=False, allow_motion=False, journal=None):
        self.mock, self.allow_motion = mock, allow_motion
        self.lock = asyncio.Lock()
        self.active = None
        self.active_motion = False
        self.boot_id = str(uuid.uuid4())
        self.loaded_code_digest = code_digest()
        self.journal = sqlite3.connect(str(journal) if journal else ':memory:')
        self.journal.execute('CREATE TABLE IF NOT EXISTS requests (id TEXT PRIMARY KEY, script TEXT, args TEXT, reserved_at REAL)')
        self.journal.commit()
        if journal:
            os.chmod(journal, 0o600)
        self.stopping = False

    async def run(self, script, args, motion=False, request_id=None, timeout=12):
        if self.stopping or self.lock.locked():
            raise ValueError('Robot busy/stopping; no command queued. Wait for completion.')
        if motion:
            if not self.allow_motion:
                raise ValueError('Motion disabled. Operator must restart with --allow-motion.')
            pending=ROOT/'.private/right-elbow-p-recovery.json'
            restoring=(script=='right_elbow_p_test.py' and '--action' in args and args[args.index('--action')+1]=='restore')
            supported_release=(script=='right_arm_hold.py' and args and args[0]=='release')
            if pending.exists() and not (restoring or supported_release):
                raise ValueError('Pending right-ID3 P restoration; restore P before further motion')
            if self.loaded_code_digest != code_digest():
                raise ValueError('Controller code changed; operator restart required before motion.')
            request_id = str(uuid.UUID(request_id))
            try:
                self.journal.execute('INSERT INTO requests VALUES (?, ?, ?, ?)',
                                     (request_id, script, json.dumps(args), time.time()))
                self.journal.commit()  # Reserve before starting; failed/uncertain attempts remain consumed.
            except sqlite3.IntegrityError:
                raise ValueError('Duplicate motion request ID; not executed again.') from None
        async with self.lock:
            # Stop may arrive while process creation is awaiting completion.
            self.active_motion = motion
            try:
                if self.mock:
                    return {'mock': True, 'script': script, 'arguments': args, 'hardware_accessed': False}
                self.active = await asyncio.create_subprocess_exec(
                    ROBOT_PYTHON, str(ROOT/'controllers'/script), *args,
                    stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                if self.stopping and motion:
                    self.active.send_signal(signal.SIGINT)
                task = asyncio.create_task(self.active.communicate())
                try:
                    out, err = await asyncio.wait_for(asyncio.shield(task), timeout)
                except (asyncio.TimeoutError, asyncio.CancelledError):
                    if self.active.returncode is None:
                        self.active.send_signal(signal.SIGINT)
                    try:
                        await asyncio.wait_for(asyncio.shield(task), 5)
                    except asyncio.TimeoutError:
                        # Keep the lock until the child exits: do not create a competing writer.
                        print('Child unresponsive; physical stop may be required.', file=sys.stderr)
                        await asyncio.shield(task)
                    raise
                result = {'mock': False, 'exit_code': self.active.returncode,
                          'stdout': out.decode()[-128000:] if script in ('left_gripper_up.py','left_shoulder_up.py','right_gripper_up.py','right_elbow_p_test.py','right_arm_test.py','left_yaw_range_test.py','left_shoulder_range_test.py','left_elbow_range_test.py','left_wrist_range_test.py','left_roll_range_test.py','left_gripper_range_test.py') else out.decode()[-64000:] if script in ('left_elbow_test.py','left_elbow_bias_test.py','left_wrist_test.py','left_roll_test.py','left_gripper_test.py') else out.decode()[-16000:], 'stderr': err.decode()[-4000:]}
                if self.active.returncode:
                    raise ValueError(json.dumps(result))
                return result
            finally:
                self.active = None
                self.active_motion = False

    async def stop(self):
        self.stopping = True
        try:
            if self.active_motion:
                if self.active and self.active.returncode is None:
                    self.active.send_signal(signal.SIGINT)
                for _ in range(100):
                    if not self.active_motion:
                        return {'interrupted': True, 'note': 'Inspect motion tool result for stop/hold acknowledgment; no physical sensing.'}
                    await asyncio.sleep(.05)
                return {'stopped': False, 'error': 'Controller unresponsive; use physical cutoff if needed.'}
            return {'interrupted': False, 'note': 'No active MCP movement. Use stop_base for wheel zero velocity; arms keep holding torque.'}
        finally:
            self.stopping = False


def build_server(controller):
    mcp = FastMCP('xlerobot', host='127.0.0.1', stateless_http=True, json_response=True)

    @mcp.tool(annotations=READ)
    def robot_info() -> dict:
        """Report connector mode and configuration; does not read hardware."""
        return {'protocol_version': PROTOCOL_VERSION, 'boot_id': controller.boot_id,
                'code_digest': controller.loaded_code_digest,
                'restart_required': controller.loaded_code_digest != code_digest(),
                'config_path': str(ROOT/'robot_config.json'),
                'config_digest': hashlib.sha256((ROOT/'robot_config.json').read_bytes()).hexdigest(),
                'mock': controller.mock, 'motion_enabled': controller.allow_motion,
                'right_elbow_p_recovery_pending': (ROOT/'.private/right-elbow-p-recovery.json').exists(),
                'configuration': json.loads((ROOT/'robot_config.json').read_text()),
                'limitations': 'No firmware watchdog, calibrated depth, collision avoidance, or autonomous arm IK.'}

    @mcp.tool(annotations=READ)
    async def joint_status(group: Literal['left-arm','right-arm','neck'], servo_id: int) -> dict:
        """Read current position, limits, voltage and faults. Does not enable torque."""
        if servo_id not in (range(7,9) if group=='neck' else range(1,7)):
            raise ValueError('Invalid servo ID for group')
        return await controller.run('joint_control.py',['status','--group',group,'--id',str(servo_id)])

    @mcp.tool(annotations=READ)
    def base_motion_state() -> dict:
        """Read controller activity and the last wheel pulse phase without opening hardware. Match request_id; saved phase may be stale."""
        path=ROOT/'.private/base-phase.json'
        return {'active_motion':controller.active_motion,'last_phase':json.loads(path.read_text()) if path.exists() else None}

    @mcp.tool(annotations=READ)
    async def base_status() -> dict:
        """Read fresh model, mode, voltage, faults and velocity from all four wheels; no writes."""
        return await controller.run('base_control.py',['status'])

    @mcp.tool(annotations=READ)
    async def camera_snapshot() -> list[TextContent | ImageContent]:
        """Capture fresh upright side-by-side camera image with 5-second warm-up. Robot must be stationary."""
        if controller.mock:
            return [TextContent(type='text',text='MOCK: no camera accessed; no image available.')]
        import tempfile
        with tempfile.TemporaryDirectory(prefix='xlerobot-frame-') as directory:
            path=Path(directory)/'frame.png'
            result=await controller.run('camera_capture.py',['--output',str(path)],timeout=55)
            return [TextContent(type='text',text=json.dumps(result)),
                    ImageContent(type='image',mimeType='image/png',data=base64.b64encode(path.read_bytes()).decode())]

    @mcp.tool(annotations=WRITE)
    async def move_base(direction: Literal['forward','backward','left','right','rotate-left','rotate-right'],
                        request_id: str, seconds: float=.5, speed: float=.1, turn_speed: float=20) -> dict:
        """Supervised base pulse; explicit user motion request required. New UUID per intentional move. Never retry uncertain motion."""
        for value, cap in [(seconds,.5),(speed,.1),(turn_speed,20)]:
            if not math.isfinite(value) or not 0<value<=cap:
                raise ValueError('Caps: 0.5 seconds, 0.1 m/s, 20 deg/s; finite positive values required')
        return await controller.run('base_control.py',[direction,'--seconds',str(seconds),'--speed',str(speed),
                                    '--turn-speed',str(turn_speed),'--request-id',request_id,'--execute'],True,request_id)

    @mcp.tool(annotations=WRITE)
    async def nudge_joint(group: Literal['left-arm','right-arm','neck'], servo_id: int,
                          delta_raw: int, request_id: str) -> dict:
        """Move one commissioned joint at most 32 ticks. Existing torque must be on; never bypass calibration gates."""
        if servo_id not in (range(7,9) if group=='neck' else range(1,7)) or not 0<abs(delta_raw)<=32:
            raise ValueError('Invalid joint or delta')
        return await controller.run('joint_control.py',['nudge','--group',group,'--id',str(servo_id),
                                    '--delta-raw',str(delta_raw),'--execute'],True,request_id)

    @mcp.tool(annotations=WRITE)
    async def test_neck_pan(request_id: str) -> dict:
        """Explicitly authorized supervised commissioning only: preload neck ID7, enable torque, +16 ticks. Leaves torque on."""
        return await controller.run('test_neck_pan.py',['--execute'],True,request_id)

    @mcp.tool(annotations=WRITE)
    async def test_neck_joint(servo_id: int, target_raw: int, expected_position: int,
                              request_id: str, enable_if_off: bool=False) -> dict:
        """Supervised neck commissioning only. One axis, bounded test window/budget; preload before explicit torque enable. Leaves holding torque on."""
        if servo_id not in (7,8) or not 0<=target_raw<=4095 or not 0<=expected_position<=4095:
            raise ValueError('Invalid neck ID or position')
        args=['--id',str(servo_id),'--target',str(target_raw),'--expected-position',str(expected_position),'--execute']
        if enable_if_off: args.append('--enable-if-off')
        return await controller.run('neck_test.py',args,True,request_id)

    @mcp.tool(annotations=WRITE)
    async def pan_torque(action: Literal['release','hold'], expected_position: int, request_id: str) -> dict:
        """Explicit manual neck alignment only. Release pan ID7 or preload its current pose and enable hold. Tilt remains holding. User must support camera."""
        if not 0<=expected_position<=4095: raise ValueError('Invalid position')
        return await controller.run('pan_torque.py',[action,'--expected-position',str(expected_position)],True,request_id)

    @mcp.tool(annotations=WRITE)
    async def left_arm_hold(action: Literal['hold','release','recover-hold'], request_id: str,
                            expected_positions: dict[str,int] | None=None) -> dict:
        """Explicit fully-supported left-arm startup only. Normal hold preloads current pose, staged one joint at a time. recover-hold permits ONLY ID3 up to32ticks outside saved limits: speed30 to32ticks inside (max64ticks travel), requires stationary feedback inside by8ticks, residual<=24, then may handoff once to the verified in-range current pose and verify holding. All other joints must be in range, all initially torqueoff. Limits/configuration unchanged. No further activations on failure; may leave partial torqueon and inward/hold targets; no automatic retry/release. Operator watches, supports all links, fingers clear/cutoff available. Release requires full support. Ordinary motion still requires all joints in range."""
        positions=expected_positions or {}
        if action in ('hold','recover-hold') and (set(positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in positions.values())):
            raise ValueError('Provide all6 observed integer positions')
        return await controller.run('left_arm_hold.py',[action,'--expected-positions',json.dumps(positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def right_arm_hold(action: Literal['hold','release','recover-hold'], request_id: str,
                            expected_positions: dict[str,int] | None=None) -> dict:
        """Explicit supported right-arm commissioning only. Set each joint current target and speed atomically, then verify holding before next joint; target writes may activate torque. recover-hold permits onlyID3 initially at most16ticks outside saved limits, commands inward atspeed30 to16ticks inside (max32ticks), verifies settling before enabling remaining joints. No limits changed. Release only when user supports arm. Normal hold commands no travel; recover-hold commands only the bounded inward elbow move; leaves holding torque on after success/failure."""
        positions=expected_positions or {}
        if action in ('hold','recover-hold') and (set(positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in positions.values())):
            raise ValueError('Provide all6 observed integer positions')
        return await controller.run('right_arm_hold.py',[action,'--expected-positions',json.dumps(positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_left_yaw(expected_positions: dict[str,int], request_id: str) -> dict:
        """One explicitly supervised left-arm ID1 yaw trial: +80 ticks, two-second pause, and return. All six joints already holding, empty arm supported. No other joint writes. Leaves torque on; never retry uncertain motion."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_yaw_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_right_yaw(expected_positions: dict[str,int], request_id: str) -> dict:
        """One explicitly supervised right-arm ID1 yaw trial: +80 ticks, two-second pause, and return. All six joints already holding, empty arm supported. No other joint writes. Leaves torque on; never retry uncertain motion."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_yaw_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_left_shoulder(expected_positions: dict[str,int], request_id: str) -> dict:
        """One explicitly supervised left-arm ID2 trial: +32 ticks, two-second pause, return. Probable shoulder lift; operator must verify mapping. Empty arm supported, all six joints already holding. No other joint writes; leaves torque on."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_shoulder_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_left_elbow(expected_positions: dict[str,int], request_id: str, direction: Literal['positive','negative']='positive', amplitude_deg: float=7.03125, slow: bool=False) -> dict:
        """One explicitly supervised left-arm ID3 trial: amplitude_deg2-7.03125degrees (default80ticks) in the chosen direction, two-second pause, return. slow=true uses30ticks/s; default100. Endpoint4/return16tick tolerance; operator verifies clearance. Empty arm supported, all six joints already holding. No other joint writes; leaves torque on."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        if not math.isfinite(amplitude_deg) or not 2<=amplitude_deg<=7.03125:raise ValueError('Amplitude must be2-7.03125degrees')
        args=['--expected-positions',json.dumps(expected_positions),'--direction',direction,'--amplitude-deg',str(amplitude_deg)]
        if slow:args.append('--slow')
        return await controller.run('left_elbow_test.py',args,True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_left_elbow_bias(expected_positions: dict[str,int], request_id: str) -> dict:
        """One explicitly supervised left-arm ID3 gravity-sag trial. Reads the existing elbow goal and its stationary residual (5-16 ticks), commands goal minus residual at speed30 (at most32ticks travel, within saved limits), waits for stationary feedback and reports whether position reached the original goal within4ticks. Biased goal stays in place; no return or retry. Empty arm supported, all six joints already holding. No other joint writes; leaves torque on."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_elbow_bias_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=15)

    @mcp.tool(annotations=WRITE)
    async def test_left_wrist(expected_positions: dict[str,int], request_id: str) -> dict:
        """One explicitly supervised left-arm ID4 trial: +32 ticks, two-second pause, return if settled. Probable wrist flex; operator must verify mapping. Supported empty arm, all six joints already holding. ID4-only writes; leaves torque on."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_wrist_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_left_roll(expected_positions: dict[str,int], request_id: str) -> dict:
        """One explicitly supervised left-arm ID5 wrist-roll trial: +32 ticks, two-second pause, return if settled. Empty arm supported, all six joints already holding. ID5-only writes; leaves torque on. Operator verifies physical direction."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_roll_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_left_gripper(expected_positions: dict[str,int], request_id: str) -> dict:
        """One explicitly supervised left-arm ID6 empty-gripper trial: +32 ticks, two-second pause, return if settled. Fingers clear, empty arm supported, all six joints already holding. ID6-only writes; leaves torque on. Operator verifies opening/closing direction."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_gripper_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_left_yaw_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised ID1 yaw range trial: +80ticks(left), center, -80ticks(right), center,2s pause at each point. Supported empty arm, all six joints holding, clear both sides. No other joint writes. Save no operating range without operator clearance observations."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_yaw_range_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_left_shoulder_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised ID2 shoulder clearance trial: +80ticks(up), center, -80ticks(down), center,2s pause at each point. Supported empty arm, all six joints holding, clear both directions. ID2-only writes. No operating range saved without operator clearance observations."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_shoulder_range_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_left_elbow_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised ID3 clearance sequence: +80ticks, center, -80ticks, center,2s pause each. Endpoints4tick, center16tick tolerance, three stationary readings. Supported empty arm, other joints holding, clearance both ways. ID3-only writes; operator verifies physical clearance before any range saved."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_elbow_range_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_left_wrist_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised ID4 wrist-flex clearance sequence: +80ticks(down), center, -80ticks(up), center,2s pause each. Supported empty arm, all six joints holding; wrist/gripper clearance both ways. ID4-only writes; operator verifies clearance before any operating range saved."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_wrist_range_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_left_roll_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised ID5 wrist-roll clearance sequence: +80ticks(clockwise), center, -80ticks(counterclockwise), center;2s pause each. View from gripper tip toward arm. Supported empty arm, six joints holding, wrist/gripper clearance. ID5-only writes; leaves torque on."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_roll_range_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_left_gripper_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised empty-gripper ID6 sequence: +80ticks(farther open), center, -80ticks(toward closed), center;2s pause each. Gripper empty, fingers clear, supported empty arm, all six joints holding. ID6-only writes; no force qualification or object grasping. Leaves torque on."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('left_gripper_range_test.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_right_shoulder(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised right-arm ID2 test: +32 ticks, monitored 2s pause, return. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Four-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','2','--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_right_elbow(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised right-arm ID3 test: +32 ticks, monitored 2s pause, return. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Four-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','3','--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_right_wrist(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised right-arm ID4 test: +32 ticks, monitored 2s pause, return. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Four-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','4','--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_right_roll(expected_positions: dict[str,int], request_id: str, direction: Literal['positive','negative']='positive') -> dict:
        """One supervised right-arm ID5 test: 32 ticks in selected encoder direction, monitored 2s pause, return. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Six-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','5','--expected-positions',json.dumps(expected_positions),'--direction',direction],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_right_gripper(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised right-arm ID6 test: +32 ticks, monitored 2s pause, return. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Four-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','6','--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=20)

    @mcp.tool(annotations=WRITE)
    async def test_right_yaw_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised right-arm ID1 clearance sequence: +80 ticks, center, -80 ticks, center; 2s monitored pauses. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Four-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','1','--expected-positions',json.dumps(expected_positions),'--clearance'],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_right_shoulder_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised right-arm ID2 clearance sequence: +80 ticks, center, -80 ticks, center; 2s monitored pauses. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Four-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','2','--expected-positions',json.dumps(expected_positions),'--clearance'],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_right_elbow_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised right-arm ID3 clearance sequence: +80 ticks, center, -80 ticks, center; 2s monitored pauses. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Four-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','3','--expected-positions',json.dumps(expected_positions),'--clearance'],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_right_wrist_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised right-arm ID4 clearance sequence: +80 ticks, center, -80 ticks, center; 2s monitored pauses. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Four-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','4','--expected-positions',json.dumps(expected_positions),'--clearance'],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_right_roll_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised right-arm ID5 clearance sequence: +80 ticks, center, -80 ticks, center; 2s monitored pauses. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Six-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','5','--expected-positions',json.dumps(expected_positions),'--clearance'],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_right_gripper_range(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised right-arm ID6 clearance sequence: +80 ticks, center, -80 ticks, center; 2s monitored pauses. All six already holding; empty arm supported, fingers clear. Mapping/direction unverified. Four-tick settling tolerance; no saved range changes. ID5 travel requires approved software limits."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_arm_test.py',['--id','6','--expected-positions',json.dumps(expected_positions),'--clearance'],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def test_right_elbow_p(request_id: str, action: Literal['trial','restore']='trial',
                                 trial_p: Literal[16,24]=16, expected_positions: dict[str,int] | None=None) -> dict:
        """One supervised right-ID3 +32tick trial and return at fixed P16 or24, then restore original16 and lock1. All six holding, empty arm supported. P is protected configuration: temporary experiment writes can persist until verified restoration. Recovery journal blocks new motion if unfinished; restore action only restores fixed P16/lock1, no travel. Trials at16/24 are separate calls; operator must reestablish the same supported pose for comparison. Four-tick accuracy criterion unchanged."""
        positions=expected_positions or {}
        if action=='trial' and (set(positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in positions.values())):
            raise ValueError('Provide all6 fresh integer positions')
        return await controller.run('right_elbow_p_test.py',['--action',action,'--p',str(trial_p),'--expected-positions',json.dumps(positions)],True,request_id,timeout=25)

    @mcp.tool(annotations=READ)
    async def preview_right_gripper_up(expected_positions: dict[str,int]) -> dict:
        """Read-only fixed2mm SO101-model +Z reach preview. Checks fresh right arm state, saved homing offsets, calibration/model limits and <=32tick targets; onlyIDs2-4 proposed. Actual mounted up direction and Cartesian accuracy require supervised observation; no collision avoidance. Returns preview hash required for motion."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):raise ValueError('All6 fresh integer positions required')
        return await controller.run('right_gripper_up.py',['preview','--expected-positions',json.dumps(expected_positions)],timeout=30)

    @mcp.tool(annotations=WRITE)
    async def move_right_gripper_up(expected_positions: dict[str,int], plan_hash: str, request_id: str) -> dict:
        """One explicitly supervised fixed2mm model +Z coordinated reach; no automatic return. All6 already holding within4ticks, empty arm supported, movement/cables/fingers clear. Recomputes hashed preview, checks calibration offsets and current pose. IDs2-4 only, <=32ticks total, <=8ticks per waypoint, speed30ticks/s; four-tick settling check. No PID/torque/EEPROM writes. Model displacement/direction are not physical proof; stop on unexpected result. Ordinary arm commissioning remains disabled."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):raise ValueError('All6 fresh integer positions required')
        if len(plan_hash)!=64 or any(c not in '0123456789abcdef' for c in plan_hash):raise ValueError('Valid preview hash required')
        return await controller.run('right_gripper_up.py',['execute','--expected-positions',json.dumps(expected_positions),'--plan-hash',plan_hash],True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def move_left_shoulder_up(expected_positions: dict[str,int], request_id: str) -> dict:
        """One supervised left-arm shoulder ID2 lift +32ticks(~2.8°), speed30(~2.6°/s), then hold. All6 already holding, empty arm supported, lift/cables/fingers clear. Known positive direction upward; gripper Cartesian trajectory uncalibrated. ID2-only writes,4tick settle check, no automatic return or torque release; no limits/PID/EEPROM changes. Explicit commissioning action; ordinary arm gates stay disabled."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):raise ValueError('All6 fresh integer positions required')
        return await controller.run('left_shoulder_up.py',['--expected-positions',json.dumps(expected_positions)],True,request_id,timeout=15)

    @mcp.tool(annotations=READ)
    async def preview_left_gripper_up(expected_positions: dict[str,int], distance_mm: float = 5, coarse: bool = False) -> dict:
        """Read-only left-gripper IK preview. coarse=true:10-20mm nominal,64ticks/joint,16ticks/stage,6stages,6deg tilt; sampled shoulder8/elbow16/wrist4tick lag must predict at least half requested model lift. Returns conservative model lift interval; no physical accuracy guarantee. Otherwise strict mode: distance_mm(2-10mm, default5) SO101-model +Z, same RobotKinematics as teleop_phone_so101.py, position-focused IK with <=3deg pitch change and sampled joint-lag checks. Live-verified left homing/ranges/resolution required; no phone input or writes. <=32ticks/joint, <=16ticks/stage, shoulder-leading sequence with stored goals bound in hash. IDs2-4 only. Model-to-body direction/metric accuracy not physically calibrated. Hash binds pose/calibration/model/config."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):raise ValueError('All6 fresh integer positions required')
        if not math.isfinite(distance_mm) or not (10<=distance_mm<=20 if coarse else 2<=distance_mm<=10):raise ValueError('distance_mm must be2-10 strict or10-20 coarse')
        return await controller.run('left_gripper_up.py',['preview','--expected-positions',json.dumps(expected_positions),'--distance-mm',str(distance_mm)]+(['--coarse'] if coarse else []),timeout=30)

    @mcp.tool(annotations=WRITE)
    async def move_left_gripper_up(expected_positions: dict[str,int], plan_hash: str, request_id: str, distance_mm: float = 5, coarse: bool = False) -> dict:
        """One supervised model +Z left-gripper IK reach, then hold; no return. Explicit coarse=true commissioning mode uses continuous bounded target updates at intermediate stages (no intentional stationary wait); final requires3stationary samples. Limits:distance_mm10-20,64ticks/joint,16ticks/stage,6stages,6deg tilt, shoulder8/elbow16/wrist4tick lag, at least half requested model lift mandatory. Preview must qualify that lag envelope. Reports achieved model lift and coarse completion; never precise completion or physical accuracy. Otherwise strict default:distance_mm2-10(default5). Uses position-focused IK, pitch change<=3deg, sampled independent joint-lag checks, shoulder-leading then elbow/wrist stages, speed30, <=32ticks/joint, <=16ticks/stage, <=4stages. All6 holding and stationary, standard left startup goal-error<=8. Intermediate stages require upward model progress and <=8tick lag toward baseline; final settling<=4ticks and requested modeled lift within1mm. No unchanged-target wait as first stage; too-small requests reject. Fresh model/hash/calibration guards and encoder-FK upward-progress guard. No phone, PID/EEPROM/torque writes. Empty arm supported, fingers/cables clear. Numerical/model completion is not measured physical displacement; operator observation required. Ordinary uncommissioned-arm gates stay disabled."""
        if set(expected_positions)!=set(map(str,range(1,7))) or any(type(v) is not int or not 0<=v<=4095 for v in expected_positions.values()):raise ValueError('All6 fresh integer positions required')
        if len(plan_hash)!=64 or any(c not in '0123456789abcdef' for c in plan_hash):raise ValueError('Valid preview hash required')
        if not math.isfinite(distance_mm) or not (10<=distance_mm<=20 if coarse else 2<=distance_mm<=10):raise ValueError('distance_mm must be2-10 strict or10-20 coarse')
        return await controller.run('left_gripper_up.py',['execute','--expected-positions',json.dumps(expected_positions),'--plan-hash',plan_hash,'--distance-mm',str(distance_mm)]+(['--coarse'] if coarse else []),True,request_id,timeout=30)

    @mcp.tool(annotations=WRITE)
    async def stop_active() -> dict:
        """Interrupt active MCP motion so base stop or joint hold cleanup runs. Not a hardware emergency stop."""
        return await controller.stop()

    @mcp.tool(annotations=WRITE)
    async def stop_base() -> dict:
        """Interrupt MCP motion, then send zero wheel velocity. No arm torque changes; cannot stop unrelated controllers."""
        stopped=await controller.stop()
        if controller.lock.locked():
            return {'stop_confirmed':False,'detail':stopped,'note':'Busy; close other controllers or use physical cutoff.'}
        return await controller.run('base_control.py',['stop','--execute'])

    return mcp


class TokenAuth:
    def __init__(self,app,token): self.app,self.token=app,token
    async def __call__(self,scope,receive,send):
        if scope['type']=='http':
            h=dict(scope.get('headers',[]))
            if b'origin' in h or not secrets.compare_digest(h.get(b'authorization',b''),('Bearer '+self.token).encode()):
                await send({'type':'http.response.start','status':403,'headers':[]})
                await send({'type':'http.response.body','body':b'Forbidden'})
                return
        await self.app(scope,receive,send)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mock',action='store_true')
    p.add_argument('--allow-motion',action='store_true')
    p.add_argument('--port',type=int,default=8765)
    p.add_argument('--token-file',type=Path,required=True)
    args=p.parse_args()
    token=args.token_file.read_text().strip()
    if len(token)<32: p.error('Token must be at least 32 characters')
    journal = args.token_file.parent / ('mock-requests.sqlite3' if args.mock else 'motion-requests.sqlite3')
    c=Controller(args.mock,args.allow_motion,journal)
    m=build_server(c)
    print(f'XLeRobot MCP: mock={args.mock}, motion={args.allow_motion}, localhost only',file=sys.stderr)
    uvicorn.run(TokenAuth(m.streamable_http_app(),token),host='127.0.0.1',port=args.port)

if __name__=='__main__': main()
