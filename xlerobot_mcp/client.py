#!/usr/bin/env python3
"""Authenticated local MCP client for Codex. Never retries a tool call."""
import argparse
import asyncio
import base64
import json
from pathlib import Path
import sys
import time
import httpx
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client

ROOT = Path(__file__).resolve().parent
URL = 'http://127.0.0.1:8765/mcp'
TOOLS = {'preview_left_gripper_up','move_left_gripper_up', 'move_left_shoulder_up', 'preview_right_gripper_up', 'move_right_gripper_up', 'base_motion_state', 'base_status', 'robot_info', 'joint_status', 'camera_snapshot', 'move_base',
         'nudge_joint', 'test_neck_pan', 'test_neck_joint', 'pan_torque', 'left_arm_hold', 'right_arm_hold', 'test_left_yaw', 'test_right_yaw', 'test_right_elbow_p', 'test_right_shoulder', 'test_right_elbow', 'test_right_wrist', 'test_right_roll', 'test_right_gripper', 'test_right_yaw_range', 'test_right_shoulder_range', 'test_right_elbow_range', 'test_right_wrist_range', 'test_right_roll_range', 'test_right_gripper_range', 'test_left_shoulder', 'test_left_elbow', 'test_left_elbow_bias', 'test_left_wrist', 'test_left_roll', 'test_left_gripper', 'test_left_yaw_range', 'test_left_shoulder_range', 'test_left_elbow_range', 'test_left_wrist_range', 'test_left_roll_range', 'test_left_gripper_range', 'stop_active', 'stop_base'}
MOTION = {'move_left_gripper_up', 'move_left_shoulder_up', 'move_right_gripper_up', 'move_base', 'nudge_joint', 'test_neck_pan', 'test_neck_joint', 'pan_torque', 'left_arm_hold', 'right_arm_hold', 'test_left_yaw', 'test_right_yaw', 'test_right_elbow_p', 'test_right_shoulder', 'test_right_elbow', 'test_right_wrist', 'test_right_roll', 'test_right_gripper', 'test_right_yaw_range', 'test_right_shoulder_range', 'test_right_elbow_range', 'test_right_wrist_range', 'test_right_roll_range', 'test_right_gripper_range', 'test_left_shoulder', 'test_left_elbow', 'test_left_elbow_bias', 'test_left_wrist', 'test_left_roll', 'test_left_gripper', 'test_left_yaw_range', 'test_left_shoulder_range', 'test_left_elbow_range', 'test_left_wrist_range', 'test_left_roll_range', 'test_left_gripper_range'}


def decode(result, output_dir):
    if result.isError:
        raise RuntimeError('; '.join(b.text for b in result.content if b.type == 'text'))
    blocks = []
    for block in result.content:
        if block.type == 'text':
            try:
                value = json.loads(block.text)
            except json.JSONDecodeError:
                value = {'text': block.text}
            blocks.append(value)
        elif block.type == 'image':
            if output_dir is None:
                raise ValueError('Camera requires --output-dir for the fresh image')
            output_dir.mkdir(parents=True, exist_ok=True)
            path = output_dir / f'robot-camera-{time.time_ns()}.png'
            with path.open('xb') as f:
                f.write(base64.b64decode(block.data, validate=True))
            blocks.append({'saved_image': str(path.resolve())})
    return blocks


async def execute(args):
    # No arbitrary URL or token arguments: never send this credential elsewhere.
    token = (ROOT/'.private/token').read_text().strip()
    async with httpx.AsyncClient(headers={'Authorization': 'Bearer '+token}, timeout=65, trust_env=False) as client:
        async with streamable_http_client(URL, http_client=client) as (read, write, _):
            async with ClientSession(read, write) as session:
                await session.initialize()
                async def call(name, arguments):
                    data = decode(await session.call_tool(name, arguments), args.output_dir)
                    visible = [{k: v for k, v in item.items() if k != 'configuration'} if isinstance(item, dict) else item for item in data]
                    print(json.dumps({'tool': name, 'results': visible}), flush=True)
                    return data
                if args.command == 'verify':
                    info = (await call('robot_info', {}))[0]
                    if info.get('protocol_version') != 2 or info.get('restart_required'):
                        raise RuntimeError('Updated server must be restarted before verification')
                    names = {t.name for t in (await session.list_tools()).tools}
                    if names != TOOLS:
                        raise RuntimeError('Unexpected MCP tool list')
                    if info.get('mock'):
                        raise RuntimeError('Mock server: hardware verification not performed')
                    for group, motor in [('left-arm', 1), ('right-arm', 1), ('neck', 7), ('neck', 8)]:
                        await call('joint_status', {'group': group, 'servo_id': motor})
                    await call('camera_snapshot', {})
                    print(json.dumps({'connection_verified': True, 'motion_commands_sent': 0,
                                      'boot_id': info['boot_id']}), flush=True)
                else:
                    params = json.loads(args.arguments)
                    if not isinstance(params, dict):
                        raise ValueError('--arguments must be a JSON object')
                    if args.tool == 'camera_snapshot' and args.output_dir is None:
                        raise ValueError('Camera requires --output-dir')
                    if args.tool in MOTION:
                        info = decode(await session.call_tool('robot_info', {}), None)[0]
                        if info.get('protocol_version') != 2 or info.get('restart_required'):
                            raise RuntimeError('Refusing motion through old or changed server; restart required')
                        if not info.get('motion_enabled'):
                            raise RuntimeError('Server motion is disabled')
                        if not params.get('request_id'):
                            raise ValueError('Supply a UUID request_id for the explicitly requested action')
                    await call(args.tool, params)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    verify = sub.add_parser('verify', help='Read-only connection, bus and camera check')
    verify.add_argument('--output-dir', type=Path, required=True)
    call = sub.add_parser('call')
    call.add_argument('tool', choices=sorted(TOOLS))
    call.add_argument('--arguments', default='{}')
    call.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    try:
        asyncio.run(execute(args))
    except (Exception, KeyboardInterrupt) as exc:
        # The MCP session wraps tool errors in exception groups; report the
        # underlying messages, and the controller result when one is embedded.
        def leaves(error):
            if isinstance(error, BaseExceptionGroup):
                for inner in error.exceptions:
                    yield from leaves(inner)
            else:
                yield error
        messages = [str(e) or type(e).__name__ for e in leaves(exc)]
        report = {'error': '; '.join(messages), 'retried': False,
                  'note': 'If a motion was dispatched, treat its outcome as uncertain; do not retry.'}
        for message in messages:
            start = message.find('{')
            try:
                tool_result = json.loads(message[start:]) if start >= 0 else None
            except json.JSONDecodeError:
                continue
            if isinstance(tool_result, dict):
                report['tool_result'] = tool_result
                report['error'] = 'Tool failed; see tool_result'
                break
        print(json.dumps(report), file=sys.stderr)
        return 1
    return 0

if __name__ == '__main__':
    sys.exit(main())
