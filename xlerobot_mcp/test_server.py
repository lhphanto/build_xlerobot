import asyncio
import json
from pathlib import Path
import secrets
import subprocess
import sys
import tempfile
from unittest.mock import patch
import httpx
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client
import server

async def unit():
    c=server.Controller(mock=True)
    try: await c.run('base_control.py',[],True,'11111111-1111-4111-8111-111111111111')
    except ValueError: pass
    else: raise AssertionError('Motion gate failed')
    c.allow_motion=True
    rid='22222222-2222-4222-8222-222222222222'
    await c.run('base_control.py',[],True,rid)
    try: await c.run('base_control.py',[],True,rid)
    except ValueError: pass
    else: raise AssertionError('Replay gate failed')
    class Child:
        returncode=None
        def __init__(self): self.done=asyncio.Event(); self.signals=[]
        def send_signal(self,sig): self.signals.append(sig); self.returncode=1; self.done.set()
        async def communicate(self): await self.done.wait(); return b'', b'interrupted'
    c=server.Controller(mock=False,allow_motion=True); child=Child()
    async def spawn(*a,**kw): return child
    with patch.object(asyncio,'create_subprocess_exec',spawn):
        running=asyncio.create_task(c.run('fake.py',[],True,'33333333-3333-4333-8333-333333333333'))
        await asyncio.sleep(.02)
        result=await c.stop()
        try: await running
        except ValueError: pass
        assert child.signals and result['interrupted']
    print('PASS: motion disabled gate, duplicate request rejection, interrupt and cleanup coordination')

async def integration():
    with tempfile.TemporaryDirectory() as tmp:
        token=secrets.token_urlsafe(32); path=Path(tmp)/'token'; path.write_text(token)
        p=subprocess.Popen([sys.executable,str(Path(__file__).with_name('server.py')),'--mock','--allow-motion','--port','18765','--token-file',str(path)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        try:
            async with httpx.AsyncClient() as client:
                for _ in range(100):
                    try:
                        r=await client.get('http://127.0.0.1:18765/mcp')
                        break
                    except httpx.ConnectError: await asyncio.sleep(.05)
                else: raise AssertionError('Server did not start')
                assert r.status_code==403
                r=await client.post('http://127.0.0.1:18765/mcp',headers={'Authorization':'Bearer '+token,'Origin':'https://example.com'})
                assert r.status_code==403
            async with httpx.AsyncClient(headers={'Authorization':'Bearer '+token}) as client:
                async with streamable_http_client('http://127.0.0.1:18765/mcp',http_client=client) as (read,write,_):
                    async with ClientSession(read,write) as session:
                        await session.initialize()
                        names={t.name for t in (await session.list_tools()).tools}
                        assert names=={'preview_left_gripper_up','move_left_gripper_up','move_left_shoulder_up','preview_right_gripper_up','move_right_gripper_up','base_motion_state','base_status','robot_info','joint_status','camera_snapshot','move_base','nudge_joint','test_neck_pan','test_neck_joint','pan_torque','left_arm_hold','right_arm_hold', 'test_left_yaw','test_right_yaw','test_right_elbow_p','test_right_shoulder','test_right_elbow','test_right_wrist','test_right_roll','test_right_gripper','test_right_yaw_range','test_right_shoulder_range','test_right_elbow_range','test_right_wrist_range','test_right_roll_range','test_right_gripper_range', 'test_left_shoulder','test_left_elbow','test_left_wrist','test_left_roll','test_left_gripper','test_left_yaw_range','test_left_shoulder_range','test_left_elbow_range','test_left_wrist_range','test_left_roll_range','test_left_gripper_range','stop_active','stop_base'},names
                        assert not (await session.call_tool('robot_info',{})).isError
                        result=await session.call_tool('move_base',{'direction':'forward','seconds':9,'request_id':'44444444-4444-4444-8444-444444444444'})
                        assert result.isError
                        bad_calls = [
                            ('move_base', {'direction':'forward', 'seconds':0, 'request_id':'invalid'}),
                            ('move_base', {'direction':'forward', 'speed':.11, 'request_id':'invalid'}),
                            ('move_base', {'direction':'forward', 'turn_speed':21, 'request_id':'invalid'}),
                            ('move_base', {'direction':'forward', 'request_id':'invalid'}),
                            ('move_base', {'direction':'unknown', 'request_id':'invalid'}),
                            ('nudge_joint', {'group':'neck','servo_id':1,'delta_raw':16,'request_id':'invalid'}),
                            ('nudge_joint', {'group':'neck','servo_id':7,'delta_raw':33,'request_id':'invalid'}),
                            ('nudge_joint', {'group':'neck','servo_id':7,'delta_raw':0,'request_id':'invalid'}),
                            ('joint_status', {'group':'left-arm','servo_id':7}),
                            ('test_left_yaw', {'expected_positions':{'1':1903},'request_id':'invalid'}),
                            ('test_left_shoulder', {'expected_positions':{'2':1301},'request_id':'invalid'}),
                            ('test_left_elbow', {'expected_positions':{'3':1768},'request_id':'invalid'}),
                            ('test_left_wrist', {'expected_positions':{'4':2167},'request_id':'invalid'}),
                            ('test_left_roll', {'expected_positions':{'5':2168},'request_id':'invalid'}),
                            ('test_left_gripper', {'expected_positions':{'6':2017},'request_id':'invalid'}),
                            ('test_left_yaw_range', {'expected_positions':{'1':1906},'request_id':'invalid'}),
                            ('test_left_shoulder_range', {'expected_positions':{'2':1066},'request_id':'invalid'}),
                            ('test_left_elbow_range', {'expected_positions':{'3':2501},'request_id':'invalid'}),
                            ('test_left_wrist_range', {'expected_positions':{'4':2173},'request_id':'invalid'}),
                            ('test_left_roll_range', {'expected_positions':{'5':2170},'request_id':'invalid'}),
                            ('test_left_gripper_range', {'expected_positions':{'6':2018},'request_id':'invalid'}),
                        ]
                        for name, params in bad_calls:
                            assert (await session.call_tool(name,params)).isError, (name,params)
                        import uuid
                        hold={'action':'hold','expected_positions':{str(m):1900 for m in range(1,7)},'request_id':str(uuid.uuid4())}
                        result=await session.call_tool('right_arm_hold',hold)
                        assert not result.isError
                        assert 'right_arm_hold.py' in result.content[0].text
                        assert (await session.call_tool('right_arm_hold',hold)).isError
                        assert (await session.call_tool('right_arm_hold',{**hold,'expected_positions':{'1':1900},'request_id':str(uuid.uuid4())})).isError
                        right_yaw={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':str(uuid.uuid4())}
                        result=await session.call_tool('test_right_yaw',right_yaw)
                        assert not result.isError and 'right_yaw_test.py' in result.content[0].text
                        assert (await session.call_tool('test_right_yaw',right_yaw)).isError
                        assert (await session.call_tool('test_right_yaw',{**right_yaw,'expected_positions':{'1':1900},'request_id':str(uuid.uuid4())})).isError
                        for motor,joint in enumerate(('yaw','shoulder','elbow','wrist','roll','gripper'),1):
                            for clearance in (False,True):
                                if motor==1 and not clearance: continue
                                tool='test_right_'+joint+('_range' if clearance else '')
                                params={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':str(uuid.uuid4())}
                                result=await session.call_tool(tool,params)
                                assert not result.isError,tool
                                payload=json.loads(result.content[0].text)
                                assert payload['script']=='right_arm_test.py'
                                args=payload['arguments']
                                assert args[args.index('--id')+1]==str(motor)
                                assert ('--clearance' in args)==clearance
                                assert (await session.call_tool(tool,params)).isError
                                for positions in ({'1':1900},{str(m):5000 for m in range(1,7)}):
                                    assert (await session.call_tool(tool,{**params,'expected_positions':positions,'request_id':str(uuid.uuid4())})).isError
                        roll={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':str(uuid.uuid4()),'direction':'negative'}
                        result=await session.call_tool('test_right_roll',roll)
                        assert not result.isError
                        payload=json.loads(result.content[0].text)
                        assert payload['arguments'][-2:]==['--direction','negative']
                        assert (await session.call_tool('test_right_roll',roll)).isError
                        assert (await session.call_tool('test_right_roll',{**roll,'direction':'invalid','request_id':str(uuid.uuid4())})).isError
                        for gain in (16,24):
                            params={'request_id':str(uuid.uuid4()),'trial_p':gain,'expected_positions':{str(m):1900 for m in range(1,7)}}
                            result=await session.call_tool('test_right_elbow_p',params)
                            assert not result.isError
                            payload=json.loads(result.content[0].text)
                            assert payload['script']=='right_elbow_p_test.py'
                            assert payload['arguments'][payload['arguments'].index('--p')+1]==str(gain)
                            assert (await session.call_tool('test_right_elbow_p',params)).isError
                        assert (await session.call_tool('test_right_elbow_p',{'request_id':str(uuid.uuid4()),'trial_p':32})).isError
                        assert (await session.call_tool('test_right_elbow_p',{'request_id':str(uuid.uuid4()),'expected_positions':{'3':1800}})).isError
                        assert not (await session.call_tool('test_right_elbow_p',{'request_id':str(uuid.uuid4()),'action':'restore'})).isError
                        up_positions={str(m):1900 for m in range(1,7)}
                        result=await session.call_tool('preview_right_gripper_up',{'expected_positions':up_positions})
                        assert not result.isError
                        up={'expected_positions':up_positions,'plan_hash':'a'*64,'request_id':str(uuid.uuid4())}
                        result=await session.call_tool('move_right_gripper_up',up)
                        assert not result.isError
                        assert json.loads(result.content[0].text)['script']=='right_gripper_up.py'
                        assert (await session.call_tool('move_right_gripper_up',up)).isError
                        for bad in ({**up,'plan_hash':'invalid'},{**up,'expected_positions':{'3':1900}}):
                            assert (await session.call_tool('move_right_gripper_up',{**bad,'request_id':str(uuid.uuid4())})).isError
                        assert (await session.call_tool('preview_right_gripper_up',{'expected_positions':{'3':1900}})).isError
                        recover={'action':'recover-hold','expected_positions':{str(m):1900 for m in range(1,7)},'request_id':str(uuid.uuid4())}
                        result=await session.call_tool('right_arm_hold',recover)
                        assert not result.isError
                        assert json.loads(result.content[0].text)['arguments'][0]=='recover-hold'
                        assert (await session.call_tool('right_arm_hold',recover)).isError
                        assert (await session.call_tool('right_arm_hold',{'action':'recover-hold','request_id':str(uuid.uuid4())})).isError
                        left_recovery={'action':'recover-hold','expected_positions':{str(m):1900 for m in range(1,7)},'request_id':str(uuid.uuid4())}
                        result=await session.call_tool('left_arm_hold',left_recovery)
                        assert not result.isError
                        forwarded=json.loads(result.content[0].text)
                        assert forwarded['script']=='left_arm_hold.py' and forwarded['arguments'][0]=='recover-hold'
                        assert (await session.call_tool('left_arm_hold',left_recovery)).isError
                        assert (await session.call_tool('left_arm_hold',{'action':'recover-hold','request_id':str(uuid.uuid4())})).isError
                        leftup={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':str(uuid.uuid4())}
                        result=await session.call_tool('move_left_shoulder_up',leftup)
                        assert not result.isError and json.loads(result.content[0].text)['script']=='left_shoulder_up.py'
                        assert (await session.call_tool('move_left_shoulder_up',leftup)).isError
                        assert (await session.call_tool('move_left_shoulder_up',{'request_id':str(uuid.uuid4()),'expected_positions':{'2':1900}})).isError
                        left_positions={str(m):1900 for m in range(1,7)}
                        assert not (await session.call_tool('preview_left_gripper_up',{'expected_positions':left_positions})).isError
                        left_cart={'expected_positions':left_positions,'plan_hash':'b'*64,'request_id':str(uuid.uuid4())}
                        result=await session.call_tool('move_left_gripper_up',left_cart)
                        assert not result.isError and json.loads(result.content[0].text)['script']=='left_gripper_up.py'
                        assert (await session.call_tool('move_left_gripper_up',left_cart)).isError
                        assert (await session.call_tool('move_left_gripper_up',{**left_cart,'plan_hash':'invalid','request_id':str(uuid.uuid4())})).isError
                        assert (await session.call_tool('preview_left_gripper_up',{'expected_positions':{'2':1900}})).isError
                        result=await session.call_tool('preview_left_gripper_up',{'expected_positions':left_positions,'distance_mm':10})
                        assert not result.isError
                        forwarded=json.loads(result.content[0].text)['arguments']
                        assert forwarded[forwarded.index('--distance-mm')+1]=='10.0'
                        result=await session.call_tool('move_left_gripper_up',{**left_cart,'distance_mm':10,'request_id':str(uuid.uuid4())})
                        assert not result.isError
                        forwarded=json.loads(result.content[0].text)['arguments']
                        assert forwarded[forwarded.index('--distance-mm')+1]=='10.0'
                        for distance in (1,10.1):
                            assert (await session.call_tool('preview_left_gripper_up',{'expected_positions':left_positions,'distance_mm':distance})).isError
                            assert (await session.call_tool('move_left_gripper_up',{**left_cart,'distance_mm':distance,'request_id':str(uuid.uuid4())})).isError
                        for tool in ('preview_left_gripper_up','move_left_gripper_up'):
                            args_coarse={'expected_positions':left_positions,'coarse':True,'distance_mm':20}
                            if tool.startswith('move_'):args_coarse.update(plan_hash='c'*64,request_id=str(uuid.uuid4()))
                            result=await session.call_tool(tool,args_coarse)
                            assert not result.isError
                            forwarded=json.loads(result.content[0].text)['arguments']
                            assert '--coarse' in forwarded and forwarded[forwarded.index('--distance-mm')+1]=='20.0'
                            for mm in (9.9,20.1):
                                invalid={**args_coarse,'distance_mm':mm}
                                if tool.startswith('move_'):invalid['request_id']=str(uuid.uuid4())
                                assert (await session.call_tool(tool,invalid)).isError
                        args={'direction':'forward' ,'request_id':'55555555-5555-4555-8555-555555555555'}
                        result=await session.call_tool('move_base',args)
                        assert not result.isError and '"mock": true' in result.content[0].text
                        assert (await session.call_tool('move_base',args)).isError
                        yaw={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'66666666-6666-4666-8666-666666666666'}
                        assert not (await session.call_tool('test_left_yaw',yaw)).isError
                        assert (await session.call_tool('test_left_yaw',yaw)).isError
                        shoulder={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'77777777-7777-4777-8777-777777777777'}
                        assert not (await session.call_tool('test_left_shoulder',shoulder)).isError
                        assert (await session.call_tool('test_left_shoulder',shoulder)).isError
                        elbow={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'88888888-8888-4888-8888-888888888888'}
                        assert not (await session.call_tool('test_left_elbow',elbow)).isError
                        assert (await session.call_tool('test_left_elbow',elbow)).isError
                        wrist={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'99999999-9999-4999-8999-999999999999'}
                        assert not (await session.call_tool('test_left_wrist',wrist)).isError
                        assert (await session.call_tool('test_left_wrist',wrist)).isError
                        roll={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'}
                        assert not (await session.call_tool('test_left_roll',roll)).isError
                        assert (await session.call_tool('test_left_roll',roll)).isError
                        gripper={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb'}
                        assert not (await session.call_tool('test_left_gripper',gripper)).isError
                        assert (await session.call_tool('test_left_gripper',gripper)).isError
                        yaw_range={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'cccccccc-cccc-4ccc-8ccc-cccccccccccc'}
                        assert not (await session.call_tool('test_left_yaw_range',yaw_range)).isError
                        assert (await session.call_tool('test_left_yaw_range',yaw_range)).isError
                        shoulder_range={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'dddddddd-dddd-4ddd-8ddd-dddddddddddd'}
                        assert not (await session.call_tool('test_left_shoulder_range',shoulder_range)).isError
                        assert (await session.call_tool('test_left_shoulder_range',shoulder_range)).isError
                        elbow_range={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'eeeeeeee-eeee-4eee-8eee-eeeeeeeeeeee'}
                        assert not (await session.call_tool('test_left_elbow_range',elbow_range)).isError
                        assert (await session.call_tool('test_left_elbow_range',elbow_range)).isError
                        wrist_range={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'ffffffff-ffff-4fff-8fff-ffffffffffff'}
                        assert not (await session.call_tool('test_left_wrist_range',wrist_range)).isError
                        assert (await session.call_tool('test_left_wrist_range',wrist_range)).isError
                        roll_range={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'12345678-1234-4234-8234-123456789abc'}
                        assert not (await session.call_tool('test_left_roll_range',roll_range)).isError
                        assert (await session.call_tool('test_left_roll_range',roll_range)).isError
                        gripper_range={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'87654321-4321-4321-8321-cba987654321'}
                        assert not (await session.call_tool('test_left_gripper_range',gripper_range)).isError
                        assert (await session.call_tool('test_left_gripper_range',gripper_range)).isError
                        reverse_elbow={'expected_positions':{str(m):1900 for m in range(1,7)},'request_id':'13243546-1324-4324-8324-132435465768','direction':'negative'}
                        assert not (await session.call_tool('test_left_elbow',reverse_elbow)).isError
                        assert (await session.call_tool('test_left_elbow',reverse_elbow)).isError
                        assert (await session.call_tool('test_left_elbow',{**reverse_elbow,'direction':'invalid'})).isError
                        result=await session.call_tool('test_left_elbow',{**reverse_elbow,'direction':'positive','amplitude_deg':5,'slow':True,'request_id':str(uuid.uuid4())})
                        assert not result.isError
                        forwarded=json.loads(result.content[0].text)['arguments']
                        assert '--slow' in forwarded and forwarded[forwarded.index('--amplitude-deg')+1]=='5.0'
                        for amplitude in (1,8):
                            assert (await session.call_tool('test_left_elbow',{**reverse_elbow,'amplitude_deg':amplitude,'request_id':str(uuid.uuid4())})).isError

                        assert not (await session.call_tool('camera_snapshot',{})).isError
                        assert not (await session.call_tool('stop_active',{})).isError
            print('PASS: real HTTP MCP handshake, 44 tools, bearer auth, browser-origin rejection, bounds, replay, mock capture/stop')
        finally:
            p.terminate(); p.wait(timeout=10)

async def main(): await unit(); await integration()
if __name__=='__main__': asyncio.run(main())
