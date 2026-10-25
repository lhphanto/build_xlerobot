"""No hardware: durable replay, lock/stop races, restart, and commissioning gates."""
import asyncio
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import uuid
import server

class Controls(unittest.IsolatedAsyncioTestCase):
    async def test_replay_survives_restart_and_uuid_alias(self):
        with tempfile.TemporaryDirectory() as tmp:
            journal=Path(tmp)/'requests.sqlite3'
            rid=str(uuid.uuid4())
            first=server.Controller(True,True,journal)
            await first.run('fake.py',[],True,rid)
            first.journal.close()
            second=server.Controller(True,True,journal)
            with self.assertRaisesRegex(ValueError,'Duplicate'):
                await second.run('fake.py',[],True,rid.upper().replace('-',''))
            second.journal.close()

    async def test_failed_spawn_consumes_request(self):
        c=server.Controller(False,True)
        rid=str(uuid.uuid4())
        with patch.object(asyncio,'create_subprocess_exec',side_effect=OSError('test failure')):
            with self.assertRaises(OSError): await c.run('fake.py',[],True,rid)
        self.assertFalse(c.lock.locked())
        with self.assertRaisesRegex(ValueError,'Duplicate'):
            await c.run('fake.py',[],True,rid)

    async def test_busy_rejected_and_stop_during_spawn(self):
        c=server.Controller(False,True)
        spawned=asyncio.Event(); release=asyncio.Event()
        class Child:
            returncode=None
            def __init__(self): self.done=asyncio.Event();self.signals=[]
            def send_signal(self,sig): self.signals.append(sig);self.returncode=1;self.done.set()
            async def communicate(self): await self.done.wait();return b'',b'interrupted'
        child=Child()
        async def spawn(*args,**kwargs):
            spawned.set(); await release.wait(); return child
        with patch.object(asyncio,'create_subprocess_exec',spawn):
            run=asyncio.create_task(c.run('fake.py',[],True,str(uuid.uuid4())))
            await spawned.wait()
            with self.assertRaisesRegex(ValueError,'busy'):
                await c.run('fake.py',[],True,str(uuid.uuid4()))
            stop=asyncio.create_task(c.stop());await asyncio.sleep(0)
            with self.assertRaisesRegex(ValueError,'busy'):
                await c.run('fake.py',[])
            release.set()
            with self.assertRaises(ValueError):await run
            self.assertTrue((await stop)['interrupted'])
            self.assertEqual(len(child.signals),1)
            self.assertFalse(c.lock.locked())

    async def test_motion_off_after_controller_restart(self):
        c=server.Controller(mock=True)
        with self.assertRaisesRegex(ValueError,'Motion disabled'):
            await c.run('fake.py',[],True,str(uuid.uuid4()))
        self.assertFalse((await c.stop())['interrupted'])

    async def test_pending_p_recovery_blocks_other_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'.private').mkdir();(root/'.private/right-elbow-p-recovery.json').write_text('{}')
            with patch.object(server,'ROOT',root),patch.object(server,'code_digest',return_value='same'):
                c=server.Controller(True,True)
                for script,args in [('base_control.py',[]),('right_arm_test.py',[]),('right_arm_hold.py',['hold']),('right_elbow_p_test.py',['--action','trial'])]:
                    with self.assertRaisesRegex(ValueError,'restoration'):
                        await c.run(script,args,True,str(uuid.uuid4()))
                self.assertTrue((await c.run('right_elbow_p_test.py',['--action','restore'],True,str(uuid.uuid4())))['mock'])
                self.assertTrue((await c.run('right_arm_hold.py',['release'],True,str(uuid.uuid4())))['mock'])
                self.assertTrue((await c.run('joint_control.py',[],False))['mock'])
                c.journal.close()

    async def test_code_changed_rejects_motion(self):
        c=server.Controller(True,True)
        with patch.object(server,'code_digest',return_value='changed'):
            with self.assertRaisesRegex(ValueError,'restart'):
                await c.run('fake.py',[],True,str(uuid.uuid4()))

    async def test_joint_commissioning_not_bypassed(self):
        path=server.ROOT/'controllers/joint_control.py'
        spec=importlib.util.spec_from_file_location('jc',path)
        jc=importlib.util.module_from_spec(spec);spec.loader.exec_module(jc)
        group={'port_verified':True}
        joint={'commissioned':False,'safe_raw_limits':[1000,3000],
               'supervised_test':{'remaining':1,'delta_raw':32,'expires_unix':9999999999}}
        state={'model':777,'mode':0,'torque':1,'moving':0,'goal':2000,'position':2000,'min':1000,'max':3000}
        with self.assertRaisesRegex(ValueError,'commissioned'):
            jc.validate_target(group,joint,state,32)
        joint['commissioned']=True
        self.assertEqual(jc.validate_target(group,joint,state,32),2032)
        state['torque']=0
        with self.assertRaisesRegex(ValueError,'torque'):
            jc.validate_target(group,joint,state,32)

if __name__=='__main__':unittest.main(verbosity=2)
