"""No hardware: fixed P trial, protected-write recovery and failure paths."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import right_elbow_p_test as pt
import test_right_arm_suite as fixtures

class ElbowP(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.RightArmSuite();self.f.setUp();self.f.motor=3
        self.f.group['port']='/dev/test-right'
        self.regs={21:16,22:32,23:0,55:1};self.writes=[];self.fail=None;self.once=True
        parent=self;base=type(self.f.pk)
        class PK(base):
            def read1ByteTxRx(self,ph,motor,addr):
                if addr in parent.regs:
                    assert motor==3
                    return parent.regs[addr],0,0
                return super().read1ByteTxRx(ph,motor,addr)
            def write1ByteTxRx(self,ph,motor,addr,value):
                assert motor==3 and (addr,value) in ((21,16),(21,24),(21,32),(55,0),(55,1))
                if addr==21:assert parent.regs[55]==0
                parent.writes.append((motor,addr,value))
                match=parent.fail==(addr,value)
                if not match or not parent.once or parent.failure_mode=='after':parent.regs[addr]=value
                if match and parent.once:
                    if parent.failure_mode!='always':parent.once=False
                    raise RuntimeError('lost register ack')
                return 0,0
        self.f.pk=PK();self.failure_mode='before';self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)/'pending.json'
        self.addCleanup(self.tmp.cleanup)
    def run_trial(self,p=24,action='trial'):
        with patch.object(pt.rt.jc,'read',self.f.read),patch.object(pt.rt.time,'sleep'),patch.object(pt.rt.time,'monotonic',self.f.clock),contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            return pt.control(self.f.sdk,None,self.f.pk,self.f.group,self.f.expected,p,self.path,action)
    def assert_restored(self):
        self.assertEqual(self.regs[21],16);self.assertEqual(self.regs[55],1);self.assertFalse(self.path.exists())
    def test_p16_baseline_has_no_protected_writes(self):
        result=self.run_trial(16)
        self.assertTrue(result['completed']);self.assertEqual(self.writes,[])
        start=self.f.expected['3'];self.assertEqual(self.f.events,[(3,start+32),(3,start)])
    def test_p24_trial_restores_gain_and_lock(self):
        result=self.run_trial()
        self.assertTrue(result['completed']);self.assertTrue(result['restoration']['verified']);self.assert_restored()
        self.assertEqual([(a,v) for m,a,v in self.writes],[(55,0),(21,24),(55,1),(55,0),(21,16),(55,1)])
        self.assertEqual(self.regs[22],32);self.assertEqual(self.regs[23],0)
    def test_durable_record_precedes_unlock(self):
        original=self.f.pk.write1ByteTxRx
        def write(ph,motor,addr,value):
            self.assertTrue(self.path.exists())
            data=json.loads(self.path.read_text());self.assertEqual(data['original_p'],16)
            return original(ph,motor,addr,value)
        self.f.pk.write1ByteTxRx=write
        self.run_trial();self.assert_restored()
    def test_invalid_original_settings_send_no_writes(self):
        for addr,value in [(21,32),(22,16),(23,1),(55,0)]:
            with self.subTest(addr=addr):
                self.regs={21:16,22:32,23:0,55:1};self.regs[addr]=value
                with self.assertRaises(ValueError):self.run_trial()
                self.assertEqual(self.writes,[]);self.assertEqual(self.f.events,[])
    def test_outstanding_target_rejects_before_gain_write(self):
        self.f.states['3']['goal']+=9
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.writes,[])
    def test_p32_trial_restores_gain_and_lock(self):
        result=self.run_trial(32)
        self.assertTrue(result['completed']);self.assertTrue(result['restoration']['verified']);self.assert_restored()
        self.assertEqual([(a,v) for m,a,v in self.writes],[(55,0),(21,32),(55,1),(55,0),(21,16),(55,1)])
    def test_measured_hands_off_sag_may_start_gain_trial(self):
        self.f.states['3']['goal']-=7  # Position 7 ticks past goal, as measured November2.
        result=self.run_trial()
        self.assertEqual(self.writes[:3],[(3,55,0),(3,21,24),(3,55,1)]);self.assert_restored()
        self.assertTrue(result['restoration']['verified'])
    def test_torque_off_rejects_before_gain_write(self):
        self.f.states['3']['torque']=0
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.writes,[])
    def test_pending_record_blocks_trial(self):
        pt.write_record(self.path,self.f.group['port'],24)
        with self.assertRaisesRegex(ValueError,'Pending'):self.run_trial()
        self.assertEqual(self.writes,[])
    def test_p24_write_failure_restores_without_motion(self):
        for mode in ('before','after'):
            with self.subTest(mode=mode):
                self.fail=(21,24);self.once=True;self.failure_mode=mode
                result=self.run_trial()
                self.assertFalse(result['completed']);self.assertEqual(self.f.events,[]);self.assert_restored()
    def test_unlock_failure_sends_no_motion(self):
        self.fail=(55,0)
        result=self.run_trial();self.assertFalse(result['completed']);self.assertEqual(self.f.events,[]);self.assert_restored()
    def test_motion_interrupt_restores_p_without_retry(self):
        self.f.problem='interrupt'
        result=self.run_trial();self.assertFalse(result['completed']);self.assert_restored()
        self.assertLessEqual(len(self.f.events),2)
    def test_motion_timeout_restores_p(self):
        self.f.problem='stuck'
        result=self.run_trial();self.assertFalse(result['completed']);self.assert_restored()
        self.assertIn('trial_failure',result)
    def test_fault_during_motion_restores_p(self):
        self.f.problem='fault'
        result=self.run_trial();self.assertFalse(result['completed']);self.assert_restored()
    def test_failed_restoration_preserves_recovery_record(self):
        self.fail=(21,16);self.failure_mode='always'
        result=self.run_trial();self.assertFalse(result['completed']);self.assertFalse(result['restoration']['verified']);self.assertTrue(self.path.exists())
        self.assertEqual(self.regs[55],1)
        self.fail=None
        result=self.run_trial(action='restore');self.assertTrue(result['completed']);self.assert_restored()
    def test_lost_restore_ack_readback_can_confirm_restoration(self):
        self.fail=(21,16);self.failure_mode='after'
        result=self.run_trial();self.assertTrue(result['restoration']['verified']);self.assert_restored()
    def test_restore_only_never_commands_targets(self):
        pt.write_record(self.path,self.f.group['port'],24);self.regs[21]=24
        for s in self.f.states.values():s['torque']=0
        result=self.run_trial(action='restore');self.assertTrue(result['completed']);self.assertEqual(self.f.events,[]);self.assert_restored()
    def test_recovery_wrong_bus_rejected(self):
        pt.write_record(self.path,'wrong-port',24)
        with self.assertRaises(ValueError):self.run_trial(action='restore')
        self.assertEqual(self.writes,[])
    def test_p32_record_restores_after_interrupted_trial(self):
        pt.write_record(self.path,self.f.group['port'],32);self.regs[21]=32
        result=self.run_trial(action='restore')
        self.assertTrue(result['completed']);self.assert_restored()
    def test_no_free_gain_or_register(self):
        with self.assertRaises(ValueError):self.run_trial(40)
        with self.assertRaises(ValueError):pt.write_byte(self.f.sdk,None,self.f.pk,23,1)
        self.assertEqual(self.writes,[])

if __name__=='__main__':unittest.main()
