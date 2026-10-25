"""No hardware: one-way slow lift, existing gates and interruption cleanup."""
import contextlib,io,sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import left_shoulder_up as left
import test_right_arm_suite as fixtures

class LeftUp(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.RightArmSuite();self.f.setUp();self.f.motor=2;self.packets=[]
        old=self.f.pk.writeTxRx
        def slow(ph,motor,addr,length,data):
            assert motor==2 and data[4:]==[30,0]
            self.packets.append((motor,data.copy()))
            return old(ph,motor,addr,length,[*data[:4],100,0])
        self.f.pk.writeTxRx=slow
    def run_move(self):
        with patch.object(left.core.jc,'read',self.f.read),patch.object(left.time,'sleep'),patch.object(left.time,'monotonic',self.f.clock),contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            return left.control(self.f.sdk,None,self.f.pk,self.f.group,self.f.expected)
    def test_single_slow_up_then_hold_without_return(self):
        start=self.f.expected['2'];r=self.run_move()
        self.assertTrue(r['completed']);self.assertEqual(self.f.events,[(2,start+32)])
        self.assertFalse(r['return_to_start']);self.assertEqual(r['speed_raw'],30)
        self.assertEqual(self.f.states['2']['goal'],start+32)
    def test_other_joint_loaded_offset_within_existing_bound_allowed(self):
        self.f.states['3']['goal']-=5
        self.assertTrue(self.run_move()['completed'])
    def test_no_torque_enable_or_range_bypass(self):
        for key,value in [('torque',0),('mode',1),('model',1),('moving',1),('position',400),('voltage_present_raw',100),('temperature_raw',60)]:
            self.setUp();self.f.states['2'][key]=value
            with self.assertRaises(ValueError):self.run_move()
            self.assertEqual(self.packets,[])
    def test_upper_limit_rejects_before_writes(self):
        self.f.group['joints']['2']['safe_raw_limits'][1]=self.f.expected['2']+31
        with self.assertRaises(ValueError):self.run_move()
        self.assertEqual(self.packets,[])
    def test_failed_ack_interrupt_power_and_other_drift_do_not_return(self):
        for problem in ('ack','interrupt','power','other','fault','read','stuck'):
            self.setUp();self.f.problem=problem
            with self.assertRaises(BaseException):self.run_move()
            self.assertLessEqual(len(self.packets),2)
            self.assertTrue(all(m==2 for m,p in self.packets))
    def test_stationary_small_error_keeps_target_without_twitch(self):
        self.f.residual=7
        with self.assertRaises(left.core.StationarySettlingTimeout):self.run_move()
        self.assertEqual(len(self.packets),1)
    def test_fixed_left_group(self):
        source=Path(left.__file__).read_text()
        self.assertIn("['groups']['left-arm']",source)
        self.assertNotIn("['groups']['right-arm']",source)
if __name__=='__main__':unittest.main()
