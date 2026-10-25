"""No hardware: all right-arm test paths, one-joint writes and failure gates."""
import contextlib
import copy
import io
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import right_arm_test as rt

class RightArmSuite(unittest.TestCase):
    def setUp(self):
        self.states={str(m):{'model':777,'min':500,'max':3500,'mode':0,'torque':1,'goal':1800+m*20,'position':1800+m*20,'moving':0,'voltage_present_raw':120,'temperature_raw':30} for m in range(1,7)}
        self.group={'port_verified':True,'joints':{str(m):{'safe_raw_limits':[500,3500],'observed_raw_limits':[500,3500]} for m in range(1,7)}}
        self.expected={m:s['position'] for m,s in self.states.items()}
        self.events=[];self.now=0.;self.motor=1;self.problem=None;self.residual=0;self.direction=1
        parent=self
        class SDK:COMM_SUCCESS=0
        self.sdk=SDK()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                assert motor==parent.motor and addr==42 and length==6 and data[2:]==[0,0,100,0]
                target=data[0]+256*data[1]
                parent.events.append((motor,target));s=parent.states[str(motor)]
                parent.direction=1 if target>=s['position'] else -1
                s['goal']=target
                if len(parent.events)==1:
                    if parent.problem=='ack':raise RuntimeError('lost ack')
                    if parent.problem=='interrupt':raise KeyboardInterrupt()
                    if parent.problem=='other':parent.states[str(1 if motor!=1 else 2)]['position']+=9
                    if parent.problem=='power':s['voltage_present_raw']=100
                    if parent.problem=='goal':s['goal']+=1
                    if parent.problem=='mode':s['mode']=1
                    if parent.problem=='limits':s['max']-=1
                    if parent.problem=='temperature':s['temperature_raw']=60
                return 0,0
            def diagnostic(self,motor,addr):
                assert motor==parent.motor
                if parent.problem=='diag-comm':return 0,1,0
                values={58:0,60:50,65:1 if parent.problem=='fault' else 0,69:5}
                if parent.problem=='moving':values[58]=100
                return values[addr],0,0
            def read1ByteTxRx(self,ph,motor,addr):return self.diagnostic(motor,addr)
            def read2ByteTxRx(self,ph,motor,addr):return self.diagnostic(motor,addr)
        self.pk=PK()
    def read(self,sdk,ph,pk,motor,key):
        if self.problem=='read' and self.events:raise RuntimeError('unavailable feedback')
        s=self.states[str(motor)]
        if motor==self.motor and key=='position' and self.events and self.problem!='stuck':
            target=s['goal']-self.direction*self.residual
            s['position']+=max(-10,min(10,target-s['position']))
        if motor==self.motor and key=='moving' and self.problem=='moving' and self.events:return 1
        return s.get(key,0)
    def clock(self):self.now+=.05;return self.now
    def run_trial(self,motor=1,clearance=False,direction='positive'):
        self.motor=motor
        with patch.object(rt.jc,'read',self.read),patch.object(rt.time,'sleep'),patch.object(rt.time,'monotonic',self.clock),contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            return rt.control(self.sdk,None,self.pk,self.group,self.expected,motor,clearance,direction)
    def test_all_six_small_and_clearance_write_only_selected_joint(self):
        for motor in range(1,7):
            for clearance in (False,True):
                with self.subTest(motor=motor,clearance=clearance):
                    self.setUp();start=self.expected[str(motor)];result=self.run_trial(motor,clearance)
                    delta=80 if clearance or motor==1 else 32
                    targets=[start+80,start,start-80,start] if clearance else [start+delta,start]
                    self.assertEqual(self.events,[(motor,t) for t in targets])
                    self.assertTrue(result['completed']);self.assertEqual(result['tolerance_raw'],6 if motor==5 else 4)
                    self.assertEqual(self.states[str(motor)]['position'],start)
    def test_all_six_negative_small_trial(self):
        for motor in range(1,7):
            with self.subTest(motor=motor):
                self.setUp();start=self.expected[str(motor)];self.run_trial(motor,direction='negative')
                delta=80 if motor==1 else 32
                self.assertEqual(self.events,[(motor,start-delta),(motor,start)])
    def test_preflight_rejects_without_writes(self):
        changes=[('model',1),('mode',1),('torque',0),('moving',1),('voltage_present_raw',109),('temperature_raw',60),('max',3499),('position',400),('goal',2500)]
        for key,value in changes:
            with self.subTest(key=key):
                self.setUp();self.states['4'][key]=value
                with self.assertRaises(ValueError):self.run_trial(2)
                self.assertEqual(self.events,[])
    def test_invalid_expected_or_id(self):
        for motor,expected in [(7,self.expected),(2,{'1':1800}),(2,{**self.expected,'2':True}),(2,{**self.expected,'2':5000})]:
            with self.subTest(motor=motor,expected=expected):
                self.expected=expected
                with self.assertRaises(ValueError):self.run_trial(motor)
                self.assertEqual(self.events,[])
    def test_unverified_port_and_stale_pose_reject(self):
        self.group['port_verified']=False
        with self.assertRaises(ValueError):self.run_trial(2)
        self.group['port_verified']=True;self.expected['2']+=5
        with self.assertRaises(ValueError):self.run_trial(2)
        self.assertEqual(self.events,[])
    def test_targets_must_fit_saved_limits(self):
        for clearance in (False,True):
            self.setUp();self.group['joints']['2']['safe_raw_limits']=[1800,1850]
            with self.assertRaises(ValueError):self.run_trial(2,clearance)
            self.assertEqual(self.events,[])
    def test_id5_null_monitored_without_config_mutation(self):
        self.group['joints']['5']={'safe_raw_limits':None,'observed_raw_limits':[0,4095]}
        self.states['5']['min']=0;self.states['5']['max']=4095
        original=copy.deepcopy(self.group)
        self.assertTrue(self.run_trial(3)['completed'])
        self.assertEqual(self.group,original)
    def test_id5_travel_blocked_without_approved_range(self):
        for clearance in (False,True):
            self.setUp();self.group['joints']['5']['safe_raw_limits']=None
            with self.assertRaisesRegex(ValueError,'ID5 travel blocked'):self.run_trial(5,clearance)
            self.assertEqual(self.events,[])
    def test_id5_calibration_change_blocks_other_trials(self):
        self.group['joints']['5']={'safe_raw_limits':None,'observed_raw_limits':[1,4095]}
        with self.assertRaises(ValueError):self.run_trial(2)
        self.assertEqual(self.events,[])
    def test_failures_stop_without_next_leg_or_other_joint_writes(self):
        for problem in ('ack','interrupt','other','power','goal','mode','limits','temperature','diag-comm','fault','read','moving','stuck'):
            with self.subTest(problem=problem):
                self.setUp();self.problem=problem
                with self.assertRaises(BaseException):self.run_trial(4,True)
                self.assertEqual(self.events[0],(4,self.expected['4']+80))
                self.assertLessEqual(len(self.events),2)
                self.assertTrue(all(m==4 for m,t in self.events))
    def test_stationary_error_is_failure_without_retarget_twitch(self):
        for motor in range(1,7):
            with self.subTest(motor=motor):
                self.setUp();self.residual=7
                with self.assertRaises(rt.StationarySettlingTimeout):self.run_trial(motor)
                self.assertEqual(len(self.events),1)
    def test_center_returns_reject_errors_above_joint_tolerance(self):
        # Arrive exactly at outward target, then stop seven ticks short on return.
        for motor in range(1,7):
            with self.subTest(motor=motor):
                self.setUp();original=self.pk.writeTxRx
                def write(*args):
                    result=original(*args)
                    if len(self.events)==2:self.residual=7
                    return result
                self.pk.writeTxRx=write
                with self.assertRaises(rt.StationarySettlingTimeout):self.run_trial(motor)
                self.assertEqual(len(self.events),2)
    def test_only_roll_accepts_five_or_six_tick_error(self):
        for motor in range(1,7):
            for error in (5,6):
                with self.subTest(motor=motor,error=error):
                    self.setUp();self.residual=error
                    if motor==5:
                        result=self.run_trial(motor)
                        self.assertTrue(result['completed'])
                        self.assertEqual(result['tolerance_raw'],6)
                        self.assertTrue(all(x['tolerance_raw']==6 for x in result['stages']))
                    else:
                        with self.assertRaises(rt.StationarySettlingTimeout):self.run_trial(motor)
    def test_roll_negative_fits_actual_approved_window(self):
        self.states['5'].update(min=0,max=4095,position=1995,goal=2000)
        self.expected['5']=1995
        self.group['joints']['5']={'safe_raw_limits':[1928,2008],'observed_raw_limits':[0,4095]}
        with self.assertRaises(ValueError):self.run_trial(5)
        self.assertEqual(self.events,[])
        result=self.run_trial(5,direction='negative')
        self.assertTrue(result['completed'])
        self.assertEqual(self.events,[(5,1963),(5,1995)])
    def test_cli_uses_right_group_only(self):
        source=Path(rt.__file__).read_text()
        self.assertIn("['groups']['right-arm']",source)
        self.assertNotIn("['groups']['left-arm']",source)
if __name__=='__main__':unittest.main()
