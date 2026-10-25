"""No hardware: inward-only startup recovery without widening limits."""
import copy,sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import right_arm_hold as rh

class Recovery(unittest.TestCase):
    def setUp(self):
        self.states={str(m):{'model':777,'min':500,'max':3500,'mode':0,'torque':0,'goal':0,'position':1800+m*20,'moving':0,'voltage_present_raw':120,'temperature_raw':30} for m in range(1,7)}
        self.states['3']['position']=3502
        self.group={'port_verified':True,'joints':{str(m):{'safe_raw_limits':[500,3500],'observed_raw_limits':[500,3500]} for m in range(1,7)}}
        self.expected={m:s['position'] for m,s in self.states.items()};self.events=[];self.now=0.;self.problem=None
        parent=self
        class SDK:COMM_SUCCESS=0
        self.sdk=SDK()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                target=data[0]+256*data[1]
                assert addr==42 and length==6 and data[2:4]==[0,0]
                assert parent.group['joints'][str(motor)]['safe_raw_limits'][0]<=target<=parent.group['joints'][str(motor)]['safe_raw_limits'][1]
                if motor==3 and parent.expected['3']>3500:assert target==3484 and data[4:]==[30,0]
                elif motor==3 and parent.expected['3']<500:assert target==516 and data[4:]==[30,0]
                else:assert target==parent.states[str(motor)]['position'] and data[4:]==[100,0]
                parent.events.append((motor,target));parent.states[str(motor)]['goal']=target;parent.states[str(motor)]['torque']=1
                if motor==3:
                    if parent.problem=='ack':raise RuntimeError('lost ack after activation')
                    if parent.problem=='interrupt':raise KeyboardInterrupt()
                    if parent.problem=='power':parent.states['3']['voltage_present_raw']=100
                    if parent.problem=='other':parent.states['4']['position']+=9
                return 0,0
            def write1ByteTxRx(self,ph,motor,addr,value):
                assert addr==40 and value==1;parent.states[str(motor)]['torque']=1;return 0,0
        self.pk=PK()
    def read(self,sdk,ph,pk,motor,key):
        s=self.states[str(motor)]
        if motor==3 and key=='position' and s['torque']==1 and self.problem!='stuck':s['position']+=max(-2,min(2,s['goal']-s['position']))
        return s.get(key,0)
    def clock(self):self.now+=.05;return self.now
    def run_hold(self,action='recover-hold'):
        with patch.object(rh.jc,'read',self.read),patch.object(rh.time,'sleep'),patch.object(rh.time,'monotonic',self.clock):
            return rh.control(self.sdk,None,self.pk,self.group,action,self.expected)
    def test_near_upper_limit_recovers_inward_and_holds(self):
        before=copy.deepcopy(self.group);r=self.run_hold()
        self.assertTrue(r['completed']);self.assertEqual(self.events,[(1,1820),(2,1840),(3,3484),(4,1880),(5,1900),(6,1920)])
        self.assertTrue(all(s['torque']==1 for s in self.states.values()));self.assertLessEqual(abs(self.states['3']['position']-3484),4)
        self.assertEqual(self.group,before)
    def test_near_lower_limit_recovers_only_inward(self):
        self.states['3']['position']=498;self.expected['3']=498
        self.assertTrue(self.run_hold()['completed']);self.assertIn((3,516),self.events)
    def test_max16tick_outside_gives_max32tick_move(self):
        for start in (484,3516):
            self.setUp();self.states['3']['position']=start;self.expected['3']=start
            self.assertTrue(self.run_hold()['completed']);target=next(t for m,t in self.events if m==3);self.assertEqual(abs(target-start),32)
    def test_more_than16ticks_outside_rejects_without_writes(self):
        for start in (483,3517):
            self.setUp();self.states['3']['position']=start;self.expected['3']=start
            with self.assertRaises(ValueError):self.run_hold()
            self.assertEqual(self.events,[])
    def test_normal_hold_still_rejects_out_of_range(self):
        with self.assertRaises(ValueError):self.run_hold('hold')
        self.assertEqual(self.events,[])
    def test_other_joint_outside_range_blocks(self):
        self.states['4']['position']=3501;self.expected['4']=3501
        with self.assertRaises(ValueError):self.run_hold()
        self.assertEqual(self.events,[])
    def test_fault_power_calibration_or_torque_blocks(self):
        for key,value in [('mode',1),('model',1),('moving',1),('torque',1),('voltage_present_raw',109),('temperature_raw',60),('max',3499)]:
            self.setUp();self.states['3'][key]=value
            with self.assertRaises(ValueError):self.run_hold()
            self.assertEqual(self.events,[])
    def test_stale_pose_blocks(self):
        self.expected['3']+=9
        with self.assertRaises(ValueError):self.run_hold()
        self.assertEqual(self.events,[])
    def test_recovery_failure_does_not_activate_remaining_joints_or_release(self):
        for issue in ('stuck','ack','interrupt','power','other'):
            self.setUp();self.problem=issue
            with self.assertRaises(BaseException):self.run_hold()
            self.assertEqual([m for m,t in self.events],[1,2,3]);self.assertEqual(self.states['3']['goal'],3484)
            self.assertTrue(all(self.states[str(m)]['torque']==0 for m in (4,5,6)))
    def test_already_in_range_just_holds_current_positions(self):
        self.states['3']['position']=2000;self.expected['3']=2000
        r=self.run_hold();self.assertTrue(r['completed']);self.assertIn((3,2000),self.events)
if __name__=='__main__':unittest.main()
