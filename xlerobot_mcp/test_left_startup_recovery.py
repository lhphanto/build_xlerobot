"""Mock serial only: explicit inward startup without limit/configuration writes."""
import copy,sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import left_arm_hold as left
import right_arm_hold as shared

class LeftStartupRecovery(unittest.TestCase):
    def setUp(self):
        self.states={str(m):{'model':777,'min':500,'max':3500,'mode':0,'torque':0,'goal':0,'position':1800+m*20,'moving':0,'voltage_present_raw':120,'temperature_raw':30} for m in range(1,7)}
        self.states['3']['position']=3520
        self.group={'port_verified':True,'joints':{str(m):{'safe_raw_limits':[500,3500],'observed_raw_limits':[500,3500]} for m in range(1,7)}}
        self.expected={m:s['position'] for m,s in self.states.items()};self.events=[];self.now=0.;self.problem=None;self.settle_error=0;self.handoff_error=0;self.elbow_writes=0;self.auto_torque=True
        parent=self
        class SDK:COMM_SUCCESS=0
        self.sdk=SDK()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                target=data[0]+256*data[1]
                assert motor in range(1,7) and addr==42 and length==6 and data[2:4]==[0,0]
                lo,hi=parent.group['joints'][str(motor)]['safe_raw_limits']
                assert lo<=target<=hi  # NO out-of-range hold/preload writes.
                if motor==3:
                    parent.elbow_writes+=1
                    if parent.elbow_writes==1 and not lo<=parent.expected['3']<=hi:
                        assert target==(hi-32 if parent.expected['3']>hi else lo+32)
                        assert abs(target-parent.expected['3'])<=64 and data[4:]==[30,0]
                    elif parent.elbow_writes>1:
                        assert target==parent.states['3']['position'] and data[4:]==[30,0]
                else:assert target==parent.states[str(motor)]['position'] and data[4:]==[100,0]
                parent.events.append((motor,target));parent.states[str(motor)]['goal']=target
                if parent.auto_torque:parent.states[str(motor)]['torque']=1
                if motor==3:
                    if parent.problem=='ack':raise RuntimeError('lost ack')
                    if parent.problem=='interrupt':raise KeyboardInterrupt()
                    if parent.problem=='handoff_ack' and parent.elbow_writes==2:raise RuntimeError('handoff lost ack')
                    if parent.problem=='power':parent.states['3']['voltage_present_raw']=100
                    if parent.problem=='other':parent.states['4']['position']+=9
                return 0,0
            def write1ByteTxRx(self,ph,motor,addr,value):
                assert addr==40 and value==1
                parent.states[str(motor)]['torque']=1
                return 0,0
        self.pk=PK()
    def read(self,sdk,ph,pk,motor,key):
        state=self.states[str(motor)]
        if motor==3 and key=='position' and state['torque']==1:
            if self.problem=='stuck':return state['position']
            if self.problem=='reverse':state['position']+=3;return state['position']
            sign=1 if self.expected['3']>3500 else -1
            residual=self.settle_error*sign if self.elbow_writes==1 else self.handoff_error*sign
            desired=state['goal']+residual
            state['position']+=max(-8,min(8,desired-state['position']))
        return state.get(key,0)
    def clock(self):self.now+=.025;return self.now
    def run_hold(self,action='recover-hold'):
        with patch.object(shared.jc,'read',self.read),patch.object(shared.time,'sleep'),patch.object(shared.time,'monotonic',self.clock):
            return left.control(self.sdk,None,self.pk,self.group,action,self.expected)
    def test_upper20tick_offset_recovers_then_holds_all_without_limit_changes(self):
        before=copy.deepcopy(self.group);result=self.run_hold()
        self.assertTrue(result['completed']);self.assertEqual(self.group,before)
        self.assertIn((3,3468),self.events)
        self.assertTrue(all(s['torque']==1 for s in self.states.values()))
        self.assertLessEqual(self.states['3']['position'],3492)
    def test_lower32tick_offset_recovers_only_inward(self):
        self.states['3']['position']=468;self.expected['3']=468
        self.assertTrue(self.run_hold()['completed']);self.assertIn((3,532),self.events)
    def test_max32tick_offset_commands_no_more_than64ticks(self):
        self.states['3']['position']=3532;self.expected['3']=3532
        self.run_hold();self.assertIn((3,3468),self.events)
    def test_beyond32_other_joint_outside_or_invalid_inner_target_reject_before_writes(self):
        for problem in ('outside','other','width'):
            self.setUp()
            if problem=='outside':self.states['3']['position']=3533;self.expected['3']=3533
            if problem=='other':self.states['4']['position']=3501;self.expected['4']=3501
            if problem=='width':
                self.states['3']['min']=3470;self.group['joints']['3']={'safe_raw_limits':[3470,3500],'observed_raw_limits':[3470,3500]}
            with self.assertRaises(ValueError):self.run_hold()
            self.assertEqual(self.events,[])
    def test_normal_hold_still_rejects_outside(self):
        with self.assertRaises(ValueError):self.run_hold('hold')
        self.assertEqual(self.events,[])
    def test_17tick_recovery_residual_can_handoff_once_only_after_in_range(self):
        self.settle_error=17;self.handoff_error=4
        result=self.run_hold();self.assertTrue(result['completed']);self.assertEqual(self.elbow_writes,2)
        rec=next(x for x in result['activation'] if x['id']==3)
        self.assertEqual(rec['holding_handoff_target'],3485)
        self.assertTrue(508<=result['states']['3']['position']<=3492)
    def test_persistent_handoff_drift_stops_without_more_activation_or_retry(self):
        self.settle_error=17;self.handoff_error=17
        with self.assertRaises(RuntimeError):self.run_hold()
        self.assertEqual([m for m,t in self.events],[1,2,3,3])
    def test_fault_interrupt_or_no_progress_keeps_partial_inward_hold_and_no_remaining_activation(self):
        for problem in ('ack','interrupt','power','other','stuck','reverse'):
            self.setUp();self.problem=problem
            with self.assertRaises(BaseException):self.run_hold()
            self.assertEqual([m for m,t in self.events],[1,2,3])
            self.assertEqual(self.states['3']['goal'],3468)
            self.assertTrue(all(self.states[str(m)]['torque']==0 for m in (4,5,6)))
    def test_handoff_ack_failure_does_not_repeat_target_write(self):
        self.settle_error=17;self.problem='handoff_ack'
        with self.assertRaises(RuntimeError):self.run_hold()
        self.assertEqual([m for m,t in self.events],[1,2,3,3])
    def test_calibration_power_torque_or_stale_pose_blocks_before_write(self):
        for key,value in (('model',1),('mode',1),('moving',1),('torque',1),('voltage_present_raw',109),('temperature_raw',60),('max',3499)):
            self.setUp();self.states['3'][key]=value
            with self.assertRaises(ValueError):self.run_hold()
            self.assertEqual(self.events,[])
        self.setUp();self.expected['3']+=9
        with self.assertRaises(ValueError):self.run_hold()
        self.assertEqual(self.events,[])
    def test_already_in_range_is_current_pose_hold_without_recovery_travel(self):
        self.states['3']['position']=2000;self.expected['3']=2000
        self.assertTrue(self.run_hold()['completed']);self.assertIn((3,2000),self.events)
    def test_non_automatic_target_torque_enables_after_readback(self):
        self.auto_torque=False
        self.assertTrue(self.run_hold()['completed'])
    def test_left_cli_retains_left_port_and_explicit_recovery_action(self):
        text=Path(left.__file__).read_text()
        self.assertIn("['groups']['left-arm']",text)
        self.assertIn('left_startup=True',text)
        self.assertIn("'recover-hold'",text)
if __name__=='__main__':unittest.main()
