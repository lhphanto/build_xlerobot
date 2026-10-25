import sys, unittest, io, json
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).parent/'controllers'))
import left_elbow_bias_test as bt

class ElbowBias(unittest.TestCase):
    def setUp(self):
        # Observed Oct 24: elbow stationary at 2701 with goal 2687 (+14 sag).
        self.states={str(m):{'model':777,'min':500,'max':3500,'mode':0,'torque':1,'goal':1900+m*10,'position':1900+m*10,'moving':0,'voltage_present_raw':121,'temperature_raw':30} for m in range(1,7)}
        self.states['3'].update(goal=2687, position=2701)
        self.group={'port_verified':True,'joints':{str(m):{'safe_raw_limits':[500,2804],'observed_raw_limits':[500,3500]} for m in range(1,7)}}
        self.expected={m:s['position'] for m,s in self.states.items()}
        self.events=[];self.final_sag=0;self.problem=None;self.now=0.
        parent=self
        class SDK:COMM_SUCCESS=0
        self.sdk=SDK()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                assert motor==3 and addr==42 and length==6 and data[2:]==[0,0,30,0]
                value=data[0]+256*data[1];parent.events.append(value)
                parent.states['3']['goal']=value
                if len(parent.events)==1 and parent.problem=='other':parent.states['4']['position']+=9
                return 0,0
            def read1ByteTxRx(self,ph,motor,addr):
                assert motor==3
                return 0,0,0
            def read2ByteTxRx(self,ph,motor,addr):
                assert motor==3
                return 0,0,0
        self.pk=PK()
    def read(self,sdk,ph,pk,motor,key):
        if motor==3 and key=='position' and self.events:
            s=self.states['3'];aim=s['goal']+self.final_sag
            s['position']+=max(-10,min(10,aim-s['position']))
        return self.states[str(motor)].get(key,0)
    def clock(self):
        self.now+=.05
        return self.now
    def run_trial(self):
        with patch.object(bt.jc,'read',self.read),patch.object(bt.time,'sleep'),patch.object(bt.time,'monotonic',self.clock),redirect_stdout(io.StringIO()) as out:
            try:return bt.control(self.sdk,None,self.pk,self.group,self.expected)
            finally:self.output=out.getvalue()
    def test_plan_overcommands_by_measured_sag(self):
        p=bt.plan(self.group,self.states,self.expected)
        self.assertEqual((p['desired_raw'],p['sag_ticks'],p['target_raw']),(2687,14,2673))
    def test_reaches_desired_when_sag_repeats(self):
        self.final_sag=14
        r=self.run_trial()
        self.assertEqual(self.events,[2673])
        self.assertTrue(r['completed']);self.assertTrue(r['reached_desired'])
        self.assertEqual(r['final_raw'],2687)
    def test_miss_is_reported_not_retried(self):
        self.final_sag=0  # No sag: elbow tracks the overcommanded target fully.
        r=self.run_trial()
        self.assertEqual(self.events,[2673])
        self.assertFalse(r['reached_desired']);self.assertEqual(r['final_error_to_desired'],-14)
    def test_small_or_large_residual_rejected_without_writes(self):
        for pos in (2690,2704):
            with self.subTest(pos=pos):
                self.states['3']['position']=pos;self.expected['3']=pos
                with self.assertRaises(ValueError):self.run_trial()
                self.assertEqual(self.events,[])
    def test_target_beyond_saved_limit_rejected(self):
        self.group['joints']['3']['safe_raw_limits']=[2680,2804]
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_other_joint_motion_holds_elbow_at_feedback(self):
        self.problem='other'
        with self.assertRaises(RuntimeError):self.run_trial()
        failure=json.loads(self.output.splitlines()[-1])
        self.assertTrue(failure['cleanup']['hold_acknowledged'])
        self.assertEqual(len(self.events),2);self.assertEqual(self.events[1],failure['cleanup']['hold_goal_raw'])
    def test_stale_expected_pose_rejected(self):
        self.expected['3']=2690
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])

if __name__=='__main__':
    unittest.main()
