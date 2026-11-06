"""No hardware: right start pose reuses the shared controller with its own fixed targets."""
import copy,sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import left_start_pose as sp
import right_start_pose as rp

class RightStart(unittest.TestCase):
    def setUp(self):
        # Pose after a +32 elbow trial: elbow low, goal offset by sag.
        positions={'1':2170,'2':1104,'3':3020,'4':2229,'5':1995,'6':1901};goals=positions|{'3':3012}
        self.states={m:{'model':777,'min':0,'max':4095,'mode':0,'torque':1,'goal':goals[m],'position':positions[m],'moving':0,'voltage_present_raw':120,'temperature_raw':30} for m in positions}
        self.group={'port_verified':True,'joints':{m:{'safe_raw_limits':[900,3049],'observed_raw_limits':[0,4095]} for m in positions}}
        self.expected=dict(positions);self.events=[];self.now=0.;self.stall=16
        parent=self
        class SDK:COMM_SUCCESS=0
        self.sdk=SDK()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                assert motor in (2,3,4) and addr==42 and length==6 and data[2:]==[0,0,30,0]
                target=data[0]+256*data[1];parent.events.append((motor,target));parent.states[str(motor)]['goal']=target;return 0,0
            def read1ByteTxRx(self,ph,motor,addr):return 0,0,0
            def read2ByteTxRx(self,ph,motor,addr):return 0,0,0
        self.pk=PK()
    def observer(self,states):return [0,0,0]
    def plan(self):return sp.make_plan(self.group,self.states,self.expected,self.observer,rp.RIGHT)
    def read(self,sdk,ph,pk,motor,name):
        s=self.states[str(motor)]
        if name=='position' and motor in (2,3,4) and self.events:
            # Elbow stops `stall` ticks short whenever its goal is above its position.
            aim=s['goal']+(self.stall if motor==3 and s['goal']<s['position'] else 0)
            s['position']+=max(-8,min(8,aim-s['position']))
        return s.get(name,0)
    def clock(self):self.now+=.02;return self.now
    def run_move(self,p=None,arm=rp.RIGHT):
        p=p or self.plan()
        with patch.object(sp.jc,'read',self.read),patch.object(sp.time,'sleep'),patch.object(sp.time,'monotonic',self.clock):
            return p,sp.execute(self.sdk,None,self.pk,self.group,copy.deepcopy(self.states),p,self.observer,arm)
    def test_targets_are_fixed_right_pose(self):
        p=self.plan();self.assertEqual(p['targets'],rp.START);self.assertEqual(p['execution_policy'],'right_start_pose_v1')
        self.assertEqual(p['waypoints'][-1],rp.START)
    def test_elbow_stall_within_tolerance_is_trimmed_to_target(self):
        p,result=self.run_move()
        self.assertTrue(result['completed'],result.get('error'))
        self.assertEqual(result['trimmed_commands'],{'3':2974})
        self.assertEqual(result['final_errors_ticks']['3'],0)
    def test_stall_at_tracking_limit_lands_within_final_tolerance(self):
        self.stall=24
        p,result=self.run_move()
        self.assertTrue(result['completed'],result.get('error'))
        self.assertEqual(result['final_errors_ticks']['3'],8)
    def test_larger_stall_stops_without_retry(self):
        self.stall=30
        p,result=self.run_move()
        self.assertFalse(result['completed']);self.assertIn('stage',result['error'])
        self.assertNotIn((3,2974),self.events)
    def test_left_plan_refused_by_right_execution(self):
        p=self.plan();p['execution_policy']='left_start_pose_v1'
        with self.assertRaises(ValueError):self.run_move(p)
        self.assertEqual(self.events,[])
    def test_no_writes_to_other_joints(self):
        self.run_move();self.assertEqual({m for m,_ in self.events}-{2,3,4},set())
if __name__=='__main__':unittest.main()
