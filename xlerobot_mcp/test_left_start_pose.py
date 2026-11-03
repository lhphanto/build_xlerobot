"""No hardware: fixed start-pose plan bounds, sag trim and stop behavior."""
import copy,io,sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import left_start_pose as sp

class StartPose(unittest.TestCase):
    def setUp(self):
        # The pose left by the third October29 lift: shoulder845, elbow2758 goal2746, wrist2112 goal2107.
        positions={'1':1908,'2':845,'3':2758,'4':2112,'5':2168,'6':2017};goals=positions|{'2':848,'3':2746,'4':2107}
        self.states={m:{'model':777,'min':500,'max':3500,'mode':0,'torque':1,'goal':goals[m],'position':positions[m],'moving':0,'voltage_present_raw':120,'temperature_raw':30} for m in positions}
        self.group={'port_verified':True,'joints':{m:{'safe_raw_limits':[650,2804],'observed_raw_limits':[500,3500]} for m in positions}}
        self.expected=dict(positions);self.events=[];self.sag={};self.problem=None;self.now=0.
        self.height=lambda states:[0,0,0]
        parent=self
        class SDK:COMM_SUCCESS=0
        self.sdk=SDK()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                assert motor in (2,3,4) and addr==42 and length==6 and data[2:]==[0,0,30,0]
                target=data[0]+256*data[1];parent.events.append((motor,target));parent.states[str(motor)]['goal']=target
                if len(parent.events)==1:
                    if parent.problem=='ack':raise RuntimeError('lost ack')
                    if parent.problem=='other':parent.states['6']['position']+=9
                    if parent.problem=='voltage':parent.states['3']['voltage_present_raw']=100
                return 0,0
            def read1ByteTxRx(self,ph,motor,addr):return (1 if parent.problem=='fault' else 0),0,0
            def read2ByteTxRx(self,ph,motor,addr):return 0,0,0
        self.pk=PK()
    def observer(self,states):return self.height(states)
    def plan(self):return sp.make_plan(self.group,self.states,self.expected,self.observer)
    def read(self,sdk,ph,pk,motor,name):
        s=self.states[str(motor)]
        if name=='position' and motor in (2,3,4) and self.events and self.problem!='stuck':
            aim=s['goal']+self.sag.get(motor,0);s['position']+=max(-8,min(8,aim-s['position']))
        return s.get(name,0)
    def clock(self):self.now+=.02;return self.now
    def run_move(self,p=None):
        p=p or self.plan()
        with patch.object(sp.jc,'read',self.read),patch.object(sp.time,'sleep'),patch.object(sp.time,'monotonic',self.clock):
            return p,sp.execute(self.sdk,None,self.pk,self.group,copy.deepcopy(self.states),p,self.observer)
    def test_plan_reaches_fixed_pose_in_bounded_stages(self):
        p=self.plan();self.assertEqual(p['waypoints'][-1],sp.START)
        prior={m:self.expected[m] for m in sp.START}
        for point in p['waypoints']:
            self.assertEqual(set(point),{'2','3','4'})
            self.assertTrue(all(abs(point[m]-prior[m])<=16 for m in point));prior=point
    def test_preview_writes_nothing(self):
        self.plan();self.assertEqual(self.events,[])
    def test_too_far_rejected(self):
        self.states['4']['position']=self.states['4']['goal']=2000;self.expected['4']=2000
        with self.assertRaisesRegex(ValueError,'reposition by hand'):self.plan()
    def test_preflight_requires_holding_fresh_pose(self):
        for mid,key,value in [('3','torque',0),('6','moving',1),('3','position',2810)]:
            with self.subTest(key=key):
                self.setUp();self.states[mid][key]=value
                with self.assertRaises(ValueError):self.plan()
        self.setUp();self.expected['2']+=5
        with self.assertRaises(ValueError):self.plan()
    def test_target_outside_saved_limits_rejected(self):
        self.group['joints']['3']['safe_raw_limits']=[650,2780]
        with self.assertRaisesRegex(ValueError,'outside saved limits'):self.plan()
    def test_modeled_gripper_envelope_rejected(self):
        self.height=lambda states:[0,0,-.08]
        with self.assertRaisesRegex(ValueError,'envelope'):self.plan()
        self.height=lambda states:[.03,0,0]
        with self.assertRaisesRegex(ValueError,'envelope'):self.plan()
    def test_hash_binds_pose_goals_and_limits(self):
        first=self.plan()['plan_hash']
        self.states['2']['goal']-=1;self.assertNotEqual(self.plan()['plan_hash'],first)
        self.setUp();self.group['joints']['3']['safe_raw_limits'][0]+=1;self.assertNotEqual(self.plan()['plan_hash'],first)
    def test_moves_only_active_joints_and_lands_on_pose(self):
        p,result=self.run_move()
        self.assertTrue(result['completed'],result.get('error'))
        self.assertEqual(result['final_errors_ticks'],{'2':0,'3':0,'4':0});self.assertEqual(result['trimmed_commands'],{})
        self.assertEqual({m for m,_ in self.events},{2,3,4})
    def test_one_trim_cancels_sag_and_is_not_repeated(self):
        self.sag={3:10,4:6}
        p,result=self.run_move()
        self.assertTrue(result['completed'],result.get('error'))
        self.assertEqual(result['trimmed_commands'],{'3':2777,'4':2144})
        self.assertEqual(result['final_errors_ticks'],{'2':0,'3':0,'4':0})
        self.assertEqual(self.events[-2:],[(3,2777),(4,2144)])
    def test_largest_tracked_sag_is_fully_trimmed(self):
        self.sag={3:16}
        p,result=self.run_move()
        self.assertTrue(result['completed'],result.get('error'))
        self.assertEqual(result['trimmed_commands'],{'3':2771})  # Target-16, the lift preflight bound.
    def test_large_overshoot_stops_at_window_without_finishing_path(self):
        self.sag={4:30}
        p,result=self.run_move()
        self.assertFalse(result['completed']);self.assertIn('escaped',result['error'])
        self.assertNotIn('trimmed_commands',result)
        # Feedback outside the window is never sent as a hold; the in-range goal stays.
        self.assertFalse(result['cleanup']['holds']['4']['acknowledged'])
        self.assertEqual(self.f_goal('4'),sp.START['4'])
    def f_goal(self,mid):return self.states[mid]['goal']
    def test_already_at_start_writes_nothing(self):
        for m,v in sp.START.items():self.states[m]['position']=self.states[m]['goal']=v;self.expected[m]=v
        p,result=self.run_move()
        self.assertTrue(p['already_at_start']);self.assertTrue(result['completed']);self.assertEqual(self.events,[])
    def test_failures_stop_and_do_not_finish_path(self):
        for issue in ('ack','other','voltage','fault'):
            with self.subTest(issue=issue):
                self.setUp();self.problem=issue
                p,result=self.run_move()
                self.assertFalse(result['completed']);self.assertIn('cleanup',result)
                self.assertNotIn((3,sp.START['3']),self.events)
    def test_stationary_stuck_joint_preserves_targets(self):
        self.problem='stuck'
        p,result=self.run_move()
        self.assertFalse(result['completed']);self.assertEqual(result['cleanup']['action'],'preserve_existing_targets')
        self.assertEqual(len(self.events),3)  # First stage only.
    def test_live_envelope_stops_motion(self):
        p=self.plan();self.height=lambda states:[0,0,-.09] if self.events else [0,0,0]
        p,result=self.run_move(p)
        self.assertFalse(result['completed']);self.assertIn('envelope',result['error'])
    def test_foreign_plan_rejected(self):
        p=self.plan();p['targets']=sp.START|{'3':2700}
        with self.assertRaises(ValueError):self.run_move(p)
        self.assertEqual(self.events,[])

if __name__=='__main__':unittest.main()
