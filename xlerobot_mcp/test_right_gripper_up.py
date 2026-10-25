"""No hardware: coordinated reach bounds and stop behavior."""
import copy,json,sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import right_gripper_up as up

class UpReach(unittest.TestCase):
    def setUp(self):
        self.states={str(m):{'model':777,'min':500,'max':3500,'mode':0,'torque':1,'goal':1800+m*20,'position':1800+m*20,'moving':0,'voltage_present_raw':120,'temperature_raw':30} for m in range(1,7)}
        self.group={'port_verified':True,'joints':{str(m):{'safe_raw_limits':[500,3500],'observed_raw_limits':[500,3500]} for m in range(1,7)}}
        self.expected={m:s['position'] for m,s in self.states.items()};self.cal={n:{'id':m,'drive_mode':0,'range_min':500,'range_max':3500,'homing_offset':m*10} for m,n in enumerate(up.NAMES,1)}
        self.offsets={str(m):m*10 for m in range(1,7)};self.events=[];self.problem=None;self.now=0.
        parent=self
        class SDK:COMM_SUCCESS=0
        self.sdk=SDK()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                assert motor in up.ACTIVE and addr==42 and length==6 and data[2:]==[0,0,30,0]
                target=data[0]+256*data[1];parent.events.append((motor,target));parent.states[str(motor)]['goal']=target
                if len(parent.events)==1:
                    if parent.problem=='ack':raise RuntimeError('lost ack')
                    if parent.problem=='interrupt':raise KeyboardInterrupt()
                    if parent.problem=='other':parent.states['6']['position']+=9
                    if parent.problem=='power':parent.states['3']['voltage_present_raw']=100
                    if parent.problem=='limits':parent.states['4']['max']-=1
                return 0,0
            def diag(self,motor,addr,size):
                assert motor in range(1,7)
                if addr==31:return ((2048 | (motor*10)) if parent.problem=='negativeoffset' else motor*10),0,0
                if addr==30:return 1,0,0
                if parent.problem=='diag':return 0,1,0
                return {58:0,60:50,65:1 if parent.problem=='fault' else 0,69:1}[addr],0,0
            def read1ByteTxRx(self,ph,motor,addr):return self.diag(motor,addr,1)
            def read2ByteTxRx(self,ph,motor,addr):return self.diag(motor,addr,2)
        self.pk=PK()
    def fake_solver(self,expected,cal):
        first={str(m):expected[str(m)]+(6 if m!=3 else -6) for m in up.ACTIVE}
        end={str(m):expected[str(m)]+(12 if m!=3 else -12) for m in up.ACTIVE}
        return {'targets':end,'waypoints':[first,end],'predicted_displacement_m':[0,0,.002],'model_residual_m':0.}
    def plan(self,execute=True,solver=None):return up.plan(self.group,self.states,self.expected,self.cal,self.offsets,b'cal',b'urdf',execute,solver or self.fake_solver)
    def read(self,sdk,ph,pk,motor,name):
        if self.events and self.problem=='read':raise RuntimeError('feedback lost')
        s=self.states[str(motor)]
        if name=='position' and motor in up.ACTIVE and self.problem!='stuck':s['position']+=max(-2,min(2,s['goal']-s['position']))
        return s.get(name,0)
    def clock(self):self.now+=.06;return self.now
    def run_reach(self,p=None,observer=None):
        if p is None:p=self.plan()
        initial=copy.deepcopy(self.states)
        with patch.object(up.rt.jc,'read',self.read),patch.object(up.time,'sleep'),patch.object(up.time,'monotonic',self.clock):
            return up.execute(self.sdk,None,self.pk,self.group,initial,p,observer)
    def test_coordinated_small_reach_only_ids234(self):
        result=self.run_reach();self.assertTrue(result['completed'])
        self.assertEqual([m for m,t in self.events],[2,3,4,2,3,4])
        self.assertTrue(all(self.states[str(m)]['torque']==1 for m in range(1,7)))
    def test_preview_accepts_torqueoff_but_execution_does_not(self):
        for s in self.states.values():s['torque']=0
        self.assertTrue(self.plan(False)['preview_only'])
        with self.assertRaises(ValueError):self.plan(True)
        self.assertEqual(self.events,[])
    def test_pose_power_and_limits_preflight(self):
        for key,value in [('model',1),('mode',1),('moving',1),('temperature_raw',60),('voltage_present_raw',109),('position',400),('max',3499)]:
            with self.subTest(key=key):
                self.setUp();self.states['3'][key]=value
                with self.assertRaises(ValueError):self.plan()
                self.assertEqual(self.events,[])
    def test_offsets_and_resolution_sign_decoding(self):
        self.assertEqual(up.calibration_offsets(self.sdk,None,self.pk),self.offsets)
        self.problem='negativeoffset';self.assertEqual(up.calibration_offsets(self.sdk,None,self.pk)['3'],-30)
        self.offsets['3']+=1
        with self.assertRaisesRegex(ValueError,'stale'):self.plan()
    def test_calibration_mismatch_blocks(self):
        for key,value in [('id',4),('drive_mode',1),('range_min',501)]:
            with self.subTest(key=key):
                self.setUp();self.cal['elbow_flex'][key]=value
                with self.assertRaises(ValueError):self.plan()
    def test_active_outstanding_goal_blocks(self):
        self.states['3']['goal']+=5
        with self.assertRaises(ValueError):self.plan()
    def test_stale_expected_pose_blocks(self):
        self.expected['2']+=5
        with self.assertRaises(ValueError):self.plan()
    def test_hash_binds_reference_calibration_model_and_limits(self):
        first=self.plan()['plan_hash']
        self.assertNotEqual(first,up.identity(self.group,self.expected,b'changed',b'urdf'))
        self.assertNotEqual(first,up.identity(self.group,self.expected,b'cal',b'changed'))
        self.group['joints']['2']['safe_raw_limits'][0]+=1
        self.assertNotEqual(first,self.plan()['plan_hash'])
    def test_no_arbitrary_joint_targets_or_large_steps(self):
        for motor in (1,5,6):
            def bad(e,c):
                p=self.fake_solver(e,c);p['targets'][str(motor)]=1800;return p
            with self.assertRaises(ValueError):self.plan(solver=bad)
        def bad(e,c):
            p=self.fake_solver(e,c);p['waypoints'][0]['2']+=33;return p
        with self.assertRaises(ValueError):self.plan(solver=bad)
    def test_too_many_waypoints_rejects(self):
        def bad(e,c):
            p=self.fake_solver(e,c);p['waypoints']*=3;return p
        with self.assertRaises(ValueError):self.plan(solver=bad)
    def test_failures_stop_without_later_waypoint(self):
        for issue in ('ack','interrupt','other','power','limits','diag','fault','read'):
            with self.subTest(issue=issue):
                self.setUp();self.problem=issue;result=self.run_reach()
                self.assertFalse(result['completed']);self.assertTrue(all(m in up.ACTIVE for m,t in self.events))
                end=self.fake_solver(self.expected,self.cal)['targets']
                self.assertFalse(any(t==end[str(m)] for m,t in self.events))
    def test_stationary_waypoint_miss_preserves_goals_without_twitch(self):
        self.problem='stuck';result=self.run_reach()
        self.assertFalse(result['completed']);self.assertEqual(result['cleanup']['action'],'preserve_existing_targets');self.assertEqual(len(self.events),3)
    def test_current_outside_limits_prevents_even_preview(self):
        self.states['3']['position']=3501
        with self.assertRaises(ValueError):self.plan(False)
    def test_observed_model_downward_or_sideways_motion_stops(self):
        for delta in ([0,0,-.001],[.002,0,0],[0,0,.004]):
            self.setUp()
            def observe(states):return delta if self.events else [0,0,0]
            result=self.run_reach(observer=observe)
            self.assertFalse(result['completed']);self.assertIn('corridor',result['error'])
            self.assertEqual(result['trace'][-1]['encoder_model_displacement_m'],delta)
            end=self.fake_solver(self.expected,self.cal)['targets']
            self.assertFalse(any(t==end[str(m)] for m,t in self.events))
    def test_settled_joint_endpoints_do_not_prove_gripper_height(self):
        result=self.run_reach(observer=lambda states:[0,0,0])
        self.assertFalse(result['completed']);self.assertIn('height is short',result['error'])
    def test_sufficient_observed_model_upward_progress_passes(self):
        def observe(states):return [0,0,max(0,states['2']['position']-self.expected['2'])/12*.002]
        result=self.run_reach(observer=observe)
        self.assertTrue(result['completed']);self.assertGreater(result['encoder_model_displacement_m'][2],.001)
    def test_preview_hash_binds_actual_joint_plan(self):
        first=self.plan()['plan_hash']
        def changed(e,c):
            result=self.fake_solver(e,c)
            result['targets']['2']+=1;result['waypoints'][-1]['2']+=1
            return result
        self.assertNotEqual(first,self.plan(solver=changed)['plan_hash'])
    def test_packet_rejects_other_ids(self):
        with self.assertRaises(ValueError):up.packet(self.sdk,None,self.pk,7,2000)
        self.assertEqual(self.events,[])
    def test_leading_progress_required_even_when_tracking_error_is_small(self):
        p=self.plan();p.update(execution_policy='left_shoulder_leading_v1',stage_minimum_heights_m=[.0002,.001],intermediate_tolerance_ticks=8,final_tolerance_ticks=4)
        self.problem='stuck'
        result=self.run_reach(p,observer=lambda states:[0,0,0])
        self.assertFalse(result['completed'])
        self.assertEqual(len(self.events),3)
    def test_leading_intermediate6tick_error_advances_but_final6tick_error_fails(self):
        p=self.plan();p['waypoints'][0]['2']=self.expected['2']+14;p['targets']['2']=self.expected['2']+22;p['waypoints'][1]['2']=p['targets']['2']
        p.update(execution_policy='left_shoulder_leading_v1',stage_minimum_heights_m=[.0002,.001],intermediate_tolerance_ticks=8,final_tolerance_ticks=4)
        original_read=self.read
        def read(sdk,ph,pk,motor,name):
            if name=='position' and motor==2 and self.events:
                self.states['2']['position']=self.states['2']['goal']-6
                return self.states['2']['position']
            return original_read(sdk,ph,pk,motor,name)
        self.read=read
        result=self.run_reach(p,observer=lambda states:[0,0,(states['2']['position']-self.expected['2'])/10000])
        self.assertFalse(result['completed'])
        self.assertTrue(any(m==2 and t==p['targets']['2'] for m,t in self.events))
        self.assertEqual(self.states['2']['goal']-self.states['2']['position'],6)
    def test_leading_plan_cannot_run_without_observer(self):
        p=self.plan();p.update(execution_policy='left_shoulder_leading_v1',stage_minimum_heights_m=[.0002,.001],intermediate_tolerance_ticks=8,final_tolerance_ticks=4)
        with self.assertRaises(ValueError):self.run_reach(p)
        self.assertEqual(self.events,[])
    def test_leading_unsampled_intermediate_overshoot_does_not_advance(self):
        p=self.plan();p.update(execution_policy='left_shoulder_leading_v1',stage_minimum_heights_m=[.0002,.001],intermediate_tolerance_ticks=8,final_tolerance_ticks=4)
        original_read=self.read
        def read(sdk,ph,pk,motor,name):
            if name=='position' and motor==2 and self.events:
                self.states['2']['position']=p['waypoints'][0]['2']+1
                return self.states['2']['position']
            return original_read(sdk,ph,pk,motor,name)
        self.read=read
        result=self.run_reach(p,observer=lambda states:[0,0,(states['2']['position']-self.expected['2'])/10000])
        self.assertFalse(result['completed'])
        self.assertFalse(any(m==2 and t==p['targets']['2'] for m,t in self.events))
if __name__=='__main__':unittest.main()
