"""No hardware: explicit coarse completion, uncertainty and strict-mode isolation."""
import copy,unittest
import test_right_gripper_up as fixtures
from unittest.mock import patch
import left_gripper_up as left

class CoarseLift(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.UpReach();self.f.setUp();self.up=fixtures.up
        # The fixture's default clock models a very slow servo. Allow the
        # larger coarse stages to traverse at the selected30tick/s speed.
        def clock():self.f.now+=.02;return self.f.now
        self.f.clock=clock
    def solver(self,e,c):
        points=[{'2':e['2']+16,'3':e['3']-12,'4':e['4']+1},
                {'2':e['2']+32,'3':e['3']-24,'4':e['4']+2},
                {'2':e['2']+40,'3':e['3']-32,'4':e['4']+3}]
        return {'targets':points[-1],'waypoints':points,'predicted_displacement_m':[0,0,.02],
                'execution_policy':'left_coarse_lift_v2','coarse':True,'joint_tolerances':{'2':8,'3':16,'4':4},
                'stage_minimum_heights_m':[.001,.004,.01]}
    def plan(self,solver=None):
        f=self.f
        return self.up.plan(f.group,f.states,f.expected,f.cal,f.offsets,b'cal',b'urdf',True,solver or self.solver,20,8,16,coarse=True)
    def observer(self,states):return [0,0,(states['2']['position']-self.f.expected['2'])/40*.02]
    def test_coarse_success_reports_achieved_lift_without_claiming_exact_request(self):
        result=self.f.run_reach(self.plan(),self.observer)
        self.assertTrue(result['completed']);self.assertEqual(result['completion_kind'],'coarse_model_lift')
        self.assertEqual(result['requested_lift_mm'],20);self.assertEqual(result['minimum_accepted_model_lift_mm'],10)
        self.assertGreaterEqual(result['encoder_model_displacement_m'][2],.01)
        self.assertLess(result['encoder_model_displacement_m'][2],.02)
    def test_zero_or_insufficient_lift_cannot_pass_even_with_joint_tolerance(self):
        for height in (0,.005):
            self.setUp();r=self.f.run_reach(self.plan(),lambda states:[0,0,height])
            self.assertFalse(r['completed'])
    def test_fault_and_interrupt_stop_before_further_stages(self):
        for problem in ('interrupt','ack','power','other','fault'):
            self.setUp();self.f.problem=problem
            r=self.f.run_reach(self.plan(),self.observer)
            self.assertFalse(r['completed'])
            self.assertFalse(any(m==2 and target==self.f.expected['2']+40 for m,target in self.f.events))
    def test_startup_elbow16_allowed_but17_and_other_joint9_rejected(self):
        self.f.states['3']['goal']-=16;self.plan()
        self.f.states['3']['goal']-=1
        with self.assertRaises(ValueError):self.plan()
        self.setUp();self.f.states['2']['goal']-=9
        with self.assertRaises(ValueError):self.plan()
    def test_total65_or_stage17_and_saved_limit_violations_reject(self):
        for mode in ('total','stage','limit'):
            self.setUp()
            def bad(e,c):
                p=self.solver(e,c)
                if mode=='total':p['targets']['2']=e['2']+65
                if mode=='stage':p['waypoints'][0]['2']=e['2']+17
                if mode=='limit':self.f.group['joints']['2']['safe_raw_limits'][1]=e['2']+35
                return p
            with self.assertRaises(ValueError):self.plan(bad)
    def test_coarse_requires_observer_fixed_tolerances_and_height_requirement(self):
        for mode in ('observer','tolerance','height'):
            p=self.plan();observer=self.observer
            if mode=='observer':observer=None
            if mode=='tolerance':p['joint_tolerances']['3']=17
            if mode=='height':p['stage_minimum_heights_m'][-1]=.001
            with self.assertRaises(ValueError):self.f.run_reach(p,observer)
            self.assertEqual(self.f.events,[])
    def test_strict_policy_cannot_inherit_coarse_distance_or_joint_bounds(self):
        f=self.f
        with self.assertRaises(ValueError):self.up.plan(f.group,f.states,f.expected,f.cal,f.offsets,b'cal',b'urdf',True,self.solver,20,8,16)
    def test_coarse_hash_binds_tracking_policy(self):
        first=self.plan()['plan_hash']
        def changed(e,c):
            p=self.solver(e,c);p['joint_tolerances']['3']=15;return p
        self.assertNotEqual(first,self.plan(changed)['plan_hash'])
    def test_mode_distance_rejected_before_any_hardware_or_solver(self):
        for distance in (2,9.9,20.1,float('nan')):
            with self.assertRaises(ValueError):left.make_plan({}, {}, {}, {}, True,distance,coarse=True)
    def test_intermediate_targets_advance_while_moving_but_final_must_stop(self):
        p=self.plan();read=self.f.read
        def moving_read(sdk,ph,pk,motor,name):
            if name=='moving' and motor in self.up.ACTIVE:
                at_final=all(self.f.states[m]['goal']==p['targets'][m] for m in p['targets'])
                return 0 if at_final else 1
            return read(sdk,ph,pk,motor,name)
        self.f.read=moving_read
        result=self.f.run_reach(p,self.observer)
        self.assertTrue(result['completed'])
        self.assertEqual(result['trajectory_mode'],'continuous_bounded_tracking')
    def test_coarse_final_moving_feedback_cannot_report_completion(self):
        p=self.plan();read=self.f.read
        def moving_read(sdk,ph,pk,motor,name):
            if name=='moving' and motor in self.up.ACTIVE:return 1
            return read(sdk,ph,pk,motor,name)
        self.f.read=moving_read
        result=self.f.run_reach(p,self.observer)
        self.assertFalse(result['completed'])
        self.assertTrue(any(m==2 and target==p['targets']['2'] for m,target in self.f.events))
    def test_old_coarse_execution_policy_rejected_before_writes(self):
        p=self.plan();p['execution_policy']='left_coarse_lift_v1'
        with self.assertRaises(ValueError):self.f.run_reach(p,self.observer)
        self.assertEqual(self.f.events,[])
if __name__=='__main__':unittest.main()
