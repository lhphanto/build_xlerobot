"""No hardware: left calibration/model delegation and unchanged right policy."""
import copy,sys,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import left_gripper_up as left
import test_right_gripper_up as fixtures

class LeftGripper(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.UpReach();self.f.setUp();self.f.group['kinematic_calibration']=copy.deepcopy(self.f.cal)
    def make_plan(self,distance_mm=5):
        with patch.object(left.reach,'URDF',SimpleNamespace(read_bytes=lambda:b'urdf')),patch.object(left,'shoulder_leading_plan',side_effect=lambda e,c,g,d:self.f.fake_solver(e,c)) as solve:
            result=left.make_plan(self.f.group,self.f.states,self.f.expected,self.f.offsets,True,distance_mm=distance_mm)
            solve.assert_called_once_with(self.f.expected,self.f.cal,{m:s['goal'] for m,s in self.f.states.items()},distance_mm)
            return result
    def test_left_uses_existing_solver_for_fixed_vertical5mm(self):
        result=self.make_plan();self.assertEqual(result['step_mm'],5)
        self.assertEqual(set(result['targets']),{'2','3','4'})
    def test_parameterized_distance_is_forwarded_and_hash_bound(self):
        original=self.make_plan()['plan_hash']
        for distance in (2,5.5,10):
            result=self.make_plan(distance)
            self.assertEqual(result['step_mm'],distance)
            self.assertNotEqual(result['plan_hash'],original)
    def test_invalid_distance_rejected_before_solver(self):
        for distance in (0,-1,1.99,10.01,float('nan'),float('inf'),True,'10'):
            with self.subTest(distance=distance),self.assertRaises(ValueError):self.make_plan(distance)
    def test_hash_binds_stored_goals_even_with_same_measured_pose(self):
        before=self.make_plan()['plan_hash']
        self.f.states['2']['goal']-=1
        self.assertNotEqual(self.make_plan()['plan_hash'],before)
    def test_live_homing_mismatch_rejects(self):
        self.f.offsets['3']+=1
        with self.assertRaisesRegex(ValueError,'stale'):self.make_plan()
    def test_left_existing_stationary5tick_hold_error_allowed_not_endpoint_tolerance(self):
        self.f.states['3']['goal']-=5
        self.assertEqual(self.make_plan()['step_mm'],5)
        with self.assertRaises(ValueError):self.f.plan(True)
    def test_more_than8tick_startup_error_rejected(self):
        self.f.states['3']['goal']-=9
        with self.assertRaises(ValueError):self.make_plan()
    def test_missing_calibration_blocks(self):
        del self.f.group['kinematic_calibration']
        with self.assertRaisesRegex(ValueError,'calibration'):self.make_plan()
    def test_left_port_and_progress_observer_are_wired(self):
        source=Path(left.__file__).read_text()
        self.assertIn("['groups']['left-arm']",source)
        self.assertIn('reach.make_observer',source)
        self.assertNotIn("['groups']['right-arm']",source)
    def coarse_plan(self,bias,distance_mm=10,wrist=0):
        calls=[]
        def solver(e,c,g,d,coarse=False):
            calls.append(g);return self.f.fake_solver(e,c)
        with patch.object(left.reach,'URDF',SimpleNamespace(read_bytes=lambda:b'urdf')),patch.object(left,'shoulder_leading_plan',side_effect=solver):
            result=left.make_plan(self.f.group,self.f.states,self.f.expected,self.f.offsets,True,distance_mm=distance_mm,coarse=True,elbow_bias_ticks=bias,wrist_bias_ticks=wrist)
        return result,calls[0]
    def test_wrist_bias_signed_in_travel_direction_and_hash_bound(self):
        self.f.states['4']['goal']-=5
        result,goals=self.coarse_plan(0,wrist=5)
        self.assertEqual(goals['4'],self.f.expected['4']);self.assertEqual(goals['3'],self.f.states['3']['goal'])
        self.assertEqual(result['wrist_bias_ticks'],5)  # Fake wrist travel is positive.
        self.assertEqual(result['measured_wrist_sag_ticks'],5)
        self.assertNotEqual(result['plan_hash'],self.coarse_plan(0,wrist=4)[0]['plan_hash'])
    def test_zero_wrist_bias_leaves_wrist_goal_and_hash_alone(self):
        self.f.states['4']['goal']-=5
        result,goals=self.coarse_plan(5)
        self.assertEqual(goals['4'],self.f.states['4']['goal'])
        self.assertEqual(result['wrist_bias_ticks'],0)
        self.assertEqual(result['plan_hash'],self.coarse_plan(5,wrist=0)[0]['plan_hash'])
    def test_bias_larger_than_planned_travel_rejected(self):
        # Oct29: a 15 mm plan moved the elbow +1 tick; bias12 would have pushed it 12 ticks down.
        with self.assertRaisesRegex(ValueError,'only -12 ticks'):self.coarse_plan(13)
        self.assertEqual(self.coarse_plan(12)[0]['elbow_bias_ticks'],-12)
    def test_invalid_wrist_bias_rejected(self):
        for wrist in (-1,9,True,2.0):
            with self.subTest(wrist=wrist),self.assertRaises(ValueError):self.coarse_plan(0,wrist=wrist)
    def test_elbow_bias_signed_in_travel_direction_and_plans_from_measured_elbow(self):
        self.f.states['3']['goal']-=12  # Stored sag-compensated command from an earlier bias trial.
        result,goals=self.coarse_plan(12)
        self.assertEqual(goals['3'],self.f.expected['3'])
        self.assertEqual(result['elbow_bias_ticks'],-12)  # Fake elbow travel is negative.
        self.assertEqual(result['measured_elbow_sag_ticks'],12)
    def test_zero_bias_keeps_stored_goals_and_hash(self):
        result,goals=self.coarse_plan(0)
        self.assertEqual(goals,{m:s['goal'] for m,s in self.f.states.items()})
        self.assertEqual(result['elbow_bias_ticks'],0)
        self.assertNotEqual(result['plan_hash'],self.coarse_plan(5)[0]['plan_hash'])
        self.assertNotEqual(self.coarse_plan(5)[0]['plan_hash'],self.coarse_plan(6)[0]['plan_hash'])
    def test_invalid_bias_rejected(self):
        for bias in (-1,17,True,2.0,'5'):
            with self.subTest(bias=bias),self.assertRaises(ValueError):self.coarse_plan(bias)
        with self.assertRaisesRegex(ValueError,'coarse'):
            with patch.object(left.reach,'URDF',SimpleNamespace(read_bytes=lambda:b'urdf')):
                left.make_plan(self.f.group,self.f.states,self.f.expected,self.f.offsets,True,distance_mm=5,elbow_bias_ticks=5)
    def test_biased_command_outside_saved_limits_rejected(self):
        self.f.group['joints']['3']['safe_raw_limits'][0]=self.f.expected['3']-16
        with self.assertRaisesRegex(ValueError,'Biased elbow'):self.coarse_plan(8)
    def biased_run(self,bias,sag,wrist_bias=0,wrist_sag=0):
        p=self.f.plan()
        p.update(execution_policy='left_coarse_lift_v2',coarse=True,step_mm=10,joint_tolerances={'2':8,'3':16,'4':4},
                 stage_minimum_heights_m=[.0002,.005],elbow_bias_ticks=bias,wrist_bias_ticks=wrist_bias)
        original=self.f.read
        sags={3:sag,4:wrist_sag}
        def read(sdk,ph,pk,motor,name):
            s=self.f.states[str(motor)]
            if name=='position' and motor in sags and self.f.events:
                # Gravity holds the joint `sag` ticks off its command.
                aim=s['goal']+sags[motor];s['position']+=max(-4,min(4,aim-s['position']));return s['position']
            return original(sdk,ph,pk,motor,name)
        self.f.read=read
        e3=self.f.expected['3']
        return p,self.f.run_reach(p,observer=lambda states:[0,0,max(0,e3-states['3']['position'])/12*.006])
    def test_bias_cancels_sag_and_lands_on_target(self):
        p,result=self.biased_run(-5,5)
        self.assertTrue(result['completed'],result.get('error'))
        self.assertEqual([t for m,t in self.f.events if m==3],[w['3']-5 for w in p['waypoints']])
        self.assertEqual(result['states']['3']['position'],p['targets']['3'])
    def test_without_bias_sag_leaves_elbow_short(self):
        p,result=self.biased_run(0,5)
        self.assertEqual(self.f.states['3']['position']-p['targets']['3'],5)
    def test_excess_bias_overshoot_fails_without_completing(self):
        p,result=self.biased_run(-10,0)
        self.assertFalse(result['completed'])
    def test_wrist_sag_alone_fails_final_tolerance_like_oct29(self):
        p,result=self.biased_run(-5,5,0,-5)
        self.assertFalse(result['completed'])
        self.assertIn("'4': 5",result['error'])  # Wrist residual exceeds its4tick allowance.
    def test_elbow_and_wrist_bias_cancel_both_sags(self):
        p,result=self.biased_run(-5,5,5,-5)
        self.assertTrue(result['completed'],result.get('error'))
        self.assertEqual([t for m,t in self.f.events if m==4],[w['4']+5 for w in p['waypoints']])
        self.assertEqual({m:result['states'][m]['position'] for m in '34'},{m:p['targets'][m] for m in '34'})
    def test_excess_wrist_bias_overshoot_fails(self):
        p,result=self.biased_run(0,0,8,0)
        self.assertFalse(result['completed'])
    def test_bias_rejected_for_non_coarse_execution(self):
        p=self.f.plan();p['elbow_bias_ticks']=-5
        with self.assertRaises(ValueError):self.f.run_reach(p)
        self.assertEqual(self.f.events,[])
    def test_height_bound_is_in_preview_identity(self):
        a=left.reach.identity(self.f.group,self.f.expected,b'cal',b'urdf',2)
        b=left.reach.identity(self.f.group,self.f.expected,b'cal',b'urdf',5)
        self.assertNotEqual(a,b)
if __name__=='__main__':unittest.main()
