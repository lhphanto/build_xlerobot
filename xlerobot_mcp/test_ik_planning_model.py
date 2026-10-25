"""Offline URDF regressions for the physically observed wrist-first dip.
Run with lerobot Python; never opens serial/camera or changes robot settings.
"""
import importlib.util,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import right_gripper_up as up
import left_gripper_up as left
AVAILABLE=importlib.util.find_spec('placo') is not None and importlib.util.find_spec('numpy') is not None

@unittest.skipUnless(AVAILABLE and up.URDF.exists(),'lerobot placo/URDF required for offline model test')
class ModelLag(unittest.TestCase):
    def setUp(self):
        limits=[(776,3132),(822,3132),(650,2804),(930,3147),(488,3774),(1453,2808)]
        self.cal={n:{'id':m,'drive_mode':0,'homing_offset':0,'range_min':lo,'range_max':hi} for m,(n,(lo,hi)) in enumerate(zip(up.NAMES,limits),1)}
        self.expected={'1':1908,'2':1211,'3':2558,'4':2179,'5':2169,'6':2017}
    def test_observed_unsafe_orientation_plan_is_rejected_under_lag(self):
        with self.assertRaisesRegex(ValueError,'joint lag'):
            up.solve(self.expected,self.cal,step_mm=5,orientation_weight=.01,lag_guard=True)
    def test_position_plan_limits_pitch_and_downward_lag(self):
        p=up.solve(self.expected,self.cal,step_mm=5,orientation_weight=0,lag_guard=True)
        self.assertLessEqual(p['model_residual_m'],.0005)
        self.assertLessEqual(p['modeled_orientation_change_deg'],3)
        self.assertGreaterEqual(p['worst_modeled_partial_height_m'],-.0005)
        self.assertTrue(all(abs(p['targets'][m]-self.expected[m])<=32 for m in p['targets']))
    def test_plan_recomputed_for_post_failure_pose(self):
        self.expected['3']=2561;self.expected['4']=2185
        p=up.solve(self.expected,self.cal,step_mm=5,orientation_weight=0,lag_guard=True)
        self.assertGreater(p['predicted_displacement_m'][2],.0045)
        self.assertGreaterEqual(p['worst_modeled_partial_height_m'],-.0005)
    def test_larger10mm_plan_preserves_existing_bounds(self):
        self.expected.update({'2':1209,'3':2561,'4':2185})
        p=up.solve(self.expected,self.cal,step_mm=10,orientation_weight=0,lag_guard=True)
        self.assertGreater(p['predicted_displacement_m'][2],.0095)
        self.assertLessEqual(p['modeled_orientation_change_deg'],3)
        self.assertGreaterEqual(p['worst_modeled_partial_height_m'],-.0005)
        previous=self.expected.copy()
        for point in p['waypoints']:
            for motor,value in point.items():
                self.assertLessEqual(abs(value-previous[motor]),8)
                self.assertLessEqual(abs(value-self.expected[motor]),32)
            previous.update(point)
    def test_leading_plan_escapes_recorded_duplicate_goals(self):
        self.expected.update({'2':1209,'3':2561,'4':2185})
        goals=self.expected|{'2':1202,'3':2559,'4':2186}
        for distance in (5,10):
            p=left.shoulder_leading_plan(self.expected,self.cal,goals,distance)
            self.assertNotEqual(p['waypoints'][0]['2'],goals['2'])
            self.assertEqual(p['waypoints'][0]['3'],goals['3'])
            self.assertEqual(p['waypoints'][0]['4'],goals['4'])
            self.assertLessEqual(abs(p['waypoints'][0]['2']-self.expected['2']),16)
            self.assertGreaterEqual(p['worst_modeled_partial_height_m'],-.0005)
            self.assertLessEqual(p['max_modeled_partial_side_m'],.0015)
            self.assertGreater(p['stage_minimum_heights_m'][0],0)
            self.assertEqual(p['final_tolerance_ticks'],4)
    def test_too_small_lift_rejects_instead_of_waiting_on_existing_target(self):
        self.expected.update({'2':1209,'3':2561,'4':2185})
        goals=self.expected|{'2':1202,'3':2559,'4':2186}
        with self.assertRaisesRegex(ValueError,'existing target'):
            left.shoulder_leading_plan(self.expected,self.cal,goals,2)
    def test_observed_stationary_elbow_lag_can_advance_without_false_success(self):
        self.expected.update({'2':1178,'3':2562,'4':2190})
        goals=self.expected|{'2':1173,'3':2555}
        p=left.shoulder_leading_plan(self.expected,self.cal,goals,10)
        observed=self.expected|{'2':1167}
        observer=up.make_observer(self.expected,self.cal)
        actual_height=observer({m:{'position':v} for m,v in observed.items()})[2]
        self.assertLess(actual_height,p['stage_nominal_heights_m'][0]-.003)
        self.assertGreater(actual_height,p['stage_minimum_heights_m'][0])
        self.assertAlmostEqual(p['stage_minimum_heights_m'][0],p['stage_arrival_minimum_heights_m'][0])
        self.assertTrue(all(abs(observed[m]-p['waypoints'][0][m])<=8 for m in ('2','3','4')))
        self.assertEqual(p['stage_minimum_heights_m'][-1],.009000000000000001)
        self.assertEqual(p['final_tolerance_ticks'],4)
        self.assertLess(actual_height,p['stage_minimum_heights_m'][-1])
    def test_coarse20mm_predicts_useful_lift_with_measured_tracking_uncertainty(self):
        self.expected.update({'1':1907,'2':1149,'3':2573,'4':2191})
        goals=self.expected|{'2':1147,'3':2561,'4':2192}
        p=left.shoulder_leading_plan(self.expected,self.cal,goals,20,coarse=True)
        self.assertEqual(p['joint_tolerances'],{'2':8,'3':16,'4':4})
        self.assertGreaterEqual(p['predicted_final_model_lift_interval_mm'][0],10)
        self.assertLessEqual(p['modeled_orientation_change_deg'],6)
        self.assertGreaterEqual(p['worst_modeled_partial_height_m'],-.0005)
        self.assertLessEqual(p['max_modeled_partial_side_m'],.0015)
        self.assertEqual(p['stage_minimum_heights_m'][-1],.01)
        self.assertTrue(all(abs(p['targets'][m]-self.expected[m])<=64 for m in p['targets']))
    def test_single_seed_can_reject_when_tracking_uncertainty_dominates(self):
        self.expected.update({'1':1907,'2':1149,'3':2573,'4':2191})
        goals=self.expected|{'2':1147,'3':2561,'4':2192}
        with self.assertRaises(ValueError):
            left._checked_plan(self.expected,self.cal,goals,15,coarse=True)
    def test_alternative_20mm_plan_recovers_blocked_pose_with_margin(self):
        self.expected.update({'1':1908,'2':1118,'3':2572,'4':2191})
        goals=self.expected|{'2':1118,'3':2568,'4':2191}
        with self.assertRaises(ValueError):
            left._checked_plan(self.expected,self.cal,goals,20,coarse=True)
        p=left.shoulder_leading_plan(self.expected,self.cal,goals,20,coarse=True)
        self.assertGreaterEqual(p['predicted_final_model_lift_interval_mm'][0],11)
        self.assertGreaterEqual(p['achieved_planning_margin_mm'],1)
        self.assertEqual(p['stage_minimum_heights_m'][-1],.01)
        self.assertLessEqual(p['candidate_path_checks'],4)
        self.assertLessEqual(p['model_residual_m'],.0005)
        self.assertLessEqual(p['modeled_orientation_change_deg'],6)
        self.assertGreaterEqual(p['worst_modeled_partial_height_m'],-.0005)
        self.assertLessEqual(p['max_modeled_partial_side_m'],.0015)
        previous=self.expected.copy()
        for point in p['waypoints']:
            self.assertTrue(all(abs(point[m]-previous[m])<=16 and abs(point[m]-self.expected[m])<=64 for m in point))
            previous.update(point)
    def test_recovered_folded_pose_uses_upward_elbow_lead_not_downward_shoulder(self):
        self.expected.update({'1':1907,'2':856,'3':2789,'4':2191})
        goals=self.expected|{'2':857,'3':2786}
        observer=up.make_observer(self.expected,self.cal)
        downward=self.expected|{'2':872,'3':2786}
        self.assertLess(observer({m:{'position':v} for m,v in downward.items()})[2],-.002)
        p=left.shoulder_leading_plan(self.expected,self.cal,goals,20,coarse=True)
        self.assertEqual(p['lead_joint_id'],3)
        self.assertEqual(p['waypoints'][0]['2'],goals['2'])
        self.assertLess(p['waypoints'][0]['3'],self.expected['3'])
        self.assertGreater(p['stage_nominal_heights_m'][0],0)
        self.assertGreaterEqual(p['predicted_final_model_lift_interval_mm'][0],11)
        self.assertLessEqual(p['predicted_final_model_lift_interval_mm'][1],21.5)
        self.assertEqual(p['stage_minimum_heights_m'][-1],.01)
        self.assertGreaterEqual(p['worst_modeled_partial_height_m'],-.0005)
        self.assertLessEqual(p['max_modeled_partial_side_m'],.0015)
        self.assertLessEqual(p['modeled_orientation_change_deg'],6)
if __name__=='__main__':unittest.main()
