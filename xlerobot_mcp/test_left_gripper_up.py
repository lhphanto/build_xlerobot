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
    def test_height_bound_is_in_preview_identity(self):
        a=left.reach.identity(self.f.group,self.f.expected,b'cal',b'urdf',2)
        b=left.reach.identity(self.f.group,self.f.expected,b'cal',b'urdf',5)
        self.assertNotEqual(a,b)
if __name__=='__main__':unittest.main()
