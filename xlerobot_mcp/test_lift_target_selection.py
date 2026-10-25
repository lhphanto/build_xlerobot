"""No hardware: bounded alternative selection preserves completion thresholds."""
import unittest
from unittest.mock import patch
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import left_gripper_up as left
import coarse_lift_targets as search

class TargetSelection(unittest.TestCase):
    def item(self,height=.012):return {'endpoint_minimum_m':height,'targets':{'2':1000,'3':2000,'4':2100}}
    def checked(self,height=.012):return {'stage_arrival_minimum_heights_m':[.002,height],'stage_minimum_heights_m':[.002,.01]}
    def run_plan(self):return left.shoulder_leading_plan({}, {}, {},20,True)
    def test_requires_extra_margin_without_lowering_runtime_lift_floor(self):
        with patch.object(search,'candidates',return_value=[self.item(.0105),self.item(.012)]),patch.object(left,'_checked_plan',return_value=self.checked()) as check:
            p=self.run_plan()
        self.assertEqual(check.call_count,1)
        self.assertEqual(p['planning_margin_mm'],1)
        self.assertEqual(p['stage_minimum_heights_m'][-1],.01)
        self.assertAlmostEqual(p['achieved_planning_margin_mm'],2)
    def test_exact_checked_path_can_reject_an_optimistic_endpoint(self):
        with patch.object(search,'candidates',return_value=[self.item(),self.item()]),patch.object(left,'_checked_plan',side_effect=[self.checked(.0109),self.checked(.012)]):
            p=self.run_plan()
        self.assertEqual(p['candidate_path_checks'],2)
    def test_rejects_if_no_candidate_has_required_margin(self):
        with patch.object(search,'candidates',return_value=[self.item(.0109)]),patch.object(left,'_checked_plan') as check:
            with self.assertRaisesRegex(ValueError,'planning margin'):self.run_plan()
        check.assert_not_called()
    def test_attempts_are_finite_and_only_offline_path_checks(self):
        with patch.object(search,'candidates',return_value=[self.item()]*10),patch.object(left,'_checked_plan',side_effect=ValueError('lag path rejected')) as check:
            with self.assertRaisesRegex(ValueError,'lag path'):self.run_plan()
        self.assertEqual(check.call_count,search.MAX_PATH_CHECKS)
    def test_rejected_path_can_use_next_bounded_candidate(self):
        with patch.object(search,'candidates',return_value=[self.item()]*2),patch.object(left,'_checked_plan',side_effect=[ValueError('side corridor'),self.checked()]):
            self.assertEqual(self.run_plan()['candidate_path_checks'],2)
    def test_strict_mode_preserves_original_path_and_avoids_search(self):
        with patch.object(search,'candidates') as choices,patch.object(left,'_checked_plan',return_value={'strict':True}) as check:
            self.assertEqual(left.shoulder_leading_plan({}, {}, {},5),{'strict':True})
        choices.assert_not_called();check.assert_called_once_with({}, {}, {},5)
if __name__=='__main__':unittest.main()
