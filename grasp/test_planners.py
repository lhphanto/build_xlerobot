"""Planner regression on the November23 box (lerobot env, ~4 min, no hardware): python grasp/test_planners.py"""
import json, os, shutil, subprocess, sys, tempfile, unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
BEFORE_GRASP = {'1': 2285, '2': 2063, '3': 1389, '4': 2916, '5': 3196, '6': 1737}   # held pose before attempt 2
REST = {'1': 2255, '2': 1002, '3': 2785, '4': 2856, '5': 3173, '6': 2160}             # folded rest after recover-hold (Nov24)
GRASPED = {'1': 2313, '2': 2447, '3': 1452, '4': 2519, '5': 3353, '6': 1745}        # after the close that carried the box


def run(script, *args):
    r = subprocess.run([PY, os.path.join(HERE, script), *args], capture_output=True, text=True, timeout=1200)
    if r.returncode:
        raise AssertionError(r.stdout[-800:] + r.stderr[-800:])
    return r.stdout


class Planners(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        cls.grasps = os.path.join(cls.tmp, 'box_grasps.json')
        shutil.copy(os.path.join(HERE, 'fixtures', 'nov23_box_grasps.json'), cls.grasps)
        run('plan_grasp.py', cls.grasps, '--pose', json.dumps(BEFORE_GRASP))
        cls.gp = json.load(open(os.path.join(cls.tmp, 'box_grasp_plan.json')))
        run('plan_basket.py', os.path.join(cls.tmp, 'box_grasp_plan.json'), '--pose', json.dumps(GRASPED))  # defaults: Nov24 basket
        cls.bp = json.load(open(os.path.join(cls.tmp, 'box_basket_plan.json')))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp)

    def test_grasp_matches_the_live_run(self):
        gp = self.gp
        self.assertIn(gp['tilt'], (20, 25, 30))                                # live run: 30 deg; -15 mm offset and the 3.5 cm grip allow less
        for live, planned in zip((0.322, -0.184, 0.010 - 0.015), gp['grasp_point']):
            self.assertAlmostEqual(live, planned, delta=0.008)                   # offset now -15 mm (Nov23 -20); grip 3.5 cm deep (Nov23 2)
        self.assertEqual(gp['grip_depth'], 0.035)
        kinds = [s['kind'] for s in gp['steps']]
        self.assertEqual(kinds[-4:], ['open', 'reach', 'close', 'close'])         # close, then a second squeeze (Nov25 slip)
        self.assertTrue(all('--expect' in s['command'] for s in gp['steps'][-4:]))  # camera checks around the grasp
        retry = gp['steps'][-1]['if_fails']
        self.assertIn('close --squeeze 24', retry[0]['command'])                  # squeeze more first
        self.assertTrue(any('reach' in r['command'] for r in retry))              # then a reachable deeper regrasp

    def test_basket_lifts_clear_carries_and_lowers(self):
        reach = [s for s in self.bp['steps'] if s['kind'] == 'reach']
        self.assertIn('test lift 4 cm', reach[0]['note']); self.assertTrue(reach[0].get('if_fails'))
        carry = [s for s in reach if s['note'].startswith('carry')][-1]
        self.assertGreaterEqual(carry['expect_bottom_z'], -0.01 + 0.035 - 0.006)  # rim clearance while carrying
        for v, d in zip(carry['target'][:2], (0.18, -0.20)):                       # hanging straight down: gripper over the drop point
            self.assertAlmostEqual(v, d, delta=0.011)
        loaded = [s for s in reach if not s['note'].startswith('lift the empty')]
        self.assertTrue(all(s.get('expect') for s in loaded))                     # every loaded move checked by camera
        lower = loaded[-1]
        self.assertTrue(lower['note'].startswith('lower'))
        self.assertAlmostEqual(lower['expect_bottom_z'], -0.01 - 0.01, delta=0.006)  # 1 cm below the rim (operator: lower)
        kinds = [s['kind'] for s in self.bp['steps']]
        self.assertLess(kinds.index('look'), kinds.index('open'))                  # camera confirms over the basket before the drop
        self.assertEqual(kinds[-3:], ['open', 'reach', 'view'])                    # release, lift the empty gripper, look


HOVER_NOV24 = {'1': 2264, '2': 2643, '3': 887, '4': 2825, '5': 2843, '6': 2160}  # grasp hover (35 deg, jaw 27.3) Nov24 morning
HOVER2_NOV24 = {'1': 2288, '2': 2410, '3': 1300, '4': 2660, '5': 3454, '6': 2160}  # second hover (35 deg, jaw 88.8)


class ReadyAndPull(unittest.TestCase):
    def test_ready_from_the_grasp_hover(self):
        # November24: no equal-step chain existed from here (1 cm near this outstretched pose costs ~270 ticks)
        steps = json.loads(run('plan_ready.py', '--pose', json.dumps(HOVER_NOV24), '--json').strip().splitlines()[-1])
        self.assertLessEqual(len(steps), 6); self.assertEqual(steps[-1]['target'], [0.2, -0.08, 0.14])

    def test_adjust_keeps_lean_and_jaw_line(self):
        steps = json.loads(run('adjust.py', '--right', '1.5', '--pose', json.dumps(HOVER2_NOV24), '--json').strip().splitlines()[-1])
        self.assertEqual(len(steps), 1); s = steps[0]
        self.assertAlmostEqual(s['jaw_yaw_deg'], 88.8, delta=1.5); self.assertEqual(s['tilt'], 35)
        self.assertAlmostEqual(s['target'][1], -0.193, delta=0.003)               # 15 mm toward -y (operator's right)

    def test_adjust_swing_left_lowers_servo_1(self):
        carry = {'1': 2671, '2': 2258, '3': 1100, '4': 3068, '5': 3179, '6': 1737}
        out = run('adjust.py', '--swing-left', '10', '--free-roll', '--pose', json.dumps(carry))
        self.assertIn("'1': 25", out.split('->')[1][:20])                           # ID1 2671 -> ~2548 (Nov24 live)

    def test_ready_chain_from_rest(self):
        steps = json.loads(run('plan_ready.py', '--pose', json.dumps(REST), '--json').strip().splitlines()[-1])
        self.assertLessEqual(len(steps), 4)
        self.assertEqual(steps[-1]['target'], [0.2, -0.08, 0.14]); self.assertEqual(steps[-1]['tilt'], 30)

    def test_side_pull_plan(self):
        tmp = tempfile.mkdtemp()
        try:
            g = os.path.join(tmp, 'far_grasps.json'); shutil.copy(os.path.join(HERE, 'fixtures', 'nov23_box_grasps.json'), g)
            ready = {'1': 2223, '2': 1513, '3': 1928, '4': 2984, '5': 3169, '6': 2160}
            run('plan_grasp.py', g, '--pose', json.dumps(ready), '--pull')
            p = json.load(open(os.path.join(tmp, 'far_pull_plan.json')))
            kinds = [s['kind'] for s in p['steps']]
            self.assertEqual(p['task'], 'pull'); self.assertGreaterEqual(p['tilt'], 60)
            self.assertEqual(kinds.count('close'), 1); self.assertEqual(kinds.count('open'), 2); self.assertEqual(kinds[-1], 'replan')
            self.assertLess(kinds.index('open'), kinds.index('close'))
            grab, pulled = p['points']['grab'], p['points']['pulled']
            self.assertGreater(np.hypot(*grab[:2]) - np.hypot(*pulled[:2]), 0.025)  # pulled toward the arm
            self.assertTrue(all(abs(a - b) < 1e-6 for a, b in zip([grab[2]], [pulled[2]])))  # at the same height (sliding)
        finally:
            shutil.rmtree(tmp)


if __name__ == '__main__':
    unittest.main()
