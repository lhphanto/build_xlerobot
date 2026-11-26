import copy
import json
from pathlib import Path
import tempfile
import unittest
import task_runner as t

POSE = {str(i): 2000 for i in range(1, 7)}

class LinearModel:
    def point(self, raw):
        return [(raw['1']-2000)/10000+.3, (raw['2']-2000)/10000, (raw['3']-2000)/10000+.1]
    def plan(self, raw, target):
        if target[0] > .5: raise ValueError('Target not reachable with current wrist roll')
        p = self.point(raw)
        targets = {str(i+1): raw[str(i+1)]+round((target[i]-p[i])*10000) for i in range(3)}
        if any(abs(targets[k]-raw[k]) > 200 for k in targets):
            raise ValueError('ID2 is 450 ticks from the target; above 400, reposition by hand')
        return {'targets': targets}


def plan(phases=None):
    return t.compile_route({'phases': phases or [{'name': 'lift', 'delta_mm': [0, 0, 10]}]}, POSE, LinearModel(), 'identity')


def feedback(ticket, completed=True):
    states = {i: dict(model=777, mode=0, torque=1, moving=0, min=0, max=4095, position=2000,
                      voltage_present_raw=120, temperature_raw=30) for i in POSE}
    result = {'completed': completed, 'plan': {'target_m': ticket['target_m']}, 'states': states,
              'final_errors_ticks': {i: 0 for i in '1234'}}
    return {'mock': False, 'exit_code': 0, 'stdout': json.dumps(result)}, result


class Planning(unittest.TestCase):
    def test_splits_travel_and_preserves_final_target(self):
        p = plan([{'name': 'rise', 'delta_mm': [0, 0, 80]}])
        self.assertGreater(len(p['actions']), 3)
        self.assertAlmostEqual(p['actions'][-1]['model_end_m'][2], .18)
        self.assertTrue(all(max(abs(a['nominal_targets'][k]-a['nominal_start'][k]) for k in a['nominal_targets']) <= 200 for a in p['actions']))
    def test_relative_phases_follow_previous_nominal_endpoint(self):
        p = plan([{'name': 'up', 'delta_mm': [0, 0, 10]}, {'name': 'back', 'delta_mm': [-10, 0, 0]}])
        self.assertEqual(p['actions'][-1]['model_end_m'], [.29, 0., .11])
    def test_unreachable_does_not_produce_partial_plan(self):
        with self.assertRaisesRegex(ValueError, 'not reachable'):
            plan([{'name': 'too far', 'target_m': [.6, 0, .1]}])
    def test_no_implicit_alternative_direction(self):
        class Bad(LinearModel):
            def plan(self, raw, target): raise ValueError('Saved limit exceeded')
        with self.assertRaisesRegex(ValueError, 'Saved limit'):
            t.compile_route({'phases':[{'name':'back','delta_mm':[-10,0,0]}]}, POSE, Bad(), 'identity')
    def test_invalid_parameters(self):
        for vector in ([0,0,float('nan')], [True,0,2], [0,0], [0,0,400]):
            with self.subTest(vector=vector), self.assertRaises(ValueError):
                plan([{'name':'bad','delta_mm':vector}])
        for value in (0, 1, 51, float('nan'), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                t.compile_route({'phases':[{'name':'a','delta_mm':[0,0,10]}]}, POSE, LinearModel(), 'identity', value)
    def test_unknown_action_refused(self):
        for phase in ({'name':'close','close':True}, {'name':'a','delta_mm':[0,0,10],'torque':0}, {'name':'bad','checkpoint':'anything'}):
            with self.assertRaises(ValueError): plan([phase])
    def test_plan_identity_binds_every_target(self):
        p = plan(); p['actions'][0]['target_m'][0] += .01
        with self.assertRaises(ValueError): t.new_run(p, 'identity')
        with self.assertRaises(ValueError): t.new_run(plan(), 'changed')
    def test_noop_rejected(self):
        with self.assertRaises(ValueError): plan([{'name':'a','delta_mm':[0,0,0]}])


class Journal(unittest.TestCase):
    def setUp(self): self.run = t.new_run(plan(), 'identity')
    def test_only_one_reservation_and_fresh_uuid(self):
        a = t.reserve(self.run, 'identity')
        with self.assertRaises(ValueError): t.reserve(self.run, 'identity')
        other = t.new_run(plan(), 'identity')
        self.assertNotEqual(a['request_id'], t.reserve(other, 'identity')['request_id'])
    def test_matching_feedback_advances(self):
        ticket = t.reserve(self.run, 'identity'); envelope, _ = feedback(ticket)
        self.assertTrue(t.record(self.run, ticket['request_id'], envelope))
        self.assertEqual(self.run['status'], 'complete')
    def test_wrong_request_cannot_advance(self):
        ticket = t.reserve(self.run, 'identity'); envelope, _ = feedback(ticket)
        with self.assertRaises(ValueError): t.record(self.run, 'wrong', envelope)
        self.assertEqual(self.run['status'], 'reserved')
    def test_failures_are_terminal_and_not_retried(self):
        changes = [lambda e,r:r.update(completed=False), lambda e,r:e.update(mock=True), lambda e,r:e.update(exit_code=1),
                   lambda e,r:r['plan'].update(target_m=[.1,0,.1]), lambda e,r:r['final_errors_ticks'].update({'2':9}),
                   lambda e,r:r['states']['2'].update(moving=1), lambda e,r:r['states']['2'].update(torque=0),
                   lambda e,r:r['states']['2'].update(temperature_raw=60), lambda e,r:r['states']['2'].update(voltage_present_raw=100),
                   lambda e,r:r['states']['2'].update(position=5000)]
        for change in changes:
            with self.subTest(change=change):
                run = t.new_run(plan(), 'identity'); ticket = t.reserve(run, 'identity'); e,r = feedback(ticket)
                change(e,r); e['stdout']=json.dumps(r)
                self.assertFalse(t.record(run,ticket['request_id'],e)); self.assertEqual(run['cursor'],0)
                with self.assertRaises(ValueError):t.reserve(run,'identity')
    def test_malformed_response_stops_and_preserves_uuid(self):
        ticket=t.reserve(self.run,'identity')
        self.assertFalse(t.record(self.run,ticket['request_id'],{'stdout':'broken'}))
        self.assertEqual(self.run['pending']['request_id'],ticket['request_id'])
    def test_client_and_native_envelopes(self):
        for wrap in (lambda e:{'results':[e]},lambda e:{'content':[{'type':'text','text':json.dumps(e)}]}):
            run=t.new_run(plan(),'identity');ticket=t.reserve(run,'identity');e,_=feedback(ticket)
            self.assertTrue(t.record(run,ticket['request_id'],wrap(e)))
    def test_checkpoint_blocks_motion_until_actual_observation(self):
        p=plan([{'name':'check','checkpoint':'grip_secure'},{'name':'up','delta_mm':[0,0,10]}]);run=t.new_run(p,'identity')
        with self.assertRaises(ValueError):t.reserve(run,'identity')
        with self.assertRaises(ValueError):t.observe(run,'placed','camera','Box visible in image')
        t.observe(run,'grip_secure','operator','Box rose and remains securely held')
        t.reserve(run,'identity')
    def test_supported_release_cannot_be_inferred_from_camera(self):
        run=t.new_run(plan([{'name':'support','checkpoint':'arm_supported'}]),'identity')
        with self.assertRaises(ValueError):t.observe(run,'arm_supported','camera','Arm appears on support')
        t.observe(run,'arm_supported','operator','All empty arm links fully supported')
        self.assertEqual(run['status'],'complete')
    def test_failure_cannot_be_bypassed_by_observation(self):
        ticket=t.reserve(self.run,'identity');t.record(self.run,ticket['request_id'],{})
        with self.assertRaises(ValueError):t.observe(self.run,'placed','operator','Everything appears fine')
    def test_atomic_roundtrip_retains_reservation_after_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'run.json';ticket=t.reserve(self.run,'identity');t.atomic_write(path,self.run)
            restored=json.loads(path.read_text())
            self.assertEqual(restored['pending'],ticket)
            with self.assertRaises(ValueError):t.reserve(restored,'identity')
            with t.locked(path):
                with self.assertRaises(BlockingIOError):
                    with t.locked(path): pass


if __name__ == '__main__': unittest.main()
