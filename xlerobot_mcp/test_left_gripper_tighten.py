"""No hardware: a direct small grip offset must never repeat the contact search."""
import unittest
from unittest.mock import patch
import test_left_gripper_grip as fixtures
import left_gripper_grip as gg

class Tighten(unittest.TestCase):
    setUp=fixtures.Grip.setUp
    read=fixtures.Grip.read
    clock=fixtures.Grip.clock
    go=fixtures.Grip.go

    def test_contact_keeps_a_single_two_tick_target(self):
        self.obstacle=1764
        r=self.go('tighten',squeeze=2)
        self.assertTrue(r['completed'],r)
        self.assertEqual(self.events,[1762])
        self.assertEqual(r['holding_position'],1764)
        self.assertEqual(r['holding_load'],50)

    def test_no_contact_does_not_search_farther(self):
        r=self.go('tighten',squeeze=4)
        self.assertTrue(r['completed'])
        self.assertEqual(self.events,[1760])
        self.assertNotIn('contact',r)  # no false grip/contact claim

    def test_bounds_reject_before_writes(self):
        for ticks in (0,5,-1,True,2.5):
            with self.subTest(ticks=ticks),self.assertRaises(ValueError):self.go('tighten',squeeze=ticks)
        self.assertEqual(self.events,[])

    def test_existing_holding_offset_rejects(self):
        self.states['6']['goal']=1760
        with self.assertRaisesRegex(ValueError,'holding offset'):self.go('tighten',squeeze=2)
        self.assertEqual(self.events,[])

    def test_floor_rejects_before_writes(self):
        self.states['6'].update(position=1462,goal=1462);self.expected['6']=1462
        with self.assertRaisesRegex(ValueError,'floor'):self.go('tighten',squeeze=2)
        self.assertEqual(self.events,[])

    def test_never_repeats_or_relaxes_an_existing_target(self):
        self.states['6']['goal']=1762
        with self.assertRaisesRegex(ValueError,'existing target'):self.go('tighten',squeeze=2)
        self.assertEqual(self.events,[])

    def test_initial_load_rejects_before_writes(self):
        with patch.object(self.pk,'read2ByteTxRx',return_value=(150,0,0)):
            with self.assertRaisesRegex(ValueError,'already loaded'):self.go('tighten',squeeze=2)
        self.assertEqual(self.events,[])

    def test_load_guard_holds_without_further_closing(self):
        original=self.pk.read2ByteTxRx
        def reading(*args):return (200,0,0) if self.events else original(*args)
        with patch.object(self.pk,'read2ByteTxRx',side_effect=reading):r=self.go('tighten',squeeze=2)
        self.assertFalse(r['completed']);self.assertIn('load guard',r['error'])
        self.assertTrue(r['cleanup']['attempted'])
        self.assertEqual(self.events[0],1762)
        self.assertTrue(all(v>=1762 for v in self.events))

    def test_other_joint_motion_stops(self):
        self.problem='other';r=self.go('tighten',squeeze=2)
        self.assertFalse(r['completed']);self.assertIn('ID2 moved',r['error'])

    def test_torque_off_rejects(self):
        self.states['3']['torque']=0
        with self.assertRaises(ValueError):self.go('tighten',squeeze=2)
        self.assertEqual(self.events,[])

    def test_interrupt_holds_without_retry_or_release(self):
        original=self.pk.read2ByteTxRx
        def reading(*args):
            if self.events:raise KeyboardInterrupt()
            return original(*args)
        with patch.object(self.pk,'read2ByteTxRx',side_effect=reading):r=self.go('tighten',squeeze=2)
        self.assertFalse(r['completed']);self.assertTrue(r['cleanup']['attempted'])
        self.assertEqual(len(self.events),2)  # command, then feedback hold

if __name__=='__main__':unittest.main()
