"""No hardware: gripper open, close-to-contact with bounded squeeze, empty close, and stop behavior."""
import sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import left_gripper_grip as gg

class Grip(unittest.TestCase):
    def setUp(self):
        # Pose after the November16 lower approach; elbow carries a 14-tick sag offset.
        pos={'1':2302,'2':2022,'3':2088,'4':1940,'5':3542,'6':1764};goal=pos|{'3':2074}
        self.states={m:{'model':777,'min':0,'max':4095,'mode':0,'torque':1,'goal':goal[m],'position':pos[m],'moving':0,'voltage_present_raw':120,'temperature_raw':30} for m in pos}
        self.group={'port_verified':True,'joints':{m:{'safe_raw_limits':[1453 if m=='6' else 500,2808 if m=='6' else 3774],'observed_raw_limits':[0,4095]} for m in pos}}
        self.expected=dict(pos);self.events=[];self.now=0.;self.obstacle=None;self.problem=None
        parent=self
        self.sdk=type('SDK',(),{'COMM_SUCCESS':0})()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                assert motor==6 and addr==42 and data[4]==gg.SPEED
                parent.events.append(data[0]+256*data[1]);parent.states['6']['goal']=data[0]+256*data[1]
                if parent.problem=='other':parent.states['2']['position']+=9
                return 0,0
            def read2ByteTxRx(self,ph,motor,addr):
                g=parent.states['6'];blocked=parent.obstacle is not None and g['goal']<parent.obstacle
                squeeze=max(0,parent.obstacle-g['goal']) if blocked else 0
                return (1024+min(1000,squeeze*25) if blocked else 40),0,0
        self.pk=PK()
    def read(self,sdk,ph,pk,motor,key):
        s=self.states[str(motor)]
        if key=='position' and motor==6 and self.events:
            aim=s['goal'] if self.obstacle is None else max(s['goal'],self.obstacle)
            s['position']+=max(-8,min(8,aim-s['position']))
        return s.get(key,0)
    def clock(self):self.now+=.02;return self.now
    def go(self,action,**kw):
        with patch.object(gg.jc,'read',self.read),patch.object(gg.time,'sleep'),patch.object(gg.time,'monotonic',self.clock):
            return gg.control(self.sdk,None,self.pk,self.group,self.expected,action,**kw)
    def test_open_to_target(self):
        r=self.go('open',target=2164)
        self.assertTrue(r['completed']);self.assertEqual(r['position'],2164);self.assertEqual(self.events,[2164])
    def test_open_full_range_at_measured_speed_does_not_time_out(self):
        # Live trace: about 8 ticks per 0.16 s monitor sample at speed 30 (two clock reads per sample), so 700 ticks takes ~14 s.
        orig=self.read
        def slow(sdk,ph,pk,motor,key):
            s=self.states['6']
            if motor==6 and key=='position' and self.events:
                s['position']+=max(-8,min(8,s['goal']-s['position']));s['moving']=int(s['position']!=s['goal']);return s['position']
            return orig(sdk,ph,pk,motor,key)
        self.read=slow
        def clock():self.now+=.08;return self.now
        self.clock=clock
        r=self.go('open',target=1764+700)
        self.assertTrue(r['completed'],r.get('error'));self.assertEqual(r['position'],2464)
        self.assertGreater(r['trace'][-1]['t'],10)  # longer than the old fixed 3 s timeout
    def test_open_bounds(self):
        for t in (1764,1700,2465,2900,'2000'):
            with self.subTest(t=t),self.assertRaises(ValueError):self.go('open',target=t)
        self.assertEqual(self.events,[])
    def test_close_stops_at_contact_and_squeezes_a_bounded_amount(self):
        self.obstacle=1690
        r=self.go('close',squeeze=12)
        self.assertTrue(r['contact']);self.assertEqual(r['contact_position'],1690);self.assertEqual(r['hold_goal'],1678)
        self.assertTrue(all(e>=1678 for e in self.events))  # never commanded past contact minus squeeze
    def test_close_without_object_stops_at_floor(self):
        r=self.go('close')
        self.assertFalse(r['contact']);self.assertEqual(min(self.events),1453+gg.FLOOR_MARGIN)
    def test_empty_close_ends_with_floor_deadband_instead_of_resending(self):
        # November20: goal1461 was reached at1463 with low load; old loop resent
        # the floor until interrupted because position remained greater than floor.
        for residual in range(1,gg.REACHED+1):
            with self.subTest(residual=residual):
                self.setUp();orig=Grip.read.__get__(self,Grip);floor=1453+gg.FLOOR_MARGIN
                def deadband(sdk,ph,pk,motor,key):
                    if len(self.events)>40:raise RuntimeError('Repeated floor command')
                    s=self.states['6']
                    if motor==6 and key=='position' and self.events and s['goal']==floor:
                        s['position']=max(floor+residual,s['position']-8)
                        return s['position']
                    return orig(sdk,ph,pk,motor,key)
                self.read=deadband
                r=self.go('close',squeeze=24)
                self.assertTrue(r['completed'],r.get('error'))
                self.assertFalse(r['contact'])
                self.assertEqual(r['position'],floor+residual)
                self.assertEqual(self.events.count(floor),1)
                self.assertNotIn('hold_goal',r)  # no squeeze without contact
                self.assertGreaterEqual(min(self.events),floor)
    def test_close_already_at_floor_is_no_contact_without_writes(self):
        floor=1453+gg.FLOOR_MARGIN
        self.states['6'].update(position=floor,goal=floor);self.expected['6']=floor
        r=self.go('close',squeeze=24)
        self.assertTrue(r['completed']);self.assertFalse(r['contact']);self.assertEqual(self.events,[])
    def test_squeeze_bounds(self):
        for s in (-1,25,True):
            with self.subTest(s=s),self.assertRaises(ValueError):self.go('close',squeeze=s)
        self.assertEqual(self.events,[])
    def test_other_joint_motion_stops_and_holds(self):
        self.problem='other'
        r=self.go('close')
        self.assertFalse(r['completed']);self.assertIn('ID2 moved',r['error']);self.assertTrue(r['cleanup']['attempted'])
    def test_open_blocked_reports_failure(self):
        self.obstacle=None;self.states['6']['position']=1764
        orig=self.read
        def stuck(sdk,ph,pk,motor,key):
            if key=='position' and motor==6:return self.states['6']['position']
            return orig(sdk,ph,pk,motor,key)
        self.read=stuck
        r=self.go('open',target=2000)
        self.assertFalse(r['completed']);self.assertIn('in the way',r['error'])
    def test_shoulder_holding_allowance_matches_reach_only_for_this_tool(self):
        import left_elbow_test as et
        self.states['2']['goal']=self.states['2']['position']-12
        self.assertTrue(self.go('close')['completed'])
        with self.assertRaises(ValueError):et.validate(self.group,self.states,self.expected,0)  # other tools keep 8
        self.events.clear();self.states['2']['goal']=self.states['2']['position']-41
        with self.assertRaises(ValueError):self.go('close')
        self.assertEqual(self.events,[])
    def test_single_bad_temperature_sample_is_reread_not_fatal(self):
        orig=self.read;state={'n':0}
        def glitch(sdk,ph,pk,motor,key):
            if motor==1 and key=='temperature_raw' and self.events:
                state['n']+=1
                if state['n']==3:return 77
            return orig(sdk,ph,pk,motor,key)
        self.read=glitch;self.obstacle=1690
        r=self.go('close')
        self.assertTrue(r['completed'] and r['contact']);self.assertEqual(r['glitches'][0]['temperature'],77)
    def test_persistent_bad_temperature_stops(self):
        orig=self.read
        def hot(sdk,ph,pk,motor,key):
            if motor==1 and key=='temperature_raw' and self.events:return 77
            return orig(sdk,ph,pk,motor,key)
        self.read=hot
        r=self.go('close')
        self.assertFalse(r['completed']);self.assertIn('temperature',r['error'])
    def test_trace_keeps_the_end_of_a_long_close(self):
        r=self.go('close')
        self.assertLessEqual(len(r['trace']),160);self.assertLessEqual(r['trace'][-1]['position'],1453+gg.FLOOR_MARGIN+gg.REACHED)
    def test_squeezed_gripper_can_be_reclosed_or_opened(self):
        # November23: after a 24-tick squeeze the gripper held 9 ticks short of its goal; re-close (retry) and open must start.
        self.states['6'].update(position=1745,goal=1721);self.expected['6']=1745;self.obstacle=1745
        r=self.go('close',squeeze=24)
        self.assertTrue(r['contact']);self.assertGreaterEqual(min(self.events),1745-16-24)
        self.events.clear();self.obstacle=None;self.states['6'].update(position=1745,goal=1721)
        self.assertTrue(self.go('open',target=2162)['completed'])
        self.events.clear();self.states['6'].update(goal=1720)  # 25 past: refused before any write
        with self.assertRaises(ValueError):self.go('open',target=2162)
        self.assertEqual(self.events,[])
    def test_preflight_requires_holding(self):
        self.states['4']['torque']=0
        with self.assertRaises(ValueError):self.go('close')
        self.assertEqual(self.events,[])
if __name__=='__main__':unittest.main()
