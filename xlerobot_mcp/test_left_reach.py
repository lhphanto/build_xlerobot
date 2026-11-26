"""Reach planning with the real SO101 model and left calibration, plus stop behavior with simulated servos."""
import copy,json,sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
try:
    import left_reach as lr
    import left_start_pose as sp
    import right_gripper_up as reach
    lr.kinematics({n:{'id':i,'range_min':1000,'range_max':3000} for i,n in enumerate(reach.NAMES[:5],1)})
    MODEL=reach.URDF.exists()
except Exception:
    MODEL=False

# Recorded hand-eye pose 1 (November 16), compact and away from joint limits.
POSE={'1':2147,'2':1649,'3':2038,'4':2179,'5':3373,'6':1764}

@unittest.skipUnless(MODEL,'lerobot placo/URDF required')
class Reach(unittest.TestCase):
    def setUp(self):
        self.group=json.loads((Path(__file__).parent/'robot_config.json').read_text())['groups']['left-arm']
        lims=self.group['joints']
        self.states={m:{'model':777,'min':lims[m]['observed_raw_limits'][0],'max':lims[m]['observed_raw_limits'][1],'mode':0,'torque':1,
                        'goal':v,'position':v,'moving':0,'voltage_present_raw':120,'temperature_raw':30} for m,v in POSE.items()}
        self.expected=dict(POSE);self.events=[];self.now=0.
        cal=self.group['kinematic_calibration']
        model,to_deg,_=lr.kinematics(cal)
        self.here=model.forward_kinematics(to_deg(POSE))[:3,3]
        self.observer=reach.make_observer(POSE,cal)
    def plan(self,target):return lr.make_plan(self.group,self.states,self.expected,target,self.observer)
    def test_small_moves_reach_target_in_the_model(self):
        model,to_deg,_=lr.kinematics(self.group['kinematic_calibration'])
        for d in ((0,0,.03),(.03,0,0),(0,.04,0)):
            with self.subTest(d=d):
                target=self.here+np.array(d);p,_=self.plan(target)
                commanded=dict(POSE);commanded.update(p['targets'])
                self.assertLess(np.linalg.norm(model.forward_kinematics(to_deg(commanded))[:3,3]-target),.005)
                self.assertEqual(set(p['targets']),{'1','2','3','4'})
                prior={m:POSE[m] for m in p['targets']}
                for w in p['waypoints']:
                    self.assertTrue(all(abs(w[m]-prior[m])<=16 for m in w));prior=w
    def test_outside_workspace_rejected_before_ik(self):
        for t in ((.9,0,.2),(.3,.5,.2),(.3,0,.6),(.3,0,float('nan'))):
            with self.subTest(t=t),self.assertRaises(ValueError):self.plan(np.array(t))
    def test_far_target_needing_large_travel_rejected(self):
        with self.assertRaises(ValueError):self.plan(self.here+np.array([0,0,-.3]))
    def test_level_jaws_on_november16_can_pose(self):
        # Live pose before the second grasp attempt: model tilt -25 deg (movable jaw lower), as the operator saw.
        pose={'1':2341,'2':2316,'3':1936,'4':1895,'5':3542,'6':1769}
        for m,v in pose.items():self.states[m].update(goal=v,position=v)
        self.expected=dict(pose);self.observer=reach.make_observer(pose,self.group['kinematic_calibration'])
        model,to_deg,_=lr.kinematics(self.group['kinematic_calibration'])
        self.assertLess(lr.jaw_tilt_deg(model,to_deg,pose),-20)
        target=np.array([.338,-.196,.016]);p,arm=self.plan(target) if False else lr.make_plan(self.group,self.states,self.expected,target,self.observer,True)
        self.assertIn('5',p['targets']);lo,hi=self.group['joints']['5']['safe_raw_limits']
        self.assertTrue(lo<=p['targets']['5']<=hi and p['targets']['5']<pose['5'])
        self.assertLess(abs(p['jaw_tilt_deg']),2.0);self.assertLess(p['ik_error_mm'],5)
        commanded=dict(pose,**p['targets'])
        self.assertLess(np.linalg.norm(model.forward_kinematics(to_deg(commanded))[:3,3]-target),.005)
        plain,_=lr.make_plan(self.group,self.states,self.expected,target,self.observer)
        self.assertNotIn('5',plain['targets']);self.assertNotEqual(plain['plan_hash'],p['plan_hash'])
    # Top-down mode, on the November23 standing-box grasp (Contact-GraspNet: jaw line 62 deg, ~11-15 deg lean).
    GRASP=np.array([.286,-.127,.023]);HOVER=np.array([.286,-.127,.053])
    def at(self,pose):
        pose=dict(pose,**{'6':1764})
        for m,v in pose.items():self.states[m].update(goal=v,position=v)
        self.expected=dict(pose);self.observer=reach.make_observer(pose,self.group['kinematic_calibration'])
    def pointed(self,**kw):return lr.make_plan(self.group,self.states,self.expected,kw.pop('target',self.GRASP),self.observer,top_down=True,**kw)
    def hover_pose(self,tilt=15):
        lim={m:self.group['joints'][m]['safe_raw_limits'] for m in '12345'}
        return lr.solve_pointing(self.group['kinematic_calibration'],{'1':2302,'2':2022,'3':2088,'4':1940,'5':3542,'6':1764},self.HOVER,lim,tilt,62)[0]
    def test_top_down_descent_reaches_and_stays_pointing(self):
        self.at(self.hover_pose())
        p,arm=self.pointed(jaw_yaw_deg=62,approach_tilt_deg=15)
        model,to_deg,_=lr.kinematics(self.group['kinematic_calibration'])
        commanded=dict(self.expected,**p['targets']);a=np.array(p['pointing']['approach'])
        self.assertEqual(set(p['targets']),{'1','2','3','4','5'});self.assertLess(p['ik_error_mm'],5)
        approach,line,reached=lr.pointing_errors(model,to_deg,commanded,a,lr.jaw_line(a,62))
        self.assertLess(np.linalg.norm(reached-self.GRASP),.005);self.assertLess(approach,3);self.assertLess(line,3)
        self.assertAlmostEqual(float(a@np.array([0,0,-1.])),np.cos(np.radians(15)),6)  # leans 15 deg off vertical
        self.assertGreater(float(a[:2]@self.GRASP[:2]),0)                            # away from the base
        self.assertLess(p['pointing']['start_approach_error_deg'],3);self.assertLess(p['pointing']['path_max_approach_error_deg'],5)
        prior={m:self.expected[m] for m in p['targets']}
        for w in p['waypoints']:
            self.assertTrue(all(abs(w[m]-prior[m])<=16 for m in w));prior=w
    def test_jaw_yaw_is_a_line_and_hash_binds_pointing(self):
        self.at(self.hover_pose())
        a,_=self.pointed(jaw_yaw_deg=62,approach_tilt_deg=15);b,_=self.pointed(jaw_yaw_deg=242,approach_tilt_deg=15)
        self.assertEqual(a['targets'],b['targets']);self.assertEqual(a['plan_hash'],b['plan_hash'])
        c,_=self.pointed(jaw_yaw_deg=62,approach_tilt_deg=20);d,_=self.pointed(jaw_yaw_deg=70,approach_tilt_deg=15)
        plain,_=lr.make_plan(self.group,self.states,self.expected,self.GRASP,self.observer)
        self.assertEqual(len({a['plan_hash'],c['plan_hash'],d['plan_hash'],plain['plan_hash']}),4)
    def test_top_down_argument_checks(self):
        self.at(self.hover_pose())
        for kw in ({'level':True},{'approach_tilt_deg':-1},{'approach_tilt_deg':91},{'approach_tilt_deg':'15'},{'jaw_yaw_deg':float('nan')},{'jaw_yaw_deg':'62'}):
            with self.subTest(kw=kw),self.assertRaises(ValueError):self.pointed(**kw)
        for kw in ({'jaw_yaw_deg':62},{'approach_tilt_deg':15}):
            with self.subTest(kw=kw),self.assertRaises(ValueError):lr.make_plan(self.group,self.states,self.expected,self.GRASP,self.observer,**kw)
    def test_unreachable_pointing_refused(self):
        # Straight down 3 cm above the box: the wrist flex would need to pass its safe limit.
        self.at(self.hover_pose())
        with self.assertRaisesRegex(ValueError,'not reachable pointing'):self.pointed(target=self.HOVER,jaw_yaw_deg=62)
    def test_large_reorientation_is_stepped_through_tilts(self):
        start={'1':2302,'2':2022,'3':2088,'4':1940,'5':3542};self.at(start)
        with self.assertRaisesRegex(ValueError,'larger approach_tilt_deg'):self.pointed(target=self.HOVER,jaw_yaw_deg=62,approach_tilt_deg=15)
        for tilt in (45,30,15):
            with self.subTest(tilt=tilt):
                p,_=self.pointed(target=self.HOVER,jaw_yaw_deg=62,approach_tilt_deg=tilt)
                self.assertTrue(all(abs(p['targets'][m]-self.expected[m])<=lr.MAX_TRAVEL for m in p['targets']))
                self.at(p['targets'])
    def test_chain_from_rest_pose_pointing_forward(self):
        # November23 after recover-hold: gripper points ~horizontally forward (97 deg from down), so start near 90 deg lean.
        self.at({'1':2186,'2':1050,'3':2789,'4':1820,'5':3188})
        with self.assertRaisesRegex(ValueError,'larger approach_tilt_deg'):self.pointed(target=self.HOVER,jaw_yaw_deg=62,approach_tilt_deg=45)
        for target,tilt in (((.25,-.10,.14),85),((.26,-.11,.12),65),((.27,-.115,.10),45),((.28,-.122,.075),30),(tuple(self.HOVER),15)):
            with self.subTest(tilt=tilt):
                p,_=self.pointed(target=np.array(target),jaw_yaw_deg=62,approach_tilt_deg=tilt)
                self.assertTrue(all(abs(p['targets'][m]-self.expected[m])<=lr.MAX_TRAVEL for m in p['targets']))
                self.at(p['targets'])
    def test_hash_binds_target(self):
        a,_=self.plan(self.here+np.array([0,0,.03]));b,_=self.plan(self.here+np.array([0,0,.031]))
        self.assertNotEqual(a['plan_hash'],b['plan_hash'])
    def test_preflight_accepts_what_a_completed_move_can_leave(self):
        target=self.here+np.array([0,0,-.02])
        # ID6 24: a gripper squeezing an object holds its goal up to the squeeze past its position (lift after a grip).
        self.assertEqual(lr.GOAL_TOLERANCE,{'1':16,'2':40,'3':24,'4':16,'5':16,'6':24})
        for m,tol in lr.GOAL_TOLERANCE.items():
            with self.subTest(m=m):
                self.states[m]['goal']=POSE[m]+tol;self.plan(target)
                self.states[m]['goal']=POSE[m]+tol+1
                with self.assertRaises(ValueError):self.plan(target)
                self.states[m]['goal']=POSE[m]
    def test_packet_only_addresses_ids_1_to_4(self):
        sdk=type('SDK',(),{'COMM_SUCCESS':0})()
        with self.assertRaises(ValueError):lr.packet(sdk,None,None,6,2000)  # gripper stays with its own tool
        with self.assertRaises(ValueError):lr.packet(sdk,None,None,7,2000)
    def _run(self,p,arm,deadband,yaw_friction=0,wrap=None):
        """Simulated servos: joints follow their goal at 8 ticks/sample; lifting, the shoulder only moves once its goal
        is more than `deadband` ticks above it and stops that far short (it fell 14-20 behind live, November16)."""
        parent=self;self.events=[]
        class SDK:COMM_SUCCESS=0
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                parent.events.append((motor,data[0]+256*data[1]));parent.states[str(motor)]['goal']=data[0]+256*data[1];return 0,0
            def read1ByteTxRx(self,ph,motor,addr):return 0,0,0
            def read2ByteTxRx(self,ph,motor,addr):return 0,0,0
        def read(sdk,ph,pk,motor,name):
            s=parent.states[str(motor)]
            if name=='position' and motor in (1,2,3,4,5) and parent.events:
                err=s['goal']-s['position'];band=deadband if motor==2 and err<0 else yaw_friction if motor==1 else 0  # gravity resists lifting only
                if abs(err)>band:s['position']+=max(-8,min(8,err-band*(1 if err>0 else -1)))
            return s.get(name,0)
        def clock():
            parent.now+=.02;return parent.now
        with patch.object(sp.jc,'read',wrap(read) if wrap else read),patch.object(sp.time,'sleep'),patch.object(sp.time,'monotonic',clock):
            return sp.execute(SDK(),None,PK(),self.group,copy.deepcopy(self.states),p,self.observer,arm)

    def test_small_lift_preserves_recorded_shoulder_support(self):
        pose={'1':2232,'2':2414,'3':1824,'4':1824,'5':3186,'6':1739}
        goals=pose|{'1':2234,'2':2390,'3':1822,'5':3188,'6':1737}
        for m,v in pose.items():self.states[m].update(position=v,goal=goals[m])
        self.expected=dict(pose)
        self.observer=reach.make_observer(pose,self.group['kinematic_calibration'])
        model,to_deg,_=lr.kinematics(self.group['kinematic_calibration'])
        here=model.forward_kinematics(to_deg(pose))[:3,3].copy()
        p,arm=self.plan(here+np.array([0,0,.005]))
        self.assertEqual(p['holding_offsets']['2'],-24)
        self.assertGreater(p['minimum_final_height_m'],.002)
        result=self._run(p,arm,24)
        self.assertTrue(result['completed'],result)
        self.assertGreaterEqual(result['achieved_model_height_change_mm'],p['minimum_final_height_m']*1000)
        self.assertTrue(all(v<=goals['2'] for m,v in self.events if m==2))
        self.assertTrue(all(abs(v-p['targets'][str(m)])<=32 for m,v in self.events if m==2))
        self.assertNotIn(6,{m for m,_ in self.events})

    def test_zero_height_cannot_report_small_lift_complete(self):
        p,arm=self.plan(self.here+np.array([0,0,.005]))
        with patch.object(self,'observer',return_value=[0,0,0]):
            result=self._run(p,arm,0)
        self.assertFalse(result['completed']);self.assertIn('progress',result['error'])

    def test_support_offsets_and_floor_are_hash_bound(self):
        target=self.here+np.array([0,0,.005]);p,arm=self.plan(target)
        other=dict(arm,preserve_hold_offsets=False)
        self.states['2']['goal']-=12
        a=sp.make_plan(self.group,self.states,self.expected,self.observer,arm)
        b=sp.make_plan(self.group,self.states,self.expected,self.observer,other)
        self.assertNotEqual(a['plan_hash'],b['plan_hash'])

    def test_small_lift_rejects_support_offset_beyond_cap(self):
        self.states['2']['goal']=POSE['2']-33
        with self.assertRaisesRegex(ValueError,'support offset'):
            self.plan(self.here+np.array([0,0,.005]))
        self.assertEqual(self.events,[])
    def test_shoulder_lift_needs_bounded_lead(self):
        target=self.here+np.array([-.02,0,.02])  # back and up: shoulder lifts against gravity
        p,arm=self.plan(target)
        self.assertLess(p['targets']['2'],POSE['2'])
        no_lead=dict(arm);no_lead.pop('lead')
        r=self._run(p,no_lead,18);self.assertFalse(r['completed']);self.assertIn('stage',r['error'])
        self.setUp();p,arm=self.plan(target)
        r=self._run(p,arm,18)
        self.assertTrue(r['completed'],r.get('error'));self.assertTrue(r['leads'])
        self.assertTrue(all(abs(l['lead'])<=arm['lead'][l['id']] for l in r['leads']))
        for m,t in p['targets'].items():  # what is left must pass the next preflight
            self.assertLessEqual(abs(self.states[m]['goal']-self.states[m]['position']),lr.GOAL_TOLERANCE[m])
    def test_single_bad_temperature_sample_is_reread(self):
        p,arm=self.plan(self.here+np.array([0,0,-.02]))
        count={'n':0};hot={'on':False}
        def wrap(sim):
            def glitchy(sdk,ph,pk,motor,name):
                if motor==1 and name=='temperature_raw' and self.events:
                    count['n']+=1
                    if count['n']==4 or hot['on']:return 110
                return sim(sdk,ph,pk,motor,name)
            return glitchy
        r=self._run(p,arm,0,wrap=wrap)
        self.assertTrue(r['completed'],r.get('error'));self.assertEqual(r['glitches'][0]['temperature'],110)
        self.setUp();p,arm=self.plan(self.here+np.array([0,0,-.02]));hot['on']=True;count['n']=0
        r=self._run(p,arm,0,wrap=wrap)
        self.assertFalse(r['completed']);self.assertIn('temperature',r['error'])
    def test_yaw_friction_and_shoulder_lift_together(self):
        # November16 back-off: shoulder lifting plus a yaw that stalled 11 ticks behind with the arm extended.
        target=self.here+np.array([-.02,-.01,.02]);p,arm=self.plan(target)
        self.assertNotEqual(p['targets']['1'],POSE['1'])
        no_lead=dict(arm);no_lead.pop('lead')
        r=self._run(p,no_lead,18,20);self.assertFalse(r['completed'])
        self.setUp();p,arm=self.plan(target)
        r=self._run(p,arm,18,20)  # yaw needing more than 16 + stage tolerance, as live
        self.assertTrue(r['completed'],r.get('error'));self.assertIn('1',{l['id'] for l in r['leads']})
    def test_execution_moves_yaw_and_holds_on_live_envelope_breach(self):
        p,arm=self.plan(self.here+np.array([0,.04,0]))
        self.assertNotEqual(p['targets']['1'],POSE['1'])  # sideways reach needs the yaw
        parent=self
        class SDK:COMM_SUCCESS=0
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                assert motor in (1,2,3,4) and addr==42
                parent.events.append((motor,data[0]+256*data[1]));parent.states[str(motor)]['goal']=data[0]+256*data[1];return 0,0
            def read1ByteTxRx(self,ph,motor,addr):return 0,0,0
            def read2ByteTxRx(self,ph,motor,addr):return 0,0,0
        def read(sdk,ph,pk,motor,name):
            s=parent.states[str(motor)]
            if name=='position' and motor in (1,2,3,4) and parent.events:s['position']+=max(-8,min(8,s['goal']-s['position']))
            return s.get(name,0)
        def clock():
            parent.now+=.02;return parent.now
        with patch.object(sp.jc,'read',read),patch.object(sp.time,'sleep'),patch.object(sp.time,'monotonic',clock):
            result=sp.execute(SDK(),None,PK(),self.group,copy.deepcopy(self.states),p,lambda states:[0,0,.5],arm)
        self.assertFalse(result['completed']);self.assertIn('envelope',result['error'])
        self.assertTrue(all(m in (1,2,3,4) for m,_ in self.events))
if __name__=='__main__':unittest.main()
