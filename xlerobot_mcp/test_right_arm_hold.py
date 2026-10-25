import sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import right_arm_hold as ah
class ArmHold(unittest.TestCase):
    def setUp(self):
        self.states={str(m):{'model':777,'min':500,'max':3500,'mode':0,'torque':0,'goal':0,'position':1500+m*30,'moving':0,'voltage_present_raw':121,'temperature_raw':30} for m in ah.IDS}
        self.group={'port_verified':True,'joints':{str(m):{'safe_raw_limits':[500,3500],'observed_raw_limits':[500,3500]} for m in ah.IDS}}
        self.expected={m:s['position'] for m,s in self.states.items()};self.events=[];self.auto=False;self.fail_motor=None;self.fault=False;self.apply_before_fail=False
        parent=self
        class SDK:COMM_SUCCESS=0
        self.sdk=SDK()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                assert addr==42 and length==6 and data[2:]==[0,0,100,0]
                target=data[0]+256*data[1];assert target==parent.states[str(motor)]['position']
                parent.events.append((motor,'target',target))
                if parent.fail_motor==motor and not parent.apply_before_fail:raise RuntimeError('atomic write failed')
                parent.states[str(motor)]['goal']=target
                if parent.auto:parent.states[str(motor)]['torque']=1
                if parent.fail_motor==motor:raise RuntimeError('ack lost after application')
                return 0,0
            def write1ByteTxRx(self,ph,motor,addr,value):
                if value==1:assert parent.states[str(motor)]['goal']==parent.states[str(motor)]['position']
                parent.events.append((motor,addr,value));parent.states[str(motor)]['torque']=value;return 0,0
        self.pk=PK()
    def read(self,sdk,ph,pk,motor,key):
        if self.fault:raise RuntimeError('servo fault')
        return self.states[str(motor)].get(key,0)
    def write(self,sdk,ph,pk,motor,addr,value):
        self.events.append((motor,addr,value))
        if addr==42:self.states[str(motor)]['goal']=value
    def run_action(self,action):
        with patch.object(ah.jc,'read',self.read),patch.object(ah.jc,'write',self.write),patch.object(ah.time,'sleep'):
            return ah.control(self.sdk,None,self.pk,self.group,action,self.expected)
    def check_success(self):
        result=self.run_action('hold');self.assertTrue(result['completed'])
        self.assertEqual([e[0] for e in self.events if e[1]=='target'],list(ah.IDS))
        self.assertTrue(all(m in ah.IDS for m,a,v in self.events))
        self.assertTrue(all(s['torque']==1 and s['position']==self.expected[m] for m,s in self.states.items()))
        return result
    def test_atomic_targets_before_explicit_enable(self):
        self.check_success()
        for m in ah.IDS:self.assertLess(self.events.index((m,'target',self.expected[str(m)])),self.events.index((m,40,1)))
    def test_target_write_auto_enable_supported(self):
        self.auto=True;result=self.check_success();self.assertFalse(any(e[1]==40 for e in self.events));self.assertTrue(all(e['torque_after_target_write']==1 for e in result['activation']))
    def test_outside_limits_no_writes(self):
        self.states['2']['position']=499
        with self.assertRaises(ValueError):self.run_action('hold')
        self.assertEqual(self.events,[])
    def test_pose_drift_no_writes(self):
        self.states['3']['position']+=20
        with self.assertRaises(ValueError):self.run_action('hold')
        self.assertEqual(self.events,[])
    def test_fault_no_writes(self):
        self.fault=True
        with self.assertRaises(RuntimeError):self.run_action('hold')
        self.assertEqual(self.events,[])
    def test_failure_never_activates_untouched_or_releases_loaded(self):
        self.auto=True;self.fail_motor=3
        with self.assertRaises(RuntimeError):self.run_action('hold')
        self.assertFalse(any(a==40 and v==0 for m,a,v in self.events));self.assertFalse(any(m>3 for m,a,v in self.events));self.assertEqual(self.states['3']['torque'],0)
        self.assertEqual(self.events[-2:],[(m,42,self.expected[str(m)]) for m in (1,2)])
    def test_lost_ack_after_activation_holds_that_joint(self):
        self.auto=True;self.fail_motor=3;self.apply_before_fail=True
        with self.assertRaises(RuntimeError):self.run_action('hold')
        self.assertEqual(self.events[-3:],[(m,42,self.expected[str(m)]) for m in (1,2,3)])
    def test_supported_release_torque_only(self):
        for s in self.states.values():s['torque']=1
        self.run_action('release');self.assertEqual(self.events,[(m,40,0) for m in ah.IDS])
    def test_id5_hold_window_does_not_approve_config(self):
        self.group['joints']['5']={'safe_raw_limits':None,'observed_raw_limits':[0,4095]}
        self.states['5']['min']=0;self.states['5']['max']=4095
        self.check_success()
        self.assertIsNone(self.group['joints']['5']['safe_raw_limits'])
    def test_id5_outside_local_hold_window_rejects(self):
        self.group['joints']['5']={'safe_raw_limits':None,'observed_raw_limits':[0,4095]}
        self.states['5']['min']=0;self.states['5']['max']=4095
        self.states['5']['position']+=9
        with self.assertRaises(ValueError):self.run_action('hold')
        self.assertEqual(self.events,[])
    def test_id5_changed_limits_rejects(self):
        self.group['joints']['5']={'safe_raw_limits':None,'observed_raw_limits':[1,4095]}
        with self.assertRaises(ValueError):self.run_action('hold')
        self.assertEqual(self.events,[])
    def test_cli_pins_right_group(self):
        source=Path(ah.__file__).read_text()
        self.assertIn("config['groups']['right-arm']",source)
        self.assertNotIn("config['groups']['left-arm']",source)
if __name__=='__main__':unittest.main(verbosity=2)
