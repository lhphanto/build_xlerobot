"""No hardware: neck hold preloads before enabling and commands no travel."""
import sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import neck_hold as nh

class NeckHold(unittest.TestCase):
    def setUp(self):
        # Observed November2: pan1997, tilt2042, torque off, goals read 0.
        self.states={'7':{'model':777,'mode':0,'moving':0,'min':973,'max':3358,'position':1997,'goal':0,'torque':0,'voltage_present_raw':120,'temperature_raw':30},
                     '8':{'model':777,'mode':0,'moving':0,'min':900,'max':3000,'position':2042,'goal':0,'torque':0,'voltage_present_raw':120,'temperature_raw':30}}
        self.group={'port_verified':True,'joints':{'7':{'commissioned':True,'safe_raw_limits':[1977,2139],'observed_raw_limits':[973,3358]},
                                                   '8':{'commissioned':True,'safe_raw_limits':[1959,2120],'observed_raw_limits':[900,3000]}}}
        self.expected={'7':1997,'8':2042};self.events=[];self.problem=None
        parent=self
        self.sdk=type('SDK',(),{'COMM_SUCCESS':0})()
        class PK:
            def write1ByteTxRx(self,ph,motor,addr,value):
                parent.events.append((motor,addr,value))
                if parent.problem=='ack':return 1,0
                parent.states[str(motor)]['torque']=value
                if parent.problem=='jump':parent.states[str(motor)]['position']+=20
                return 0,0
        self.pk=PK()
    def read(self,s,p,k,m,key):return self.states[str(m)].get(key,0)
    def write(self,s,p,k,m,addr,value):
        self.events.append((m,addr,value))
        if addr==42:self.states[str(m)]['goal']=value
    def run_hold(self):
        with patch.object(nh.jc,'read',self.read),patch.object(nh.jc,'write',self.write),patch.object(nh.time,'sleep'):
            return nh.control(self.sdk,None,self.pk,self.group,self.expected)
    def test_preloads_current_pose_before_enable_tilt_first(self):
        result=self.run_hold()
        self.assertEqual(self.events,[(8,44,0),(8,46,100),(8,42,2042),(8,40,1),(7,44,0),(7,46,100),(7,42,1997),(7,40,1)])
        self.assertEqual(result['enabled'],[8,7]);self.assertEqual(result['torques'],{'8':1,'7':1})
    def test_already_holding_joint_gets_no_writes(self):
        self.states['8'].update(torque=1,goal=2042)
        self.assertEqual(self.run_hold()['enabled'],[7])
        self.assertTrue(all(m==7 for m,_,_ in self.events))
    def test_any_bad_joint_blocks_all_writes(self):
        for mid,key,value in [('7','position',1970),('8','position',2125),('7','moving',1),('8','mode',1),('7','voltage_present_raw',100),('8','temperature_raw',60),('7','max',3000)]:
            with self.subTest(mid=mid,key=key):
                self.setUp();self.states[mid][key]=value;self.expected={'7':self.states['7']['position'],'8':self.states['8']['position']}
                with self.assertRaises(ValueError):self.run_hold()
                self.assertEqual(self.events,[])
    def test_stale_expected_pose_blocks(self):
        self.expected['7']+=9
        with self.assertRaises(ValueError):self.run_hold()
        self.assertEqual(self.events,[])
    def test_outstanding_target_blocks_takeover(self):
        self.states['7'].update(torque=1,goal=2050)
        with self.assertRaises(ValueError):self.run_hold()
        self.assertEqual(self.events,[])
    def test_uncommissioned_joint_blocks(self):
        self.group['joints']['8']['commissioned']=False
        with self.assertRaises(ValueError):self.run_hold()
        self.assertEqual(self.events,[])
    def test_lost_ack_stops_before_next_joint(self):
        self.problem='ack'
        with self.assertRaises(RuntimeError):self.run_hold()
        self.assertFalse(any(m==7 for m,_,_ in self.events))
    def test_displacement_after_enable_stops(self):
        self.problem='jump'
        with self.assertRaisesRegex(RuntimeError,'moved after enabling'):self.run_hold()
        self.assertFalse(any(m==7 for m,_,_ in self.events))
if __name__=='__main__':unittest.main()
