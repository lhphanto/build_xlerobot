import sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import pan_torque as pt
class TestPan(unittest.TestCase):
    def setUp(self):
        self.state={'model':777,'mode':0,'moving':0,'min':973,'max':3358,'position':1955,'goal':1955,'torque':1,'voltage_present_raw':121}
        self.group={'port_verified':True,'joints':{'7':{'observed_raw_limits':[973,3358]}}};self.tilt=1;self.events=[]
        self.sdk=type('SDK',(),{'COMM_SUCCESS':0})()
        parent=self
        class PK:
            def write1ByteTxRx(self,ph,motor,addr,value):
                parent.events.append((motor,addr,value));parent.state['torque']=value;return 0,0
        self.pk=PK()
    def read(self,s,p,k,m,key):return self.state.get(key,0) if m==7 else (self.tilt if key=='torque' else 0)
    def write(self,s,p,k,m,addr,value):
        self.events.append((m,addr,value))
        if addr==42:self.state['goal']=value
    def run_action(self,action):
        with patch.object(pt.jc,'read',self.read),patch.object(pt.jc,'write',self.write),patch.object(pt.time,'sleep'):
            return pt.control(self.sdk,None,self.pk,self.group,action,1955)
    def test_release_only_pan_torque(self):
        result=self.run_action('release');self.assertEqual(self.events,[(7,40,0)]);self.assertEqual(result['torque'],0);self.assertEqual(result['tilt_torque_after'],1)
    def test_hold_preloads_before_enable(self):
        self.state.update(torque=0,goal=1000);self.run_action('hold');self.assertEqual(self.events,[(7,44,0),(7,46,100),(7,42,1955),(7,40,1)])
    def test_fault_has_no_writes(self):
        with patch.object(pt.jc,'read',side_effect=RuntimeError('fault')):
            with self.assertRaises(RuntimeError):pt.control(self.sdk,None,self.pk,self.group,'release',1955)
        self.assertEqual(self.events,[])
    def test_tilt_off_rejected(self):
        self.tilt=0
        with self.assertRaises(ValueError):self.run_action('release')
        self.assertEqual(self.events,[])
    def test_pending_motion_rejected(self):
        self.state['goal']=2050
        with self.assertRaises(ValueError):self.run_action('release')
        self.assertEqual(self.events,[])
if __name__=='__main__':unittest.main(verbosity=2)
