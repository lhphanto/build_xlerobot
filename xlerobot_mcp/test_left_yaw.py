import sys, unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).parent/'controllers'))
import left_yaw_test as yt

class YawTrial(unittest.TestCase):
    def setUp(self):
        self.states={str(m):{'model':777,'min':500,'max':3500,'mode':0,'torque':1,'goal':1900+m*10,'position':1900+m*10,'moving':0,'voltage_present_raw':121,'temperature_raw':30} for m in range(1,7)}
        self.group={'port_verified':True,'joints':{str(m):{'safe_raw_limits':[500,3500],'observed_raw_limits':[500,3500]} for m in range(1,7)}}
        self.expected={m:s['position'] for m,s in self.states.items()}
        self.events=[];self.problem=None;self.now=0.;self.step=True;self.pauses=[]
        parent=self
        class SDK:COMM_SUCCESS=0
        self.sdk=SDK()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                assert motor==1 and addr==42 and length==6 and data[2:]==[0,0,100,0]
                value=data[0]+256*data[1];parent.events.append(value);parent.pauses.append(parent.now)
                parent.states['1']['goal']=value
                if len(parent.events)==1:
                    if parent.problem=='ack':raise RuntimeError('lost ack')
                    if parent.problem=='interrupt':raise KeyboardInterrupt()
                    if parent.problem=='other':parent.states['3']['position']+=9
                    if parent.problem=='voltage':parent.states['2']['voltage_present_raw']=100
                return 0,0
        self.pk=PK()
    def read(self,sdk,ph,pk,motor,key):
        if self.problem=='fault' and self.events:raise RuntimeError('servo fault')
        if motor==1 and key=='position' and self.step:
            s=self.states['1'];d=s['goal']-s['position'];s['position']+=max(-10,min(10,d))
        return self.states[str(motor)].get(key,0)
    def clock(self):
        self.now+=.05
        return self.now
    def run_trial(self):
        with patch.object(yt.jc,'read',self.read),patch.object(yt.time,'sleep'),patch.object(yt.time,'monotonic',self.clock):
            return yt.control(self.sdk,None,self.pk,self.group,self.expected)
    def test_round_trip_and_no_other_joint_writes(self):
        result=self.run_trial();self.assertTrue(result['completed']);self.assertEqual(self.events,[1990,1910]);self.assertEqual(self.states['1']['position'],1910);self.assertGreaterEqual(self.pauses[1]-self.pauses[0],2);self.assertEqual(result['delta_raw'],80)
    def test_outstanding_goal_no_write(self):
        self.states['2']['goal']+=10
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_stale_pose_no_write(self):
        self.expected['1']+=5
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_limit_no_write(self):
        self.group['joints']['1']['safe_raw_limits'][1]=1930
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_torque_off_no_write(self):
        self.states['4']['torque']=0
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_other_joint_drift_stops_without_return(self):
        self.problem='other'
        with self.assertRaises(RuntimeError):self.run_trial()
        self.assertEqual(len(self.events),2);self.assertNotEqual(self.events[-1],1910)
    def test_voltage_drop_stops_without_return(self):
        self.problem='voltage'
        with self.assertRaises(RuntimeError):self.run_trial()
        self.assertEqual(len(self.events),2);self.assertNotEqual(self.events[-1],1910)
    def test_lost_ack_holds_without_return(self):
        self.problem='ack'
        with self.assertRaises(RuntimeError):self.run_trial()
        self.assertEqual(self.events,[1990,1920])
    def test_interruption_holds_without_return(self):
        self.problem='interrupt'
        with self.assertRaises(KeyboardInterrupt):self.run_trial()
        self.assertEqual(self.events,[1990,1920])
    def test_timeout_no_return(self):
        self.step=False
        with self.assertRaises(RuntimeError):self.run_trial()
        self.assertEqual(self.events,[1990,1910])
    def test_fault_no_extra_write_when_feedback_unavailable(self):
        self.problem='fault'
        with self.assertRaises(RuntimeError):self.run_trial()
        self.assertEqual(self.events,[1990])
if __name__=='__main__':unittest.main(verbosity=2)
