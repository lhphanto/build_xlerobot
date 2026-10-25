import sys, unittest, io, json
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).parent/'controllers'))
import left_elbow_range_test as yt

class ElbowRangeTrial(unittest.TestCase):
    def setUp(self):
        self.states={str(m):{'model':777,'min':500,'max':3500,'mode':0,'torque':1,'goal':1900+m*10,'position':1900+m*10,'moving':0,'voltage_present_raw':121,'temperature_raw':30} for m in range(1,7)}
        self.group={'port_verified':True,'joints':{str(m):{'safe_raw_limits':[500,3500],'observed_raw_limits':[500,3500]} for m in range(1,7)}}
        self.expected={m:s['position'] for m,s in self.states.items()}
        self.events=[];self.problem=None;self.now=0.;self.step=True;self.pauses=[];self.load_raw=24;self.phase_current_raw=0;self.speed_raw=0;self.center_error=0;self.endpoint_error=0
        parent=self
        class SDK:COMM_SUCCESS=0
        self.sdk=SDK()
        class PK:
            def writeTxRx(self,ph,motor,addr,length,data):
                assert motor==3 and addr==42 and length==6 and data[2:]==[0,0,100,0]
                value=data[0]+256*data[1];parent.events.append(value);parent.pauses.append(parent.now)
                parent.states['3']['goal']=value
                if len(parent.events)==1:
                    if parent.problem=='ack':raise RuntimeError('lost ack')
                    if parent.problem=='interrupt':raise KeyboardInterrupt()
                    if parent.problem=='other':parent.states['4']['position']+=9
                    if parent.problem=='voltage':parent.states['3']['voltage_present_raw']=100
                return 0,0
            def read1ByteTxRx(self,ph,motor,addr):
                assert motor==3 and addr==65
                return (4 if parent.problem=='status' else 0),0,0
            def read2ByteTxRx(self,ph,motor,addr):
                assert motor==3 and addr in (58,60,69)
                if parent.problem=='diag_comm':return 0,-1,0
                return ({60:parent.load_raw,69:parent.phase_current_raw,58:parent.speed_raw}[addr]),0,0
        self.pk=PK()
    def read(self,sdk,ph,pk,motor,key):
        if self.problem=='fault' and self.events:raise RuntimeError('servo fault')
        if motor==3 and key=='position' and self.step and self.events:
            s=self.states['3'];d=s['goal']-s['position'];s['position']+=max(-10,min(10,d))
            if len(self.events) in (2,4) and self.center_error:s['position']=s['goal']+self.center_error
            if len(self.events)==1 and self.endpoint_error:s['position']=s['goal']-self.endpoint_error
        if motor==3 and key=='moving' and len(self.events) in (2,4) and self.problem=='center_moving':return 1
        return self.states[str(motor)].get(key,0)
    def clock(self):
        self.now+=.05
        return self.now
    def run_trial(self):
        with patch.object(yt.jc,'read',self.read),patch.object(yt.time,'sleep'),patch.object(yt.time,'monotonic',self.clock):
            return yt.control(self.sdk,None,self.pk,self.group,self.expected)
    def test_round_trip_and_no_other_joint_writes(self):
        result=self.run_trial();self.assertTrue(result['completed']);self.assertEqual(self.events,[2010,1930,1850,1930]);self.assertEqual(self.states['3']['position'],1930);self.assertGreaterEqual(self.pauses[1]-self.pauses[0],2);self.assertEqual(result['delta_raw'],80);self.assertEqual({r['stage'] for r in result['trace']},{'positive','center-after-positive','negative','center-after-negative'});self.assertTrue(all(r['present_load_raw']==24 for r in result['trace']))
    def test_outstanding_goal_no_write(self):
        self.states['3']['goal']+=17
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_stale_pose_no_write(self):
        self.expected['3']+=5
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_limit_no_write(self):
        self.group['joints']['3']['safe_raw_limits'][1]=1940
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_torque_off_no_write(self):
        self.states['3']['torque']=0
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_other_joint_drift_stops_without_return(self):
        self.problem='other'
        with self.assertRaises(RuntimeError):self.run_trial()
        self.assertEqual(len(self.events),2);self.assertNotEqual(self.events[-1],1930)
    def test_voltage_drop_stops_without_return(self):
        self.problem='voltage'
        with self.assertRaises(RuntimeError):self.run_trial()
        self.assertEqual(len(self.events),2);self.assertNotEqual(self.events[-1],1930)
    def test_lost_ack_holds_without_return(self):
        self.problem='ack'
        with self.assertRaises(RuntimeError):self.run_trial()
        self.assertEqual(self.events,[2010,1940])
    def test_interruption_holds_without_return(self):
        self.problem='interrupt'
        with self.assertRaises(KeyboardInterrupt):self.run_trial()
        self.assertEqual(self.events,[2010,1940])
    def test_timeout_no_return(self):
        self.step=False
        with self.assertRaises(RuntimeError):self.run_trial()
        self.assertEqual(self.events,[2010,1930])
    def test_fault_no_extra_write_when_feedback_unavailable(self):
        self.problem='fault'
        with self.assertRaises(RuntimeError):self.run_trial()
        self.assertEqual(self.events,[2010])
    def test_timeout_preserves_tracking_and_cleanup_evidence(self):
        self.step=False
        output=io.StringIO()
        with redirect_stdout(output),self.assertRaises(RuntimeError):self.run_trial()
        result=json.loads(output.getvalue())
        self.assertFalse(result['completed']);self.assertFalse(result['return_attempted'])
        self.assertTrue(result['cleanup']['hold_acknowledged'])
        self.assertTrue(all(r['error_ticks']==80 for r in result['trace']))
    def test_servo_status_fault_holds_without_return(self):
        self.problem='status';output=io.StringIO()
        with redirect_stdout(output),self.assertRaises(RuntimeError):self.run_trial()
        result=json.loads(output.getvalue())
        self.assertEqual(result['trace'][0]['servo_status_raw'],4)
        self.assertTrue(result['cleanup']['hold_acknowledged']);self.assertEqual(len(self.events),2)
    def test_diagnostic_comm_failure_holds_without_return(self):
        self.problem='diag_comm';output=io.StringIO()
        with redirect_stdout(output),self.assertRaises(RuntimeError):self.run_trial()
        result=json.loads(output.getvalue());self.assertTrue(result['cleanup']['hold_acknowledged']);self.assertEqual(len(self.events),2)
    def test_negative_endpoint_outside_limits_no_write(self):
        self.group['joints']['3']['safe_raw_limits'][0]=1860
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_center_plus_sixteen_accepted_on_both_returns(self):
        self.center_error=16;r=self.run_trial();self.assertTrue(r['completed'])
        self.assertEqual([x['tolerance_raw'] for x in r['stages']],[4,16,4,16])
        self.assertEqual(self.events,[2010,1930,1850,1930])
    def test_center_minus_sixteen_accepted(self):
        self.center_error=-16;self.assertTrue(self.run_trial()['completed'])
    def test_center_seventeen_rejected_without_second_excursion(self):
        self.center_error=17;out=io.StringIO()
        with redirect_stdout(out),self.assertRaises(RuntimeError):self.run_trial()
        r=json.loads(out.getvalue());self.assertFalse(r['completed']);self.assertEqual(r['failed_stage'],'center-after-positive')
        self.assertEqual(self.events,[2010,1930,1947])
    def test_endpoint_five_still_rejected(self):
        self.endpoint_error=5;out=io.StringIO()
        with redirect_stdout(out),self.assertRaises(RuntimeError):self.run_trial()
        r=json.loads(out.getvalue());self.assertEqual(r['failed_stage'],'positive');self.assertFalse(r['return_attempted'])
    def test_center_must_be_stationary(self):
        self.center_error=7;self.problem='center_moving';out=io.StringIO()
        with redirect_stdout(out),self.assertRaises(RuntimeError):self.run_trial()
        r=json.loads(out.getvalue());self.assertEqual(r['failed_stage'],'center-after-positive')
    def test_observed_thirteen_tick_returns_now_accepted_without_reset(self):
        self.center_error=13;r=self.run_trial();self.assertTrue(r['completed'])
        self.assertEqual(self.events,[2010,1930,1850,1930]);self.assertEqual(self.states['3']['goal'],1930)
        self.assertTrue(all(st['torque']==1 for st in self.states.values()))
    def test_stationary_endpoint_failure_still_preserves_target(self):
        self.endpoint_error=13;out=io.StringIO()
        with redirect_stdout(out),self.assertRaises(yt.StationarySettlingTimeout):self.run_trial()
        r=json.loads(out.getvalue());self.assertFalse(r['completed']);self.assertEqual(r['cleanup']['action'],'preserve_existing_target')
        self.assertEqual(self.events,[2010])
    def test_high_output_keeps_legacy_failure_hold(self):
        self.endpoint_error=13;self.load_raw=151;out=io.StringIO()
        with redirect_stdout(out),self.assertRaises(RuntimeError):self.run_trial()
        r=json.loads(out.getvalue());self.assertEqual(r['failure_kind'],'other');self.assertTrue(r['cleanup']['hold_acknowledged'])
        self.assertEqual(self.events,[2010,1997])
    def test_nonzero_speed_keeps_legacy_failure_hold(self):
        self.endpoint_error=13;self.speed_raw=50;out=io.StringIO()
        with redirect_stdout(out),self.assertRaises(RuntimeError):self.run_trial()
        r=json.loads(out.getvalue());self.assertEqual(r['failure_kind'],'other');self.assertTrue(r['cleanup']['hold_acknowledged'])
    def test_high_phase_current_keeps_legacy_failure_hold(self):
        self.endpoint_error=13;self.phase_current_raw=17;out=io.StringIO()
        with redirect_stdout(out),self.assertRaises(RuntimeError):self.run_trial()
        r=json.loads(out.getvalue());self.assertEqual(r['failure_kind'],'other');self.assertTrue(r['cleanup']['hold_acknowledged'])
    def test_existing_id3_residual_sixteen_allowed_without_preload(self):
        self.states['3']['goal']-=16;r=self.run_trial();self.assertTrue(r['completed'])
        self.assertEqual(self.events[0],2010)
    def test_other_joint_goal_error_nine_still_rejected(self):
        self.states['2']['goal']+=9
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
    def test_existing_id3_goal_outside_limits_rejected(self):
        self.states['3']['min']=1920;self.group['joints']['3']['observed_raw_limits'][0]=1920
        self.group['joints']['3']['safe_raw_limits'][0]=1920;self.states['3']['goal']=1919
        with self.assertRaises(ValueError):self.run_trial()
        self.assertEqual(self.events,[])
if __name__=='__main__':unittest.main(verbosity=2)
