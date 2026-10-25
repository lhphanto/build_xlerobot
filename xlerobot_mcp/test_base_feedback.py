import sys,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import base_control as b
class SDK:
    COMM_SUCCESS=0
    def __init__(self):self.calls=[];self.closed=False;self.mode=1;self.fault=0;self.vel={m:0 for m in b.IDS};self.stuck=False
    def PortHandler(self,port):return self
    def PacketHandler(self,p):return self
    def openPort(self):return True
    def setBaudRate(self,baud):return True
    def closePort(self):self.closed=True
    def read1ByteTxRx(self,ph,motor,addr):return self.read2ByteTxRx(ph,motor,addr)
    def read2ByteTxRx(self,ph,motor,addr):
        values={3:777,33:self.mode,40:1,46:self.vel[motor],58:100 if self.stuck else self.vel[motor],62:121,63:25,66:0}
        return values[addr],0,self.fault
    def write1ByteTxRx(self,ph,motor,addr,value):return self.write2ByteTxRx(ph,motor,addr,value)
    def write2ByteTxRx(self,ph,motor,addr,value):
        self.calls.append((motor,addr,value))
        if addr==46:self.vel[motor]=value
        return 0,0
class BaseTests(unittest.TestCase):
    def args(self,action):return SimpleNamespace(port='/mock',action=action,seconds=.01,speed=.1,turn_speed=20)
    def test_status_no_writes(self):
        sdk=SDK();b.run_live(self.args('status'),sdk);self.assertEqual(sdk.calls,[]);self.assertTrue(sdk.closed)
    def test_pulse_wheel_only_no_mode_changes(self):
        sdk=SDK();b.run_live(self.args('forward'),sdk)
        self.assertTrue(any(a==46 and v for m,a,v in sdk.calls));self.assertTrue(all(m in b.IDS and a in (40,46) for m,a,v in sdk.calls));self.assertEqual(sdk.calls[-4:],[(m,46,0) for m in b.IDS]);self.assertTrue(sdk.closed)
    def test_invalid_mode_never_moves(self):
        sdk=SDK();sdk.mode=0
        with self.assertRaises(ValueError):b.run_live(self.args('forward'),sdk)
        self.assertTrue(all(a==46 and v==0 for m,a,v in sdk.calls))
    def test_fault_never_moves(self):
        sdk=SDK();sdk.fault=1
        with self.assertRaises(RuntimeError):b.run_live(self.args('forward'),sdk)
        self.assertTrue(all(a==46 and v==0 for m,a,v in sdk.calls));self.assertTrue(sdk.closed)
    def test_interrupt_stops(self):
        sdk=SDK();original=b.write;nonzeros=0
        def write(*args,**kw):
            nonlocal nonzeros
            if args[4]==46 and args[5]!=0:
                nonzeros+=1
                if nonzeros==2:raise KeyboardInterrupt()
            return original(*args,**kw)
        with patch.object(b,'write',write):
            with self.assertRaises(KeyboardInterrupt):b.run_live(self.args('forward'),sdk)
        self.assertEqual(sdk.calls[-4:],[(m,46,0) for m in b.IDS]);self.assertTrue(sdk.closed)
    def test_diagnostic_file_failure_cannot_block_stop(self):
        sdk=SDK();args=self.args('forward');args.request_id='test-request'
        with patch.object(b.tempfile,'mkstemp',side_effect=OSError('disk unavailable')):
            b.run_live(args,sdk)
        self.assertEqual(sdk.calls[-4:],[(m,46,0) for m in b.IDS]);self.assertTrue(sdk.closed)

    def test_stop_feedback_rejection(self):
        sdk=SDK();sdk.stuck=True
        with patch.object(b.time,'monotonic',side_effect=[0,0,3]):
            with self.assertRaises(RuntimeError):b.verify_stop(sdk,sdk,sdk)
if __name__=='__main__':unittest.main(verbosity=2)
