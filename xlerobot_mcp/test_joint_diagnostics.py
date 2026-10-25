import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import joint_control as jc
class Diagnostics(unittest.TestCase):
    def test_status_reads_only_and_reports_faults(self):
        class SDK:COMM_SUCCESS=0
        class PK:
            def read1ByteTxRx(self,ph,m,addr):return addr,0,1 if addr==65 else 0
            def read2ByteTxRx(self,ph,m,addr):return addr*10,0,0
            def getRxPacketError(self,error):return 'fault'
            def __getattr__(self,name):raise AssertionError('Unexpected method: '+name)
        state,faults=jc.diagnostic_status(SDK(),None,PK(),3)
        self.assertEqual(state['torque_limit_raw'],480)
        self.assertEqual(state['homing_offset_raw'],310)
        self.assertEqual(state['dead_zone_positive_raw'],26)
        self.assertEqual(state['motor_phase_current_raw'],690)
        self.assertEqual(set(faults),{'servo_status_raw'})
        self.assertNotIn('torque_limit_raw',jc.REGS)
    def test_comm_failure_not_silenced(self):
        class SDK:COMM_SUCCESS=0
        class PK:
            def read2ByteTxRx(self,*args):return 0,-1,0
        with self.assertRaises(RuntimeError):jc.diagnostic_status(SDK(),None,PK(),3)
if __name__=='__main__':unittest.main(verbosity=2)
