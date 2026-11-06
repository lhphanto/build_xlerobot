"""No hardware: saved right-ID3 gain change is torque-off, fixed-value and verified."""
import sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import right_elbow_p_set as ps

class SetP(unittest.TestCase):
    def setUp(self):
        self.state={'model':777,'mode':0,'moving':0,'torque':0,'position':3011,'goal':2995,'voltage_present_raw':120}
        self.regs={21:16,22:32,23:0,55:1};self.writes=[];self.fail=None
        self.sdk=type('SDK',(),{'COMM_SUCCESS':0})();self.group={'port_verified':True}
        parent=self
        class PK:
            def read1ByteTxRx(self,ph,motor,addr):
                assert motor==3
                return parent.regs[addr],0,0
            def write1ByteTxRx(self,ph,motor,addr,value):
                assert motor==3
                parent.writes.append((addr,value))
                if parent.fail==(addr,value):return 1,0
                parent.regs[addr]=value;return 0,0
        self.pk=PK();self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.record=Path(self.tmp.name)/'pending.json'
    def read(self,s,p,k,m,key):
        assert m==3
        return self.state.get(key,0)
    def run_set(self,p=32):
        with patch.object(ps.jc,'read',self.read):
            return ps.control(self.sdk,None,self.pk,self.group,p,self.record)
    def test_sets_p32_locks_and_reports(self):
        result=self.run_set()
        self.assertEqual(self.writes,[(55,0),(21,32),(55,1)]);self.assertEqual(self.regs[21],32);self.assertEqual(self.regs[55],1)
        self.assertTrue(result['changed']);self.assertEqual(result['before']['P'],16)
    def test_can_undo_to_p16(self):
        self.regs[21]=32
        self.assertEqual(self.run_set(16)['p_raw'],16);self.assertEqual(self.regs[21],16)
    def test_same_value_writes_nothing(self):
        self.regs[21]=32
        self.assertFalse(self.run_set()['changed']);self.assertEqual(self.writes,[])
    def test_only_fixed_values(self):
        for p in (0,24,40,True,'32'):
            with self.subTest(p=p),self.assertRaises(ValueError):self.run_set(p)
        self.assertEqual(self.writes,[])
    def test_requires_torque_off_and_quiet_joint(self):
        for key,value in [('torque',1),('moving',1),('mode',1),('model',1),('voltage_present_raw',100)]:
            with self.subTest(key=key):
                self.setUp();self.state[key]=value
                with self.assertRaises(ValueError):self.run_set()
                self.assertEqual(self.writes,[])
    def test_unexpected_settings_block(self):
        for addr,value in [(21,20),(22,16),(23,1),(55,0)]:
            with self.subTest(addr=addr):
                self.setUp();self.regs[addr]=value
                with self.assertRaises(ValueError):self.run_set()
                self.assertEqual(self.writes,[])
    def test_pending_recovery_blocks(self):
        self.record.write_text('{}')
        with self.assertRaisesRegex(ValueError,'Pending'):self.run_set()
        self.assertEqual(self.writes,[])
    def test_failed_gain_write_still_relocks(self):
        self.fail=(21,32)
        with self.assertRaises(RuntimeError):self.run_set()
        self.assertEqual(self.writes,[(55,0),(21,32),(55,1)]);self.assertEqual(self.regs[55],1);self.assertEqual(self.regs[21],16)
    def test_unverified_bus_blocks(self):
        self.group['port_verified']=False
        with self.assertRaises(ValueError):self.run_set()
        self.assertEqual(self.writes,[])
if __name__=='__main__':unittest.main()
