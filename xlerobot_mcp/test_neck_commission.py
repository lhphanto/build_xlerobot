import copy
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
import neck_test as nt

class NeckTest(unittest.TestCase):
    def setUp(self):
        self.group={'port_verified':True,'joints':{'7':{'observed_raw_limits':[973,3358]},'8':{'observed_raw_limits':[874,3039]}}}
        self.session={'enabled':True,'expires_unix':time.time()+600,'joints':{'8':{'center_raw':2040,'radius_raw':91,'remaining_moves':6}}}
        self.state={'model':777,'mode':0,'torque':0,'position':2040,'goal':0,'moving':0,'min':874,'max':3039,'voltage_present_raw':121,'temperature_raw':30,'voltage_min_raw':40,'voltage_max_raw':140}
    def test_rejections(self):
        cases=[({'position':2050},2072,False),({'torque':0},2072,False),({'torque':0},2131,True),({'voltage_present_raw':150},2072,True),({'moving':1},2072,True),({'model':1},2072,True),({'max':3040},2072,True),({'temperature_raw':60},2072,True),({'torque':1,'goal':2200},2072,False),({},2132,True)]
        for changes,target,enable in cases:
            with self.subTest(changes=changes,target=target):
                with self.assertRaises(ValueError):nt.validate(self.group,self.session,8,self.state|changes,target,2040,enable)
    def test_expiry_budget_and_width(self):
        for alteration in ['expiry','budget','width']:
            s=copy.deepcopy(self.session)
            if alteration=='expiry':s['expires_unix']=0
            if alteration=='budget':s['joints']['8']['remaining_moves']=0
            if alteration=='width':s['joints']['8']['radius_raw']=92
            with self.assertRaises(ValueError):nt.validate(self.group,s,8,self.state,2072,2040,True)
    def test_preload_before_enable_and_one_axis(self):
        events=[]; state=self.state
        class SDK:COMM_SUCCESS=0
        class Packet:
            def write1ByteTxRx(self,ph,motor,reg,value):
                assert state['goal']==state['position']==2040
                events.append(('enable',motor,reg,value));state['torque']=value;return 0,0
        def read(sdk,ph,pk,motor,key):return state[key] if motor==8 else 0
        def write(sdk,ph,pk,motor,reg,value):
            events.append(('write',motor,reg,value))
            if reg==42:state['goal']=value;state['position']=value
        def nudge(sdk,ph,pk,motor,target):
            self.assertEqual(state['torque'],1);events.append(('nudge',motor,target));state['goal']=target;state['position']=target;return target
        with patch.object(nt.jc,'read',read),patch.object(nt.jc,'write',write),patch.object(nt.jc,'nudge',nudge),patch.object(nt.time,'sleep'):
            nt.execute(SDK(),None,Packet(),self.group,self.session,8,2072,2040,True,lambda:events.append(('consume',)))
        self.assertEqual(events[0],('consume',))
        self.assertLess(events.index(('write',8,42,2040)),events.index(('enable',8,40,1)))
        self.assertEqual(state['position'],2072)
        self.assertTrue(all(e[1]==8 for e in events if e[0]!='consume'))
    def test_fault_before_write(self):
        with patch.object(nt.jc,'read',side_effect=RuntimeError('servo fault')),patch.object(nt.jc,'write') as write:
            with self.assertRaises(RuntimeError):nt.execute(None,None,None,self.group,self.session,8,2072,2040,True,lambda:self.fail('consumed'))
            write.assert_not_called()
    def test_failure_holds_without_torque_release(self):
        state=self.state|{'torque':1,'goal':2040};writes=[]
        with patch.object(nt.jc,'read',side_effect=lambda s,p,k,m,key:state[key] if m==8 else 0),patch.object(nt.jc,'nudge',side_effect=RuntimeError('timeout')),patch.object(nt.jc,'write',side_effect=lambda s,p,k,m,r,v:writes.append((m,r,v))):
            with self.assertRaises(RuntimeError):nt.execute(None,None,None,self.group,self.session,8,2072,2040,False,lambda:None)
        self.assertEqual(writes,[(8,42,2040)])
if __name__=='__main__':unittest.main(verbosity=2)
