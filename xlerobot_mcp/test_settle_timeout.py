import copy,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent/'controllers'))
from settle_timeout import stationary_timeout_evidence

class StationaryTimeoutGate(unittest.TestCase):
    def rows(self):
        return [dict(elapsed_s=t,stage='return',target_raw=2000,goal_raw=2000,position_raw=2013,
                     moving=0,present_speed_raw=0,torque=1,servo_status_raw=0,
                     voltage_raw=120,temperature_c=32,present_load_raw=112,motor_phase_current_raw=7)
                for t in (1.,1.08,1.16)]
    def test_observed_stationary_low_output_case(self):
        result=stationary_timeout_evidence(self.rows(),2000,'return')
        self.assertEqual(result['residual_ticks'],13)
    def test_direction_bit_handled_for_low_output(self):
        rows=self.rows()
        for row in rows:row['present_load_raw']=1024+112
        self.assertIsNotNone(stationary_timeout_evidence(rows,2000,'return'))
    def test_unsafe_or_uncertain_samples_rejected(self):
        for field,value in [('goal_raw',2001),('target_raw',2001),('stage','out'),('moving',1),
                            ('present_speed_raw',50),('torque',0),('servo_status_raw',4),
                            ('voltage_raw',109),('voltage_raw',131),('temperature_c',60),
                            ('present_load_raw',151),('present_load_raw',2048),
                            ('motor_phase_current_raw',17),('position_raw',2017)]:
            with self.subTest(field=field,value=value):
                rows=self.rows();rows[-1][field]=value
                self.assertIsNone(stationary_timeout_evidence(rows,2000,'return'))
    def test_sample_count_and_timing_must_be_fresh_and_ordered(self):
        rows=self.rows()
        self.assertIsNone(stationary_timeout_evidence(rows[:2],2000,'return'))
        for times in [(1.,1.02,1.04),(1.,1.3,1.6),(1.1,1.,1.2)]:
            changed=copy.deepcopy(rows)
            for row,t in zip(changed,times):row['elapsed_s']=t
            self.assertIsNone(stationary_timeout_evidence(changed,2000,'return'))
    def test_position_drift_rejected_even_if_moving_flag_zero(self):
        rows=self.rows();rows[0]['position_raw']=2010
        self.assertIsNone(stationary_timeout_evidence(rows,2000,'return'))
    def test_missing_feedback_not_assumed_zero(self):
        rows=self.rows();del rows[-1]['present_speed_raw']
        self.assertIsNone(stationary_timeout_evidence(rows,2000,'return'))
if __name__=='__main__':unittest.main(verbosity=2)
