"""Read-only classification of a narrowly verified stationary settling timeout."""

class StationarySettlingTimeout(RuntimeError):
    def __init__(self, message, evidence):
        super().__init__(message)
        self.evidence = evidence


def stationary_timeout_evidence(recent, target, stage):
    """Return evidence only; never write hardware or accept a failed motion.

    Conservative eligibility for preserving an existing gravity-support target:
    three validated samples spanning >=0.1s, <=2tick variation, <=16tick error,
    zero speed/moving, torque on, fault-free, and low output/current. These
    diagnostic bands are not force, cable-current or payload qualification.
    Callers must already have checked all joints, limits and target identity.
    """
    if len(recent) != 3:
        return None
    positions = []
    try:
        for row in recent:
            if row['stage'] != stage or row['target_raw'] != target or row['goal_raw'] != target:
                return None
            if row['moving'] != 0 or row['present_speed_raw'] != 0 or row['torque'] != 1 or row['servo_status_raw'] != 0:
                return None
            if not 110 <= row['voltage_raw'] <= 130 or not 0 <= row['temperature_c'] < 60:
                return None
            if not 0 <= row['present_load_raw'] <= 2047 or (row['present_load_raw'] & 1023) > 150:
                return None
            if not 0 <= row['motor_phase_current_raw'] <= 16:
                return None
            if abs(row['position_raw'] - target) > 16:
                return None
            positions.append(row['position_raw'])
        times = [row['elapsed_s'] for row in recent]
        if not times[0] < times[1] < times[2] or times[2] - times[0] < .1 or times[2] - times[0] > .5:
            return None
    except (KeyError, TypeError):
        return None
    if max(positions) - min(positions) > 2:
        return None
    return {'samples':3, 'span_seconds':round(times[-1]-times[0],4),
            'goal_raw':target, 'positions_raw':positions,
            'residual_ticks':positions[-1]-target,
            'note':'Positioning test still failed. Existing target and torque retained; no new target sent.'}
