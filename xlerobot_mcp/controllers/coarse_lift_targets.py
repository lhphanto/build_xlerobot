"""Bounded offline IK alternatives, evaluated with LeRobot FK; never opens hardware."""
import itertools, math

MAX_RANKED = 12
MAX_ENDPOINT_CHECKS = 128
MAX_PATH_CHECKS = 4
PLANNING_MARGIN_M = .001


def candidates(expected, calibration, distance_mm):
    import numpy as np
    import xml.etree.ElementTree as ET
    import right_gripper_up as reach
    from lerobot.model.kinematics import RobotKinematics

    seed = reach.solve(expected, calibration, step_mm=distance_mm,
                       orientation_weight=0, lag_guard=False, coarse=True)
    names = reach.NAMES[:5]
    with reach.solver_stdout_to_stderr():
        model = RobotKinematics(str(reach.URDF), joint_names=list(names))
    limits = {j.attrib['name']: (float(j.find('limit').attrib['lower']),
                                float(j.find('limit').attrib['upper']))
              for j in ET.parse(reach.URDF).getroot().findall('joint')
              if j.find('limit') is not None}

    def pose(raw, check=False):
        q = np.array([(raw[str(calibration[n]['id'])]
                       - (calibration[n]['range_min'] + calibration[n]['range_max']) / 2)
                      * 360 / 4095 for n in names])
        if check and any(not limits[n][0] <= np.deg2rad(q[i]) <= limits[n][1]
                         for i, n in enumerate(names)):
            raise ValueError('Alternative target outside URDF limits')
        return model.forward_kinematics(q).copy()

    origin = pose(expected, True)
    wanted = origin[:3, 3].copy()
    wanted[2] += distance_mm / 1000
    ranked = {}
    low = max(expected['2'] - 64, seed['targets']['2'] - 32)
    high = min(expected['2'] + 64, seed['targets']['2'] + 32)
    for shoulder in range(low, high + 1, 2):
        raw = expected | seed['targets']
        raw['2'] = shoulder
        # Explore the redundant elbow/wrist solution for this shoulder angle.
        # This is offline math: intermediate solver values are never commands.
        for _ in range(8):
            position = pose(raw)[:3, 3]
            error = (wanted - position)[[0, 2]]
            if np.linalg.norm(error) < 1e-6:
                break
            columns = []
            for mid in ('3', '4'):
                shifted = raw.copy()
                shifted[mid] += 1
                columns.append((pose(shifted)[:3, 3] - position)[[0, 2]])
            try:
                change = np.linalg.solve(np.array(columns).T, error)
            except np.linalg.LinAlgError:
                break
            if not np.isfinite(change).all():
                break
            for mid, step in zip(('3', '4'), change):
                raw[mid] += float(np.clip(step, -16, 16))
        for elbow_shift, wrist_shift in itertools.product((-1, 0, 1), repeat=2):
            point = {mid: int(round(value)) for mid, value in raw.items()}
            point['3'] += elbow_shift
            point['4'] += wrist_shift
            if any(abs(point[str(i)] - expected[str(i)]) > 64
                   or not calibration[n]['range_min'] <= point[str(i)] <= calibration[n]['range_max']
                   for i, n in enumerate(reach.NAMES, 1)):
                continue
            try:
                transform = pose(point, True)
            except ValueError:
                continue
            residual = float(np.linalg.norm(transform[:3, 3] - wanted))
            angle = math.degrees(math.acos(float(np.clip(
                (np.trace(origin[:3, :3].T @ transform[:3, :3]) - 1) / 2, -1, 1))))
            if residual > .0005 or angle > 6:
                continue
            useful_leader = False; initial_progress = []
            for mid, tolerance in (('2', 8), ('3', 16), ('4', 4)):
                travel = point[mid] - expected[mid]
                if abs(travel) < 16:
                    continue
                initial = expected.copy()
                initial[mid] += int(np.sign(travel)) * 16
                lift = pose(initial)[:3, 3] - origin[:3, 3]
                if lift[2] > .0002 and np.linalg.norm(lift[:2]) <= .001:
                    useful_leader = True
                    partial = expected.copy()
                    partial[mid] += int(np.sign(travel)) * (16 - tolerance)
                    initial_progress.append(float(pose(partial)[2,3] - origin[2,3]))
            if not useful_leader:
                continue
            delta = transform[:3, 3] - origin[:3, 3]
            estimate = float(delta[2])
            for mid, tolerance in (('2', 8), ('3', 16), ('4', 4)):
                shifted = point.copy()
                lag = min(tolerance, abs(point[mid] - expected[mid]))
                shifted[mid] -= int(np.sign(point[mid] - expected[mid])) * lag
                estimate += float(pose(shifted)[2, 3] - transform[2, 3])
            key = tuple(point[str(i)] for i in reach.ACTIVE)
            ranked[key] = {
                'targets': {str(i): point[str(i)] for i in reach.ACTIVE},
                'predicted_displacement_m': delta.tolist(), 'model_residual_m': residual,
                'modeled_orientation_change_deg': angle, 'orientation_weight': 0,
                'linear_minimum_m': estimate,'initial_progress_m':max(initial_progress),
            }
    # The linear estimate only ranks candidates. Actual acceptance always uses
    # exhaustive integer arrival-lag checks and the complete staged path checks.
    shortlist = sorted(ranked.values(), key=lambda p: (
        p['initial_progress_m']<=.0002,-p['linear_minimum_m'], p['model_residual_m'], tuple(p['targets'].values())))[:MAX_ENDPOINT_CHECKS]
    for candidate in shortlist:
        arrivals = []
        for mid, tolerance in (('2', 8), ('3', 16), ('4', 4)):
            target = candidate['targets'][mid]
            direction = 1 if target >= expected[mid] else -1
            arrivals.append([target - direction * lag
                             for lag in range(min(tolerance, abs(target - expected[mid])) + 1)])
        heights = [float(pose(expected | dict(zip(('2', '3', '4'), values)))[2, 3] - origin[2, 3])
                   for values in itertools.product(*arrivals)]
        candidate['endpoint_minimum_m'] = min(heights)
        candidate['endpoint_maximum_m'] = max(heights)
    feasible = [p for p in shortlist if p['endpoint_maximum_m'] <= distance_mm/1000+.0015]
    return sorted(feasible, key=lambda p: (
        p['initial_progress_m']<=.0002,-p['endpoint_minimum_m'], p['model_residual_m'], tuple(p['targets'].values())))[:MAX_RANKED]
