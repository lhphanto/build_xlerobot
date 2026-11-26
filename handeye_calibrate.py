#!/usr/bin/env python3
"""Camera-to-left-arm (eye-to-hand) calibration with an ArUco marker on the gripper.

capture: with the left arm holding still and the neck locked, take one stereo snapshot
         through the robot's MCP service, read the six left-arm joints, find the marker in
         the left half, and save the pose to <dir>/pose_NN.json. Read-only; no motion.
solve:   combine all saved poses with cv2.calibrateHandEye to get where the camera (left
         half's lens) sits relative to the arm base, and report per-pose residuals.

The arm pose comes from the SO101 URDF and the left arm's saved kinematic calibration,
the same model the lift planner uses; it has not been checked against the real arm, so
large residuals can mean the model, not the camera, is off.

Example:
    python handeye_calibrate.py capture --marker-mm 30
    python handeye_calibrate.py solve
"""
import argparse
import glob
import json
import os
import subprocess
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
MCP = os.path.join(ROOT, 'xlerobot_mcp')
CLIENT_PY = os.path.join(MCP, '.venv', 'bin', 'python')
URDF = os.path.join(ROOT, 'SO101', 'so101_new_calib.urdf')
NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
DICT = cv2.aruco.DICT_4X4_50


def call(tool, args=None, *extra):
    r = subprocess.run([CLIENT_PY, os.path.join(MCP, 'client.py'), 'call', tool, '--arguments', json.dumps(args or {}), *extra],
                       capture_output=True, text=True, timeout=120)
    if r.returncode:
        sys.exit(f'{tool} failed: {r.stderr.strip()[-400:]}')
    return json.loads(r.stdout)['results']


def joints():
    out = {}
    for i in range(1, 7):
        st = json.loads(call('joint_status', {'group': 'left-arm', 'servo_id': i})[0]['stdout'])['state']
        out[str(i)] = st
    return out


def gripper_pose(positions, cal):
    """4x4 gripper pose in the arm base frame (meters), from raw joint positions."""
    from lerobot.model.kinematics import RobotKinematics
    model = RobotKinematics(URDF, joint_names=list(NAMES))
    q = np.array([(positions[str(cal[n]['id'])] - (cal[n]['range_min'] + cal[n]['range_max']) / 2) * 360 / 4095 for n in NAMES])
    return model.forward_kinematics(q)


def marker_pose(image, cam, marker_m):
    """Marker pose in the left-half lens frame from fisheye-undistorted corners, or None."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT), cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or 0 not in ids.ravel():
        return None
    c = corners[list(ids.ravel()).index(0)].reshape(-1, 1, 2).astype(np.float64)
    c = cv2.cornerSubPix(gray, c.astype(np.float32), (5, 5), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)).astype(np.float64)
    K, D = np.array(cam['camera_matrix']), np.array(cam['distortion']).reshape(4, 1)
    # Undistort to normalized coordinates, then solve PnP as an ideal pinhole camera.
    norm = cv2.fisheye.undistortPoints(c, K, D)
    h = marker_m / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, norm, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    return {'rvec': rvec.ravel().tolist(), 'tvec_m': tvec.ravel().tolist(), 'corners_px': c.reshape(-1, 2).tolist()}


def capture(args):
    os.makedirs(args.dir, exist_ok=True)
    cal = json.load(open(os.path.join(MCP, 'robot_config.json')))['groups']['left-arm']['kinematic_calibration']
    cam = json.load(open(args.camera))['left']
    before = joints()
    if any(s['torque'] != 1 for s in before.values()):
        sys.exit('All six left-arm joints must be holding; hold the arm first')
    neck = {i: json.loads(call('joint_status', {'group': 'neck', 'servo_id': i})[0]['stdout'])['state']['position'] for i in (7, 8)}
    shot = call('camera_snapshot', None, '--output-dir', args.dir)
    image_path = shot[1]['saved_image']
    after = joints()
    # A held arm settles a few ticks under load; the frame is taken late in the capture, so
    # the after-snapshot reading is used. Larger drift means the arm was moving.
    drift = {m: after[m]['position'] - before[m]['position'] for m in after}
    moved = {m: d for m, d in drift.items() if abs(d) > 5}
    if moved:
        sys.exit(f'Arm moved during the snapshot {moved}; not saved')
    image = cv2.imread(image_path)
    left_half = image[:, :image.shape[1] // 2]
    marker = marker_pose(left_half, cam, args.marker_mm / 1000)
    if marker is None:
        sys.exit(f'Marker not found in the left half of {image_path}; adjust the pose and capture again')
    positions = {m: s['position'] for m, s in after.items()}
    existing = glob.glob(os.path.join(args.dir, 'pose_*.json'))
    n = 1 + max([int(os.path.basename(p)[5:7]) for p in existing], default=0)
    record = {'pose': n, 'image': image_path, 'joints_raw': positions, 'joint_drift_during_capture': drift, 'neck_raw': neck,
              'gripper_in_base': gripper_pose(positions, cal).tolist(), 'marker_in_camera': marker,
              'marker_mm': args.marker_mm}
    path = os.path.join(args.dir, f'pose_{n:02d}.json')
    with open(path, 'w') as f:
        json.dump(record, f, indent=2)
    t = np.array(marker['tvec_m']) * 100
    print(f'saved {path}: marker {t[2]:.1f} cm in front, {t[0]:+.1f} cm right, {t[1]:+.1f} cm down; joints {positions}')


def solve(args):
    records = [json.load(open(p)) for p in sorted(glob.glob(os.path.join(args.dir, 'pose_*.json')))]
    if len(records) < 4:
        sys.exit(f'{len(records)} poses; capture at least 4 (10-12 recommended)')
    spread = max(abs(r['neck_raw'][k] - records[0]['neck_raw'][k]) for r in records for k in records[0]['neck_raw'])
    if spread > 4:
        print(f'WARNING: neck moved between poses by up to {spread} ticks; results will be inconsistent')
    G = [np.array(r['gripper_in_base']) for r in records]
    C = [_pose(r) for r in records]
    # Unknowns: X = camera in arm base, Y = marker on gripper, with G_i Y = X C_i for every pose.
    X = _park(G, C)
    Y = _average_pose([np.linalg.inv(g) @ X @ c for g, c in zip(G, C)])
    X, Y = _refine(G, C, X, Y)
    errors = [float(np.linalg.norm((g @ Y)[:3, 3] - (X @ c)[:3, 3]) * 1000) for g, c in zip(G, C)]
    cam_in_base, offset = X, Y
    print(f'camera (left-half lens) at {np.round(X[:3, 3] * 100, 1)} cm in arm base frame; '
          f'marker position residual mean {np.mean(errors):.1f} mm, max {np.max(errors):.1f} mm')
    out = {'method': 'park+least_squares', 'camera_left_lens_in_arm_base': cam_in_base.tolist(), 'marker_in_gripper': offset.tolist(),
           'per_pose_residual_mm': errors, 'poses': [r['pose'] for r in records], 'neck_raw': records[0]['neck_raw'],
           'note': 'Valid only while the neck stays at neck_raw. Arm pose from the unverified SO101 model.'}
    path = os.path.join(args.dir, 'handeye_result.json')
    with open(path, 'w') as f:
        json.dump(out, f, indent=2)
    print(f'per-pose residual mm {[round(e, 1) for e in errors]}; saved {path}')


def _park(G, C):
    """Closed-form AX = XB (Park and Martin) from all pose pairs; OpenCV 5 dropped calibrateHandEye."""
    alphas, betas, rows, rhs = [], [], [], []
    pairs = [(i, j) for i in range(len(G)) for j in range(i + 1, len(G))]
    for i, j in pairs:
        A = G[j] @ np.linalg.inv(G[i])
        B = C[j] @ np.linalg.inv(C[i])
        alphas.append(cv2.Rodrigues(A[:3, :3])[0].ravel()); betas.append(cv2.Rodrigues(B[:3, :3])[0].ravel())
    # R_A R_X = R_X R_B, so the rotation axes satisfy alpha = R_X beta: a Kabsch fit.
    H = np.array(betas).T @ np.array(alphas)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    for i, j in pairs:
        A = G[j] @ np.linalg.inv(G[i]); B = C[j] @ np.linalg.inv(C[i])
        rows.append(A[:3, :3] - np.eye(3)); rhs.append(R @ B[:3, 3] - A[:3, 3])
    t = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)[0]
    X = np.eye(4); X[:3, :3] = R; X[:3, 3] = t
    return X


def _average_pose(poses):
    rots = np.array([p[:3, :3] for p in poses]).sum(axis=0)
    U, _, Vt = np.linalg.svd(rots)
    out = np.eye(4); out[:3, :3] = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    out[:3, 3] = np.mean([p[:3, 3] for p in poses], axis=0)
    return out


def _refine(G, C, X, Y):
    """Least squares over both transforms: marker position and orientation agreement in every pose."""
    from scipy.optimize import least_squares

    def unpack(v):
        a = np.eye(4); a[:3, :3] = cv2.Rodrigues(v[:3])[0]; a[:3, 3] = v[3:6]
        b = np.eye(4); b[:3, :3] = cv2.Rodrigues(v[6:9])[0]; b[:3, 3] = v[9:]
        return a, b

    def residual(v):
        a, b = unpack(v); res = []
        for g, c in zip(G, C):
            lhs, rhs = g @ b, a @ c
            res.extend(lhs[:3, 3] - rhs[:3, 3])                          # meters
            res.extend(0.05 * (lhs[:3, :3] - rhs[:3, :3]).ravel())       # weight rotation like ~5 cm lever
        return np.array(res)

    v0 = np.concatenate([cv2.Rodrigues(X[:3, :3])[0].ravel(), X[:3, 3], cv2.Rodrigues(Y[:3, :3])[0].ravel(), Y[:3, 3]])
    return unpack(least_squares(residual, v0).x)


def _pose(record):
    m = np.eye(4)
    m[:3, :3] = cv2.Rodrigues(np.array(record['marker_in_camera']['rvec']))[0]
    m[:3, 3] = record['marker_in_camera']['tvec_m']
    return m


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['capture', 'solve'])
    parser.add_argument('--dir', default=os.path.join(ROOT, 'samples', 'hand_eye'))
    parser.add_argument('--camera', default=os.path.join(ROOT, 'samples', 'calibration', 'calibration_fisheye.json'))
    parser.add_argument('--marker-mm', type=float, default=30.0, help='measured side of the printed black square')
    args = parser.parse_args()
    capture(args) if args.command == 'capture' else solve(args)


if __name__ == '__main__':
    main()
