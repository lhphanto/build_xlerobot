#!/usr/bin/env python3
"""Turn a pixel in the left camera image into a left-arm reach target, and optionally preview the reach.

The pixel is in the raw left half of the stereo frame (1920x1080, as saved by the robot's
snapshots). Depth comes from fisheye-rectified stereo matching (as in calculate_depth.py);
the camera-to-arm calibration then gives the point in the left arm base frame (x forward,
y left, z up, meters). The target is placed --hover-cm above that point so the gripper
stops short of the surface. Nothing moves; --preview only asks the server for a plan.

Examples:
    python reach_point.py --pixel 960 700                     # fresh snapshot
    python reach_point.py --pixel 960 700 --image frame.png   # existing side-by-side frame
    python reach_point.py --pixel 960 700 --preview
"""
import argparse
import json
import os
import subprocess
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
MCP = os.path.join(ROOT, 'xlerobot_mcp')
CAMERA = os.path.join(ROOT, 'samples', 'calibration', 'calibration_fisheye.json')
HANDEYE = os.path.join(ROOT, 'samples', 'hand_eye', 'handeye_result.json')
NECK_TOLERANCE = 6  # ticks; beyond this the camera-to-arm calibration no longer applies
# Disparity is about 36 / depth(m) px, so 256 px covers depths down to about 14 cm, which the arm reaches.
NUM_DISP = 256


def call(tool, args=None, *extra):
    r = subprocess.run([os.path.join(MCP, '.venv', 'bin', 'python'), os.path.join(MCP, 'client.py'), 'call', tool,
                        '--arguments', json.dumps(args or {}), *extra], capture_output=True, text=True, timeout=180)
    if r.returncode:
        report = json.loads(r.stderr.strip().splitlines()[-1])
        res = report.get('tool_result') or {}
        sys.exit(f'{tool} failed: {(res.get("stderr") or report.get("error", ""))[-400:]}')
    return json.loads(r.stdout)['results']


def rectifier(cal, size):
    K1, D1 = np.array(cal['left']['camera_matrix']), np.array(cal['left']['distortion']).reshape(4, 1)
    K2, D2 = np.array(cal['right']['camera_matrix']), np.array(cal['right']['distortion']).reshape(4, 1)
    R, T = np.array(cal['stereo']['R']), np.array(cal['stereo']['T_mm']).reshape(3, 1) / 1000.0
    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(K1, D1, K2, D2, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, balance=0.0)
    maps = (cv2.fisheye.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_16SC2),
            cv2.fisheye.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_16SC2))
    return K1, D1, R1, P1, Q, maps


def disparity_map(left_r, right_r, block=7, num_disp=NUM_DISP):
    sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=num_disp, blockSize=block, P1=8 * block ** 2,
                                 P2=32 * block ** 2, disp12MaxDiff=1, preFilterCap=63, uniquenessRatio=10,
                                 speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_HH)
    return sgbm.compute(left_r, right_r).astype(np.float32) / 16.0


def point_from_pixel(frame, u, v, window=9):
    """Arm-base point (m) for raw left-half pixel (u, v), plus diagnostics."""
    h, w = frame.shape[:2]
    left, right = frame[:, :w // 2], frame[:, w // 2:]
    size = (left.shape[1], left.shape[0])
    if not (0 <= u < size[0] and 0 <= v < size[1]):
        raise ValueError(f'Pixel ({u}, {v}) outside the {size[0]}x{size[1]} left image')
    cal = json.load(open(CAMERA))
    K1, D1, R1, P1, Q, (map1, map2) = rectifier(cal, size)
    gray = lambda img: cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    left_r, right_r = cv2.remap(gray(left), *map1, cv2.INTER_LINEAR), cv2.remap(gray(right), *map2, cv2.INTER_LINEAR)
    disp = disparity_map(left_r, right_r)
    # Raw pixel -> rectified pixel, so it indexes the disparity map.
    ur, vr = cv2.fisheye.undistortPoints(np.array([[[u, v]]], np.float64), K1, D1, R=R1, P=P1).ravel()
    x0, y0 = int(round(ur)), int(round(vr))
    half = window // 2
    patch = disp[max(0, y0 - half):y0 + half + 1, max(0, x0 - half):x0 + half + 1]
    valid = patch[patch > 0]
    if valid.size < 0.3 * window * window:
        raise ValueError(f'No reliable depth at that pixel ({valid.size} of {window * window} valid); pick a textured spot')
    d = float(np.median(valid))
    # A match at the end of the search range is usually a clipped wrong match, not a real depth.
    if d >= NUM_DISP - 8:
        raise ValueError(f'Disparity {d:.0f} px is at the search limit; the point is closer than about {36 / NUM_DISP:.2f} m')
    X, Y, Z, W = Q @ np.array([ur, vr, d, 1.0])
    p_rect = np.array([X, Y, Z]) / W
    p_lens = R1.T @ p_rect                                    # rectified -> original left lens frame
    handeye = json.load(open(HANDEYE))
    p_base = (np.array(handeye['camera_left_lens_in_arm_base']) @ np.append(p_lens, 1.0))[:3]
    return p_base, {'disparity_px': d, 'depth_m': float(p_rect[2]), 'rectified_pixel': [float(ur), float(vr)],
                    'valid_fraction': float(valid.size / patch.size), 'neck_raw': handeye['neck_raw']}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--pixel', type=float, nargs=2, required=True, metavar=('U', 'V'), help='raw left-half pixel')
    parser.add_argument('--image', help='existing side-by-side frame (default: take a fresh snapshot)')
    parser.add_argument('--hover-cm', type=float, default=5.0, help='stop this far above the point (arm z)')
    parser.add_argument('--preview', action='store_true', help='ask the server for a reach plan (no motion)')
    args = parser.parse_args()

    out_dir = os.path.join(ROOT, 'samples', 'reach')
    if args.image:
        path = args.image
    else:
        os.makedirs(out_dir, exist_ok=True)
        path = call('camera_snapshot', None, '--output-dir', out_dir)[1]['saved_image']
    frame = cv2.imread(path)
    if frame is None:
        sys.exit(f'Cannot read {path}')
    if not args.image or args.preview:
        neck = {m: json.loads(call('joint_status', {'group': 'neck', 'servo_id': m})[0]['stdout'])['state']['position'] for m in (7, 8)}
        ref = json.load(open(HANDEYE))['neck_raw']
        off = {m: neck[m] - ref[str(m)] for m in neck}
        if any(abs(o) > NECK_TOLERANCE for o in off.values()):
            sys.exit(f'Neck moved since the camera-to-arm calibration (offset {off} ticks); results would be wrong')
    u, v = args.pixel
    point, info = point_from_pixel(frame, u, v)
    target = point + np.array([0, 0, args.hover_cm / 100])
    print(f'image {path}')
    print(f'pixel ({u:.0f}, {v:.0f}): depth {info["depth_m"]:.3f} m (disparity {info["disparity_px"]:.1f} px, '
          f'{info["valid_fraction"] * 100:.0f}% valid nearby)')
    print(f'point in arm base frame: x {point[0]:.3f}  y {point[1]:.3f}  z {point[2]:.3f} m')
    print(f'reach target ({args.hover_cm:g} cm above): {np.round(target, 3).tolist()}')
    marked = frame.copy()
    cv2.circle(marked, (int(u), int(v)), 18, (0, 0, 255), 3)
    cv2.drawMarker(marked, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
    marked_path = os.path.splitext(path)[0] + '_target.png'
    cv2.imwrite(marked_path, marked)
    print(f'marked image {marked_path}')
    if args.preview:
        joints = {str(i): json.loads(call('joint_status', {'group': 'left-arm', 'servo_id': i})[0]['stdout'])['state']['position'] for i in range(1, 7)}
        res = call('preview_left_reach', {'expected_positions': joints, 'target_m': target.tolist()})[0]
        plan = json.loads([l for l in res['stdout'].splitlines() if l.startswith('{')][-1])['plan']
        print(f'preview: {len(plan["waypoints"])} stages, joint targets {plan["targets"]}, IK error {plan["ik_error_mm"]:.1f} mm, '
              f'gripper move {np.round(np.array(plan["move_m"]) * 100, 1).tolist()} cm, plan_hash {plan["plan_hash"]}')


if __name__ == '__main__':
    main()
