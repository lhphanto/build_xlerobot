#!/usr/bin/env python3
"""Zhang's checkerboard calibration for the two halves of the XLeRobot stereo camera.

Reads pair_NN_left_half.png / pair_NN_right_half.png from --dir, calibrates each lens
(cv2.calibrateCamera), then the pair (cv2.stereoCalibrate with fixed intrinsics), and
writes a JSON file with intrinsics, distortion, reprojection errors and the
right-relative-to-left rotation and translation.

"left"/"right" are image halves after the capture's 180-degree rotation, not
confirmed physical lens sides.

Example:
    python calibrate_stereo.py --dir samples/calibration --cols 9 --rows 6 --square-mm 25.4
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

DETECT_FLAGS = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE


def board_points(cols, rows, square_mm):
    """3-D corner coordinates on the board plane (z = 0), in millimetres."""
    grid = np.zeros((cols * rows, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    return grid


def find_corners(path, pattern):
    gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(path)
    # The sector-based detector is sub-pixel accurate; the classic one is a fallback
    # for poses it misses, refined with cornerSubPix.
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=DETECT_FLAGS)
    if not ok:
        ok, corners = cv2.findChessboardCorners(
            gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return (corners if ok else None), gray.shape[::-1]


def per_view_errors(objpoints, imgpoints, rvecs, tvecs, K, dist, fisheye=False):
    errors = []
    for obj, img, r, t in zip(objpoints, imgpoints, rvecs, tvecs):
        if fisheye:
            projected, _ = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3).astype(np.float64), r, t, K, dist)
        else:
            projected, _ = cv2.projectPoints(obj, r, t, K, dist)
        errors.append(float(np.sqrt(np.mean(np.sum((img.reshape(-1, 1, 2) - projected) ** 2, axis=2)))))
    return errors


def calibrate_lens(name, objpoints, imgpoints, size, flags, fisheye=False):
    if fisheye:
        # cv2.fisheye wants (N,1,3)/(N,1,2) float64 points and fits k1-k4 of the equidistant model.
        obj = [o.reshape(-1, 1, 3).astype(np.float64) for o in objpoints]
        img = [p.reshape(-1, 1, 2).astype(np.float64) for p in imgpoints]
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        # Without a starting focal length the fisheye solver diverges on few, central views,
        # so seed it from the standard model; with fewer than 8 views fit only k1, k2.
        _, K0, _, _, _ = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)
        fisheye_flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
                         | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS)
        if len(objpoints) < 8:
            fisheye_flags |= cv2.fisheye.CALIB_FIX_K3 | cv2.fisheye.CALIB_FIX_K4
            print(f'{name}: fewer than 8 views, fitting fisheye k1 and k2 only')
        rms, K, dist, rvecs, tvecs = cv2.fisheye.calibrate(
            obj, img, size, K0, np.zeros((4, 1)), flags=fisheye_flags, criteria=criteria)
    else:
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, size, None, None, flags=flags)
    errors = per_view_errors(objpoints, imgpoints, rvecs, tvecs, K, dist, fisheye)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f'\n{name} lens: RMS reprojection error {rms:.3f} px over {len(objpoints)} views')
    print(f'  fx {fx:.1f}  fy {fy:.1f}  cx {cx:.1f}  cy {cy:.1f}')
    # Undistort the left/right edge midpoints to normalized rays; this includes distortion,
    # which a focal-length-only estimate ignores. Pinhole models are unreliable this far out.
    edges = np.array([[[0, cy]], [[size[0] - 1, cy]]], np.float64)
    rays = (cv2.fisheye.undistortPoints(edges, K, dist) if fisheye else cv2.undistortPoints(edges, K, dist)).reshape(-1, 2)
    fov = np.degrees(abs(np.arctan(rays[0, 0])) + abs(np.arctan(rays[1, 0])))
    print(f'  horizontal field of view about {fov:.1f} deg (extrapolated beyond the board corners)')
    print('  distortion', np.round(dist.ravel(), 4).tolist())
    print('  per-view error px', [round(e, 3) for e in errors])
    return {'rms_px': rms, 'camera_matrix': K.tolist(), 'distortion': dist.ravel().tolist(),
            'per_view_error_px': errors}


def coverage(imgpoints, size):
    pts = np.concatenate([p.reshape(-1, 2) for p in imgpoints])
    return {'x_min': float(pts[:, 0].min()), 'x_max': float(pts[:, 0].max()),
            'y_min': float(pts[:, 1].min()), 'y_max': float(pts[:, 1].max()),
            'width': size[0], 'height': size[1]}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dir', default='samples/calibration')
    parser.add_argument('--cols', type=int, default=9, help='inner corners per row')
    parser.add_argument('--rows', type=int, default=6, help='inner corners per column')
    parser.add_argument('--square-mm', type=float, default=25.4)
    parser.add_argument('--rational', action='store_true',
                        help='8-coefficient rational distortion model; needs many more views than 4')
    parser.add_argument('--max-view-error', type=float, default=0,
                        help='drop the worst view, one at a time, until every view fits within this many '
                             'pixels (0 keeps all views); at least 3 views are always kept')
    parser.add_argument('--fisheye', action='store_true',
                        help='cv2.fisheye equidistant model (k1-k4), for wide-angle lenses')
    parser.add_argument('--out', default=None, help='output JSON (default <dir>/calibration.json)')
    parser.add_argument('--undistort', action='store_true', help='also write undistorted_*.png previews')
    args = parser.parse_args()

    pattern = (args.cols, args.rows)
    pairs = sorted(glob.glob(os.path.join(args.dir, 'pair_*_left_half.png')))
    if not pairs:
        sys.exit(f'No pair_*_left_half.png images in {args.dir}')
    if args.fisheye and args.rational:
        sys.exit('--fisheye and --rational are different models; choose one')
    flags = cv2.CALIB_RATIONAL_MODEL if args.rational else 0
    obj = board_points(args.cols, args.rows, args.square_mm)
    # Each lens uses every image where its own half sees the whole board, so edge views
    # that the other lens cuts off still count; the stereo step needs both halves.
    views, size = {'left': {}, 'right': {}}, None
    for left_path in pairs:
        stem = os.path.basename(left_path).replace('_left_half.png', '')
        found = []
        for side, path in (('left', left_path), ('right', left_path.replace('_left_half.png', '_right_half.png'))):
            corners, half_size = find_corners(path, pattern)
            if size is None:
                size = half_size
            if half_size != size:
                sys.exit(f'{stem}: image size differs from {size}')
            if corners is not None:
                views[side][stem] = corners
                found.append(side)
        print(f'{stem}: board found in {" and ".join(found) or "neither"} half' + ('' if len(found) == 1 else 's'))

    for side in ('left', 'right'):
        # Zhang's method needs at least three views of the plane; more and varied is better.
        if len(views[side]) < 3:
            sys.exit(f'Only {len(views[side])} usable {side}-half views; at least 3 are required')
        if not args.max_view_error:
            continue
        # Blurred or badly detected views pull every lens parameter, so drop the worst
        # view (standard model, which stays stable) until all fit.
        while len(views[side]) > 3:
            stems, pts = list(views[side]), list(views[side].values())
            _, K, dist, rvecs, tvecs = cv2.calibrateCamera([obj] * len(pts), pts, size, None, None, flags=flags)
            errors = per_view_errors([obj] * len(pts), pts, rvecs, tvecs, K, dist)
            k = int(np.argmax(errors))
            if errors[k] <= args.max_view_error:
                break
            print(f'{stems[k]} {side} half: dropped, view error {errors[k]:.2f} px above {args.max_view_error} px')
            del views[side][stems[k]]
        print(f'{side} half: {len(views[side])} views kept')

    model = 'fisheye' if args.fisheye else 'rational' if args.rational else 'standard'
    used = [stem for stem in views['left'] if stem in views['right']]
    result = {'pairs': used, 'image_size': list(size), 'pattern_inner_corners': list(pattern),
              'square_mm': args.square_mm, 'distortion_model': model,
              'note': 'left/right are image halves after 180-degree rotation, not confirmed physical lenses'}
    for side in ('left', 'right'):
        pts = list(views[side].values())
        result[side] = calibrate_lens(f'{side.capitalize()}-half', [obj] * len(pts), pts, size, flags, args.fisheye)
        result[side]['views'] = list(views[side])
        cov = coverage(pts, size)
        result[side]['corner_coverage_px'] = cov
        print(f'{side} corners cover x {cov["x_min"]:.0f}-{cov["x_max"]:.0f}, y {cov["y_min"]:.0f}-{cov["y_max"]:.0f} of {size[0]}x{size[1]}')
    if len(used) < 3:
        sys.exit(f'Only {len(used)} views seen fully by both halves; stereo needs at least 3')
    objpoints = [obj] * len(used)
    left_pts = [views['left'][s] for s in used]
    right_pts = [views['right'][s] for s in used]
    print(f'\nStereo uses {len(used)} views seen by both halves')

    # Stereo: keep each lens's intrinsics and solve only the rigid transform between them.
    K1, d1 = np.array(result['left']['camera_matrix']), np.array(result['left']['distortion'])
    K2, d2 = np.array(result['right']['camera_matrix']), np.array(result['right']['distortion'])
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    if args.fisheye:
        d1, d2 = d1.reshape(4, 1), d2.reshape(4, 1)
        obj = [o.reshape(-1, 1, 3).astype(np.float64) for o in objpoints]
        out = cv2.fisheye.stereoCalibrate(
            obj, [p.reshape(-1, 1, 2).astype(np.float64) for p in left_pts],
            [p.reshape(-1, 1, 2).astype(np.float64) for p in right_pts],
            K1, d1, K2, d2, size, flags=cv2.fisheye.CALIB_FIX_INTRINSIC, criteria=criteria)
        # Returns rms, K1, D1, K2, D2, R, T (newer OpenCV appends per-view rvecs/tvecs).
        rms, R, T = out[0], out[5], out[6]
        # The fisheye API gives no E/F; derive them from R, T and the intrinsics.
        tx = np.array([[0, -T[2, 0], T[1, 0]], [T[2, 0], 0, -T[0, 0]], [-T[1, 0], T[0, 0], 0]])
        E = tx @ R
        F = np.linalg.inv(K2).T @ E @ np.linalg.inv(K1)
    else:
        rms, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
            objpoints, left_pts, right_pts, K1, d1, K2, d2, size,
            criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC)
    T = np.asarray(T, dtype=np.float64).reshape(3, 1)
    angle = float(np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0])))
    baseline = float(np.linalg.norm(T))
    print(f'\nStereo: RMS {rms:.3f} px, baseline {baseline:.1f} mm, relative rotation {angle:.2f} deg')
    print('  T (mm)', np.round(T.ravel(), 2).tolist())
    result['stereo'] = {'rms_px': rms, 'R': R.tolist(), 'T_mm': T.ravel().tolist(), 'baseline_mm': baseline,
                        'rotation_deg': angle, 'E': E.tolist(), 'F': F.tolist()}

    out = args.out or os.path.join(args.dir, 'calibration.json')
    with open(out, 'w') as f:
        json.dump(result, f, indent=2)
    print(f'\nSaved {out}')

    if args.undistort:
        for stem in used:
            for side, K, d in (('left', K1, d1), ('right', K2, d2)):
                img = cv2.imread(os.path.join(args.dir, f'{stem}_{side}_half.png'))
                path = os.path.join(args.dir, f'undistorted_{model}_{stem}_{side}.png')
                if args.fisheye:
                    # balance=0.5 keeps most of the wide view instead of cropping to the valid centre.
                    Knew = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, d, size, np.eye(3), balance=0.5)
                    cv2.imwrite(path, cv2.fisheye.undistortImage(img, K, d, Knew=Knew))
                else:
                    cv2.imwrite(path, cv2.undistort(img, K, d))
        print('Wrote undistorted previews')


if __name__ == '__main__':
    main()
