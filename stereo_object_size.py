#!/usr/bin/env python3
"""Read-only object sizing from an upright calibrated stereo snapshot.

ROIs use raw LEFT-HALF pixels, x y width height. Fit a selected tabletop,
then measure the selected object's height and visible projected width in 3D.
No camera/serial access, VO motion, robot targets, or learned models.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from reach_point import rectifier, disparity_map

ROOT = Path(__file__).resolve().parent
CALIBRATION = ROOT / 'samples/calibration/calibration_fisheye.json'
TRUSTED = (300, 150, 1550, 880)  # CALIBRATION_RESULTS.md; conservative for both halves


class MeasurementError(ValueError):
    pass


def rectangle_mask(shape, rectangles):
    h, w = shape
    mask = np.zeros(shape, np.uint8)
    for rect in rectangles:
        if len(rect) != 4 or any(type(v) is not int for v in rect):
            raise MeasurementError('Each ROI needs four integer pixels: x y width height')
        x, y, rw, rh = rect
        if rw <= 0 or rh <= 0 or x < 0 or y < 0 or x + rw > w or y + rh > h:
            raise MeasurementError(f'ROI {rect} outside {w}x{h} left half')
        mask[y:y+rh, x:x+rw] = 255
    return mask


def fit_plane(points, threshold=.003, min_fraction=.60, iterations=450):
    """Deterministic RANSAC + SVD. Normal faces the camera; meters throughout."""
    points = np.asarray(points, float)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 100:
        raise MeasurementError('Too few valid tabletop points (need at least100)')
    rng = np.random.default_rng(17)
    sample = points[rng.choice(len(points), min(5000, len(points)), replace=False)]
    best = np.zeros(len(sample), bool)
    for _ in range(iterations):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(b-a, c-a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        inside = np.abs((sample-a) @ n) < threshold
        if inside.sum() > best.sum():
            best = inside
    # A three-point hypothesis is noisy; refine it before enforcing the final
    # inlier fraction, rather than rejecting a recoverable plane prematurely.
    if best.mean() < max(.30, min_fraction-.20):
        raise MeasurementError('Selected tabletop is not one sufficiently supported plane')
    selected = sample[best]
    for _ in range(5):
        centre = selected.mean(axis=0)
        _, singular, axes = np.linalg.svd(selected-centre, full_matrices=False)
        if singular[1] / np.sqrt(len(selected)) < .010:
            raise MeasurementError('Tabletop selection is too narrow or collinear')
        n = axes[-1]
        d = -float(centre @ n)
        if d < 0:
            n, d = -n, -d
        inside = np.abs(points @ n+d) < threshold
        selected = points[inside]
        if len(selected) < 100:
            raise MeasurementError('Tabletop plane lost support while refining')
    fraction = float(inside.mean())
    rmse = float(np.sqrt(np.mean((selected @ n+d)**2)))
    if fraction < min_fraction or rmse > threshold or d < .03:
        raise MeasurementError('Tabletop plane quality inadequate')
    return n, d, {'inlier_fraction': fraction, 'rmse_mm': rmse*1000,
                   'points': int(len(points)), 'inliers': int(inside.sum()),
                   'principal_spread_mm': (singular[:2]/np.sqrt(len(selected))*1000).tolist()}


def dimensions(points, normal, offset):
    """Height above support and visible camera-horizontal width; no hidden size claim."""
    points = np.asarray(points, float)
    if len(points) < 80 or not np.isfinite(points).all():
        raise MeasurementError('Too few valid selected object points (need at least80)')
    n = np.asarray(normal, float)
    if not np.isclose(np.linalg.norm(n), 1., atol=1e-5):
        raise MeasurementError('Support normal must be a unit vector')
    heights = points @ n + offset
    if np.quantile(heights, .98) <= .006:
        raise MeasurementError('Object does not rise reliably above the support plane')
    # Camera-right projected onto the tabletop; unlike raw pixel width, this
    # corrects perspective and camera tilt, but not hidden faces or object yaw.
    axis = np.array([1., 0., 0.])
    axis -= n * (axis @ n)
    norm = np.linalg.norm(axis)
    if norm < .2:
        raise MeasurementError('Camera-horizontal width direction is degenerate')
    axis /= norm
    horizontal = points @ axis
    width = float(np.quantile(horizontal, .99)-np.quantile(horizontal, .01))
    height = float(np.quantile(heights, .98))
    if not .003 <= width <= .50 or not .006 <= height <= .50:
        raise MeasurementError('Dimension outside the supported3-500mm width/6-500mm height range')
    return {'height_above_table_mm': height*1000, 'visible_width_mm': width*1000,
            'top_height_band_mm': (np.quantile(heights, [.95, .995])*1000).tolist(),
            'width_axis_camera': axis.tolist(), 'object_points': int(len(points))}


def make_cloud(frame, calibration):
    if frame is None or frame.shape != (1080, 3840, 3):
        raise MeasurementError('Expected upright3840x1080 side-by-side BGR frame')
    shape = (1080, 1920)
    _, _, R1, P1, Q, maps = rectifier(calibration, (1920, 1080))
    left = cv2.cvtColor(frame[:, :1920], cv2.COLOR_BGR2GRAY)
    right = cv2.cvtColor(frame[:, 1920:], cv2.COLOR_BGR2GRAY)
    lg = cv2.remap(left, *maps[0], cv2.INTER_LINEAR)
    rg = cv2.remap(right, *maps[1], cv2.INTER_LINEAR)
    disp = disparity_map(lg, rg)
    # Explicit reverse consistency in addition to SGBM's internal check.
    reverse = np.fliplr(disparity_map(np.fliplr(rg).copy(), np.fliplr(lg).copy()))
    yy, xx = np.indices(shape)
    xr = np.rint(xx-disp).astype(np.int32)
    xrc = np.clip(xr, 0, shape[1]-1)
    trust = rectangle_mask(shape, [(TRUSTED[0], TRUSTED[1],
                                   TRUSTED[2]-TRUSTED[0], TRUSTED[3]-TRUSTED[1])])
    tl = cv2.remap(trust, *maps[0], cv2.INTER_NEAREST) > 0
    tr = cv2.remap(trust, *maps[1], cv2.INTER_NEAREST) > 0
    cloud = cv2.reprojectImageTo3D(disp, Q)
    consistent = np.abs(disp-reverse[yy, xrc]) <= 1.5
    valid = (disp > .5) & (disp < 248) & (xr >= 0) & (xr < 1920)
    valid &= tl & tr[yy, xrc] & consistent & np.isfinite(cloud).all(axis=2)
    valid &= (cloud[:, :, 2] >= .15) & (cloud[:, :, 2] <= 1.5)
    return cloud, valid, disp, maps[0], R1, P1


def selected_component(cloud, valid, object_mask, normal, offset, seed=None):
    heights = np.full(valid.shape, np.nan)
    heights[valid] = cloud[valid] @ normal+offset
    candidates = valid & (object_mask > 0) & (heights > .006) & (heights < .50)
    # Close small stereo holes for component labelling only; invented pixels
    # are NEVER included in the measured3D cloud.
    labels_mask = cv2.morphologyEx(candidates.astype(np.uint8), cv2.MORPH_CLOSE,
                                 np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(labels_mask)
    eligible = [i for i in range(1, count) if np.count_nonzero(candidates & (labels == i)) >= 80]
    if not eligible:
        raise MeasurementError('No sufficiently connected object depth above the tabletop')
    if seed is None:
        chosen = max(eligible, key=lambda i: stats[i, cv2.CC_STAT_AREA])
    else:
        distances = []
        for i in eligible:
            y, x = np.nonzero(candidates & (labels == i))
            distances.append((float(np.min((x-seed[0])**2+(y-seed[1])**2)), i))
        dist, chosen = min(distances)
        if dist > 15**2:
            raise MeasurementError('No reliable object component close to the selected seed')
    return candidates & (labels == chosen)


def estimate(frame, calibration, object_roi, table_rois, seed_raw=None, cache=None):
    if frame is None or frame.shape != (1080, 3840, 3):
        raise MeasurementError('Expected upright3840x1080 side-by-side BGR frame')
    raw_shape = (1080, 1920)
    obj_raw = rectangle_mask(raw_shape, [object_roi])
    table_raw = rectangle_mask(raw_shape, table_rois)
    table_raw[obj_raw > 0] = 0
    key = hashlib.sha256(frame.tobytes()+json.dumps(calibration, sort_keys=True).encode()
                         + b'object-size-stereo-v1').hexdigest()
    _, _, R1, P1, _, maps = rectifier(calibration, (1920, 1080))
    cached = False
    if cache and Path(cache).exists():
        with np.load(cache, allow_pickle=False) as z:
            if str(z['key']) == key:
                cloud, valid, disp = z['cloud'], z['valid'], z['disparity']
                cached = True
    if not cached:
        cloud, valid, disp, _, R1, P1 = make_cloud(frame, calibration)
        if cache:
            Path(cache).parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(cache, key=key, cloud=cloud, valid=valid, disparity=disp)
    obj = cv2.remap(obj_raw, *maps[0], cv2.INTER_NEAREST)
    table = cv2.remap(table_raw, *maps[0], cv2.INTER_NEAREST)
    table_valid = valid & (table > 0)
    coverage = float(table_valid.sum()/max(1, (table > 0).sum()))
    if coverage < .15:
        raise MeasurementError('Less than15% of selected tabletop has trustworthy stereo depth')
    n, d, plane = fit_plane(cloud[table_valid])
    seed = None
    if seed_raw is not None:
        x, y = seed_raw
        rx, ry, rw, rh = object_roi
        if not rx <= x < rx+rw or not ry <= y < ry+rh:
            raise MeasurementError('Object seed must be inside its ROI')
        K = np.asarray(calibration['left']['camera_matrix'])
        D = np.asarray(calibration['left']['distortion']).reshape(4, 1)
        seed = cv2.fisheye.undistortPoints(np.array([[[x, y]]], float), K, D, R=R1, P=P1).ravel()
    chosen = selected_component(cloud, valid, obj, n, d, seed)
    metrics = dimensions(cloud[chosen], n, d)
    # Estimate sampling stability; this is NOT a total/calibrated accuracy
    # interval and does not account for systematic stereo/calibration errors.
    rng = np.random.default_rng(29)
    support = cloud[table_valid]
    alternative_heights = []
    for _ in range(4):
        subset = support[rng.choice(len(support), max(100, len(support)//2), replace=False)]
        nn, dd, _ = fit_plane(subset, iterations=450)
        alternative_heights.append(float(np.quantile(cloud[chosen] @ nn+dd, .98))*1000)
    spread = float(np.ptp(alternative_heights))
    if spread > 10:
        raise MeasurementError(f'Tabletop fit unstable: height changes {spread:.1f}mm under resampling')
    trusted = rectangle_mask(raw_shape, [(300, 150, 1250, 730)])
    trusted_fraction = float(np.count_nonzero((obj_raw > 0) & (trusted > 0))/np.count_nonzero(obj_raw))
    warnings = ['Width describes the visible camera-horizontal extent; hidden or rotated faces may be wider.',
                'Height assumes the selected object rests on the selected flat tabletop.',
                'The top-height band describes point spread, not a calibrated accuracy interval.']
    if trusted_fraction < .98:
        warnings.append('Part of the ROI lies outside calibration coverage; those pixels were excluded.')
    result = {'ok': True, 'method': 'fisheye stereo + tabletop plane; no robot motion or VO required',
              **metrics, 'quality': 'approximate_visible_surface',
              'plane': {'normal_camera': n.tolist(), 'offset_m': float(d), **plane},
              'plane_resampling_height_spread_mm': spread,
              'table_valid_depth_fraction': coverage,
              'object_valid_depth_fraction': float(chosen.sum()/max(1, (obj > 0).sum())),
              'object_roi_trusted_fraction': trusted_fraction,
              'median_object_depth_mm': float(np.median(cloud[chosen, 2])*1000),
              'raw_object_roi': list(object_roi), 'raw_table_rois': [list(r) for r in table_rois],
              'warnings': warnings}
    return result, chosen, disp


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--image', required=True, type=Path)
    p.add_argument('--calibration', default=CALIBRATION, type=Path)
    p.add_argument('--object-roi', nargs=4, type=int, required=True, metavar=('X', 'Y', 'W', 'H'))
    p.add_argument('--table-roi', nargs=4, type=int, action='append', required=True, metavar=('X', 'Y', 'W', 'H'))
    p.add_argument('--seed', nargs=2, type=float)
    p.add_argument('--cache', type=Path, help='Optional depth cache bound to exact image and calibration')
    p.add_argument('--output', type=Path, help='JSON report; also writes an annotated LEFT image')
    args = p.parse_args()
    try:
        frame = cv2.imread(str(args.image))
        if frame is None or frame.shape != (1080, 3840, 3):
            raise MeasurementError('Image must be the upright3840x1080 stereo snapshot')
        calibration = json.loads(args.calibration.read_text())
        result, _, _ = estimate(frame, calibration, args.object_roi, args.table_roi, args.seed, args.cache)
        result.update(image=str(args.image.resolve()), calibration=str(args.calibration.resolve()))
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            overlay = frame[:, :1920].copy()
            for roi, colour in [(args.object_roi, (255, 200, 0))]+[(r, (0, 220, 0)) for r in args.table_roi]:
                x, y, w, h = roi
                cv2.rectangle(overlay, (x, y), (x+w, y+h), colour, 2)
            label = f"height ~{result['height_above_table_mm']:.1f} mm   visible width ~{result['visible_width_mm']:.1f} mm"
            cv2.putText(overlay, label, (30, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 3)
            path = args.output.with_name(args.output.stem+'_annotated.png')
            if not cv2.imwrite(str(path), overlay):
                raise MeasurementError('Could not save annotated image')
            result['annotated_image'] = str(path.resolve())
            args.output.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result))
        return 0
    except (MeasurementError, OSError, KeyError, cv2.error) as error:
        result = {'ok': False, 'error': str(error), 'dimensions_available': False}
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
