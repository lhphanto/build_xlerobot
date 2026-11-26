#!/usr/bin/env python3
"""Camera check between steps (contact_graspnet env). No motion.

    python grasp/look.py                                   # fresh snapshot only (path printed, left image preview saved)
    python grasp/look.py --object X Y Z [--expect-bottom-z Z] [--tolerance 0.03]
    python grasp/look.py --image FRAME.png --object X Y Z  # on an existing frame

--object: where the object should be (arm frame, m): a point on its body, below the jaws when held. The point is
projected into the left image, SlimSAM segments the object from that pixel, stereo depth gives its points in the arm
frame, and the measured centre / top / bottom are compared with the expectation: OK if the centre is within
--tolerance horizontally and (if given) the bottom within it vertically, otherwise CHECK; UNSEEN when no plausible
object mask is found (hidden behind the arm). Several seeds around the point are tried (see measure()). The neck must be in the
camera-to-arm calibration pose (within 6 ticks). The annotated crop shows the expected point (red), mask (green) and
measured centre (yellow). Prints one JSON line last; appends it to samples/grasp/steps.jsonl.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, HERE]
import reach_point as rp  # noqa: E402
import sam_click as sc  # noqa: E402

MCP = os.path.join(ROOT, 'xlerobot_mcp'); OUT = os.path.join(ROOT, 'samples', 'grasp'); LOG = os.path.join(OUT, 'steps.jsonl')


def client(tool, args, *extra):
    r = subprocess.run([os.path.join(MCP, '.venv', 'bin', 'python'), os.path.join(MCP, 'client.py'), 'call', tool, '--arguments', json.dumps(args), *extra],
                       capture_output=True, text=True, timeout=180)
    if r.returncode:
        sys.exit(f'{tool} failed: {r.stderr[-300:]}')
    return json.loads(r.stdout)['results']


def snapshot():
    os.makedirs(OUT, exist_ok=True)
    path = client('camera_snapshot', {}, '--output-dir', OUT)[1]['saved_image']
    ref = json.load(open(rp.HANDEYE))['neck_raw']
    for m in (7, 8):
        pos = json.loads(client('joint_status', {'group': 'neck', 'servo_id': m})[0]['stdout'])['state']['position']
        if abs(pos - ref[str(m)]) > rp.NECK_TOLERANCE:
            sys.exit(f'Neck ID{m} at {pos}, calibration pose {ref[str(m)]}: camera-to-arm positions would be wrong')
    return path


SEEDS = ((0, 0, 0), (0, 0, -0.03), (0, 0, 0.03), (0, 0.015, 0), (0, -0.015, 0))  # around the expected point (m)
MAX_EXTENT = 0.12  # m; wider masks are the desk sheet, the arm or the background, not a hand-sized object


def measure(frame, point, gripper=None):
    """Segment the object around an expected arm-frame point; arm-frame stats of its depth points.

    November24 the single projected seed often landed on the arm, which hides the object from this camera at the hover,
    descent and carry, and SAM returned the arm, a gripper servo or the desk sheet (false CHECKs). Now several seeds
    are tried; points above the gripper point + 3 cm (the arm itself) are dropped when the gripper point is known;
    masks wider than MAX_EXTENT are rejected. The first plausible mask near the expectation wins, else the nearest
    plausible one; with none, the result has 'unseen' (the object is hidden), which is not a CHECK."""
    rect, K1, D1, R1, P1, Q, maps = sc.rectified(frame)
    T = np.array(json.load(open(rp.HANDEYE))['camera_left_lens_in_arm_base']); Ti = np.linalg.inv(T)
    gray = lambda i: cv2.cvtColor(i, cv2.COLOR_BGR2GRAY)
    w, h = frame.shape[1] // 2, frame.shape[0]
    disp = rp.disparity_map(cv2.remap(gray(frame[:, :w]), *maps[0], cv2.INTER_LINEAR), cv2.remap(gray(frame[:, w:]), *maps[1], cv2.INTER_LINEAR))
    xyz = cv2.reprojectImageTo3D(disp, Q); good = (disp > 0) & (disp < rp.NUM_DISP - 8)
    T_lens = np.eye(4); T_lens[:3, :3] = R1.T; Tb = T @ T_lens
    point = np.asarray(point, float); tried, best = [], None
    for off in SEEDS:
        q = point + off; p_lens = Ti[:3, :3] @ q + Ti[:3, 3]
        raw, _ = cv2.fisheye.projectPoints(p_lens.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K1, D1)
        u, v = raw.ravel()
        if not (0 <= u < w and 0 <= v < h):
            tried.append({'seed': [round(float(u)), round(float(v))], 'reject': 'outside the image'}); continue
        mask, iou, rc, _ = sc.segment(rect, [[u, v]], K1, D1, R1, P1)
        pts = xyz[good & mask] @ Tb[:3, :3].T + Tb[:3, 3]
        pts = pts[np.linalg.norm(pts - point, axis=1) < 0.25]  # drop far mismatches
        if gripper is not None:
            pts = pts[pts[:, 2] < gripper[2] + 0.03]
        info = {'seed': [round(float(u)), round(float(v))], 'mask_px': int(mask.sum()), 'mask_score': round(float(iou.max()), 3), 'points': int(len(pts))}
        ext = float(max(np.ptp(np.percentile(pts[:, k], [5, 95])) for k in (0, 1))) if len(pts) >= 100 else None
        if len(pts) < 100:
            info['reject'] = f'only {len(pts)} depth points'
        elif ext > MAX_EXTENT:
            info['reject'] = f'mask {ext * 100:.0f} cm wide (not the object)'
        if 'reject' in info:
            tried.append(info); continue
        c = np.median(pts, axis=0)
        info.update(centre=c.round(3).tolist(), top_z=round(float(np.percentile(pts[:, 2], 95)), 3), bottom_z=round(float(np.percentile(pts[:, 2], 5)), 3),
                    extent_m=round(ext, 3))
        off_h = float(np.hypot(*(c[:2] - point[:2]))); tried.append(info)
        if best is None or off_h < best[0]:
            best = (off_h, info, mask, rc[0])
        if off_h <= 0.03:
            break
    if best is None:
        return {'unseen': True, 'error': 'object not visible (hidden by the arm, or no plausible mask)', 'tried': tried}, rect, None, None
    return dict(best[1], tried=tried), rect, best[2], best[3]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image'); ap.add_argument('--object', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
    ap.add_argument('--expect-bottom-z', type=float); ap.add_argument('--tolerance', type=float, default=0.03)
    ap.add_argument('--label', default='look')
    ap.add_argument('--gripper', type=float, nargs=3, metavar=('X', 'Y', 'Z'), help='gripper point (arm frame): points above it + 3 cm are the arm')
    a = ap.parse_args()
    path = a.image or snapshot(); frame = cv2.imread(path)
    stem = os.path.join(OUT, os.path.splitext(os.path.basename(path))[0])
    result = {'step': 'look', 'label': a.label, 'image': path}
    if not a.object:
        cv2.imwrite(stem + '_look.jpg', cv2.resize(frame[:, :frame.shape[1] // 2], None, fx=.5, fy=.5))
        result['preview'] = stem + '_look.jpg'
    else:
        t = time.time(); info, rect, mask, click = measure(frame, a.object, a.gripper); result.update(info, expected=a.object, seconds=round(time.time() - t, 1))
        if 'centre' in info:
            off = float(np.hypot(info['centre'][0] - a.object[0], info['centre'][1] - a.object[1]))
            result['horizontal_offset_m'] = round(off, 3)
            ok = off <= a.tolerance
            if a.expect_bottom_z is not None:
                result['expected_bottom_z'] = a.expect_bottom_z; result['bottom_offset_m'] = round(info['bottom_z'] - a.expect_bottom_z, 3)
                ok = ok and abs(result['bottom_offset_m']) <= a.tolerance
            result['verdict'] = 'OK' if ok else 'CHECK'
        else:
            result['verdict'] = 'UNSEEN' if info.get('unseen') else 'CHECK'
        if mask is not None:
            vis = rect.copy(); vis[mask] = (0.5 * vis[mask] + [0, 127, 0]).astype(np.uint8)
            cv2.drawMarker(vis, (int(click[0]), int(click[1])), (255, 0, 0), cv2.MARKER_CROSS, 30, 3)
            ys, xs = np.nonzero(mask)
            crop = vis[max(0, ys.min() - 200):ys.max() + 200, max(0, xs.min() - 250):xs.max() + 250]
            cv2.imwrite(stem + '_look.jpg', cv2.cvtColor(crop, cv2.COLOR_RGB2BGR)); result['preview'] = stem + '_look.jpg'
        print(f"{a.label}: {result['verdict']}  expected {a.object}, measured centre {result.get('centre')} top {result.get('top_z')} "
              f"bottom {result.get('bottom_z')} (offset {result.get('horizontal_offset_m')} m){'; ' + result['error'] if 'error' in result else ''}")
    os.makedirs(OUT, exist_ok=True)
    with open(LOG, 'a') as f:
        f.write(json.dumps(dict(result, time=time.strftime('%Y-%m-%dT%H:%M:%S'))) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
