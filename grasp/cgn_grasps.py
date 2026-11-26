#!/usr/bin/env python3
"""Contact-GraspNet grasps for one masked object in a stereo frame, in the left arm base frame (contact_graspnet env). No motion.

Depth: fisheye-rectified SGBM (reach_point.py). The mask (sam_click.py) is intersected with points more than 1.5 cm
above a RANSAC desk plane fitted around the object. Grasps are converted to the arm frame and tagged with what our
5-joint arm can do (near top-down, or leaning in the vertical plane through the arm base) and the jaw width.

    python grasp/cgn_grasps.py FRAME.png --mask samples/grasp/<frame>_mask.npy
writes samples/grasp/<frame>_grasps.json (object + grasps) and _grasps.jpg (top six drawn; yellow = best)
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import torch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CGN = os.environ.get('CGN_ROOT', os.path.expanduser('~/Documents/contact_graspnet_pytorch'))
sys.path[:0] = [ROOT, CGN, CGN + '/contact_graspnet_pytorch', CGN + '/Pointnet_Pointnet2_pytorch']
import reach_point as rp  # noqa: E402

FINGER_CENTER_Z = 0.1034  # Panda grasp frame: jaw-pad centre along the approach
MAX_WIDTH = 0.075         # SO101 jaws open ~88 mm (ID6 2162); keep clearance


def load_model():
    _load = torch.load
    def safe_load(f, *a, **k):  # the checkpoint stores numpy scalars; allow only those, on CPU
        with torch.serialization.safe_globals([np.core.multiarray.scalar, np.dtype, np.dtypes.Float64DType, np.dtypes.Int64DType]):
            return _load(f, map_location='cpu', weights_only=True)
    torch.load = safe_load
    from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
    from contact_graspnet_pytorch import config_utils
    from contact_graspnet_pytorch.checkpoints import CheckpointIO
    cfg = config_utils.load_config(CGN + '/checkpoints/contact_graspnet', batch_size=3, arg_configs=[])
    est = GraspEstimator(cfg)
    CheckpointIO(checkpoint_dir=CGN + '/checkpoints/contact_graspnet/checkpoints', model=est.model).load('model.pt')
    return est


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('frame'); ap.add_argument('--mask', required=True)
    ap.add_argument('--out-dir', default=os.path.join(ROOT, 'samples', 'grasp'))
    a = ap.parse_args()
    frame = cv2.imread(a.frame); sam = np.load(a.mask).astype(bool)
    h, w = frame.shape[:2]; left, right = frame[:, :w // 2], frame[:, w // 2:]; size = (left.shape[1], left.shape[0])
    handeye = json.load(open(rp.HANDEYE))
    K1, D1, R1, P1, Q, (m1, m2) = rp.rectifier(json.load(open(rp.CAMERA)), size)
    gray = lambda i: cv2.cvtColor(i, cv2.COLOR_BGR2GRAY)
    lr_, rr = cv2.remap(gray(left), *m1, cv2.INTER_LINEAR), cv2.remap(gray(right), *m2, cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(cv2.remap(left, *m1, cv2.INTER_LINEAR), cv2.COLOR_BGR2RGB)
    disp = rp.disparity_map(lr_, rr)
    xyz = cv2.reprojectImageTo3D(disp, Q)
    depth = np.where((disp <= 0) | (disp >= rp.NUM_DISP - 8), 0, xyz[..., 2]).astype(np.float32)
    depth[(depth < 0.1) | (depth > 2.0)] = 0
    T_lens = np.eye(4); T_lens[:3, :3] = R1.T
    T_base_cam = np.array(handeye['camera_left_lens_in_arm_base']) @ T_lens
    pts = (xyz.reshape(-1, 3) @ T_base_cam[:3, :3].T + T_base_cam[:3, 3]).reshape(xyz.shape)
    ok = depth > 0
    ys, xs = np.nonzero(sam); x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()

    # Desk plane: RANSAC on a ring around the object (cables and the desk edge spoil a plain least-squares fit).
    ring = np.zeros_like(ok); ring[max(0, y0 - 120):y1 + 120, max(0, x0 - 150):x1 + 150] = True; ring[max(0, y0 - 10):y1 + 10, max(0, x0 - 10):x1 + 10] = False
    P = pts[ring & ok]; rng = np.random.default_rng(0); best = None
    for _ in range(400):
        q = P[rng.choice(len(P), 3, replace=False)]; n = np.cross(q[1] - q[0], q[2] - q[0])
        if np.linalg.norm(n) < 1e-9: continue
        n /= np.linalg.norm(n); n = n if n[2] > 0 else -n
        if n[2] < 0.9: continue
        inl = np.abs(P @ n - n @ q[0]) < 0.006
        if best is None or inl.sum() > best[0].sum(): best = (inl, n)
    Pi = P[best[0]]; A = np.c_[Pi[:, :2], np.ones(len(Pi))]; coef, *_ = np.linalg.lstsq(A, Pi[:, 2], rcond=None)
    resid = Pi[:, 2] - A @ coef
    height = pts[..., 2] - (pts[..., 0] * coef[0] + pts[..., 1] * coef[1] + coef[2])
    seg = (sam & ok & (height > 0.015) & (height < 0.30)).astype(np.uint8)
    bp = pts[seg > 0]
    if len(bp) < 200:
        sys.exit(f'Only {len(bp)} object points with depth above the desk; re-click or retake the frame')
    centre = np.median(bp, axis=0)
    desk_z = float(coef[0] * centre[0] + coef[1] * centre[1] + coef[2])
    obj = {'desk_z': round(desk_z, 4), 'top_z': round(float(np.percentile(bp[:, 2], 95)), 4),
           'height': round(float(np.percentile(height[seg > 0], 95)), 4),
           'x': [round(float(v), 3) for v in np.percentile(bp[:, 0], [5, 95])], 'y': [round(float(v), 3) for v in np.percentile(bp[:, 1], [5, 95])],
           'centre_xy': [round(float(centre[0]), 3), round(float(centre[1]), 3)], 'points': int(len(bp)),
           'desk_fit_mm': round(float(np.std(resid) * 1000), 1)}
    print(f'desk z {desk_z:.3f} (fit {obj["desk_fit_mm"]} mm); object top z {obj["top_z"]:.3f}, height {obj["height"]:.3f}, '
          f'x {obj["x"]}, y {obj["y"]}, {len(bp)} points')

    est = load_model(); K = P1[:3, :3]; t = time.time()
    pc_full, pc_segments, _ = est.extract_point_clouds(depth, K, segmap=seg, rgb=rgb, z_range=[0.1, 1.5])
    grasps, scores, _, openings = est.predict_scene_grasps(pc_full, pc_segments=pc_segments, local_regions=True, filter_grasps=True, forward_passes=3)
    G, S = grasps.get(1, np.zeros((0, 4, 4))), scores.get(1, np.zeros(0))
    W = openings.get(1, np.full(len(S), np.nan)) if isinstance(openings, dict) else np.full(len(S), np.nan)
    print(f'Contact-GraspNet {time.time() - t:.1f} s: {len(G)} grasps on the object')
    rows = []
    for g, s, wd in zip(G, S, np.atleast_1d(W)):
        Tb = T_base_cam @ g; ap_, close = Tb[:3, 2], Tb[:3, 0]
        c = (Tb @ [0, 0, FINGER_CENTER_Z, 1])[:3]
        down = float(np.degrees(np.arccos(np.clip(-ap_[2], -1, 1))))
        radial = c[:2] / np.linalg.norm(c[:2]); side = np.array([-radial[1], radial[0]])
        out_of_plane = float(np.degrees(np.arcsin(np.clip(abs(ap_[:2] @ side), 0, 1))))
        lean_out = float(ap_[:2] @ radial) >= 0  # leaning away from the base is the lean our arm can make
        reach = down < 20 or (out_of_plane < 15 and lean_out)
        rows.append(dict(score=float(s), width_mm=float(wd) * 1000, down_deg=down, out_of_plane_deg=out_of_plane,
                         reachable=bool(reach and (np.isnan(wd) or wd <= MAX_WIDTH)), centre=c.round(3).tolist(),
                         approach=ap_.round(3).tolist(), jaw_axis=close.round(3).tolist(),
                         jaw_yaw_deg=round(float(np.degrees(np.arctan2(close[1], close[0])) % 180), 1)))
    rows.sort(key=lambda r: (not r['reachable'], r['down_deg'] > 20, -r['score']))
    stem = os.path.join(a.out_dir, os.path.splitext(os.path.basename(a.frame))[0]); os.makedirs(a.out_dir, exist_ok=True)
    json.dump({'frame': os.path.abspath(a.frame), 'mask': os.path.abspath(a.mask), 'object': obj, 'grasps': rows}, open(stem + '_grasps.json', 'w'), indent=1)
    for r in rows[:5]:
        print(f"  score {r['score']:.2f} centre {r['centre']} {r['down_deg']:.0f} deg from down, jaw line {r['jaw_yaw_deg']} deg, width {r['width_mm']:.0f} mm, reachable {r['reachable']}")
    print(f"{sum(r['reachable'] for r in rows)} of {len(rows)} usable by our arm")

    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR); Tcb = np.linalg.inv(T_base_cam)
    def px(p): pc = Tcb[:3, :3] @ p + Tcb[:3, 3]; return tuple(int(v) for v in (K @ pc)[:2] / pc[2])
    for i, r in enumerate([r for r in rows if r['reachable']][:6]):
        c = np.array(r['centre']); j = np.array(r['jaw_axis']) * r['width_mm'] / 2000; col = (0, 255, 255) if i == 0 else (255, 0, 255)
        cv2.line(img, px(c - j), px(c + j), col, 2); cv2.circle(img, px(c), 4, col, -1)
        cv2.line(img, px(c), px(c - np.array(r['approach']) * 0.06), (255, 255, 0), 1)
    cv2.imwrite(stem + '_grasps.jpg', cv2.resize(img[max(0, y0 - 150):y1 + 150, max(0, x0 - 200):x1 + 200], None, fx=2, fy=2))
    print(f'grasps {stem}_grasps.json  overlay {stem}_grasps.jpg')


if __name__ == '__main__':
    main()
