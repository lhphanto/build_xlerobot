#!/usr/bin/env python3
"""SlimSAM object mask from one or more clicks on the raw left image (contact_graspnet env). No motion.

The mask is in the fisheye-rectified left image, the same pixels cgn_grasps.py uses for depth.
Click the object's top and one face (one click on a face alone can miss the top).

    python grasp/sam_click.py FRAME.png --click 958 930 --click 958 1010
writes samples/grasp/<frame>_mask.npy and _mask.jpg
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
sys.path.insert(0, ROOT)
import reach_point as rp  # noqa: E402

MODEL = 'Zigeng/SlimSAM-uniform-50'
_CACHE = []  # model, processor (loaded once per process)


def segment(rect_rgb, raw_clicks, K1, D1, R1, P1):
    """SlimSAM mask (rectified pixels) from raw left-image clicks; returns (mask, scores, rectified clicks, seconds)."""
    from transformers import SamModel, SamProcessor
    rc = cv2.fisheye.undistortPoints(np.array(raw_clicks, float).reshape(-1, 1, 2), K1, D1, R=R1, P=P1).reshape(-1, 2)
    t = time.time()
    if not _CACHE:
        _CACHE.extend([SamModel.from_pretrained(MODEL).eval(), SamProcessor.from_pretrained(MODEL)])
    model, proc = _CACHE
    inp = proc(rect_rgb, input_points=[[rc.tolist()]], return_tensors='pt')
    with torch.no_grad():
        out = model(**inp)
    masks = proc.image_processor.post_process_masks(out.pred_masks, inp['original_sizes'], inp['reshaped_input_sizes'])[0][0].numpy()
    iou = out.iou_scores[0, 0].numpy()
    return masks[int(np.argmax(iou))], iou, rc, time.time() - t


def rectified(frame):
    """(rectified left RGB, K1, D1, R1, P1, Q, maps) for a side-by-side frame."""
    left = frame[:, :frame.shape[1] // 2]; size = (left.shape[1], left.shape[0])
    K1, D1, R1, P1, Q, maps = rp.rectifier(json.load(open(rp.CAMERA)), size)
    return cv2.cvtColor(cv2.remap(left, *maps[0], cv2.INTER_LINEAR), cv2.COLOR_BGR2RGB), K1, D1, R1, P1, Q, maps


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('frame')
    ap.add_argument('--click', type=float, nargs=2, action='append', required=True, metavar=('U', 'V'), help='raw left-half pixel on the object')
    ap.add_argument('--out-dir', default=os.path.join(ROOT, 'samples', 'grasp'))
    a = ap.parse_args()
    f = cv2.imread(a.frame)
    if f is None:
        sys.exit(f'Cannot read {a.frame}')
    rect, K1, D1, R1, P1, Q, _ = rectified(f)
    m, iou, rc, secs = segment(rect, a.click, K1, D1, R1, P1)
    print(f'SlimSAM {secs:.1f} s; candidate scores {iou.round(3).tolist()}; using the best')
    os.makedirs(a.out_dir, exist_ok=True)
    stem = os.path.join(a.out_dir, os.path.splitext(os.path.basename(a.frame))[0])
    np.save(stem + '_mask.npy', m.astype(np.uint8))
    vis = rect.copy(); vis[m] = (0.5 * vis[m] + [0, 127, 0]).astype(np.uint8)
    for u, v in rc:
        cv2.circle(vis, (int(u), int(v)), 6, (255, 255, 0), -1)
    ys, xs = np.nonzero(m)
    crop = vis[max(0, ys.min() - 150):ys.max() + 150, max(0, xs.min() - 200):xs.max() + 200]
    cv2.imwrite(stem + '_mask.jpg', cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
    print(f'mask {stem}_mask.npy  (check {stem}_mask.jpg covers the whole object, top included)')


if __name__ == '__main__':
    main()
