#!/usr/bin/env python3
"""Pick still, varied checkerboard frames from a stereo calibration video.

Each kept frame is saved like the snapshot pairs (pair_NN_left_half.png,
pair_NN_right_half.png, pair_NN_side_by_side.png), numbered after the pairs
already in --dir, so calibrate_stereo.py uses them directly.

Frames are kept only when the board is found (in both halves, or either with
--either-half, for edge views one lens cuts off) and has barely moved
since the previous sampled frame (motion blur ruins corners). From those, a
greedy farthest-point choice spreads the selection over board position, size
and tilt.

Example:
    python extract_calib_frames.py samples/calibration/calib_video.mp4 --count 12
"""
import argparse
import glob
import os
import re
import sys

import cv2
import numpy as np

SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)


def detect(gray, pattern):
    """Fast detection at half resolution, then corner refinement at full resolution."""
    small = cv2.resize(gray, None, fx=.5, fy=.5, interpolation=cv2.INTER_AREA)
    ok, corners = cv2.findChessboardCornersSB(small, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        return cv2.cornerSubPix(gray, (corners * 2).astype(np.float32), (7, 7), (-1, -1), SUBPIX)
    # Slower full-resolution search catches tilted or small boards the quick pass misses.
    ok, corners = cv2.findChessboardCornersSB(
        gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE)
    return corners if ok else None


def descriptor(corners, pattern, size):
    """Board centre, apparent size and two tilt cues, each roughly on a 0-1 scale."""
    cols, rows = pattern
    pts = corners.reshape(rows, cols, 2)
    w, h = size
    centre = pts.reshape(-1, 2).mean(axis=0)
    area = cv2.contourArea(np.array([pts[0, 0], pts[0, -1], pts[-1, -1], pts[-1, 0]], np.float32))
    top, bottom = np.linalg.norm(pts[0, -1] - pts[0, 0]), np.linalg.norm(pts[-1, -1] - pts[-1, 0])
    left, right = np.linalg.norm(pts[-1, 0] - pts[0, 0]), np.linalg.norm(pts[-1, -1] - pts[0, -1])
    # Unequal opposite edges mean the board is tilted toward or away from the camera.
    return np.array([centre[0] / w, centre[1] / h, np.sqrt(area / (w * h)),
                     np.log(top / bottom), np.log(left / right)])


def farthest_point(features, count):
    chosen = [int(np.argmax(features[:, 2]))]  # Start from the largest view of the board.
    while len(chosen) < min(count, len(features)):
        dist = np.min(np.linalg.norm(features[:, None, :] - features[None, chosen, :], axis=2), axis=1)
        chosen.append(int(np.argmax(dist)))
    return chosen


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('video')
    parser.add_argument('--dir', default='samples/calibration', help='where pair_NN images go')
    parser.add_argument('--cols', type=int, default=9)
    parser.add_argument('--rows', type=int, default=6)
    parser.add_argument('--count', type=int, default=12, help='frames to keep')
    parser.add_argument('--every', type=int, default=5, help='examine every Nth frame')
    parser.add_argument('--max-motion', type=float, default=2.0,
                        help='largest mean corner movement, in pixels, between examined frames; '
                             '0 disables, for low frame rates where frames are far apart in time')
    parser.add_argument('--either-half', action='store_true',
                        help='keep frames where only one half sees the whole board (edge views)')
    parser.add_argument('--dry-run', action='store_true', help='report the choice without writing images')
    args = parser.parse_args()

    pattern = (args.cols, args.rows)
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit(f'Cannot open {args.video}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    candidates, previous, index, examined = [], None, -1, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        index += 1
        if index % args.every:
            continue
        examined += 1
        h, w = frame.shape[:2]
        if w % 2:
            sys.exit(f'Expected an even-width side-by-side frame; got {w}x{h}')
        halves = [frame[:, :w // 2], frame[:, w // 2:]]
        found = [detect(cv2.cvtColor(half, cv2.COLOR_BGR2GRAY), pattern) for half in halves]
        seen = [i for i, c in enumerate(found) if c is not None]
        if not seen or (len(seen) < 2 and not args.either_half):
            previous = None
            continue
        # Track motion and pose in one half consistently while it stays visible.
        side = seen[0]
        corners = found[side]
        motion = (None if previous is None or previous[0] != side
                  else float(np.mean(np.linalg.norm(corners - previous[1], axis=2))))
        previous = (side, corners)
        # The first detection after a gap has no motion estimate; skip it to be safe.
        if args.max_motion > 0 and (motion is None or motion > args.max_motion):
            continue
        candidates.append({'frame': index, 'time_s': index / fps, 'motion_px': motion, 'image': frame.copy(),
                           'halves': '+'.join(('left', 'right')[i] for i in seen),
                           'feature': descriptor(corners, pattern, (w // 2, h))})
    cap.release()
    print(f'Examined {examined} frames; {len(candidates)} were still with the board fully visible'
          + (' in at least one half' if args.either_half else ' in both halves'))
    if not candidates:
        sys.exit('No usable frames; move more slowly and keep the whole board in view')

    features = np.array([c['feature'] for c in candidates])
    # Scale each cue by its spread so position, size and tilt count equally.
    spread = features.std(axis=0)
    picks = farthest_point(features / np.where(spread > 1e-9, spread, 1), args.count)
    existing = [int(m.group(1)) for p in glob.glob(os.path.join(args.dir, 'pair_*_left_half.png'))
                if (m := re.search(r'pair_(\d+)_left_half', p))]
    number = max(existing, default=0)
    for k in sorted(picks, key=lambda k: candidates[k]['frame']):
        c = candidates[k]; number += 1
        x, y, s = c['feature'][:3]
        moved = 'n/a' if c['motion_px'] is None else f'{c["motion_px"]:.1f}px'
        print(f'pair_{number:02d}: frame {c["frame"]} t={c["time_s"]:.1f}s  board centre ({x * 100:.0f}%, {y * 100:.0f}%)'
              f'  size {s * 100:.0f}%  moved since previous examined frame {moved}  ({c["halves"]})')
        if args.dry_run:
            continue
        img = c['image']; half = img.shape[1] // 2
        cv2.imwrite(os.path.join(args.dir, f'pair_{number:02d}_side_by_side.png'), img)
        cv2.imwrite(os.path.join(args.dir, f'pair_{number:02d}_left_half.png'), img[:, :half])
        cv2.imwrite(os.path.join(args.dir, f'pair_{number:02d}_right_half.png'), img[:, half:])
    cover = features[picks]
    print(f'Selected centres span x {cover[:, 0].min() * 100:.0f}-{cover[:, 0].max() * 100:.0f}%,'
          f' y {cover[:, 1].min() * 100:.0f}-{cover[:, 1].max() * 100:.0f}% of the left half')


if __name__ == '__main__':
    main()
