#!/usr/bin/env python3
"""Capture a fresh frame from the named USB camera; never control motors.

By default it reads the continuous stream (camera_stream.py), starting it if needed and waiting until it has run
--warmup-seconds (default 30) and its frames agree in colour and brightness, so exposure and white balance are settled.
November24: cold 5 s captures came out very yellow (red/blue 7-26 at 8-9 s after opening); brightness and colour settled
at about 23 s (red/blue 1.64). The stream stays running, so only the first snapshot after it starts waits.
The returned frame is always one written after this request. --no-stream or a stream failure uses a cold capture.
"""
import argparse
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

import camera_stream


def settle_stats(path):
    """(red/blue, green/blue, mean brightness) of the left half, or None without OpenCV."""
    try:
        import cv2
    except ImportError:
        return None
    f = cv2.imread(str(path))
    if f is None:
        return None
    f = cv2.resize(f[:, :f.shape[1] // 2], None, fx=0.25, fy=0.25).astype(float)
    b, g, r = (max(f[..., i].mean(), 1.0) for i in range(3))
    return (r / b, g / b, float(f.mean()))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--name', default='3D USB Camera')
    p.add_argument('--warmup-seconds', type=float, default=30.0,
                   help='Stream: minimum running time before a frame is used; cold capture: video discarded (default: 30)')
    p.add_argument('--no-stream', action='store_true', help='cold capture only')
    args = p.parse_args()
    if not math.isfinite(args.warmup_seconds) or not 0 <= args.warmup_seconds <= 60:
        p.error('--warmup-seconds must be finite and between 0 and 60')
    ffmpeg = shutil.which('ffmpeg')
    if not ffmpeg:
        p.error('ffmpeg is not installed or not on PATH')
    result = subprocess.run([ffmpeg, '-hide_banner', '-f', 'avfoundation',
                             '-list_devices', 'true', '-i', ''], capture_output=True, text=True, timeout=15)
    video = result.stderr.split('AVFoundation video devices:')[-1].split('AVFoundation audio devices:')[0]
    matches = [idx for idx, name in re.findall(r'\[(\d+)\] (.+)', video) if name.strip() == args.name]
    if len(matches) != 1:
        raise RuntimeError(f'Expected exactly one camera named {args.name!r}; found {len(matches)}. '
                           'Check macOS Camera permission for the host app and camera connection.\n' + result.stderr)
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise RuntimeError('Use a new output filename to prevent stale frame confusion')
    started = time.time()
    if not args.no_stream:
        try:
            st = camera_stream.ensure(matches[0], args.name); camera_stream.touch()
            ready = max(started, st['started_unix'] + args.warmup_seconds)
            deadline = time.time() + args.warmup_seconds + 30
            prev = None; stable = False
            while time.time() < deadline:
                mtime = camera_stream.LATEST.stat().st_mtime if camera_stream.LATEST.exists() else 0
                if mtime > ready:
                    shutil.copyfile(camera_stream.LATEST, output)
                    look = settle_stats(output)
                    if look is None or time.time() - st['started_unix'] > args.warmup_seconds + 15:
                        stable = True  # no OpenCV, or the stream has long been settled: any fresh frame
                    elif prev and mtime - prev[0] >= 1.0:
                        stable = all(abs(a - b) <= tol for a, b, tol in zip(look, prev[1], (0.05, 0.05, 6)))
                    if not stable:
                        if prev is None or mtime - prev[0] >= 1.0:
                            prev = (mtime, look)
                        output.unlink(missing_ok=True); time.sleep(0.25); continue
                    print(json.dumps({'image': str(output), 'camera': args.name, 'source': 'stream',
                                      'capture_started_unix': started, 'capture_finished_unix': time.time(),
                                      'stream_running_s': round(time.time() - st['started_unix'], 1),
                                      'colour': None if look is None else {'r_over_b': round(look[0], 2), 'g_over_b': round(look[1], 2), 'mean': round(look[2], 1)},
                                      'warmup_seconds': args.warmup_seconds, 'rotation_degrees': 180, 'depth_validated': False}))
                    return
                time.sleep(0.1)
            note = 'stream produced no fresh frame in time; cold capture used'
        except Exception as exc:
            note = f'stream unavailable ({exc}); cold capture used'
        print(note, file=sys.stderr)
    try:
        result = subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-f', 'avfoundation',
                                 '-framerate', '30', '-i', matches[0] + ':none',
                                 '-vf', f'setpts=PTS-STARTPTS,select=gte(t\\,{args.warmup_seconds}),hflip,vflip', '-frames:v', '1', '-update', '1', str(output)],
                                capture_output=True, text=True, timeout=30 + args.warmup_seconds)
        if result.returncode or not output.exists() or output.stat().st_size == 0:
            raise RuntimeError('Camera capture failed: ' + result.stderr)
    except BaseException:
        output.unlink(missing_ok=True)
        raise
    print(json.dumps({'image': str(output), 'camera': args.name,
                      'capture_started_unix': started, 'capture_finished_unix': time.time(),
                      'warmup_seconds': args.warmup_seconds, 'rotation_degrees': 180, 'depth_validated': False, 'source': 'cold'}))


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
