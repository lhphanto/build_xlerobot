#!/usr/bin/env python3
"""Capture a fresh frame from the named USB camera; never control motors."""
import argparse
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--name', default='3D USB Camera')
    p.add_argument('--warmup-seconds', type=float, default=5.0,
                   help='Discard this many seconds of incoming video before saving (default: 5)')
    args = p.parse_args()
    if not math.isfinite(args.warmup_seconds) or not 0 <= args.warmup_seconds <= 15:
        p.error('--warmup-seconds must be finite and between 0 and 15')
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
                      'warmup_seconds': args.warmup_seconds, 'rotation_degrees': 180, 'depth_validated': False}))


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
