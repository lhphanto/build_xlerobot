#!/usr/bin/env python3
"""Keep the USB stereo camera streaming so exposure and white balance stay settled; never controls motors.

    camera_stream.py run --index N     # foreground supervisor (started detached by ensure()); exits when idle
    camera_stream.py status | stop

ffmpeg reads the camera continuously at 30 fps and writes the latest upright (180 degree rotated) frame twice a
second to latest.png via an atomic rename, so readers never see a partial file. The supervisor stops ffmpeg after
IDLE_SECONDS without a snapshot request (camera_capture.py touches last_used) or if ffmpeg exits.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

DIR = Path(tempfile.gettempdir()) / 'xlerobot-camera-stream'
LATEST = DIR / 'latest.png'
STATE = DIR / 'state.json'
LAST_USED = DIR / 'last_used'
IDLE_SECONDS = 1800
FPS_OUT = 2


def read_state():
    try:
        s = json.loads(STATE.read_text())
        os.kill(s['pid'], 0)
        return s
    except (OSError, ValueError, KeyError):
        return None


def touch():
    DIR.mkdir(parents=True, exist_ok=True)
    LAST_USED.write_text(str(time.time()))


def ensure(index, name):
    """Start the stream if it is not running; returns its state (pid, started_unix)."""
    s = read_state()
    if s and s.get('index') == index:
        return s
    if s:
        stop()
    DIR.mkdir(parents=True, exist_ok=True)
    LATEST.unlink(missing_ok=True)
    touch()
    log = open(DIR / 'stream.log', 'a')
    p = subprocess.Popen([sys.executable, os.path.abspath(__file__), 'run', '--index', index, '--name', name],
                         stdout=log, stderr=log, stdin=subprocess.DEVNULL, start_new_session=True)
    s = {'pid': p.pid, 'index': index, 'name': name, 'started_unix': time.time()}
    STATE.write_text(json.dumps(s))
    return s


def stop():
    s = read_state()
    if s:
        try:
            os.killpg(s['pid'], signal.SIGTERM)
        except OSError:
            pass
    STATE.unlink(missing_ok=True)
    return s


def run(index, name):
    ffmpeg = shutil.which('ffmpeg')
    if not ffmpeg:
        sys.exit('ffmpeg is not installed or not on PATH')
    child = subprocess.Popen([ffmpeg, '-hide_banner', '-loglevel', 'error', '-f', 'avfoundation', '-framerate', '30',
                              '-i', index + ':none', '-vf', f'hflip,vflip,fps={FPS_OUT}', '-f', 'image2', '-update', '1',
                              '-atomic_writing', '1', '-y', str(LATEST)], stdin=subprocess.DEVNULL)
    def finish(*_):
        child.terminate()
        try:
            child.wait(5)
        except subprocess.TimeoutExpired:
            child.kill()
        sys.exit(0)
    signal.signal(signal.SIGTERM, finish); signal.signal(signal.SIGINT, finish)
    print(time.strftime('%H:%M:%S'), f'streaming {name} (index {index}) pid {child.pid}', flush=True)
    while child.poll() is None:
        try:
            idle = time.time() - float(LAST_USED.read_text())
        except (OSError, ValueError):
            idle = 0
        if idle > IDLE_SECONDS:
            print(time.strftime('%H:%M:%S'), f'idle {idle:.0f} s; stopping', flush=True)
            finish()
        time.sleep(2)
    print(time.strftime('%H:%M:%S'), f'ffmpeg exited {child.returncode}', flush=True)
    STATE.unlink(missing_ok=True)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('action', choices=['run', 'status', 'stop'])
    p.add_argument('--index'); p.add_argument('--name', default='3D USB Camera')
    a = p.parse_args()
    if a.action == 'run':
        run(a.index, a.name)
    elif a.action == 'stop':
        print(json.dumps({'stopped': stop()}))
    else:
        s = read_state()
        age = time.time() - LATEST.stat().st_mtime if LATEST.exists() else None
        print(json.dumps({'running': bool(s), 'state': s, 'latest_frame_age_s': age}))


if __name__ == '__main__':
    main()
