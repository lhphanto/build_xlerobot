#!/usr/bin/env python3
"""Record a side-by-side stereo video from the '3D USB Camera' for stereo_vo_map.py.

Only reads the camera; never moves the robot. Drive or carry the robot yourself while it records.

Frames are captured with OpenCV (AVFoundation backend), which gets a real 30 fps at 3840x1080 from
this camera. ffmpeg's own avfoundation input only delivered ~1 fps at that size, so it is used here
just for encoding (frames are piped in).

The frames get the same 180-degree rotation as xlerobot_mcp's camera_snapshot, because the camera is
mounted upside down and the calibration (calibration_fisheye.json) was done on upright frames.
Check that the first frame looks upright, like samples/stereo_side_by_side.png.

Requirements: pip install opencv-python numpy; brew install ffmpeg

Example:
    python slam/record_stereo.py samples/slam/living_room_03.mp4 --seconds 60 --fps 10
Stop early with Ctrl-C (the file is still closed properly).
"""
import argparse
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2

WIDTH, HEIGHT = 3840, 1080


def find_camera(ffmpeg, name):
    """AVFoundation index of the camera. The order changes when cameras are plugged in, so look it up
    by name each time. OpenCV's AVFoundation backend uses the same order."""
    result = subprocess.run([ffmpeg, "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                            capture_output=True, text=True, timeout=15)
    video = result.stderr.split("AVFoundation video devices:")[-1].split("AVFoundation audio devices:")[0]
    matches = [idx for idx, dev in re.findall(r"\[(\d+)\] (.+)", video) if dev.strip() == name]
    if len(matches) != 1:
        sys.exit(f"Expected exactly one camera named {name!r}, found {len(matches)}.\n{video}")
    return int(matches[0])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("output", help="output .mp4 path (must not exist)")
    p.add_argument("--seconds", type=float, default=60.0, help="recording length (default 60)")
    p.add_argument("--fps", type=float, default=10.0, help="frames per second saved, <= 30 (default 10)")
    p.add_argument("--name", default="3D USB Camera", help="AVFoundation camera name")
    p.add_argument("--warmup", type=float, default=3.0,
                   help="seconds dropped at the start while auto-exposure settles (default 3)")
    p.add_argument("--no-rotate", action="store_true", help="skip the 180-degree rotation")
    args = p.parse_args()

    ffmpeg = shutil.which("ffmpeg") or sys.exit("ffmpeg is not installed (brew install ffmpeg)")
    out = Path(args.output).resolve()
    if out.exists():
        sys.exit(f"{out} exists; use a new name")
    out.parent.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(find_camera(ffmpeg, args.name), cv2.CAP_AVFOUNDATION)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    ok, frame = cap.read()
    if not ok or frame.shape[:2] != (HEIGHT, WIDTH):
        sys.exit(f"camera did not give {WIDTH}x{HEIGHT} frames (got {None if not ok else frame.shape})")

    # near-lossless H.264: compression artifacts hurt feature matching and stereo
    enc = subprocess.Popen([ffmpeg, "-hide_banner", "-loglevel", "warning", "-y",
                            "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{WIDTH}x{HEIGHT}", "-r", f"{args.fps:g}",
                            "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "12",
                            "-pix_fmt", "yuv420p", str(out)], stdin=subprocess.PIPE)

    # Encode in a separate thread so a slow encoder never stalls the camera reads.
    frames = queue.Queue(maxsize=60)

    def writer():
        while (f := frames.get()) is not None:
            enc.stdin.write(f.tobytes())
        enc.stdin.close()

    thread = threading.Thread(target=writer)
    thread.start()

    print(f"warming up {args.warmup:g} s, then recording {args.seconds:g} s at {args.fps:g} fps -> {out} "
          "(Ctrl-C to stop early)")
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.warmup:
        cap.read()

    saved = reads = dropped = 0
    start = time.monotonic()
    next_t = 0.0
    try:
        while (now := time.monotonic() - start) < args.seconds:
            ok, frame = cap.read()
            if not ok:
                print("\ncamera read failed; stopping")
                break
            reads += 1
            if now < next_t:
                continue
            next_t += 1.0 / args.fps
            if not args.no_rotate:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            try:
                frames.put_nowait(frame)
                saved += 1
            except queue.Full:
                dropped += 1
            if saved % 10 == 0:
                print(f"\r{now:5.1f} s  saved {saved}  camera {reads / max(now, 1e-6):4.1f} fps", end="", flush=True)
    except KeyboardInterrupt:
        pass
    elapsed = time.monotonic() - start
    cap.release()
    frames.put(None)
    thread.join()
    enc.wait()
    print(f"\nsaved {saved} frames in {elapsed:.1f} s ({saved / max(elapsed, 1e-6):.1f} fps; "
          f"camera gave {reads / max(elapsed, 1e-6):.1f} fps) -> {out}")
    if dropped:
        print(f"WARNING: the encoder fell behind and {dropped} frames were dropped; try a lower --fps")


if __name__ == "__main__":
    main()
