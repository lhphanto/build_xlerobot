#!/usr/bin/env python3
"""Batch disparity for a side-by-side stereo recording: SGBM or RAFT-Stereo, saved per frame.

Meant to run on a remote machine (CPU or GPU). Uses exactly the same fisheye rectification as
stereo_vo_map.py, so the saved disparities line up with its rectified left camera.

Inputs (copy these to the remote machine):
    samples/slam/living_room_03.mp4                 side-by-side 3840 x 1080, already upright
    samples/calibration/calibration_fisheye.json    stereo fisheye calibration
    slam/stereo_depth_batch.py                      this script

Setup:
    pip install opencv-python numpy                 # SGBM only
    pip install torch                               # + RAFT-Stereo (CUDA build on a GPU machine)
    git clone https://github.com/princeton-vl/RAFT-Stereo.git
    cd RAFT-Stereo && bash download_models.sh       # -> models/raftstereo-middlebury.pth, raftstereo-realtime.pth, ...
    (download_models.sh uses wget and "mkdir models -p"; on macOS fetch the zip with curl instead)

Run (frame selection: --every N, --start/--end, or --frames 0,10,20):
    python stereo_depth_batch.py VIDEO CALIB OUT_DIR --method sgbm --size half
    python stereo_depth_batch.py VIDEO CALIB OUT_DIR --method sgbm --size full       # needs ~2-3 GB RAM
    python stereo_depth_batch.py VIDEO CALIB OUT_DIR --method raft --model middlebury --raft-repo RAFT-Stereo
    python stereo_depth_batch.py VIDEO CALIB OUT_DIR --method raft --model realtime   --raft-repo RAFT-Stereo

Rough speed per frame (half size): SGBM ~0.05 s; RAFT middlebury ~0.3-1 s on a T4-class GPU, tens of
seconds on CPU; RAFT realtime several times faster. Try --end 20 first to check speed and memory.

Output (OUT_DIR):
    meta.json            method, size, rectified intrinsics fx fy cx cy, baseline_m, list of frames
    disp_000123.npz      key "disp": float16 H x W disparity in pixels of the rectified left image,
                         NaN where invalid. Depth Z = fx * baseline_m / disp.
    preview_000123.jpg   (with --previews) rectified left image | colorized depth, for a quick check
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

RAW_SIZE = (1920, 1080)                        # per lens
SIZES = {"half": (960, 540), "full": (1920, 1080)}

# RAFT-Stereo settings per checkpoint, from the RAFT-Stereo README
RAFT_MODELS = {
    "middlebury": dict(ckpt="raftstereo-middlebury.pth", iters=32, hidden_dims=[128] * 3, shared_backbone=False,
                       n_downsample=2, n_gru_layers=3, slow_fast_gru=False),
    "eth3d":      dict(ckpt="raftstereo-eth3d.pth", iters=32, hidden_dims=[128] * 3, shared_backbone=False,
                       n_downsample=2, n_gru_layers=3, slow_fast_gru=False),
    "realtime":   dict(ckpt="raftstereo-realtime.pth", iters=7, hidden_dims=[128] * 3, shared_backbone=True,
                       n_downsample=3, n_gru_layers=2, slow_fast_gru=True),
}


# ---------------------------------------------------------------------------------------------
# Rectification (same as stereo_vo_map.py)
# ---------------------------------------------------------------------------------------------
def make_rectifier(calib_path, size):
    cal = json.load(open(calib_path))
    K1 = np.array(cal["left"]["camera_matrix"]); D1 = np.array(cal["left"]["distortion"]).reshape(4, 1)
    K2 = np.array(cal["right"]["camera_matrix"]); D2 = np.array(cal["right"]["distortion"]).reshape(4, 1)
    R_lr = np.array(cal["stereo"]["R"])
    T_lr = np.array(cal["stereo"]["T_mm"]).reshape(3, 1) / 1000.0
    R1, R2, P1, P2, _ = cv2.fisheye.stereoRectify(K1, D1, K2, D2, RAW_SIZE, R_lr, T_lr,
                                                  flags=cv2.CALIB_ZERO_DISPARITY, newImageSize=size,
                                                  balance=0.0, fov_scale=1.0)
    map_l = cv2.fisheye.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_16SC2)
    map_r = cv2.fisheye.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_16SC2)
    intr = dict(fx=float(P1[0, 0]), fy=float(P1[1, 1]), cx=float(P1[0, 2]), cy=float(P1[1, 2]),
                baseline_m=float(-P2[0, 3] / P2[0, 0]))

    def rectify(frame):
        left = cv2.remap(frame[:, :RAW_SIZE[0]], *map_l, cv2.INTER_LINEAR)
        right = cv2.remap(frame[:, RAW_SIZE[0]:], *map_r, cv2.INTER_LINEAR)
        return left, right

    return rectify, intr


# ---------------------------------------------------------------------------------------------
# Depth methods: each returns (H x W float32 disparity, NaN = invalid)
# ---------------------------------------------------------------------------------------------
def make_sgbm(size_name, block=5):
    ndisp = 96 if size_name == "half" else 192     # max disparity ~ fx * b / 0.25 m min depth
    sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=ndisp, blockSize=block,
                                 P1=8 * block**2, P2=32 * block**2, disp12MaxDiff=1, preFilterCap=63,
                                 uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
                                 mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def run(left_bgr, right_bgr):
        gl = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
        d = sgbm.compute(gl, gr).astype(np.float32) / 16.0   # OpenCV returns disparity * 16
        d[d <= 0.5] = np.nan
        return d

    return run


def pick_device(name):
    import torch
    if name != "auto":
        return name
    if torch.cuda.is_available():
        return "cuda"
    if getattr(torch.backends, "mps", None) and torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def make_raft(repo, model_name, device, iters=None, mixed_precision=None):
    import torch
    repo = Path(repo).resolve()
    sys.path[:0] = [str(repo), str(repo / "core")]
    from raft_stereo import RAFTStereo           # noqa: E402  (from the RAFT-Stereo repo)
    from utils.utils import InputPadder          # noqa: E402

    cfg = RAFT_MODELS[model_name]
    args = argparse.Namespace(
        hidden_dims=cfg["hidden_dims"], corr_implementation="reg", shared_backbone=cfg["shared_backbone"],
        corr_levels=4, corr_radius=4, n_downsample=cfg["n_downsample"], context_norm="batch",
        slow_fast_gru=cfg["slow_fast_gru"], n_gru_layers=cfg["n_gru_layers"],
        mixed_precision=(device == "cuda") if mixed_precision is None else mixed_precision)
    model = RAFTStereo(args)
    ckpt = repo / "models" / cfg["ckpt"]
    state = torch.load(ckpt, map_location="cpu", weights_only=True)
    model.load_state_dict({k.removeprefix("module."): v for k, v in state.items()})
    model = model.to(device).eval()
    n_iters = iters or cfg["iters"]

    def to_tensor(bgr):                          # RAFT-Stereo expects RGB, 0-255 float, N x 3 x H x W
        return torch.from_numpy(np.ascontiguousarray(bgr[:, :, ::-1])).permute(2, 0, 1).float()[None].to(device)

    def run(left_bgr, right_bgr):
        a, b = to_tensor(left_bgr), to_tensor(right_bgr)
        padder = InputPadder(a.shape, divis_by=32)
        a, b = padder.pad(a, b)
        with torch.no_grad():
            _, flow = model(a, b, iters=n_iters, test_mode=True)
        d = -padder.unpad(flow).squeeze().float().cpu().numpy()   # flow along x is minus the disparity
        d[d <= 0.5] = np.nan
        return d

    return run


# ---------------------------------------------------------------------------------------------
def colorize_depth(disp, intr, max_depth=4.0):
    z = intr["fx"] * intr["baseline_m"] / disp
    zc = np.clip(np.nan_to_num(z, nan=0.0) / max_depth, 0, 1)
    img = cv2.applyColorMap((255 - zc * 255).astype(np.uint8), cv2.COLORMAP_TURBO)   # red = near
    img[~(np.isfinite(z) & (z < max_depth))] = 0
    return img


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("video"); p.add_argument("calib"); p.add_argument("out_dir")
    p.add_argument("--method", choices=["sgbm", "raft"], required=True)
    p.add_argument("--size", choices=list(SIZES), default="half", help="rectified size per lens (default half)")
    p.add_argument("--model", choices=list(RAFT_MODELS), default="middlebury", help="RAFT-Stereo checkpoint")
    p.add_argument("--raft-repo", default="RAFT-Stereo", help="path to the cloned RAFT-Stereo repo (with models/)")
    p.add_argument("--device", default="auto", help="auto | cuda | cuda:1 | mps | cpu")
    p.add_argument("--iters", type=int, help="RAFT GRU iterations (default: 32 middlebury, 7 realtime)")
    p.add_argument("--every", type=int, default=1, help="process every Nth frame (default 1)")
    p.add_argument("--start", type=int, default=0); p.add_argument("--end", type=int, help="stop before this frame")
    p.add_argument("--frames", help="comma-separated frame numbers instead of --every/--start/--end")
    p.add_argument("--previews", type=int, default=50, help="save a preview jpg every N processed frames (0 = none)")
    a = p.parse_args()

    out = Path(a.out_dir); out.mkdir(parents=True, exist_ok=True)
    size = SIZES[a.size]
    rectify, intr = make_rectifier(a.calib, size)
    if a.method == "sgbm":
        run, label = make_sgbm(a.size), f"sgbm_{a.size}"
    else:
        device = pick_device(a.device)
        run, label = make_raft(a.raft_repo, a.model, device, a.iters), f"raft_{a.model}_{a.size}"
        print("device:", device)

    cap = cv2.VideoCapture(a.video)
    n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if a.frames:
        wanted = sorted({int(x) for x in a.frames.split(",")})
    else:
        wanted = list(range(a.start, min(a.end or n_total, n_total), a.every))
    wanted_set = set(wanted)
    print(f"{label}: {len(wanted)} of {n_total} frames, rectified {size[0]}x{size[1]}, "
          f"fx {intr['fx']:.1f}, baseline {intr['baseline_m'] * 1000:.1f} mm -> {out}")

    done, t0 = [], time.perf_counter()
    for i in range(max(wanted) + 1):
        if i not in wanted_set:
            if not cap.grab():                     # skip without decoding into numpy
                break
            continue
        ok, frame = cap.read()
        if not ok:
            break
        if frame.shape[:2] != (1080, 3840):
            sys.exit(f"frame {i}: expected 3840 x 1080 side-by-side, got {frame.shape}")
        left, right = rectify(frame)
        t = time.perf_counter()
        disp = run(left, right)
        dt = time.perf_counter() - t
        np.savez_compressed(out / f"disp_{i:06d}.npz", disp=disp.astype(np.float16))
        done.append(i)
        if a.previews and (len(done) - 1) % a.previews == 0:
            cv2.imwrite(str(out / f"preview_{i:06d}.jpg"), np.hstack([left, colorize_depth(disp, intr)]))
        valid = np.mean(np.isfinite(disp)) * 100
        print(f"frame {i:5d}  {dt:6.2f} s  valid {valid:3.0f}%  ({len(done)}/{len(wanted)}, "
              f"elapsed {time.perf_counter() - t0:7.1f} s)", flush=True)

    meta = dict(method=label, video=str(a.video), size=list(size), frames=done, **intr,
                note="disp_XXXXXX.npz['disp']: float16 disparity (px) in the rectified left image; "
                     "NaN = invalid; depth = fx * baseline_m / disp")
    (out / "meta.json").write_text(json.dumps(meta, indent=1))
    print(f"saved {len(done)} frames in {time.perf_counter() - t0:.1f} s to {out}")


if __name__ == "__main__":
    main()
