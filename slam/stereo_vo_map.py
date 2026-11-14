# %% [markdown]
# # Stereo visual odometry + 2D occupancy grid for the XLeRobot fisheye stereo camera
# A small "slambook chapter 13"-style pipeline built from the pieces in job_search_round2:
#   1. rectify each side-by-side frame with the fisheye stereo calibration (pinhole, row-aligned)
#   2. SGBM disparity -> metric depth for the rectified left camera
#   3. tracking: ORB matches between the current frame and the last KEYFRAME,
#      3D points from the keyframe's depth, solvePnPRansac, then Huber Gauss-Newton BA (Section 6.7)
#   4. mapping: at each keyframe, project depth into a gravity-aligned world frame and update a
#      log-odds occupancy grid (floor = free, things at robot height = occupied, ray-casting for free space)
#   5. dense map: fuse the depth of EVERY frame into a TSDF volume (slambook Ch. 12) -> colored mesh
#   6. visualize everything in Rerun: camera, trajectory, point cloud, mesh, occupancy grid, per frame
#
#   1b. loop closure: recognize places seen before, then pose-graph optimization (slambook Ch. 10-11)
#
# Requirements: `pip install opencv-python numpy scipy matplotlib rerun-sdk open3d`
#   (on macOS, open3d also needs `brew install libusb`)
# Record input with: `python slam/record_stereo.py samples/slam/living_room_01.mp4 --seconds 60`

# %%
import json
import os
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import open3d.core as o3c
import rerun as rr
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

np.set_printoptions(precision=4, suppress=True)

# %% [markdown]
# ## Settings

# %%
REPO = "/Users/huiliu/Documents/build_xlerobot"
CALIB_PATH = f"{REPO}/samples/calibration/calibration_fisheye.json"
VIDEO = f"{REPO}/samples/slam/living_room_01.mp4"     # side-by-side 3840 x 1080, upright
# Optional precomputed disparities from stereo_depth_batch.py (e.g. RAFT-Stereo run on a GPU machine).
# None = SGBM here. When set, ONLY the frames in that folder are used (e.g. every 3rd), for tracking,
# the occupancy grid and the TSDF mesh; outputs go to <video>_slam_<folder name>.
DEPTH_DIR = None                  # e.g. f"{REPO}/samples/slam/output_raft_mb"
OUT_DIR = os.path.splitext(VIDEO)[0] + "_slam" + (f"_{os.path.basename(DEPTH_DIR.rstrip('/'))}" if DEPTH_DIR else "")

PROC_SIZE = (960, 540)            # rectified size per lens; half resolution keeps it ~10 fps
MIN_DEPTH, MAX_DEPTH = 0.25, 4.0  # meters; stereo depth beyond ~4 m is too noisy (baseline 6 cm)

# keyframes: start a new one when tracking against the old one gets weak or the camera moved enough
KF_MIN_INLIERS = 120
KF_MAX_TRANSLATION = 0.20         # m
KF_MAX_ROTATION = 15.0            # deg
MIN_TRACK_INLIERS = 30            # below this the frame counts as lost
MAX_STEP = 0.30                   # m per frame; larger jumps are rejected as tracking failures
HUBER_K = 2.0                     # pixels

# loop closure + pose graph
LOOP_CLOSURE = True
LOOP_MIN_GAP = 8                  # only compare keyframes at least this many keyframes apart
LOOP_MIN_MATCHES = 150            # ORB matches needed before trying the geometric check
LOOP_MIN_INLIERS = 80             # PnP inliers needed to accept a loop
LOOP_MAX_PER_KF = 3               # keep the best few loops per keyframe
LOOP_MAX_JUMP = (1.0, 30.0)       # m, deg: reject loops that disagree with odometry by more (false matches)
PG_SIGMA_T, PG_SIGMA_R = 0.02, 1.0   # m, deg: uncertainty of one edge (odometry and loops alike)

# world frame: z up, origin on the floor under the first camera position, x = first view direction
CAMERA_HEIGHT = 1.20              # m, lens above the floor (tape-measured; was 1.22 m before the
                                  # camera was tilted down, i.e. for living_room_01/02).
                                  # None = estimate from the bottom of the images (furniture biases it low)

# occupancy grid
GRID_RES = 0.05                   # m per cell
GRID_SIZE = 12.0                  # m, square, centered on the start position
FLOOR_TOL = 0.06                  # |height| below this = floor = free (at 1 m; grows with distance)
FLOOR_TOL_PER_M = 0.08            # extra floor tolerance per meter of depth: stereo error grows with range
OBST_MARGIN = 0.05                # obstacles start this far above the floor tolerance
OBST_MAX = 1.40                   # m; points higher than this don't block the robot
MAP_RANGE = 3.5                   # m, ignore farther depth for the map
L_OCC, L_FREE, L_CLAMP = 0.85, -0.4, 4.0   # log-odds update per keyframe

# dense map: TSDF fusion of every frame's depth
TSDF_VOXEL = 0.02                 # m; finer than ~1.5 cm is below what 6 cm stereo resolves
TSDF_TRUNC = 0.08                 # m; truncation distance, a few times the depth noise at ~1.5 m
TSDF_MAX_DEPTH = 2.5              # m; depth error is ~5 cm at 2 m and ~12 cm at 3 m, so drop the far part
FUSE_EVERY = 1                    # fuse every Nth tracked frame (1 = all)

VIEWER = "spawn"                  # "spawn" = Rerun desktop viewer, "save" = OUT_DIR/map.rrd only,
                                  # "notebook" = inline viewer in Jupyter
MAX_FRAMES = None                 # e.g. 100 for a quick test
os.makedirs(OUT_DIR, exist_ok=True)

# %% [markdown]
# ## Calibration and rectification maps
# `newImageSize` makes the rectified pinhole camera directly at PROC_SIZE, so its fx, fy, cx, cy
# (from P1) are the half-resolution intrinsics. Everything below works in this rectified left camera.

# %%
cal = json.load(open(CALIB_PATH))
K1 = np.array(cal["left"]["camera_matrix"]);  D1 = np.array(cal["left"]["distortion"]).reshape(4, 1)
K2 = np.array(cal["right"]["camera_matrix"]); D2 = np.array(cal["right"]["distortion"]).reshape(4, 1)
R_lr = np.array(cal["stereo"]["R"])
T_lr = np.array(cal["stereo"]["T_mm"]).reshape(3, 1) / 1000.0     # meters, X_right = R X_left + T
RAW_SIZE = (1920, 1080)

R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K1, D1, K2, D2, RAW_SIZE, R_lr, T_lr, flags=cv2.CALIB_ZERO_DISPARITY,
    newImageSize=PROC_SIZE, balance=0.0, fov_scale=1.0)
map_l = cv2.fisheye.initUndistortRectifyMap(K1, D1, R1, P1, PROC_SIZE, cv2.CV_16SC2)
map_r = cv2.fisheye.initUndistortRectifyMap(K2, D2, R2, P2, PROC_SIZE, cv2.CV_16SC2)

fx, fy, cx, cy = P1[0, 0], P1[1, 1], P1[0, 2], P1[1, 2]
K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
BASELINE = -P2[0, 3] / P2[0, 0]
W, H = PROC_SIZE
print(f"rectified camera: fx {fx:.1f} fy {fy:.1f} cx {cx:.1f} cy {cy:.1f}, baseline {BASELINE * 1000:.1f} mm, "
      f"horizontal FOV {np.degrees(2 * np.arctan(W / 2 / fx)):.0f} deg")


def rectify(frame_bgr):
    """Side-by-side BGR frame -> rectified left BGR, left gray, right gray at PROC_SIZE."""
    left, right = frame_bgr[:, :RAW_SIZE[0]], frame_bgr[:, RAW_SIZE[0]:]
    left_r = cv2.remap(left, *map_l, cv2.INTER_LINEAR)
    right_r = cv2.remap(right, *map_r, cv2.INTER_LINEAR)
    return left_r, cv2.cvtColor(left_r, cv2.COLOR_BGR2GRAY), cv2.cvtColor(right_r, cv2.COLOR_BGR2GRAY)


# %% [markdown]
# ## Stereo depth (SGBM, Z = f b / d)

# %%
BLOCK = 5
sgbm = cv2.StereoSGBM_create(
    minDisparity=0, numDisparities=96, blockSize=BLOCK,
    P1=8 * BLOCK**2, P2=32 * BLOCK**2, disp12MaxDiff=1, preFilterCap=63,
    uniquenessRatio=10, speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)


def stereo_depth(gray_l, gray_r):
    disp = sgbm.compute(gray_l, gray_r).astype(np.float32) / 16.0
    depth = np.full(disp.shape, np.nan, np.float32)
    ok = disp > 0.5
    depth[ok] = fx * BASELINE / disp[ok]
    depth[(depth < MIN_DEPTH) | (depth > MAX_DEPTH)] = np.nan
    return depth


# Precomputed disparity (stereo_depth_batch.py): same rectification, possibly at 2x this size.
depth_meta = None
if DEPTH_DIR:
    depth_meta = json.load(open(f"{DEPTH_DIR}/meta.json"))
    DEPTH_SCALE = depth_meta["size"][0] / W
    assert DEPTH_SCALE in (1, 2) and abs(depth_meta["fx"] / DEPTH_SCALE - fx) < 0.5 \
        and abs(depth_meta["baseline_m"] - BASELINE) < 1e-4, \
        f"{DEPTH_DIR} was made with a different calibration or size: {depth_meta}"
    DEPTH_FRAMES = set(depth_meta["frames"])
    print(f"depth from {DEPTH_DIR} ({depth_meta['method']}, {len(DEPTH_FRAMES)} frames)")


def frame_depth(i, gray_l, gray_r):
    """Depth for video frame i: from DEPTH_DIR if set, else SGBM on the rectified pair."""
    if depth_meta is None:
        return stereo_depth(gray_l, gray_r)
    disp = np.load(f"{DEPTH_DIR}/disp_{i:06d}.npz")["disp"].astype(np.float32)
    if DEPTH_SCALE != 1:              # full-size disparity -> this size: pixels and disparity both shrink
        disp = cv2.resize(disp, (W, H), interpolation=cv2.INTER_NEAREST) / DEPTH_SCALE
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = fx * BASELINE / disp
    depth[~((depth >= MIN_DEPTH) & (depth <= MAX_DEPTH))] = np.nan
    return depth.astype(np.float32)


def backproject(u, v, z):
    """Pixels + depth -> 3D points in the rectified left camera frame (x right, y down, z forward)."""
    return np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])


# %% [markdown]
# ## Features
# Cross-checked brute-force Hamming matching: a match is kept only if each point is the other's
# best match. It replaces the book's "2 x min distance" rule, which depends on the single best match.

# %%
orb = cv2.ORB_create(nfeatures=2000)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)


def detect(gray):
    kps, des = orb.detectAndCompute(gray, None)
    return np.float64([k.pt for k in kps]).reshape(-1, 2), des


def match(des_a, des_b, max_dist=50):
    if des_a is None or des_b is None:
        return np.zeros((0, 2), int)
    ms = [m for m in matcher.match(des_a, des_b) if m.distance < max_dist]
    return np.array([(m.queryIdx, m.trainIdx) for m in ms], int).reshape(-1, 2)


# %% [markdown]
# ## Pose refinement: Huber Gauss-Newton on SE(3) (same as pose_estimation_3d2d_ba.py)

# %%
def hat(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def se3_exp(xi):
    rho, phi = xi[:3], xi[3:]
    theta = np.linalg.norm(phi)
    if theta < 1e-10:
        Rm, J = np.eye(3) + hat(phi), np.eye(3) + 0.5 * hat(phi)
    else:
        A = hat(phi / theta)
        Rm = np.eye(3) + np.sin(theta) * A + (1 - np.cos(theta)) * A @ A
        J = np.eye(3) + (1 - np.cos(theta)) / theta * A + (theta - np.sin(theta)) / theta * A @ A
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rm, J @ rho
    return T


def reproject(T, P3):
    pc = P3 @ T[:3, :3].T + T[:3, 3]
    return np.column_stack([fx * pc[:, 0] / pc[:, 2] + cx, fy * pc[:, 1] / pc[:, 2] + cy]), pc


def refine_pose_huber(T, P3, p2, k=HUBER_K, iterations=10):
    last = np.inf
    for _ in range(iterations):
        proj, pc = reproject(T, P3)
        e = p2 - proj
        r = np.linalg.norm(e, axis=1)
        w = np.where(r <= k, 1.0, k / np.maximum(r, 1e-12))
        cost = np.sum(np.where(r <= k, r**2, 2 * k * r - k**2))
        if cost >= last:
            break
        X, Y, Z = pc.T
        iz, iz2 = 1 / Z, 1 / Z**2
        J = np.zeros((len(pc), 2, 6))
        J[:, 0] = np.column_stack([-fx * iz, 0 * X, fx * X * iz2, fx * X * Y * iz2, -fx - fx * X * X * iz2, fx * Y * iz])
        J[:, 1] = np.column_stack([0 * X, -fy * iz, fy * Y * iz2, fy + fy * Y * Y * iz2, -fy * X * Y * iz2, -fy * X * iz])
        Hm = np.einsum("n,nij,nik->jk", w, J, J)
        b = -np.einsum("n,nij,ni->j", w, J, e)
        dx = np.linalg.solve(Hm, b)
        T, last = se3_exp(dx) @ T, cost
        if np.linalg.norm(dx) < 1e-7:
            break
    return T


def rt_to_T(rvec, tvec):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
    return T


def rotation_deg(Rm):
    return np.degrees(np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1)))


# %% [markdown]
# ## Tracking one frame against a keyframe (3D-2D, PnP + BA)
# T_kc maps keyframe-camera points into the current camera: X_cur = T_kc X_kf.

# %%
def track(kf, pts, des, T_guess):
    m = match(kf["des"], des)
    if len(m) == 0:
        return None, 0
    P3 = kf["P3"][m[:, 0]]
    p2 = pts[m[:, 1]]
    ok = ~np.isnan(P3[:, 2])
    P3, p2 = P3[ok], p2[ok]
    if len(P3) < MIN_TRACK_INLIERS:
        return None, len(P3)
    rvec0 = cv2.Rodrigues(T_guess[:3, :3])[0]
    tvec0 = T_guess[:3, 3].reshape(3, 1).copy()
    success, rvec, tvec, inl = cv2.solvePnPRansac(
        P3, p2, K, None, rvec0, tvec0, useExtrinsicGuess=True,
        iterationsCount=200, reprojectionError=2.0, confidence=0.999)
    if not success or inl is None or len(inl) < MIN_TRACK_INLIERS:
        return None, 0 if inl is None else len(inl)
    T = rt_to_T(rvec, tvec)
    # Refine with Huber BA on everything within 5 px of the RANSAC pose (inliers + near-inliers).
    proj, pc = reproject(T, P3)
    near = (np.linalg.norm(p2 - proj, axis=1) < 5.0) & (pc[:, 2] > 0)
    T = refine_pose_huber(T, P3[near], p2[near])
    return T, len(inl)


# %% [markdown]
# ## World frame: "up" and the floor, estimated after tracking
# Tracking runs in the "odom" frame (= the first camera). Afterwards we find a gravity-aligned world:
#   - up: the robot drives on a flat floor, so the camera path lies in a plane parallel to the floor;
#     its normal (smallest PCA direction of the path) is "up". Needs the path to span some area
#     (side steps or a turn); otherwise fall back to the average camera orientation.
#   - camera height: CAMERA_HEIGHT if set, else the median height of points in the bottom rows of
#     the keyframes (mostly floor), taken relative to each keyframe's own camera so drift cancels.
#   - planar snap: on a flat floor with a rigid mount every pose has the same height, pitch and roll;
#     only x, y and heading change. Snapping keyframes to that removes the VO's tilt/height drift,
#     which would otherwise lift far-away floor into the obstacle band (1 deg tilt = 5 cm at 3 m).

# %%
def rot_a_to_b(a, b):
    """Smallest rotation taking unit vector a to unit vector b."""
    v, c = np.cross(a, b), a @ b
    s2 = v @ v
    if s2 < 1e-16:
        return np.eye(3)
    Kx = hat(v)
    return np.eye(3) + Kx + Kx @ Kx * ((1 - c) / s2)


def estimate_world(traj_o, R_oc, keyframes):
    """Returns T_wo (odom -> world: z up, z = 0 on the floor, x = first view direction) and the
    camera height."""
    up_guess = -np.mean([R[:, 1] for R in R_oc], axis=0)       # cameras are roughly upright
    up_guess /= np.linalg.norm(up_guess)
    c = traj_o - traj_o.mean(0)
    ev, evec = np.linalg.eigh(c.T @ c / max(len(c), 1))
    extent = np.sqrt(np.maximum(ev, 0))
    up, how = up_guess, "average camera orientation (path too short or straight for a plane fit)"
    if extent[1] > 0.15:                                       # path spans an area, not just a line
        n = evec[:, 0] * np.sign(evec[:, 0] @ up_guess)
        if np.degrees(np.arccos(np.clip(n @ up_guess, -1, 1))) < 35:
            up, how = n, f"plane of the camera path ({extent[2]:.2f} x {extent[1]:.2f} m)"

    if CAMERA_HEIGHT is not None:
        cam_h, how_h = CAMERA_HEIGHT, "set"
    else:
        below = []
        for kf in keyframes:                                   # bottom quarter of each keyframe
            v, u = np.mgrid[int(0.75 * H):H:4, 0:W:4]
            z = kf["depth"][v, u].astype(np.float32)
            ok = ~np.isnan(z)
            h = (backproject(u[ok], v[ok], z[ok]) @ kf["T_oc"][:3, :3].T) @ up   # relative to that camera
            below.append(h[h < -0.3])
        below = np.concatenate(below)
        if len(below) > 1000:
            cam_h, how_h = -np.median(below), f"median of {len(below)} bottom-of-image points"
        else:
            cam_h, how_h = 1.0, "NOT FOUND, guessed"
    floor_h = np.mean(traj_o @ up) - cam_h
    fwd = np.array([0.0, 0.0, 1.0]) - up[2] * up               # first camera's view direction, leveled
    fwd /= np.linalg.norm(fwd)
    T = np.eye(4)
    T[:3, :3] = np.vstack([fwd, np.cross(up, fwd), up])        # rows = world x, y, z in odom coords
    T[:3, 3] = [0, 0, -floor_h]
    down = np.mean([R[:, 1] for R in R_oc], 0)
    tilt = np.degrees(np.arccos(np.clip(-down @ up / np.linalg.norm(down), -1, 1)))
    print(f"up from {how}\ncamera height {cam_h:.3f} m ({how_h}); camera mount tilted {tilt:.1f} deg from level; "
          f"VO height wobble +-{np.std(traj_o @ up) * 100:.1f} cm (removed by the planar snap)")
    return T, cam_h


def snap_planar(T_wc, cam_h, down_w):
    """Keep x, y and heading; set height to cam_h and the camera's down axis to the mount's down_w."""
    T = T_wc.copy()
    T[:3, :3] = rot_a_to_b(T[:3, 1], down_w) @ T[:3, :3]
    T[2, 3] = cam_h
    return T


# %% [markdown]
# ## Occupancy grid (log-odds)
# For each depth point in world coordinates (tol = floor tolerance, growing with depth):
#   - floor point (|z| < tol): its cell and the cells along the ray from the camera are free
#   - obstacle point (tol + OBST_MARGIN < z < OBST_MAX): its cell is occupied, the cells before it are free
# Each keyframe adds L_OCC / L_FREE once per touched cell, clamped, so repeated views firm it up and
# a moving person leaves only a faint trace.

# %%
N_CELLS = int(round(GRID_SIZE / GRID_RES))
logodds = np.zeros((N_CELLS, N_CELLS), np.float32)        # [ix, iy] with x forward, y left


def to_cell(xy):
    return np.floor(xy / GRID_RES + N_CELLS / 2).astype(int)


def integrate(depth, T_wc, step=4, max_rays=5000, rng=np.random.default_rng(1)):
    v, u = np.mgrid[0:H:step, 0:W:step]
    z = depth[v, u]
    ok = ~np.isnan(z) & (z < MAP_RANGE)
    Pw = backproject(u[ok], v[ok], z[ok]) @ T_wc[:3, :3].T + T_wc[:3, 3]
    height = Pw[:, 2]
    tol = FLOOR_TOL + FLOOR_TOL_PER_M * np.maximum(z[ok] - 1.0, 0)   # stereo error grows with depth
    floor = np.abs(height) < tol
    obst = (height > tol + OBST_MARGIN) & (height < OBST_MAX)
    ends, is_obst = Pw[floor | obst, :2], obst[floor | obst]
    if len(ends) > max_rays:
        pick = rng.choice(len(ends), max_rays, replace=False)
        ends, is_obst = ends[pick], is_obst[pick]
    if len(ends) == 0:
        return
    cam = T_wc[:2, 3]
    d = np.linalg.norm(ends - cam, axis=1)
    t = np.linspace(0, 1, int(MAP_RANGE / (GRID_RES / 2)), endpoint=False)
    samples = cam + (ends - cam)[:, None, :] * t[None, :, None]          # rays x steps x 2
    before_end = (t[None, :] * d[:, None]) < (d[:, None] - GRID_RES)       # stop one cell short
    free = np.concatenate([samples[before_end], ends[~is_obst]])
    occ = ends[is_obst]

    def cells(xy):
        c = to_cell(xy)
        c = c[((c >= 0) & (c < N_CELLS)).all(1)]
        return np.unique(c, axis=0)

    occ_c, free_c = cells(occ), cells(free)
    if len(occ_c):
        occ_set = set(map(tuple, occ_c))
        free_c = np.array([c for c in map(tuple, free_c) if c not in occ_set]).reshape(-1, 2)
        logodds[occ_c[:, 0], occ_c[:, 1]] += L_OCC
    if len(free_c):
        logodds[free_c[:, 0], free_c[:, 1]] += L_FREE
    np.clip(logodds, -L_CLAMP, L_CLAMP, out=logodds)


def grid_image(traj_xy=None):
    """RGB image of the grid: unknown gray, free white, occupied black; forward (x) points up."""
    img = np.full((N_CELLS, N_CELLS, 3), 128, np.uint8)
    img[logodds < 2 * L_FREE + 1e-3] = 255                 # seen free in at least 2 keyframes
    img[logodds > 2 * L_OCC - 1e-3] = 0                    # seen occupied in at least 2 keyframes
    if traj_xy is not None and len(traj_xy):
        c = to_cell(np.asarray(traj_xy))
        c = c[((c >= 0) & (c < N_CELLS)).all(1)]
        img[c[:, 0], c[:, 1]] = (230, 60, 30)
    # rows = x (flip so forward is up), cols = y (flip so left is left)
    return img[::-1, ::-1].copy()


# %% [markdown]
# ## Raw map point cloud (keyframes, voxel-averaged, colored)
# Unfused: each voxel averages the points that land in it, so it is noisy at range. The TSDF mesh in
# pass 3 is the cleaner dense map; this cloud is kept to compare against it.

# %%
VOXEL = 0.02
cloud_sum = {}                                             # voxel key -> [sum xyz, sum rgb, count]


def add_cloud(depth, bgr, T_wc, step=3):
    v, u = np.mgrid[0:H:step, 0:W:step]
    z = depth[v, u]
    ok = ~np.isnan(z) & (z < MAP_RANGE)
    Pw = backproject(u[ok], v[ok], z[ok]) @ T_wc[:3, :3].T + T_wc[:3, 3]
    rgb = bgr[v[ok], u[ok]][:, ::-1].astype(np.float64)
    keys, inv = np.unique(np.floor(Pw / VOXEL).astype(np.int64), axis=0, return_inverse=True)
    inv = inv.ravel()
    sums_p = np.zeros((len(keys), 3)); np.add.at(sums_p, inv, Pw)
    sums_c = np.zeros((len(keys), 3)); np.add.at(sums_c, inv, rgb)
    counts = np.bincount(inv, minlength=len(keys))
    for key, sp, sc, n in zip(map(tuple, keys), sums_p, sums_c, counts):
        acc = cloud_sum.get(key)
        if acc is None:
            cloud_sum[key] = [sp, sc, n]
        else:
            acc[0] += sp; acc[1] += sc; acc[2] += n


def cloud_arrays():
    acc = list(cloud_sum.values())
    n = np.array([a[2] for a in acc], np.float64)[:, None]
    return np.array([a[0] for a in acc]) / n, (np.array([a[1] for a in acc]) / n).astype(np.uint8)


# %% [markdown]
# ## Rerun setup
# Tracking is logged under world/odom (first-camera frame); once the world frame is known, one static
# transform on world/odom puts it z-up, and the map (built per keyframe) is logged at each keyframe's
# frame index, so scrubbing the timeline shows the map growing.

# %%
rr.init("xlerobot_stereo_vo")
if VIEWER == "spawn":
    rr.spawn()
elif VIEWER == "notebook":
    rr.notebook_show()
if VIEWER == "save":
    rr.save(f"{OUT_DIR}/map.rrd")       # open later with: rerun map.rrd
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
rr.log("world/odom/camera/image", rr.Pinhole(focal_length=[fx, fy], principal_point=[cx, cy],
                                             width=W, height=H, camera_xyz=rr.ViewCoordinates.RDF), static=True)


def log_tracking(i, T_oc, bgr, depth, traj_o):
    rr.set_time("frame", sequence=i)
    rr.log("world/odom/camera", rr.Transform3D(translation=T_oc[:3, 3], mat3x3=T_oc[:3, :3]))
    rr.log("world/odom/camera/image/rgb", rr.Image(bgr, color_model="BGR").compress(jpeg_quality=70))
    rr.log("world/odom/trajectory", rr.LineStrips3D([np.array(traj_o)], colors=[230, 60, 30]))
    if depth is not None:
        rr.log("world/odom/camera/image/depth", rr.DepthImage(np.nan_to_num(depth), meter=1.0))


def log_map(i, traj_w, with_cloud=True):
    rr.set_time("frame", sequence=i)
    if with_cloud:                      # the whole cloud is re-logged, so not at every keyframe
        pts, cols = cloud_arrays()
        rr.log("world/map/cloud", rr.Points3D(pts, colors=cols, radii=VOXEL * 0.6))
    occ = np.argwhere(logodds > 2 * L_OCC - 1e-3)
    centers = np.column_stack([(occ + 0.5 - N_CELLS / 2) * GRID_RES, np.zeros(len(occ))])
    rr.log("world/map/occupied", rr.Points3D(centers, colors=[20, 20, 20], radii=GRID_RES / 2))
    rr.log("occupancy_grid", rr.Image(grid_image(traj_w[:, :2])))


# %% [markdown]
# ## Check that the input matches the calibration
# After rectification, a point must lie on the same row in both images, with positive disparity.
# If the frames are rotated or the halves swapped, this fails on every frame. It also fails while
# one lens's auto-exposure is still settling (washed-out image), so the map starts at the first
# frame that passes.

# %%
def stereo_check(gray_l, gray_r):
    pl, dl = detect(gray_l)
    pr, dr = detect(gray_r)
    m = match(dl, dr)
    if len(m) < 100:
        return False, f"only {len(m)} left/right matches"
    dy = pr[m[:, 1], 1] - pl[m[:, 0], 1]
    disp = pl[m[:, 0], 0] - pr[m[:, 1], 0]
    med_dy, med_disp = np.median(dy), np.median(disp[np.abs(dy) < 2]) if (np.abs(dy) < 2).any() else -1
    return abs(med_dy) < 1 and med_disp > 0, f"{len(m)} matches, median row offset {med_dy:.1f} px, median disparity {med_disp:.1f} px"


cap = cv2.VideoCapture(VIDEO)
fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
ok, frame = cap.read()
assert ok and frame.shape[:2] == (1080, 3840), f"expected 3840 x 1080 side-by-side frames, got {frame.shape}"
cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
print(f"{n_total} frames at {fps:.1f} fps")

# %% [markdown]
# ## Pass 1: tracking (visual odometry)
# Duplicate frames are skipped: if the camera delivers fewer frames than the recording rate, ffmpeg
# repeats frames, and a repeated frame carries no information.

# %%
def make_keyframe(i, bgr, gray_l, gray_r, pts, des, T_oc, depth=None):
    depth = frame_depth(i, gray_l, gray_r) if depth is None else depth
    z = depth[np.clip(pts[:, 1].round().astype(int), 0, H - 1), np.clip(pts[:, 0].round().astype(int), 0, W - 1)]
    return {"i": i, "pts": pts, "des": des, "P3": backproject(pts[:, 0], pts[:, 1], z), "T_oc": T_oc,
            "depth": depth.astype(np.float16), "bgr": bgr}


traj_o, R_oc, frame_ids, log_rows, keyframes, ref_kf = [], [], [], [], [], []
kf = None
T_kc_last = np.eye(4)
T_oc_last = np.eye(4)
last_small = None
n_dup = 0
t_start = time.perf_counter()
i = -1
while True:
    ok, frame = cap.read()
    i += 1
    if not ok or (MAX_FRAMES and i >= MAX_FRAMES):
        break
    if depth_meta is not None and i not in DEPTH_FRAMES:   # no precomputed depth for this frame
        continue
    small = cv2.resize(frame, (480, 135), interpolation=cv2.INTER_AREA).astype(np.int16)
    if last_small is not None and np.abs(small - last_small).mean() < 0.1:
        n_dup += 1
        continue
    last_small = small
    bgr, gl, gr = rectify(frame)
    pts, des = detect(gl)

    if kf is None:                                          # first good frame starts the odometry
        passed, info = stereo_check(gl, gr)
        print(f"frame {i}: stereo check {'passed' if passed else 'FAILED'} ({info})")
        if not passed:
            if i >= 30:
                raise RuntimeError("30 frames failed the left/right check: wrong rotation or swapped halves?")
            continue
        T_oc = np.eye(4)
        kf = make_keyframe(i, bgr, gl, gr, pts, des, T_oc)
        new_kf, n_inl, status = True, len(pts), "init"
    else:
        T_kc, n_inl = track(kf, pts, des, T_kc_last)
        status = "ok"
        if T_kc is not None:
            T_oc_new = kf["T_oc"] @ np.linalg.inv(T_kc)
            if np.linalg.norm(T_oc_new[:3, 3] - T_oc_last[:3, 3]) > MAX_STEP:
                T_kc, status = None, "jump"
        if T_kc is None:
            # Lost: keep the last pose and restart from a fresh keyframe here. The map stays usable,
            # but any motion during the lost frames is missing from the trajectory.
            status = status if status == "jump" else "lost"
            T_oc, new_kf = T_oc_last.copy(), True
        else:
            T_oc, T_kc_last = T_oc_new, T_kc
            new_kf = (n_inl < KF_MIN_INLIERS or np.linalg.norm(T_kc[:3, 3]) > KF_MAX_TRANSLATION
                      or rotation_deg(T_kc[:3, :3]) > KF_MAX_ROTATION)
        if new_kf:
            kf = make_keyframe(i, bgr, gl, gr, pts, des, T_oc)
            kf["after_lost"] = T_kc is None        # restarted after tracking loss: motion unknown
            T_kc_last = np.eye(4)

    if new_kf:
        keyframes.append(kf)
    ref_kf.append(len(keyframes) - 1)                     # keyframe this frame's pose is relative to
    T_oc_last = T_oc
    traj_o.append(T_oc[:3, 3].copy())
    R_oc.append(T_oc[:3, :3].copy())
    frame_ids.append(i)
    log_rows.append((i, status, n_inl, new_kf))
    log_tracking(i, T_oc, bgr, kf["depth"].astype(np.float32) if new_kf else None, traj_o)
    if status != "ok" or len(traj_o) % 20 == 1:
        print(f"frame {i:4d}/{n_total}  {status:5s}  inliers {n_inl:4d}  keyframes {len(keyframes):3d}  "
              f"pos {T_oc[:3, 3].round(3)}")

cap.release()
elapsed = time.perf_counter() - t_start
traj_o = np.array(traj_o)
n_lost = sum(s in ("lost", "jump") for _, s, _, _ in log_rows)
fps_used = fps * len(DEPTH_FRAMES) / max(n_total, 1) if depth_meta is not None else fps
print(f"\n{len(traj_o)} distinct frames ({n_dup} duplicates skipped, so ~{fps_used * len(traj_o) / max(len(traj_o) + n_dup, 1):.1f} "
      f"real fps among the frames used) in {elapsed:.1f} s; "
      f"{len(keyframes)} keyframes, {n_lost} lost/rejected frames")
if n_dup > len(traj_o):
    print("WARNING: most frames were duplicates. Tracking works best at >= 5 real fps; "
          "fix the camera mode or drive more slowly.")
print(f"path length {np.sum(np.linalg.norm(np.diff(traj_o, axis=0), axis=1)):.2f} m, "
      f"end - start {np.linalg.norm(traj_o[-1] - traj_o[0]):.3f} m")

# %% [markdown]
# ## Pass 1b: loop closure and pose-graph optimization (slambook Ch. 10-11)
# Odometry chains keyframe to keyframe, so small errors add up (drift). A loop closure is a second,
# independent measurement between two keyframes far apart in time that see the same place:
#   1. candidates: ORB-match each keyframe against all keyframes >= LOOP_MIN_GAP older. (ORB-SLAM uses a
#      bag-of-words index for this; with ~50 keyframes in a room, brute force is fast enough.)
#   2. geometric check: PnP-RANSAC of the new keyframe's features against the old keyframe's 3D points
#      (the same track() as in odometry). Enough inliers -> relative pose Z_ij between keyframes i and j.
#      It does NOT need the same pose: just enough overlap in view for ORB to match (viewpoint change
#      up to ~30-45 deg, scale up to ~2x).
#   3. pose graph: nodes = keyframe poses T_i (camera -> odom), edges = relative pose measurements Z_ij
#      (consecutive keyframes from odometry, plus the loops). Minimize over all T (T_0 fixed):
#          sum_edges || r_ij ||^2,   r_ij = ( t / sigma_t , rotvec / sigma_r ) of  E_ij = Z_ij^-1 T_i^-1 T_j
#      E_ij is the identity when the poses agree with the measurement. (The book uses the SE(3) log map for
#      r_ij; translation + rotation vector of E_ij is the common g2o-style simplification.) A Huber-type
#      robust loss limits the damage of a wrong loop.
#   4. every frame keeps its pose relative to its keyframe: T_frame' = T_kf' T_kf^-1 T_frame.

# %%
def rel(T_a, T_b):
    """Pose of b in a's frame: T_a^-1 T_b."""
    return np.linalg.inv(T_a) @ T_b


def find_loops(keyframes):
    loops = []
    for j in range(len(keyframes)):
        kj = keyframes[j]
        found = []
        for i in range(j - LOOP_MIN_GAP + 1):
            ki = keyframes[i]
            if len(match(ki["des"], kj["des"])) < LOOP_MIN_MATCHES:
                continue
            guess = rel(kj["T_oc"], ki["T_oc"])                  # i in j's frame, from odometry
            T_ji, n_inl = track(ki, kj["pts"], kj["des"], guess)  # same 3D-2D check as tracking
            if T_ji is None or n_inl < LOOP_MIN_INLIERS:
                continue
            Z_ij = np.linalg.inv(T_ji)                           # j in i's frame, measured
            D = rel(rel(ki["T_oc"], kj["T_oc"]), Z_ij)           # loop vs odometry disagreement
            jump_t, jump_r = np.linalg.norm(D[:3, 3]), rotation_deg(D[:3, :3])
            if jump_t > LOOP_MAX_JUMP[0] or jump_r > LOOP_MAX_JUMP[1]:
                continue
            found.append((n_inl, i, j, Z_ij, jump_t, jump_r))
        loops += sorted(found, key=lambda x: -x[0])[:LOOP_MAX_PER_KF]
    return loops


def optimize_pose_graph(poses, edges):
    """poses: list of 4x4 (camera -> odom); edges: (i, j, Z_ij). Returns optimized poses."""
    n = len(poses)
    x0 = np.concatenate([np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]] for T in poses[1:]])
    sig = np.r_[np.full(3, np.radians(PG_SIGMA_R)), np.full(3, PG_SIGMA_T)]
    Zinv = [np.linalg.inv(Z) for _, _, Z in edges]

    def unpack(x):
        Ts = [poses[0]]
        for k in range(n - 1):
            T = np.eye(4)
            T[:3, :3] = Rotation.from_rotvec(x[6 * k:6 * k + 3]).as_matrix()
            T[:3, 3] = x[6 * k + 3:6 * k + 6]
            Ts.append(T)
        return Ts

    def residuals(x):
        Ts = unpack(x)
        r = []
        for (i, j, _), Zi in zip(edges, Zinv):
            E = Zi @ np.linalg.inv(Ts[i]) @ Ts[j]
            r.append(np.r_[Rotation.from_matrix(E[:3, :3]).as_rotvec(), E[:3, 3]] / sig)
        return np.concatenate(r)

    sol = least_squares(residuals, x0, loss="huber", f_scale=3.0, method="trf")
    return unpack(sol.x), residuals(x0), residuals(sol.x)


if LOOP_CLOSURE and len(keyframes) > LOOP_MIN_GAP:
    t0 = time.perf_counter()
    loops = find_loops(keyframes)
    print(f"\nloop closure: {len(loops)} loops among {len(keyframes)} keyframes "
          f"({time.perf_counter() - t0:.1f} s)")
    for n_inl, i, j, _, jt, jr in loops[:15]:
        print(f"  keyframe {i:3d} (frame {keyframes[i]['i']:4d}) <-> {j:3d} (frame {keyframes[j]['i']:4d}): "
              f"{n_inl:4d} inliers, odometry off by {jt * 100:5.1f} cm, {jr:4.1f} deg")
    if loops:
        old = [k["T_oc"].copy() for k in keyframes]
        odo = [(a, a + 1, rel(old[a], old[a + 1])) for a in range(len(old) - 1)]
        # a restart after tracking loss measured no motion: make that edge very weak
        odo_w = [(a, b, Z) for a, b, Z in odo if not keyframes[b].get("after_lost")]
        edges = odo_w + [(i, j, Z) for _, i, j, Z, _, _ in loops]
        new, r0, r1 = optimize_pose_graph(old, edges)
        nl = 6 * len(loops)
        print(f"pose graph: {len(odo_w)} odometry + {len(loops)} loop edges; loop residual RMS "
              f"{np.sqrt(np.mean(r0[-nl:] ** 2)):.1f} -> {np.sqrt(np.mean(r1[-nl:] ** 2)):.1f} sigma, "
              f"odometry residual RMS {np.sqrt(np.mean(r1[:-nl] ** 2)):.2f} sigma")
        moves = [np.linalg.norm(a[:3, 3] - b[:3, 3]) for a, b in zip(old, new)]
        print(f"keyframes moved by up to {max(moves) * 100:.1f} cm (median {np.median(moves) * 100:.1f} cm)")
        # move every frame with its keyframe, keyframes onto their optimized poses
        delta = [Tn @ np.linalg.inv(To) for Tn, To in zip(new, old)]
        for f in range(len(traj_o)):
            T = np.eye(4); T[:3, :3] = R_oc[f]; T[:3, 3] = traj_o[f]
            T = delta[ref_kf[f]] @ T
            R_oc[f], traj_o[f] = T[:3, :3].copy(), T[:3, 3].copy()
        for k, T in zip(keyframes, new):
            k["T_oc"] = T
        print(f"after loop closure: end - start {np.linalg.norm(traj_o[-1] - traj_o[0]):.3f} m")
        rr.log("world/odom/trajectory_before_loops", rr.LineStrips3D([np.array([T[:3, 3] for T in old])],
                                                                      colors=[120, 120, 120]), static=True)
        rr.log("world/odom/loops", rr.LineStrips3D(
            [[new[i][:3, 3], new[j][:3, 3]] for _, i, j, _, _, _ in loops], colors=[40, 160, 255]), static=True)
        rr.log("world/odom/trajectory_optimized", rr.LineStrips3D([traj_o], colors=[230, 60, 30]), static=True)

# %% [markdown]
# ## Pass 2: world frame, occupancy grid and point cloud

# %%
T_wo, cam_h = estimate_world(traj_o, R_oc, keyframes)
rr.log("world/odom", rr.Transform3D(translation=T_wo[:3, 3], mat3x3=T_wo[:3, :3]), static=True)
traj = traj_o @ T_wo[:3, :3].T + T_wo[:3, 3]                  # world coordinates, z = camera height
traj[:, 2] = cam_h                                             # planar snap for the saved path
stamps = np.array(frame_ids) / fps
down_w = np.mean([(T_wo @ k["T_oc"])[:3, 1] for k in keyframes], 0)
down_w /= np.linalg.norm(down_w)                              # the mount's fixed "down" in world

for n, kf in enumerate(keyframes):
    T_wc = snap_planar(T_wo @ kf["T_oc"], cam_h, down_w)
    depth = kf["depth"].astype(np.float32)
    integrate(depth, T_wc)
    add_cloud(depth, kf["bgr"], T_wc)
    log_map(kf["i"], traj[: frame_ids.index(kf["i"]) + 1], with_cloud=n % 5 == 4 or n == len(keyframes) - 1)
print(f"raw cloud: {len(cloud_sum)} points ({VOXEL * 100:.0f} cm voxels, keyframes only)")

# %% [markdown]
# ## Pass 3: dense map by TSDF fusion (slambook Ch. 12)
# A TSDF (truncated signed distance field) stores, per voxel, the distance to the nearest surface
# along the camera ray: + in front, - behind, clipped to +-TSDF_TRUNC. Each frame updates every voxel
# it sees with a running weighted average:
#     D <- (W D + d) / (W + 1),   W <- W + 1
# so the stereo noise of one frame averages out over the many frames that see the same surface.
# The surface is the zero crossing of D, extracted with marching cubes as a triangle mesh.
#
# Every tracked frame is used (not only keyframes), with its pose from pass 1 snapped to the plane,
# so the video is read and SGBM is run a second time. Pixels at depth edges ("flying pixels" that
# interpolate between foreground and background) are removed first.

# %%
def clean_depth(depth):
    """Drop pixels far from TSDF_MAX_DEPTH and pixels on depth discontinuities."""
    d = depth.copy()
    d[~(d < TSDF_MAX_DEPTH)] = np.nan                       # also catches NaN
    filled = np.nan_to_num(d, nan=0)
    k = np.ones((5, 5), np.uint8)
    edge = (cv2.dilate(filled, k) - cv2.erode(np.where(filled > 0, filled, 99).astype(np.float32), k)) > 0.05 * filled
    d[edge] = np.nan
    return np.nan_to_num(d, nan=0).astype(np.float32)        # Open3D: 0 = no depth


# Open3D's tensor TSDF: voxels live in 16^3 blocks that are allocated only where surfaces are seen,
# so the volume can cover the whole room. (The older o3d.pipelines.integration.ScalableTSDFVolume
# returned an empty mesh with open3d 0.20 on macOS / Python 3.13.)
tsdf = o3d.t.geometry.VoxelBlockGrid(
    attr_names=("tsdf", "weight", "color"), attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
    attr_channels=((1), (1), (3)), voxel_size=TSDF_VOXEL, block_resolution=16, block_count=50000,
    device=o3c.Device("CPU:0"))
K_t = o3c.Tensor(K, o3c.float64)
pose_of = {frame_ids[k]: k for k in range(0, len(frame_ids), FUSE_EVERY)}   # video frame -> pose index

t_start = time.perf_counter()
cap = cv2.VideoCapture(VIDEO)
i = -1
n_fused = 0
while True:
    ok, frame = cap.read()
    i += 1
    if not ok or i > frame_ids[-1]:
        break
    k = pose_of.get(i)
    if k is None:
        continue
    T_oc = np.eye(4); T_oc[:3, :3] = R_oc[k]; T_oc[:3, 3] = traj_o[k]
    T_wc = snap_planar(T_wo @ T_oc, cam_h, down_w)
    bgr, gl, gr = rectify(frame)
    depth_img = o3d.t.geometry.Image(o3c.Tensor(clean_depth(frame_depth(i, gl, gr))))
    color_img = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(bgr[:, :, ::-1], np.float32) / 255))
    T_cw = o3c.Tensor(np.linalg.inv(T_wc), o3c.float64)     # Open3D wants camera-from-world
    blocks = tsdf.compute_unique_block_coordinates(depth_img, K_t, T_cw, 1.0, TSDF_MAX_DEPTH, TSDF_TRUNC / TSDF_VOXEL)
    tsdf.integrate(blocks, depth_img, color_img, K_t, T_cw, 1.0, TSDF_MAX_DEPTH, TSDF_TRUNC / TSDF_VOXEL)
    n_fused += 1
    if n_fused % 100 == 0:
        print(f"fused {n_fused}/{len(pose_of)} frames")
cap.release()

# keep only voxels seen in >= 3 frames: a surface must be confirmed, single-frame noise is dropped
mesh = tsdf.extract_triangle_mesh(weight_threshold=3.0).to_legacy()
# drop small disconnected blobs (isolated noise) from the mesh
tri_cluster, cluster_n_tri, _ = mesh.cluster_connected_triangles()
tri_cluster, cluster_n_tri = np.asarray(tri_cluster), np.asarray(cluster_n_tri)
mesh.remove_triangles_by_mask(cluster_n_tri[tri_cluster] < 200)
mesh.remove_unreferenced_vertices()
mesh.compute_vertex_normals()
dense = tsdf.extract_point_cloud(weight_threshold=3.0).to_legacy()
print(f"TSDF: fused {n_fused} frames in {time.perf_counter() - t_start:.1f} s -> mesh with "
      f"{len(mesh.triangles)} triangles, dense cloud with {len(dense.points)} points")

o3d.io.write_triangle_mesh(f"{OUT_DIR}/mesh.ply", mesh)
o3d.io.write_point_cloud(f"{OUT_DIR}/dense_cloud.ply", dense)
rr.set_time("frame", sequence=frame_ids[-1])
rr.log("world/map/mesh", rr.Mesh3D(
    vertex_positions=np.asarray(mesh.vertices), triangle_indices=np.asarray(mesh.triangles),
    vertex_normals=np.asarray(mesh.vertex_normals),
    vertex_colors=(np.asarray(mesh.vertex_colors) * 255).astype(np.uint8)))

# %% [markdown]
# ## Save results and plot the grid with the trajectory

# %%
np.save(f"{OUT_DIR}/occupancy_logodds.npy", logodds)
np.savetxt(f"{OUT_DIR}/trajectory_xyz.txt", np.column_stack([stamps, traj]), fmt="%.4f",
           header="time x y z  (world: x forward, y left, z up = camera height above floor, m)")
cv2.imwrite(f"{OUT_DIR}/occupancy.png", cv2.cvtColor(grid_image(traj[:, :2]), cv2.COLOR_RGB2BGR))

half = GRID_SIZE / 2
plt.figure(figsize=(7, 7))
plt.imshow(grid_image(), extent=[half, -half, -half, half])   # x forward = up, y left = left
plt.plot(traj[:, 1], traj[:, 0], color="tab:red", lw=2, label="camera path")
plt.plot(traj[0, 1], traj[0, 0], "go", label="start")
span = max(1.5, np.abs(traj[:, :2]).max() + MAP_RANGE)
plt.xlim(span, -span); plt.ylim(-span, span)
plt.xlabel("y, left (m)"); plt.ylabel("x, forward (m)")
plt.title("occupancy grid (white free, black occupied, gray unknown)")
plt.legend(); plt.show()
print("saved to", OUT_DIR)
