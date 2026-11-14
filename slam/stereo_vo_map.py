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
# Run:  `python slam/stereo_vo_map.py --video samples/slam/living_room_03.mp4 [--depth-dir ...] [--viewer save]`
#   (or run the cells in order in a notebook; edit Config below instead of passing flags)
#
# Code layout: every setting lives in `Config`; each stage is a function or small class that gets
# what it needs as arguments, and `main()` at the bottom wires the stages together in order.
#
# Pose convention (slambook): T_ab is a 4x4 rigid transform that maps a point's coordinates in frame b
# into frame a, X_a = T_ab X_b; equivalently, it is the pose of frame b expressed in frame a.
# Frames:  c = a camera (rectified left lens: x right, y down, z forward)
#          k = a keyframe's camera
#          o = "odom", the first tracked camera (tracking runs in this frame)
#          w = world: z up, z = 0 on the floor, origin under the first camera, x = first view direction

# %%
import argparse
import dataclasses
import json
import os
import time
from dataclasses import dataclass, field

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
# All tunable values in one place. Each stage below reads only the fields it needs (listed in its
# docstring), so it is clear where every number comes from.

# %%
REPO = "/Users/huiliu/Documents/build_xlerobot"


@dataclass
class Config:
    # input / output
    calib_path: str = f"{REPO}/samples/calibration/calibration_fisheye.json"
    video: str = f"{REPO}/samples/slam/living_room_01.mp4"   # side-by-side 3840 x 1080, upright
    # Optional precomputed disparities from stereo_depth_batch.py (e.g. RAFT-Stereo run on a GPU machine).
    # None = SGBM here. When set, ONLY the frames in that folder are used (e.g. every 3rd), for tracking,
    # the occupancy grid and the TSDF mesh; outputs go to <video>_slam_<folder name>.
    depth_dir: str | None = None      # e.g. f"{REPO}/samples/slam/output_raft_mb"
    out_dir: str | None = None        # None = derived from video and depth_dir, see output_dir()
    viewer: str = "spawn"             # "spawn" = Rerun desktop viewer, "save" = <out_dir>/map.rrd only,
                                      # "notebook" = inline viewer in Jupyter
    max_frames: int | None = None     # e.g. 100 for a quick test

    # stereo depth
    proc_size: tuple = (960, 540)     # rectified size per lens; half resolution keeps it ~10 fps
    min_depth: float = 0.25           # m
    max_depth: float = 4.0            # m; stereo depth beyond ~4 m is too noisy (baseline 6 cm)

    # tracking; a new keyframe starts when tracking against the old one gets weak or the camera moved enough
    kf_min_inliers: int = 120
    kf_max_translation: float = 0.20  # m
    kf_max_rotation: float = 15.0     # deg
    min_track_inliers: int = 30       # below this the frame counts as lost
    max_step: float = 0.30            # m per frame; larger jumps are rejected as tracking failures
    huber_k: float = 2.0              # pixels

    # loop closure + pose graph
    loop_closure: bool = True
    loop_min_gap: int = 8             # only compare keyframes at least this many keyframes apart
    loop_min_matches: int = 150       # ORB matches needed before trying the geometric check
    loop_min_inliers: int = 80        # PnP inliers needed to accept a loop
    loop_max_per_kf: int = 3          # keep the best few loops per keyframe
    loop_max_jump: tuple = (1.0, 30.0)  # m, deg: reject loops that disagree with odometry by more
    pg_sigma_t: float = 0.02          # m: uncertainty of one pose-graph edge (odometry and loops alike)
    pg_sigma_r: float = 1.0           # deg

    # world frame
    camera_height: float | None = 1.20  # m, lens above the floor (tape-measured; was 1.22 m before the
                                        # camera was tilted down, i.e. for living_room_01/02).
                                        # None = estimate from the bottom of the images (biased low by furniture)

    # occupancy grid
    grid_res: float = 0.05            # m per cell
    grid_size: float = 12.0           # m, square, centered on the start position
    floor_tol: float = 0.06           # |height| below this = floor = free (at 1 m; grows with distance)
    floor_tol_per_m: float = 0.08     # extra floor tolerance per meter of depth: stereo error grows with range
    obst_margin: float = 0.05         # obstacles start this far above the floor tolerance
    obst_max: float = 1.40            # m; points higher than this don't block the robot
    map_range: float = 3.5            # m, ignore farther depth for the grid and the raw cloud
    l_occ: float = 0.85               # log-odds update per keyframe for an occupied cell
    l_free: float = -0.4              # ... for a free cell
    l_clamp: float = 4.0              # log-odds stay within +-l_clamp

    # raw point cloud (keyframes only)
    cloud_voxel: float = 0.02         # m

    # dense map: TSDF fusion of every frame's depth
    tsdf_voxel: float = 0.02          # m; finer than ~1.5 cm is below what 6 cm stereo resolves
    tsdf_trunc: float = 0.08          # m; truncation distance, a few times the depth noise at ~1.5 m
    tsdf_max_depth: float = 2.5       # m; depth error is ~5 cm at 2 m and ~12 cm at 3 m, so drop the far part
    fuse_every: int = 1               # fuse every Nth tracked frame (1 = all)

    def output_dir(self):
        """Output folder: out_dir if set, else <video>_slam[_<depth_dir name>] next to the video."""
        if self.out_dir:
            return self.out_dir
        suffix = f"_{os.path.basename(self.depth_dir.rstrip('/'))}" if self.depth_dir else ""
        return os.path.splitext(self.video)[0] + "_slam" + suffix


def parse_args(argv=None):
    """Build a Config from the defaults above plus command-line overrides.

    Args:
        argv: list of argument strings; None = sys.argv. Unknown arguments are ignored, so this also
            works inside Jupyter (which passes its own `-f kernel.json`).
    Returns:
        Config.
    """
    p = argparse.ArgumentParser(description="Stereo VO + occupancy grid + TSDF mesh for the XLeRobot camera")
    p.add_argument("--video", help="side-by-side stereo .mp4 from record_stereo.py")
    p.add_argument("--calib", dest="calib_path", help="calibration JSON from calibrate_stereo.py --fisheye")
    p.add_argument("--depth-dir", help="folder of precomputed disparities from stereo_depth_batch.py")
    p.add_argument("--out-dir", help="output folder (default: <video>_slam[_<depth dir>])")
    p.add_argument("--viewer", choices=["spawn", "save", "notebook"])
    p.add_argument("--max-frames", type=int, help="stop after this many video frames")
    p.add_argument("--no-loop-closure", dest="loop_closure", action="store_const", const=False)
    args, _ = p.parse_known_args(argv)
    return dataclasses.replace(Config(), **{k: v for k, v in vars(args).items() if v is not None})


# %% [markdown]
# ## Pose helpers (SE(3) and SO(3))

# %%
def hat(v):
    """Skew-symmetric matrix of a 3-vector: hat(a) @ b == np.cross(a, b)."""
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def se3_exp(xi):
    """SE(3) exponential map.

    Args:
        xi: 6-vector (rho, phi): translation part rho, rotation vector phi (radians).
    Returns:
        4x4 transform.
    """
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


def rt_to_T(rvec, tvec):
    """OpenCV rotation vector + translation -> 4x4 transform."""
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
    return T


def rotation_deg(Rm):
    """Rotation angle (deg) of a 3x3 rotation matrix."""
    return np.degrees(np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1)))


def rel(T_a, T_b):
    """Relative pose T_a^-1 T_b: frame b expressed in frame a, when T_a and T_b share a parent frame."""
    return np.linalg.inv(T_a) @ T_b


def rot_a_to_b(a, b):
    """Smallest rotation matrix taking unit vector a to unit vector b."""
    v, c = np.cross(a, b), a @ b
    s2 = v @ v
    if s2 < 1e-16:
        return np.eye(3)
    Kx = hat(v)
    return np.eye(3) + Kx + Kx @ Kx * ((1 - c) / s2)


# %% [markdown]
# ## Calibration and rectification
# `newImageSize` makes the rectified pinhole camera directly at proc_size, so its fx, fy, cx, cy
# (from P1) are the half-resolution intrinsics. Everything after this works in the rectified left camera.

# %%
class StereoCamera:
    """The rectified stereo pair: rectification maps plus the pinhole model of the rectified left camera.

    Attributes:
        size: (W, H) of the rectified images.
        K: 3x3 intrinsics of the rectified left camera; fx, fy, cx, cy are its entries (pixels).
        baseline: distance between the rectified cameras (m), so depth = fx * baseline / disparity.
    """
    RAW_SIZE = (1920, 1080)               # one half of the side-by-side frame

    def __init__(self, calib_path, proc_size):
        """
        Args:
            calib_path: JSON from `calibrate_stereo.py --fisheye` (per-lens K, distortion; stereo R, T_mm).
            proc_size: (W, H) of the rectified images.
        """
        cal = json.load(open(calib_path))
        K1 = np.array(cal["left"]["camera_matrix"]);  D1 = np.array(cal["left"]["distortion"]).reshape(4, 1)
        K2 = np.array(cal["right"]["camera_matrix"]); D2 = np.array(cal["right"]["distortion"]).reshape(4, 1)
        R_lr = np.array(cal["stereo"]["R"])
        T_lr = np.array(cal["stereo"]["T_mm"]).reshape(3, 1) / 1000.0     # meters, X_right = R X_left + T
        # R1, R2: rotate each lens into the common rectified orientation; P1, P2: new pinhole projections.
        R1, R2, P1, P2, _ = cv2.fisheye.stereoRectify(
            K1, D1, K2, D2, self.RAW_SIZE, R_lr, T_lr, flags=cv2.CALIB_ZERO_DISPARITY,
            newImageSize=proc_size, balance=0.0, fov_scale=1.0)
        self.map_l = cv2.fisheye.initUndistortRectifyMap(K1, D1, R1, P1, proc_size, cv2.CV_16SC2)
        self.map_r = cv2.fisheye.initUndistortRectifyMap(K2, D2, R2, P2, proc_size, cv2.CV_16SC2)
        self.size = tuple(proc_size)
        self.fx, self.fy, self.cx, self.cy = P1[0, 0], P1[1, 1], P1[0, 2], P1[1, 2]
        self.K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])
        self.baseline = -P2[0, 3] / P2[0, 0]
        W = self.size[0]
        print(f"rectified camera: fx {self.fx:.1f} fy {self.fy:.1f} cx {self.cx:.1f} cy {self.cy:.1f}, "
              f"baseline {self.baseline * 1000:.1f} mm, horizontal FOV {np.degrees(2 * np.arctan(W / 2 / self.fx)):.0f} deg")

    def rectify(self, frame_bgr):
        """Split a side-by-side frame and rectify both halves.

        Args:
            frame_bgr: 1080 x 3840 x 3 uint8 frame; left lens in columns 0-1919, right lens in 1920-3839.
        Returns:
            (left BGR, left gray, right gray), each at self.size.
        """
        w = self.RAW_SIZE[0]
        left, right = frame_bgr[:, :w], frame_bgr[:, w:]
        left_r = cv2.remap(left, *self.map_l, cv2.INTER_LINEAR)
        right_r = cv2.remap(right, *self.map_r, cv2.INTER_LINEAR)
        return left_r, cv2.cvtColor(left_r, cv2.COLOR_BGR2GRAY), cv2.cvtColor(right_r, cv2.COLOR_BGR2GRAY)

    def backproject(self, u, v, z):
        """Pixels + depth -> 3D points in the rectified left camera frame (x right, y down, z forward).

        Args:
            u, v: pixel coordinates, arrays of length N.
            z: depth along the optical axis (m), length N.
        Returns:
            N x 3 array (m).
        """
        return np.column_stack([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z])

    def project(self, T_cp, P):
        """Transform points into the camera and project them to pixels.

        Args:
            T_cp: 4x4 transform from the points' frame p into this camera.
            P: N x 3 points in frame p.
        Returns:
            (N x 2 pixels, N x 3 points in camera coordinates).
        """
        pc = P @ T_cp[:3, :3].T + T_cp[:3, 3]
        uv = np.column_stack([self.fx * pc[:, 0] / pc[:, 2] + self.cx, self.fy * pc[:, 1] / pc[:, 2] + self.cy])
        return uv, pc


# %% [markdown]
# ## Stereo depth (Z = f b / d)
# Either SGBM on the rectified pair, or disparities precomputed by stereo_depth_batch.py
# (same rectification, possibly at 2x this size).

# %%
class DepthSource:
    """Metric depth for the rectified left image of a video frame.

    Attributes:
        frames: set of video frame indices that have depth, or None = every frame (SGBM).
        meta: meta.json of the precomputed folder, or None for SGBM.
    """

    def __init__(self, cam, min_depth, max_depth, depth_dir=None):
        """
        Args:
            cam: StereoCamera (gives fx, baseline and the image size).
            min_depth, max_depth: depth outside this range (m) is set to NaN.
            depth_dir: folder from stereo_depth_batch.py (meta.json + disp_XXXXXX.npz), or None = SGBM.
        """
        self.cam, self.min_depth, self.max_depth, self.depth_dir = cam, min_depth, max_depth, depth_dir
        self.meta, self.frames = None, None
        if depth_dir:
            self.meta = json.load(open(f"{depth_dir}/meta.json"))
            self.scale = self.meta["size"][0] / cam.size[0]
            assert self.scale in (1, 2) and abs(self.meta["fx"] / self.scale - cam.fx) < 0.5 \
                and abs(self.meta["baseline_m"] - cam.baseline) < 1e-4, \
                f"{depth_dir} was made with a different calibration or size: {self.meta}"
            self.frames = set(self.meta["frames"])
            print(f"depth from {depth_dir} ({self.meta['method']}, {len(self.frames)} frames)")
        else:
            block = 5
            self.sgbm = cv2.StereoSGBM_create(
                minDisparity=0, numDisparities=96, blockSize=block,
                P1=8 * block**2, P2=32 * block**2, disp12MaxDiff=1, preFilterCap=63,
                uniquenessRatio=10, speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def has(self, i):
        """True if video frame i has depth."""
        return self.frames is None or i in self.frames

    def depth(self, i, gray_l, gray_r):
        """Depth image for video frame i.

        Args:
            i: video frame index (used to find the precomputed file).
            gray_l, gray_r: rectified gray images (used by SGBM).
        Returns:
            H x W float32 depth (m) of the rectified left camera, NaN = no depth.
        """
        fx, b = self.cam.fx, self.cam.baseline
        if self.depth_dir is None:
            disp = self.sgbm.compute(gray_l, gray_r).astype(np.float32) / 16.0   # SGBM: fixed point x16
            depth = np.full(disp.shape, np.nan, np.float32)
            ok = disp > 0.5
            depth[ok] = fx * b / disp[ok]
            depth[(depth < self.min_depth) | (depth > self.max_depth)] = np.nan
            return depth
        disp = np.load(f"{self.depth_dir}/disp_{i:06d}.npz")["disp"].astype(np.float32)
        if self.scale != 1:           # full-size disparity -> this size: pixels and disparity both shrink
            disp = cv2.resize(disp, self.cam.size, interpolation=cv2.INTER_NEAREST) / self.scale
        with np.errstate(divide="ignore", invalid="ignore"):
            depth = fx * b / disp
        depth[~((depth >= self.min_depth) & (depth <= self.max_depth))] = np.nan
        return depth.astype(np.float32)


# %% [markdown]
# ## Features
# Cross-checked brute-force Hamming matching: a match is kept only if each point is the other's
# best match. It replaces the book's "2 x min distance" rule, which depends on the single best match.

# %%
class OrbFeatures:
    """ORB detector + cross-checked brute-force matcher."""

    def __init__(self, n_features=2000):
        self.orb = cv2.ORB_create(nfeatures=n_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    def detect(self, gray):
        """Args: gray image. Returns: (N x 2 float64 pixel positions, N x 32 uint8 descriptors or None)."""
        kps, des = self.orb.detectAndCompute(gray, None)
        return np.float64([k.pt for k in kps]).reshape(-1, 2), des

    def match(self, des_a, des_b, max_dist=50):
        """Match two descriptor sets.

        Args:
            des_a, des_b: descriptor arrays from detect() (either may be None).
            max_dist: largest Hamming distance kept.
        Returns:
            M x 2 int array of (index into a, index into b).
        """
        if des_a is None or des_b is None:
            return np.zeros((0, 2), int)
        ms = [m for m in self.matcher.match(des_a, des_b) if m.distance < max_dist]
        return np.array([(m.queryIdx, m.trainIdx) for m in ms], int).reshape(-1, 2)


def stereo_check(gray_l, gray_r, features):
    """Check that a rectified pair matches the calibration: matched points on the same row, positive disparity.

    If the frames are rotated or the halves swapped, this fails on every frame. It also fails while one
    lens's auto-exposure is still settling (washed-out image), so the map starts at the first frame that passes.

    Args:
        gray_l, gray_r: rectified gray images.
        features: OrbFeatures.
    Returns:
        (passed, human-readable details).
    """
    pl, dl = features.detect(gray_l)
    pr, dr = features.detect(gray_r)
    m = features.match(dl, dr)
    if len(m) < 100:
        return False, f"only {len(m)} left/right matches"
    dy = pr[m[:, 1], 1] - pl[m[:, 0], 1]
    disp = pl[m[:, 0], 0] - pr[m[:, 1], 0]
    med_dy, med_disp = np.median(dy), np.median(disp[np.abs(dy) < 2]) if (np.abs(dy) < 2).any() else -1
    return abs(med_dy) < 1 and med_disp > 0, f"{len(m)} matches, median row offset {med_dy:.1f} px, median disparity {med_disp:.1f} px"


# %% [markdown]
# ## Keyframes and tracking one frame against a keyframe (3D-2D: PnP + Huber BA)
# A keyframe keeps its ORB features with their 3D positions (from its depth). A later frame is tracked
# by matching its features to the keyframe's and solving PnP, which gives T_ck: keyframe -> current camera.

# %%
@dataclass
class Keyframe:
    i: int                  # video frame index
    pts: np.ndarray         # N x 2 ORB keypoint pixels in the rectified left image
    des: np.ndarray         # N x 32 ORB descriptors
    P3: np.ndarray          # N x 3 keypoints in this keyframe's camera frame (m); NaN where no depth
    T_oc: np.ndarray        # 4x4 pose: this camera -> odom
    depth: np.ndarray       # H x W float16 depth (m), NaN = none
    bgr: np.ndarray         # rectified left color image
    after_lost: bool = False  # started right after a tracking loss: motion from the previous keyframe unknown


def make_keyframe(i, bgr, pts, des, T_oc, depth, cam, after_lost=False):
    """Create a Keyframe, looking up each keypoint's depth to get its 3D position.

    Args:
        i: video frame index.
        bgr: rectified left color image.
        pts, des: ORB keypoints (N x 2) and descriptors of the rectified left image.
        T_oc: 4x4 pose of this camera in odom.
        depth: H x W float32 depth (m) of this frame.
        cam: StereoCamera.
        after_lost: True if tracking was lost just before this keyframe.
    """
    W, H = cam.size
    z = depth[np.clip(pts[:, 1].round().astype(int), 0, H - 1), np.clip(pts[:, 0].round().astype(int), 0, W - 1)]
    return Keyframe(i=i, pts=pts, des=des, P3=cam.backproject(pts[:, 0], pts[:, 1], z), T_oc=T_oc,
                    depth=depth.astype(np.float16), bgr=bgr, after_lost=after_lost)


def refine_pose_huber(T, P3, p2, K, k, iterations=10):
    """Gauss-Newton on SE(3) with Huber weights, minimizing reprojection error (same as pose_estimation_3d2d_ba.py).

    Args:
        T: 4x4 initial transform from the points' frame into the camera.
        P3: N x 3 points; p2: N x 2 observed pixels.
        K: 3x3 camera intrinsics.
        k: Huber threshold (pixels): residuals beyond it get weight k / r.
        iterations: maximum Gauss-Newton steps.
    Returns:
        Refined 4x4 transform.
    """
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    last = np.inf
    for _ in range(iterations):
        pc = P3 @ T[:3, :3].T + T[:3, 3]
        proj = np.column_stack([fx * pc[:, 0] / pc[:, 2] + cx, fy * pc[:, 1] / pc[:, 2] + cy])
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


class Tracker:
    """3D-2D tracking: a keyframe's 3D points vs another image's 2D features."""

    def __init__(self, cam, features, min_inliers, huber_k):
        """
        Args:
            cam: StereoCamera (intrinsics K).
            features: OrbFeatures (for matching).
            min_inliers: fewer PnP inliers than this = tracking failed.
            huber_k: Huber threshold (pixels) for the pose refinement.
        """
        self.cam, self.features, self.min_inliers, self.huber_k = cam, features, min_inliers, huber_k

    def track(self, kf, pts, des, T_guess):
        """Estimate the pose of an image relative to a keyframe.

        Args:
            kf: Keyframe (its descriptors and 3D points P3).
            pts, des: ORB keypoints (N x 2) and descriptors of the image to track.
            T_guess: 4x4 initial guess of T_ck (keyframe -> image camera), e.g. the previous frame's.
        Returns:
            (T_ck or None if tracking failed, number of PnP inliers).
        """
        m = self.features.match(kf.des, des)
        if len(m) == 0:
            return None, 0
        P3 = kf.P3[m[:, 0]]
        p2 = pts[m[:, 1]]
        ok = ~np.isnan(P3[:, 2])
        P3, p2 = P3[ok], p2[ok]
        if len(P3) < self.min_inliers:
            return None, len(P3)
        rvec0 = cv2.Rodrigues(T_guess[:3, :3])[0]
        tvec0 = T_guess[:3, 3].reshape(3, 1).copy()
        success, rvec, tvec, inl = cv2.solvePnPRansac(
            P3, p2, self.cam.K, None, rvec0, tvec0, useExtrinsicGuess=True,
            iterationsCount=200, reprojectionError=2.0, confidence=0.999)
        if not success or inl is None or len(inl) < self.min_inliers:
            return None, 0 if inl is None else len(inl)
        T = rt_to_T(rvec, tvec)
        # Refine with Huber BA on everything within 5 px of the RANSAC pose (inliers + near-inliers).
        proj, pc = self.cam.project(T, P3)
        near = (np.linalg.norm(p2 - proj, axis=1) < 5.0) & (pc[:, 2] > 0)
        T = refine_pose_huber(T, P3[near], p2[near], self.cam.K, self.huber_k)
        return T, len(inl)


# %% [markdown]
# ## Occupancy grid (log-odds)
# For each depth point in world coordinates (tol = floor tolerance, growing with depth):
#   - floor point (|z| < tol): its cell and the cells along the ray from the camera are free
#   - obstacle point (tol + obst_margin < z < obst_max): its cell is occupied, the cells before it are free
# Each keyframe adds l_occ / l_free once per touched cell, clamped, so repeated views firm it up and
# a moving person leaves only a faint trace.

# %%
class OccupancyGrid:
    """2D log-odds grid in the world frame, indexed [ix, iy] with x forward, y left, centered on the start."""

    def __init__(self, cfg):
        """
        Args:
            cfg: Config; uses grid_res, grid_size, floor_tol, floor_tol_per_m, obst_margin, obst_max,
                map_range, l_occ, l_free, l_clamp.
        """
        self.res, self.size = cfg.grid_res, cfg.grid_size
        self.floor_tol, self.floor_tol_per_m = cfg.floor_tol, cfg.floor_tol_per_m
        self.obst_margin, self.obst_max, self.map_range = cfg.obst_margin, cfg.obst_max, cfg.map_range
        self.l_occ, self.l_free, self.l_clamp = cfg.l_occ, cfg.l_free, cfg.l_clamp
        self.n = int(round(self.size / self.res))
        self.logodds = np.zeros((self.n, self.n), np.float32)
        self.rng = np.random.default_rng(1)                   # for subsampling rays

    def to_cell(self, xy):
        """Args: K x 2 world x, y (m). Returns: K x 2 int cell indices (may be outside the grid)."""
        return np.floor(xy / self.res + self.n / 2).astype(int)

    def integrate(self, depth, T_wc, cam, step=4, max_rays=5000):
        """Update the grid with one depth image.

        Args:
            depth: H x W depth (m), NaN = none.
            T_wc: 4x4 camera -> world pose (gravity-aligned, z = 0 on the floor).
            cam: StereoCamera (for back-projection).
            step: use every step-th pixel in each direction.
            max_rays: randomly keep at most this many rays.
        """
        W, H = cam.size
        v, u = np.mgrid[0:H:step, 0:W:step]
        z = depth[v, u]
        ok = ~np.isnan(z) & (z < self.map_range)
        Pw = cam.backproject(u[ok], v[ok], z[ok]) @ T_wc[:3, :3].T + T_wc[:3, 3]
        height = Pw[:, 2]
        tol = self.floor_tol + self.floor_tol_per_m * np.maximum(z[ok] - 1.0, 0)   # stereo error grows with depth
        floor = np.abs(height) < tol
        obst = (height > tol + self.obst_margin) & (height < self.obst_max)
        ends, is_obst = Pw[floor | obst, :2], obst[floor | obst]
        if len(ends) > max_rays:
            pick = self.rng.choice(len(ends), max_rays, replace=False)
            ends, is_obst = ends[pick], is_obst[pick]
        if len(ends) == 0:
            return
        cam_xy = T_wc[:2, 3]
        d = np.linalg.norm(ends - cam_xy, axis=1)
        t = np.linspace(0, 1, int(self.map_range / (self.res / 2)), endpoint=False)
        samples = cam_xy + (ends - cam_xy)[:, None, :] * t[None, :, None]       # rays x steps x 2
        before_end = (t[None, :] * d[:, None]) < (d[:, None] - self.res)        # stop one cell short
        free = np.concatenate([samples[before_end], ends[~is_obst]])
        occ = ends[is_obst]

        def cells(xy):
            c = self.to_cell(xy)
            c = c[((c >= 0) & (c < self.n)).all(1)]
            return np.unique(c, axis=0)

        occ_c, free_c = cells(occ), cells(free)
        if len(occ_c):
            occ_set = set(map(tuple, occ_c))
            free_c = np.array([c for c in map(tuple, free_c) if c not in occ_set]).reshape(-1, 2)
            self.logodds[occ_c[:, 0], occ_c[:, 1]] += self.l_occ
        if len(free_c):
            self.logodds[free_c[:, 0], free_c[:, 1]] += self.l_free
        np.clip(self.logodds, -self.l_clamp, self.l_clamp, out=self.logodds)

    def occupied_centers(self):
        """World x, y, z=0 of cells seen occupied in at least 2 keyframes, as K x 3."""
        occ = np.argwhere(self.logodds > 2 * self.l_occ - 1e-3)
        return np.column_stack([(occ + 0.5 - self.n / 2) * self.res, np.zeros(len(occ))])

    def image(self, traj_xy=None):
        """RGB image of the grid: unknown gray, free white, occupied black; forward (x) points up.

        Args:
            traj_xy: optional K x 2 world x, y path to draw in red.
        """
        img = np.full((self.n, self.n, 3), 128, np.uint8)
        img[self.logodds < 2 * self.l_free + 1e-3] = 255      # seen free in at least 2 keyframes
        img[self.logodds > 2 * self.l_occ - 1e-3] = 0         # seen occupied in at least 2 keyframes
        if traj_xy is not None and len(traj_xy):
            c = self.to_cell(np.asarray(traj_xy))
            c = c[((c >= 0) & (c < self.n)).all(1)]
            img[c[:, 0], c[:, 1]] = (230, 60, 30)
        # rows = x (flip so forward is up), cols = y (flip so left is left)
        return img[::-1, ::-1].copy()


# %% [markdown]
# ## Raw map point cloud (keyframes, voxel-averaged, colored)
# Unfused: each voxel averages the points that land in it, so it is noisy at range. The TSDF mesh in
# pass 3 is the cleaner dense map; this cloud is kept to compare against it.

# %%
class VoxelCloud:
    """Colored point cloud that averages all points falling in the same voxel."""

    def __init__(self, voxel, max_range):
        """
        Args:
            voxel: voxel edge length (m).
            max_range: ignore depth farther than this (m).
        """
        self.voxel, self.max_range = voxel, max_range
        self.sums = {}                                        # voxel key -> [sum xyz, sum rgb, count]

    def __len__(self):
        return len(self.sums)

    def add(self, depth, bgr, T_wc, cam, step=3):
        """Add one depth image.

        Args:
            depth: H x W depth (m), NaN = none.
            bgr: matching rectified left color image.
            T_wc: 4x4 camera -> world pose.
            cam: StereoCamera (for back-projection).
            step: use every step-th pixel in each direction.
        """
        W, H = cam.size
        v, u = np.mgrid[0:H:step, 0:W:step]
        z = depth[v, u]
        ok = ~np.isnan(z) & (z < self.max_range)
        Pw = cam.backproject(u[ok], v[ok], z[ok]) @ T_wc[:3, :3].T + T_wc[:3, 3]
        rgb = bgr[v[ok], u[ok]][:, ::-1].astype(np.float64)
        keys, inv = np.unique(np.floor(Pw / self.voxel).astype(np.int64), axis=0, return_inverse=True)
        inv = inv.ravel()
        sums_p = np.zeros((len(keys), 3)); np.add.at(sums_p, inv, Pw)
        sums_c = np.zeros((len(keys), 3)); np.add.at(sums_c, inv, rgb)
        counts = np.bincount(inv, minlength=len(keys))
        for key, sp, sc, n in zip(map(tuple, keys), sums_p, sums_c, counts):
            acc = self.sums.get(key)
            if acc is None:
                self.sums[key] = [sp, sc, n]
            else:
                acc[0] += sp; acc[1] += sc; acc[2] += n

    def arrays(self):
        """Returns: (K x 3 float points in world m, K x 3 uint8 RGB colors)."""
        acc = list(self.sums.values())
        n = np.array([a[2] for a in acc], np.float64)[:, None]
        return np.array([a[0] for a in acc]) / n, (np.array([a[1] for a in acc]) / n).astype(np.uint8)


# %% [markdown]
# ## Rerun logging
# Tracking is logged under world/odom (first-camera frame); once the world frame is known, one static
# transform on world/odom puts it z-up, and the map (built per keyframe) is logged at each keyframe's
# frame index, so scrubbing the timeline shows the map growing.

# %%
def init_rerun(viewer, out_dir, cam):
    """Start Rerun and log the static camera model.

    Args:
        viewer: "spawn" (desktop viewer), "save" (write <out_dir>/map.rrd) or "notebook".
        out_dir: output folder (for "save").
        cam: StereoCamera (pinhole model for the image view).
    """
    rr.init("xlerobot_stereo_vo")
    if viewer == "spawn":
        rr.spawn()
    elif viewer == "notebook":
        rr.notebook_show()
    elif viewer == "save":
        rr.save(f"{out_dir}/map.rrd")       # open later with: rerun map.rrd
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/odom/camera/image", rr.Pinhole(
        focal_length=[cam.fx, cam.fy], principal_point=[cam.cx, cam.cy],
        width=cam.size[0], height=cam.size[1], camera_xyz=rr.ViewCoordinates.RDF), static=True)


def log_tracking(i, T_oc, bgr, depth, traj_o):
    """Log one tracked frame.

    Args:
        i: video frame index (Rerun timeline "frame").
        T_oc: 4x4 camera -> odom pose.
        bgr: rectified left color image.
        depth: depth image to show, or None (only logged at keyframes).
        traj_o: list of camera positions in odom so far.
    """
    rr.set_time("frame", sequence=i)
    rr.log("world/odom/camera", rr.Transform3D(translation=T_oc[:3, 3], mat3x3=T_oc[:3, :3]))
    rr.log("world/odom/camera/image/rgb", rr.Image(bgr, color_model="BGR").compress(jpeg_quality=70))
    rr.log("world/odom/trajectory", rr.LineStrips3D([np.array(traj_o)], colors=[230, 60, 30]))
    if depth is not None:
        rr.log("world/odom/camera/image/depth", rr.DepthImage(np.nan_to_num(depth), meter=1.0))


def log_map(i, grid, cloud, traj_w, with_cloud=True):
    """Log the map state after a keyframe.

    Args:
        i: video frame index of the keyframe.
        grid: OccupancyGrid.
        cloud: VoxelCloud.
        traj_w: K x 3 world camera positions up to this frame.
        with_cloud: also re-log the whole point cloud (slow, so not at every keyframe).
    """
    rr.set_time("frame", sequence=i)
    if with_cloud:
        pts, cols = cloud.arrays()
        rr.log("world/map/cloud", rr.Points3D(pts, colors=cols, radii=cloud.voxel * 0.6))
    rr.log("world/map/occupied", rr.Points3D(grid.occupied_centers(), colors=[20, 20, 20], radii=grid.res / 2))
    rr.log("occupancy_grid", rr.Image(grid.image(traj_w[:, :2])))


def log_mesh(i, mesh):
    """Log an Open3D legacy TriangleMesh at video frame i."""
    rr.set_time("frame", sequence=i)
    rr.log("world/map/mesh", rr.Mesh3D(
        vertex_positions=np.asarray(mesh.vertices), triangle_indices=np.asarray(mesh.triangles),
        vertex_normals=np.asarray(mesh.vertex_normals),
        vertex_colors=(np.asarray(mesh.vertex_colors) * 255).astype(np.uint8)))


# %% [markdown]
# ## Pass 1: tracking (visual odometry)
# Duplicate frames are skipped: if the camera delivers fewer frames than the recording rate, ffmpeg
# repeats frames, and a repeated frame carries no information.

# %%
@dataclass
class Trajectory:
    """Result of tracking: one pose per used video frame, plus the keyframes."""
    poses: np.ndarray                 # N x 4 x 4, T_oc (camera -> odom) per used frame
    frame_ids: list                   # N video frame indices
    ref_kf: list                      # N indices into keyframes: the keyframe each pose was tracked against
    keyframes: list                   # Keyframe objects
    fps: float                        # video frame rate (for time stamps)
    statuses: list = field(default_factory=list)   # N of "init" / "ok" / "lost" / "jump"

    @property
    def positions(self):
        """N x 3 camera positions in odom."""
        return self.poses[:, :3, 3]


def open_video(path):
    """Open a side-by-side stereo video and check its frame size.

    Returns:
        (cv2.VideoCapture positioned at frame 0, fps, frame count).
    """
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    ok, frame = cap.read()
    assert ok and frame.shape[:2] == (1080, 3840), f"expected 3840 x 1080 side-by-side frames, got {frame.shape}"
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    print(f"{n_total} frames at {fps:.1f} fps")
    return cap, fps, n_total


def run_tracking(cfg, cam, depth_src, features, tracker):
    """Track every usable video frame against the current keyframe.

    Args:
        cfg: Config; uses video, max_frames, max_step, kf_min_inliers, kf_max_translation, kf_max_rotation.
        cam: StereoCamera.
        depth_src: DepthSource.
        features: OrbFeatures.
        tracker: Tracker.
    Returns:
        Trajectory in the odom frame (= the first tracked camera).
    """
    cap, fps, n_total = open_video(cfg.video)
    poses, positions, frame_ids, ref_kf, keyframes, statuses = [], [], [], [], [], []
    kf = None                         # current keyframe
    T_ck_last = np.eye(4)             # last tracked pose relative to kf: the guess for the next frame
    T_oc_last = np.eye(4)
    last_small = None
    n_dup = 0
    t_start = time.perf_counter()
    i = -1
    while True:
        ok, frame = cap.read()
        i += 1
        if not ok or (cfg.max_frames and i >= cfg.max_frames):
            break
        if not depth_src.has(i):                                # no precomputed depth for this frame
            continue
        small = cv2.resize(frame, (480, 135), interpolation=cv2.INTER_AREA).astype(np.int16)
        if last_small is not None and np.abs(small - last_small).mean() < 0.1:
            n_dup += 1
            continue
        last_small = small
        bgr, gl, gr = cam.rectify(frame)      # rectified left color, left gray, right gray
        pts, des = features.detect(gl)        # ORB keypoints and descriptors of the left image

        if kf is None:                                          # first good frame starts the odometry
            passed, info = stereo_check(gl, gr, features)
            print(f"frame {i}: stereo check {'passed' if passed else 'FAILED'} ({info})")
            if not passed:
                if i >= 30:
                    raise RuntimeError("30 frames failed the left/right check: wrong rotation or swapped halves?")
                continue
            T_oc = np.eye(4)
            kf = make_keyframe(i, bgr, pts, des, T_oc, depth_src.depth(i, gl, gr), cam)
            new_kf, n_inl, status = True, len(pts), "init"
        else:
            T_ck, n_inl = tracker.track(kf, pts, des, T_ck_last)
            status = "ok"
            if T_ck is not None:
                T_oc_new = kf.T_oc @ np.linalg.inv(T_ck)          # T_oc = T_ok T_kc
                if np.linalg.norm(T_oc_new[:3, 3] - T_oc_last[:3, 3]) > cfg.max_step:
                    T_ck, status = None, "jump"
            if T_ck is None:
                # Lost: keep the last pose and restart from a fresh keyframe here. The map stays usable,
                # but any motion during the lost frames is missing from the trajectory.
                status = status if status == "jump" else "lost"
                T_oc, new_kf = T_oc_last.copy(), True
            else:
                T_oc, T_ck_last = T_oc_new, T_ck
                new_kf = (n_inl < cfg.kf_min_inliers or np.linalg.norm(T_ck[:3, 3]) > cfg.kf_max_translation
                          or rotation_deg(T_ck[:3, :3]) > cfg.kf_max_rotation)
            if new_kf:
                kf = make_keyframe(i, bgr, pts, des, T_oc, depth_src.depth(i, gl, gr), cam,
                                   after_lost=T_ck is None)
                T_ck_last = np.eye(4)

        if new_kf:
            keyframes.append(kf)
        ref_kf.append(len(keyframes) - 1)
        T_oc_last = T_oc
        poses.append(T_oc.copy())
        positions.append(T_oc[:3, 3].copy())
        frame_ids.append(i)
        statuses.append(status)
        log_tracking(i, T_oc, bgr, kf.depth.astype(np.float32) if new_kf else None, positions)
        if status != "ok" or len(poses) % 20 == 1:
            print(f"frame {i:4d}/{n_total}  {status:5s}  inliers {n_inl:4d}  keyframes {len(keyframes):3d}  "
                  f"pos {T_oc[:3, 3].round(3)}")
    cap.release()

    traj = Trajectory(np.array(poses), frame_ids, ref_kf, keyframes, fps, statuses)
    elapsed = time.perf_counter() - t_start
    p = traj.positions
    n_lost = sum(s in ("lost", "jump") for s in statuses)
    fps_used = fps * len(depth_src.frames) / max(n_total, 1) if depth_src.frames is not None else fps
    print(f"\n{len(p)} distinct frames ({n_dup} duplicates skipped, so ~{fps_used * len(p) / max(len(p) + n_dup, 1):.1f} "
          f"real fps among the frames used) in {elapsed:.1f} s; "
          f"{len(keyframes)} keyframes, {n_lost} lost/rejected frames")
    if n_dup > len(p):
        print("WARNING: most frames were duplicates. Tracking works best at >= 5 real fps; "
              "fix the camera mode or drive more slowly.")
    print(f"path length {np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)):.2f} m, "
          f"end - start {np.linalg.norm(p[-1] - p[0]):.3f} m")
    return traj


# %% [markdown]
# ## Pass 1b: loop closure and pose-graph optimization (slambook Ch. 10-11)
# Odometry chains keyframe to keyframe, so small errors add up (drift). A loop closure is a second,
# independent measurement between two keyframes far apart in time that see the same place:
#   1. candidates: ORB-match each keyframe against all keyframes >= loop_min_gap older. (ORB-SLAM uses a
#      bag-of-words index for this; with ~50 keyframes in a room, brute force is fast enough.)
#   2. geometric check: PnP-RANSAC of the new keyframe's features against the old keyframe's 3D points
#      (the same Tracker.track() as in odometry). Enough inliers -> relative pose Z_ij between keyframes i and j.
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
@dataclass
class Loop:
    n_inliers: int
    i: int                  # older keyframe index
    j: int                  # newer keyframe index
    Z_ij: np.ndarray        # measured 4x4 pose of keyframe j in keyframe i's frame
    jump_t: float           # disagreement with odometry (m)
    jump_r: float           # ... (deg)


def find_loops(keyframes, tracker, features, cfg):
    """Find pairs of keyframes, far apart in time, that see the same place.

    Args:
        keyframes: list of Keyframe (with odometry poses T_oc).
        tracker: Tracker (geometric check).
        features: OrbFeatures (candidate matching).
        cfg: Config; uses loop_min_gap, loop_min_matches, loop_min_inliers, loop_max_jump, loop_max_per_kf.
    Returns:
        list of Loop.
    """
    loops = []
    for j in range(len(keyframes)):
        kj = keyframes[j]
        found = []
        for i in range(j - cfg.loop_min_gap + 1):
            ki = keyframes[i]
            if len(features.match(ki.des, kj.des)) < cfg.loop_min_matches:
                continue
            guess = rel(kj.T_oc, ki.T_oc)                       # T_ji from odometry
            T_ji, n_inl = tracker.track(ki, kj.pts, kj.des, guess)   # same 3D-2D check as tracking
            if T_ji is None or n_inl < cfg.loop_min_inliers:
                continue
            Z_ij = np.linalg.inv(T_ji)                          # j in i's frame, measured
            D = rel(rel(ki.T_oc, kj.T_oc), Z_ij)                # loop vs odometry disagreement
            jump_t, jump_r = np.linalg.norm(D[:3, 3]), rotation_deg(D[:3, :3])
            if jump_t > cfg.loop_max_jump[0] or jump_r > cfg.loop_max_jump[1]:
                continue
            found.append(Loop(n_inl, i, j, Z_ij, jump_t, jump_r))
        loops += sorted(found, key=lambda lp: -lp.n_inliers)[:cfg.loop_max_per_kf]
    return loops


def optimize_pose_graph(poses, edges, sigma_t, sigma_r):
    """Pose-graph optimization with a Huber loss; the first pose stays fixed.

    Args:
        poses: list of 4x4 initial poses (camera -> odom), one per node.
        edges: list of (i, j, Z_ij), Z_ij = measured pose of node j in node i's frame.
        sigma_t: translation uncertainty of one edge (m).
        sigma_r: rotation uncertainty of one edge (deg).
    Returns:
        (optimized poses, residuals before, residuals after); residuals are in units of sigma,
        6 per edge in the order of `edges`.
    """
    n = len(poses)
    x0 = np.concatenate([np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]] for T in poses[1:]])
    sig = np.r_[np.full(3, np.radians(sigma_r)), np.full(3, sigma_t)]
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


def close_loops(traj, tracker, features, cfg):
    """Detect loops and correct the trajectory with a pose graph, in place.

    Args:
        traj: Trajectory; its poses and its keyframes' T_oc are replaced by the optimized ones.
        tracker, features: as for find_loops().
        cfg: Config; uses loop_* and pg_sigma_t, pg_sigma_r.
    Returns:
        list of Loop (empty if none were found).
    """
    keyframes = traj.keyframes
    t0 = time.perf_counter()
    loops = find_loops(keyframes, tracker, features, cfg)
    print(f"\nloop closure: {len(loops)} loops among {len(keyframes)} keyframes ({time.perf_counter() - t0:.1f} s)")
    for lp in loops[:15]:
        print(f"  keyframe {lp.i:3d} (frame {keyframes[lp.i].i:4d}) <-> {lp.j:3d} (frame {keyframes[lp.j].i:4d}): "
              f"{lp.n_inliers:4d} inliers, odometry off by {lp.jump_t * 100:5.1f} cm, {lp.jump_r:4.1f} deg")
    if not loops:
        return loops

    old = [k.T_oc.copy() for k in keyframes]
    odo = [(a, a + 1, rel(old[a], old[a + 1])) for a in range(len(old) - 1)]
    # a restart after tracking loss measured no motion: leave that edge out
    odo = [(a, b, Z) for a, b, Z in odo if not keyframes[b].after_lost]
    edges = odo + [(lp.i, lp.j, lp.Z_ij) for lp in loops]
    new, r0, r1 = optimize_pose_graph(old, edges, cfg.pg_sigma_t, cfg.pg_sigma_r)
    nl = 6 * len(loops)
    print(f"pose graph: {len(odo)} odometry + {len(loops)} loop edges; loop residual RMS "
          f"{np.sqrt(np.mean(r0[-nl:] ** 2)):.1f} -> {np.sqrt(np.mean(r1[-nl:] ** 2)):.1f} sigma, "
          f"odometry residual RMS {np.sqrt(np.mean(r1[:-nl] ** 2)):.2f} sigma")
    moves = [np.linalg.norm(a[:3, 3] - b[:3, 3]) for a, b in zip(old, new)]
    print(f"keyframes moved by up to {max(moves) * 100:.1f} cm (median {np.median(moves) * 100:.1f} cm)")

    # move every frame with its keyframe, keyframes onto their optimized poses
    delta = [Tn @ np.linalg.inv(To) for Tn, To in zip(new, old)]
    for f in range(len(traj.poses)):
        traj.poses[f] = delta[traj.ref_kf[f]] @ traj.poses[f]
    for k, T in zip(keyframes, new):
        k.T_oc = T
    p = traj.positions
    print(f"after loop closure: end - start {np.linalg.norm(p[-1] - p[0]):.3f} m")
    rr.log("world/odom/trajectory_before_loops", rr.LineStrips3D([np.array([T[:3, 3] for T in old])],
                                                                  colors=[120, 120, 120]), static=True)
    rr.log("world/odom/loops", rr.LineStrips3D(
        [[new[lp.i][:3, 3], new[lp.j][:3, 3]] for lp in loops], colors=[40, 160, 255]), static=True)
    rr.log("world/odom/trajectory_optimized", rr.LineStrips3D([p], colors=[230, 60, 30]), static=True)
    return loops


# %% [markdown]
# ## World frame: "up" and the floor, estimated after tracking
# Tracking runs in the "odom" frame (= the first camera). Afterwards we find a gravity-aligned world:
#   - up: the robot drives on a flat floor, so the camera path lies in a plane parallel to the floor;
#     its normal (smallest PCA direction of the path) is "up". Needs the path to span some area
#     (side steps or a turn); otherwise fall back to the average camera orientation.
#   - camera height: cfg.camera_height if set, else the median height of points in the bottom rows of
#     the keyframes (mostly floor), taken relative to each keyframe's own camera so drift cancels.
#   - planar snap: on a flat floor with a rigid mount every pose has the same height, pitch and roll;
#     only x, y and heading change. Snapping poses to that removes the VO's tilt/height drift,
#     which would otherwise lift far-away floor into the obstacle band (1 deg tilt = 5 cm at 3 m).

# %%
@dataclass
class World:
    T_wo: np.ndarray        # 4x4 odom -> world (z up, z = 0 on the floor, x = first view direction)
    cam_h: float            # camera height above the floor (m)
    down_w: np.ndarray      # the camera mount's fixed "down" axis (camera +y) in world coordinates

    def T_wc(self, T_oc):
        """Camera -> world pose of an odom pose, snapped to the plane: keeps x, y and heading, sets the
        height to cam_h and the camera's down axis to down_w."""
        T = self.T_wo @ T_oc
        T[:3, :3] = rot_a_to_b(T[:3, 1], self.down_w) @ T[:3, :3]
        T[2, 3] = self.cam_h
        return T

    def positions(self, traj):
        """N x 3 world camera positions of a Trajectory, z set to the camera height."""
        p = traj.positions @ self.T_wo[:3, :3].T + self.T_wo[:3, 3]
        p[:, 2] = self.cam_h
        return p


def estimate_world(traj, cam, camera_height):
    """Find the gravity-aligned world frame from the tracked trajectory.

    Args:
        traj: Trajectory (poses in odom, keyframes with depth).
        cam: StereoCamera (image size, back-projection; used when camera_height is None).
        camera_height: lens height above the floor (m), or None to estimate it from the keyframes' depth.
    Returns:
        World.
    """
    traj_o, R_oc = traj.positions, traj.poses[:, :3, :3]
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

    if camera_height is not None:
        cam_h, how_h = camera_height, "set"
    else:
        W, H = cam.size
        below = []
        for kf in traj.keyframes:                              # bottom quarter of each keyframe
            v, u = np.mgrid[int(0.75 * H):H:4, 0:W:4]
            z = kf.depth[v, u].astype(np.float32)
            ok = ~np.isnan(z)
            h = (cam.backproject(u[ok], v[ok], z[ok]) @ kf.T_oc[:3, :3].T) @ up   # relative to that camera
            below.append(h[h < -0.3])
        below = np.concatenate(below)
        if len(below) > 1000:
            cam_h, how_h = -np.median(below), f"median of {len(below)} bottom-of-image points"
        else:
            cam_h, how_h = 1.0, "NOT FOUND, guessed"
    floor_h = np.mean(traj_o @ up) - cam_h
    fwd = np.array([0.0, 0.0, 1.0]) - up[2] * up               # first camera's view direction, leveled
    fwd /= np.linalg.norm(fwd)
    T_wo = np.eye(4)
    T_wo[:3, :3] = np.vstack([fwd, np.cross(up, fwd), up])     # rows = world x, y, z in odom coords
    T_wo[:3, 3] = [0, 0, -floor_h]
    down = np.mean([R[:, 1] for R in R_oc], 0)
    tilt = np.degrees(np.arccos(np.clip(-down @ up / np.linalg.norm(down), -1, 1)))
    print(f"up from {how}\ncamera height {cam_h:.3f} m ({how_h}); camera mount tilted {tilt:.1f} deg from level; "
          f"VO height wobble +-{np.std(traj_o @ up) * 100:.1f} cm (removed by the planar snap)")
    down_w = np.mean([(T_wo @ k.T_oc)[:3, 1] for k in traj.keyframes], 0)
    down_w /= np.linalg.norm(down_w)
    return World(T_wo, cam_h, down_w)


# %% [markdown]
# ## Pass 2: occupancy grid and raw point cloud from the keyframes

# %%
def build_keyframe_maps(traj, world, cam, cfg):
    """Integrate every keyframe's depth into the occupancy grid and the raw cloud, logging as it goes.

    Args:
        traj: Trajectory (keyframes with depth, color and pose).
        world: World (odom -> world, planar snap).
        cam: StereoCamera.
        cfg: Config; passed to OccupancyGrid, plus cloud_voxel and map_range for the cloud.
    Returns:
        (OccupancyGrid, VoxelCloud).
    """
    grid = OccupancyGrid(cfg)
    cloud = VoxelCloud(cfg.cloud_voxel, cfg.map_range)
    traj_w = world.positions(traj)
    keyframes = traj.keyframes
    for n, kf in enumerate(keyframes):
        T_wc = world.T_wc(kf.T_oc)
        depth = kf.depth.astype(np.float32)
        grid.integrate(depth, T_wc, cam)
        cloud.add(depth, kf.bgr, T_wc, cam)
        log_map(kf.i, grid, cloud, traj_w[: traj.frame_ids.index(kf.i) + 1],
                with_cloud=n % 5 == 4 or n == len(keyframes) - 1)
    print(f"raw cloud: {len(cloud)} points ({cloud.voxel * 100:.0f} cm voxels, keyframes only)")
    return grid, cloud


# %% [markdown]
# ## Pass 3: dense map by TSDF fusion (slambook Ch. 12)
# A TSDF (truncated signed distance field) stores, per voxel, the distance to the nearest surface
# along the camera ray: + in front, - behind, clipped to +-tsdf_trunc. Each frame updates every voxel
# it sees with a running weighted average:
#     D <- (W D + d) / (W + 1),   W <- W + 1
# so the stereo noise of one frame averages out over the many frames that see the same surface.
# The surface is the zero crossing of D, extracted with marching cubes as a triangle mesh.
#
# Every tracked frame is used (not only keyframes), with its pose from pass 1 snapped to the plane,
# so the video is read and the depth computed a second time. Pixels at depth edges ("flying pixels"
# that interpolate between foreground and background) are removed first.

# %%
def clean_depth(depth, max_depth):
    """Prepare depth for TSDF fusion: drop pixels beyond max_depth and on depth discontinuities.

    Args:
        depth: H x W depth (m), NaN = none.
        max_depth: farthest depth kept (m).
    Returns:
        H x W float32 depth with 0 = no depth (Open3D's convention).
    """
    d = depth.copy()
    d[~(d < max_depth)] = np.nan                            # also catches NaN
    filled = np.nan_to_num(d, nan=0)
    k = np.ones((5, 5), np.uint8)
    edge = (cv2.dilate(filled, k) - cv2.erode(np.where(filled > 0, filled, 99).astype(np.float32), k)) > 0.05 * filled
    d[edge] = np.nan
    return np.nan_to_num(d, nan=0).astype(np.float32)


def fuse_tsdf(traj, world, cam, depth_src, cfg):
    """Fuse the depth of every (fuse_every-th) tracked frame into a TSDF and extract a mesh.

    Args:
        traj: Trajectory (poses and their video frame indices).
        world: World (planar-snapped camera -> world poses).
        cam: StereoCamera (rectification, intrinsics).
        depth_src: DepthSource.
        cfg: Config; uses video, tsdf_voxel, tsdf_trunc, tsdf_max_depth, fuse_every.
    Returns:
        (Open3D legacy TriangleMesh, Open3D legacy PointCloud), both in world coordinates.
    """
    # Open3D's tensor TSDF: voxels live in 16^3 blocks that are allocated only where surfaces are seen,
    # so the volume can cover the whole room. (The older o3d.pipelines.integration.ScalableTSDFVolume
    # returned an empty mesh with open3d 0.20 on macOS / Python 3.13.)
    tsdf = o3d.t.geometry.VoxelBlockGrid(
        attr_names=("tsdf", "weight", "color"), attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
        attr_channels=((1), (1), (3)), voxel_size=cfg.tsdf_voxel, block_resolution=16, block_count=50000,
        device=o3c.Device("CPU:0"))
    K_t = o3c.Tensor(cam.K, o3c.float64)
    trunc_voxels = cfg.tsdf_trunc / cfg.tsdf_voxel
    fids = traj.frame_ids
    pose_of = {fids[k]: k for k in range(0, len(fids), cfg.fuse_every)}   # video frame -> pose index

    t_start = time.perf_counter()
    cap = cv2.VideoCapture(cfg.video)
    i = -1
    n_fused = 0
    while True:
        ok, frame = cap.read()
        i += 1
        if not ok or i > fids[-1]:
            break
        k = pose_of.get(i)
        if k is None:
            continue
        T_wc = world.T_wc(traj.poses[k])
        bgr, gl, gr = cam.rectify(frame)
        depth_img = o3d.t.geometry.Image(o3c.Tensor(clean_depth(depth_src.depth(i, gl, gr), cfg.tsdf_max_depth)))
        color_img = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(bgr[:, :, ::-1], np.float32) / 255))
        T_cw = o3c.Tensor(np.linalg.inv(T_wc), o3c.float64)     # Open3D wants camera-from-world
        blocks = tsdf.compute_unique_block_coordinates(depth_img, K_t, T_cw, 1.0, cfg.tsdf_max_depth, trunc_voxels)
        tsdf.integrate(blocks, depth_img, color_img, K_t, T_cw, 1.0, cfg.tsdf_max_depth, trunc_voxels)
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
    return mesh, dense


# %% [markdown]
# ## Save results and plot the grid with the trajectory

# %%
def save_results(out_dir, traj, world, grid, mesh, dense):
    """Write occupancy_logodds.npy, occupancy.png, trajectory_xyz.txt, mesh.ply and dense_cloud.ply.

    Args:
        out_dir: output folder.
        traj: Trajectory (frame ids and fps for the time stamps).
        world: World (world-frame positions).
        grid: OccupancyGrid.
        mesh, dense: Open3D mesh and point cloud from fuse_tsdf().
    """
    traj_w = world.positions(traj)
    stamps = np.array(traj.frame_ids) / traj.fps
    np.save(f"{out_dir}/occupancy_logodds.npy", grid.logodds)
    np.savetxt(f"{out_dir}/trajectory_xyz.txt", np.column_stack([stamps, traj_w]), fmt="%.4f",
               header="time x y z  (world: x forward, y left, z up = camera height above floor, m)")
    cv2.imwrite(f"{out_dir}/occupancy.png", cv2.cvtColor(grid.image(traj_w[:, :2]), cv2.COLOR_RGB2BGR))
    o3d.io.write_triangle_mesh(f"{out_dir}/mesh.ply", mesh)
    o3d.io.write_point_cloud(f"{out_dir}/dense_cloud.ply", dense)
    print("saved to", out_dir)


def plot_grid(grid, traj_w, map_range):
    """Show the occupancy grid with the camera path in matplotlib.

    Args:
        grid: OccupancyGrid.
        traj_w: N x 3 world camera positions.
        map_range: depth range used for the map (m); sets the zoom.
    """
    half = grid.size / 2
    plt.figure(figsize=(7, 7))
    plt.imshow(grid.image(), extent=[half, -half, -half, half])   # x forward = up, y left = left
    plt.plot(traj_w[:, 1], traj_w[:, 0], color="tab:red", lw=2, label="camera path")
    plt.plot(traj_w[0, 1], traj_w[0, 0], "go", label="start")
    span = max(1.5, np.abs(traj_w[:, :2]).max() + map_range)
    plt.xlim(span, -span); plt.ylim(-span, span)
    plt.xlabel("y, left (m)"); plt.ylabel("x, forward (m)")
    plt.title("occupancy grid (white free, black occupied, gray unknown)")
    plt.legend(); plt.show()


# %% [markdown]
# ## Run all passes

# %%
def main(cfg):
    """Run the whole pipeline.

    Args:
        cfg: Config.
    Returns:
        dict with the intermediate results (cam, traj, loops, world, grid, cloud, mesh, dense),
        handy for inspecting in a notebook.
    """
    out_dir = cfg.output_dir()
    os.makedirs(out_dir, exist_ok=True)
    cam = StereoCamera(cfg.calib_path, cfg.proc_size)
    depth_src = DepthSource(cam, cfg.min_depth, cfg.max_depth, cfg.depth_dir)
    features = OrbFeatures()
    tracker = Tracker(cam, features, cfg.min_track_inliers, cfg.huber_k)
    init_rerun(cfg.viewer, out_dir, cam)

    traj = run_tracking(cfg, cam, depth_src, features, tracker)                  # pass 1
    loops = []
    if cfg.loop_closure and len(traj.keyframes) > cfg.loop_min_gap:
        loops = close_loops(traj, tracker, features, cfg)                       # pass 1b
    world = estimate_world(traj, cam, cfg.camera_height)
    rr.log("world/odom", rr.Transform3D(translation=world.T_wo[:3, 3], mat3x3=world.T_wo[:3, :3]), static=True)
    grid, cloud = build_keyframe_maps(traj, world, cam, cfg)                     # pass 2
    mesh, dense = fuse_tsdf(traj, world, cam, depth_src, cfg)                    # pass 3
    log_mesh(traj.frame_ids[-1], mesh)

    save_results(out_dir, traj, world, grid, mesh, dense)
    plot_grid(grid, world.positions(traj), cfg.map_range)
    return dict(cam=cam, traj=traj, loops=loops, world=world, grid=grid, cloud=cloud, mesh=mesh, dense=dense)


if __name__ == "__main__":
    results = main(parse_args())
