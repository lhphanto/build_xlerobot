# XLeRobot stereo camera calibration

Calibrated November 6, 2026. Source data: `calibration_fisheye.json` in this folder.

## Setup

| Item | Value |
|---|---|
| Camera | 3D USB Camera, one side-by-side frame of 3840 × 1080 |
| Image per lens | 1920 × 1080 (left and right halves of the frame) |
| Orientation | Frames rotated 180° at capture (`hflip,vflip`), as the robot's snapshots are |
| Board | 9 × 6 inner corners, 25.4 mm squares |
| Method | Zhang's method per lens, then stereo with intrinsics fixed |
| Lens model | OpenCV fisheye (equidistant), k1–k4 |
| Views used | 24 per lens, all 24 seen by both halves; `pair_17` dropped (error above 2 px) |
| Sources | 4 snapshots (`pair_01`–`04`) and 20 still frames from three videos (`pair_05`–`25`) |

**"Left" and "right" mean the left and right halves of the rotated frame.** The 180° rotation swaps the halves, so the left half may come from the robot's physical right lens. That mapping is not confirmed.

## Intrinsics

| | Left half | Right half |
|---|---|---|
| fx (px) | 600.892 | 602.116 |
| fy (px) | 604.389 | 604.957 |
| cx (px) | 967.549 | 980.867 |
| cy (px) | 522.636 | 555.336 |
| Skew | 0 (fixed) | 0 (fixed) |
| RMS reprojection error | 0.702 px | 0.699 px |

Camera matrices (pixels):

```
K_left  = [[600.892,   0.000, 967.549],
           [  0.000, 604.389, 522.636],
           [  0.000,   0.000,   1.000]]

K_right = [[602.116,   0.000, 980.867],
           [  0.000, 604.957, 555.336],
           [  0.000,   0.000,   1.000]]
```

Fisheye distortion, D = [k1, k2, k3, k4], for θ_d = θ (1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸):

```
D_left  = [0.247534, 0.331279, -0.106190, -0.067889]
D_right = [0.214596, 0.495423, -0.421499,  0.144361]
```

Use these only with `cv2.fisheye` functions (for example `cv2.fisheye.undistortImage`, `cv2.fisheye.undistortPoints`), not the standard `cv2.undistort`.

## Position difference between the two lenses

**The lenses are 60.48 mm apart, almost purely sideways, and rotated 0.46° relative to each other.**

Position of the right-half lens's optical centre, in the left-half lens's coordinate frame (x right, y down, z forward along the view):

| Axis | Offset |
|---|---|
| x (sideways) | **+60.48 mm** |
| y (vertical) | −0.08 mm |
| z (forward) | +0.58 mm |
| Distance (baseline) | **60.48 mm** |

Rotation of the right-half lens relative to the left: 0.46° in total (rotation vector 0.222°, −0.397°, −0.082° about x, y, z).

OpenCV's stereo output, which maps a point from left-lens coordinates to right-lens coordinates as X_right = R · X_left + T:

```
R = [[ 0.999975,  0.001410, -0.006934],
     [-0.001437,  0.999992, -0.003863],
     [ 0.006929,  0.003873,  0.999968]]

T = [-60.472, 0.168, -0.994]  mm
```

T is in the right lens's frame, so its numbers differ slightly from the position table above, which is −Rᵀ · T.

Stereo RMS reprojection error: 0.795 px.

## Quality checks

| Check | Result |
|---|---|
| Focal lengths agree between lenses | ✅ fx 600.9 vs 602.1 (0.2% apart) |
| Offset is mostly sideways | ✅ vertical 0.08 mm, forward 0.58 mm |
| Small relative rotation | ✅ 0.46° |
| Low reprojection error | ✅ 0.70 px per lens, 0.79 px stereo |
| Stable as views were added | ✅ fx moved under 1% from 18 to 24 views; baseline 60.1–60.5 mm |
| Plausible field of view | ⚠️ right ≈ 121° × 85°; left ≈ 155° horizontal, which is not plausible for matching lenses |
| Corners cover the whole image | ⚠️ left x 311–1564, y 146–848; right x 233–1466, y 172–880 (of 1920 × 1080) |

## Where to trust it

**Trust it within roughly the corner-covered area of each half:** x ≈ 300–1550 and y ≈ 150–880. That is about the central 70% of the width and most of the height.

Outside that area the distortion is extrapolated, so corrections and depth there are not reliable. The left half's implausible 155° horizontal field of view comes from that extrapolation near its right edge, not from the lens. Covering the outer strips (board about 1 m to the side at 60 cm distance, and at the bottom of the view), or fitting only k1 and k2 for the left lens, should fix it.

## How to reproduce

From `~/Documents/build_xlerobot`:

```sh
python calibrate_stereo.py --dir samples/calibration --fisheye --max-view-error 2 --out samples/calibration/calibration_fisheye.json
```

Adding frames from a new video (`--either-half` keeps edge views that only one lens sees fully):

```sh
python extract_calib_frames.py samples/calibration/<video>.mp4 --count 6 --every 1 --max-motion 12 --either-half
```

The camera delivers about 1 frame per second at full resolution, so hold each board pose still for about 4 seconds when recording.

## Files in this folder

| File | Contents |
|---|---|
| `calibration_fisheye.json` | **Current result**, all values above plus per-view errors and E/F matrices |
| `calibration.json` | Outdated: standard (pinhole) model fitted to only 5 pairs. Do not use. |
| `pair_NN_left_half.png`, `pair_NN_right_half.png`, `pair_NN_side_by_side.png` | Calibration images |
| `undistorted_fisheye_*.png` | Corrected previews from the current result |
| `calib_video*.mp4` | Source videos (large, not needed once frames are extracted) |
