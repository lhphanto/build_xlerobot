# Camera-to-left-arm calibration (version 1)

Calibrated November 16, 2026. Source: `handeye_result.json` in this folder, from `handeye_calibrate.py`.

## Valid only at this neck pose

| Neck joint | Raw position |
|---|---|
| Pan (ID 7) | 2012 |
| Tilt (ID 8) | 2286–2287 (target 2288) |

The camera is on the neck. **If the neck moves, this result no longer applies.** The tilt operating limit was raised to 2295 on the same day so this downward view of the desk is allowed.

## Result

Camera (left-half lens) pose in the left arm's base frame (meters). Arm base frame from the SO101 model: x forward, y left, z up.

```
T_base_camera = [[-0.17642, -0.37593,  0.90970,  0.02695],
                 [-0.98336,  0.10811, -0.14602, -0.12585],
                 [-0.04346, -0.92032, -0.38874,  0.44162],
                 [ 0.0,      0.0,      0.0,      1.0    ]]
```

| | Value |
|---|---|
| Camera position | 2.7 cm forward, 12.6 cm right (y = −12.6), 44.2 cm up from the arm base |
| Viewing direction (optical axis) | (0.91, −0.15, −0.39): forward and about 23° down |
| Image "down" in base frame | (−0.38, 0.11, −0.92): mostly straight down |
| Marker on gripper | about 44 mm, 3 mm, −73 mm from the gripper frame |

## Accuracy

8 poses, marker 17–48 cm from the camera, spanning about 35 cm side to side and 3–19 cm below the camera.

| | Value |
|---|---|
| Camera–arm agreement per pose | 1.5–22.3 mm, mean 8.5 mm |
| Camera position change when any one pose is left out | 0.6–10.3 mm |
| Camera rotation change when any one pose is left out | 0.2–1.3° |

**Expect roughly 1 cm error** when converting a camera point to an arm target: less near the robot, more at full reach. The error grows with how far the arm is stretched (the 48 cm pose is off by 22 mm), which points to the arm model or link bending rather than the camera. Good enough to move near an object; marginal for precise grasping. Improving it means also solving for small joint-angle offsets in the arm model (about 15 poses).

## Converting a depth point to arm coordinates

`calculate_depth.py` produces points in the **rectified** left camera frame. This calibration uses the **original** left lens frame. Undo the rectification rotation `R1` first:

```python
import json
import numpy as np

T_base_camera = np.array(json.load(open("samples/hand_eye/handeye_result.json"))["camera_left_lens_in_arm_base"])

def rectified_to_arm_base(p_rect, R1):
    """p_rect: (x, y, z) meters from reprojectImageTo3D; R1: from cv2.fisheye.stereoRectify."""
    p_lens = R1.T @ np.asarray(p_rect, dtype=float)      # rectified -> original left lens frame
    return (T_base_camera @ np.append(p_lens, 1.0))[:3]  # left lens -> arm base, meters
```

For a point taken from the raw (unrectified) image with `solvePnP` or `cv2.fisheye.undistortPoints`, skip the `R1` step.

## How it was done

- ArUco marker DICT_4X4_50 id 0, 30 mm (`aruco_marker_letter.pdf`), fixed to the gripper's fixed jaw.
- For each pose the arm was placed by hand and held with `left_arm_hold`; one stereo snapshot gave the marker pose (fisheye calibration plus `solvePnP`); the six joint readings gave the gripper pose through the SO101 model and the left arm's kinematic calibration.
- Solved camera-to-base and marker-to-gripper together: Park–Martin closed form, then least-squares refinement. OpenCV 5's Python build lacks `calibrateHandEye`, so the script does this itself; checked on synthetic data (exact without noise; 2.1 mm and 0.4° with 2 mm noise).
- Poses with the marker on a different arm link were discarded (`rejected_marker_on_arm/`).

## Reproduce or extend

From `~/Documents/build_xlerobot`, with the neck at the pose above:

```sh
python handeye_calibrate.py capture --marker-mm 30   # one pose; arm must be holding
python handeye_calibrate.py solve
```
