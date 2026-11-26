# Grasp and basket skill (left arm)

Pick an object off the desk with the SO101 left arm and drop it into the black basket. Perception (SlimSAM +
Contact-GraspNet on the stereo camera) and model-checked planning produce step lists; `step.py` executes them through
the xlerobot MCP server, one supervised step (or a bounded range of steps) per operator go, with a camera check after
each. Standing juice box, ~3.3 x 4.9 x 13.5 cm.

**Rules for agents:** nothing moves without the operator's go for that step or range; never bypass a safety gate;
never print the server token; the operator restarts the server after code changes in `xlerobot_mcp/` (changes in
`grasp/` need no restart). Server capabilities: `../xlerobot_mcp/README.md` ("Current capabilities").

| Script | Env | What it does |
|---|---|---|
| `step.py` | any | Runner: `status`, `camera`, `hold`, `ready`, `reach`, `adjust`, `open`, `close`, `look`, `view`, `run PLAN --steps a-b` |
| `look.py` | contact_graspnet | Camera check: snapshot; with an expected object point: segment + depth, compare (OK / CHECK / UNSEEN) |
| `sam_click.py` | contact_graspnet | SlimSAM object mask from 1–2 clicks on the raw left image |
| `cgn_grasps.py` | contact_graspnet | Contact-GraspNet grasps for the masked object (arm frame) + object size/top/desk height |
| `plan_grasp.py` | lerobot | Grasp task: chooses a grasp, offset, tilt; approach chain, open, descend, close, retry steps (or a side pull) |
| `plan_basket.py` | lerobot | Basket task: test lift, lifts, carry, camera check over the basket, lower, open, lift away, view |
| `plan_ready.py` | lerobot | Chain from the current pose to the ready pose (`step.py ready` uses it) |
| `adjust.py` | lerobot | Operator correction from the current pose (`step.py adjust` uses it) |
| `common.py` | lerobot | Arm model (FK, IK via `xlerobot_mcp/controllers/left_reach.py`, chains), ready pose, held-object model |
| `test_planners.py` | lerobot | Planner regression (no hardware, ~4 min) |
| `test_look.py` | contact_graspnet | Camera-check replay on November24 frames (no hardware, ~1 min) |

Envs: `E=~/miniforge3/envs/contact_graspnet/bin/python`, `L=~/miniforge3/envs/lerobot/bin/python`. Plans, frames and
the step log go to `samples/grasp/` (`steps.jsonl` has every step and camera check). Run everything from the repo root.
Done end to end three times: November23, November24 (operator corrections over the basket) and November25 (no
corrections with the current defaults; the box twisted in a 2 cm grip during the carry, hence the 3.5 cm grip below).

## Start

1. `python grasp/step.py hold` — warms up the camera (first snapshot ~30 s; frames before that are yellow), turns the
   neck torque on in place if it is off (`neck_hold`, no travel; November25 it was off at the basket view), then
   torque on in place. If it refuses because a joint rests past its limit, the operator repositions by hand with
   support (November24: the limp elbow rested 34 ticks past; folding it in a few degrees fixed it). Recover-hold is
   used automatically when only the elbow is up to 32 ticks past.
2. `python grasp/step.py ready` — camera warm check, then plans (up to ~1 min) and runs the chain to the ready pose:
   gripper (0.20, −0.08, 0.14), 30° lean, out of the camera's view of the desk. 2–3 steps from the folded rest; also
   works from a grasp hover (4 steps; it keeps the jaw line and grows steps as the joint limit allows).

## Task 1: grasp

1. **Frame** (arm at ready, so it does not hide the object): `$E grasp/look.py` → prints the snapshot path; the
   `_look.jpg` preview is the left image at half scale (multiply pixel coordinates by 2 for the raw image).
2. **Segment**: click the object's top and one face (raw left-image pixels):
   `$E grasp/sam_click.py FRAME --click U1 V1 --click U2 V2` → check `<frame>_mask.jpg` covers the whole object.
3. **Grasps**: `$E grasp/cgn_grasps.py FRAME --mask samples/grasp/<frame>_mask.npy` → `<frame>_grasps.json` and
   overlay `<frame>_grasps.jpg`; prints the object's top z, height and footprint. Rerun perception whenever the arm
   was between the camera and the object (an occluded frame gives only side grasps).
4. **Plan**: `$L grasp/plan_grasp.py samples/grasp/<frame>_grasps.json` (live joints) → `<frame>_grasp_plan.json`:
   - grasp: top-down (<20°) first, then within 1 cm of the object's footprint centre, then best score (`--index N`
     overrides). November24 the top score sat on the box's near corner;
   - target = grasp centre + `--offset` (default (0, −0.015) m; operator corrections toward −y: 20, 20 mm on
     November23, 15 mm on November24; none needed November25; cause unknown) at `--grip-depth` (3.5 cm) below the
     top. The gripper point is at the fingertips, so this is how much of the object the jaws overlap: 1 cm slipped
     (November23), 2 cm carried but the 13.6 cm box twisted and slipped in the jaws later (November25);
   - the smallest lean (15–45°) whose hover (`--hover` 5 cm above) and grasp chain from the current pose;
   - if no top-down grasp is reachable: a side pull (below).
5. **Run**: `python grasp/step.py run PLAN --steps 1-N` for the approach to the hover (one go). The hover check should
   be OK. Then the operator looks: if the jaws are off, correct with `step.py adjust` (e.g. `--right 1.5`) — the
   operator's directions are from behind the arm: right = −y, forward = +x. Then `open` (skipped when already open),
   descend (camera reports UNSEEN: the wrist hides the box; the operator confirms it is between the jaws), `close`,
   then the plan's **re-squeeze** (a second `close --squeeze 24`; run both on one go: `--steps K-K+1`).
6. **Grip check**: `close` prints the push (holding − goal ticks), ≥ 7 OK after the re-squeeze. It can settle a few
   ticks lower as cardboard gives (November25: 1749 → 1742 over the carry). Weak: the plan's retry (close again, or
   regrasp 1 cm deeper). A push of 11 / holding load 84 with a 2 cm grip still let the box twist on November25: depth
   matters more than squeeze for a tall box. Every `close` re-contacts and sets the goal `--squeeze` past the new
   contact, replacing the earlier squeeze: a small one weakens the grip (November25 `--squeeze 2` after a 24: push
   8 → 2, load 64 → 24; another `--squeeze 24` gave push 9, load 76, jaws 1744 → 1710). Always re-squeeze with 24.

### Side pull (object too far for a top-down grasp)

When no 15–45° top-down grasp chains from the current pose, `plan_grasp.py` plans a side pull instead (or `--pull`),
`<frame>_pull_plan.json`: approach with the gripper leaning 60–90°, jaws across the line from the base, at the
object's mid-height; open; advance 1 cm past its centre; close lightly (12-tick squeeze); slide it toward the arm to
~0.35 m (3–8 cm); open; back away and up. Then redo perception and `plan_grasp.py`. (Model-checked, not yet run live.)

## Task 2: lift, carry, drop into the basket

1. **Plan** (jaws closed on the object): `$L grasp/plan_basket.py samples/grasp/<frame>_grasp_plan.json` (live joints,
   ~40 s) → `<frame>_basket_plan.json`. Defaults are the November24 basket: `--drop 0.18 -0.20` (object centre at
   release, = the gripper point where it went in), `--rim-z -0.01`. If the basket moved, measure a rim point and a
   point inside with `python reach_point.py --pixel U V --image FRAME`.
   - The held object hangs **straight down** below the gripper point (it pivots in the jaws; camera depth on three
     lifts), `hang` = height − grip depth. The old along-the-lean model put it ~6 cm too far out.
   - Steps: test lift 4 cm (camera must see it rise; else back down and retry the grip) → lifts until the bottom is
     `--clearance` (3.5 cm) above the rim → carry (wrist roll free) with the gripper over the drop point → **camera
     check over the basket** (needs OK: object within 3 cm of the drop point, bottom above the rim) → lower until the
     bottom is `--release` (−0.01: 1 cm below the rim; the operator wanted lower than +1 cm) → open → lift the empty
     gripper 5 cm → `view` (camera tilted to 2350, snapshot, back to the calibration pose).
2. **Run**: `python grasp/step.py run PLAN --steps 1-K` up to the check (one go), then the rest. The run stops on a
   failed step or a camera CHECK; UNSEEN stops only the check step. A loaded reach that stalls (yaw falls behind), or
   that ends just outside the server's 8-tick final tolerance with no error (the loaded shoulder sags: November25 test
   lift ended 9 ticks off), is re-run once automatically from where it holds; a finish within 16 ticks after that is
   accepted and the camera check decides. November25 the carry-2 check gave a false CHECK (the mask caught only the
   top of the box near the jaws, bottom measured 4 cm high): look at the `_look.jpg` before acting on a CHECK.
3. **Corrections over the basket**: `python grasp/step.py adjust --free-roll ...` (roll free while carrying):
   `--retract CM`/`--extend CM` (toward/away from the base), `--swing-left DEG` (servo 1 down) / `--swing-right DEG`
   (servo 1 up), `--up/--down CM`. If the operator names a servo ("move servo 1/2"), confirm the direction in these
   terms before moving: November24 "retract, e.g. servo 1" was first taken the wrong way.

## Camera checks (look.py)

The camera sits behind and above the arm. At the hover, descent and carry the arm hides part or all of the object.
`look.py` projects the expected object point, tries several seeds around it, drops depth points above the gripper
point + 3 cm (the arm), rejects masks wider than 12 cm (desk sheet, arm), and reports:
OK (within `--tolerance` 3 cm), CHECK (seen somewhere else: look before continuing), UNSEEN (hidden: not evidence of
a problem; the operator's eyes decide where it matters, e.g. before closing). The basket interior is below the view
at the calibration tilt; `view` tilts down to 2350 (limit 2356) for a snapshot only (depth needs the calibration pose).

## Things to watch

- **Shoulder temperature.** ID2 climbed 31 → 39 °C over a November24 carry (49 °C on November23). `step.py` warns at
  50 °C; the controllers stop at 60 °C. Don't park the loaded arm; rest it (release with support) between attempts.
- **Bogus ID1 temperatures** (80–150 °C) during motion are re-read and reported as `glitches`; a sustained bad value stops.
- **Over the basket the camera barely sees the box** (November25: a 1 cm sliver of mask at the frame edge gave the OK).
  The operator's eyes decide before the drop; the post-drop `view` shows the box in the basket.
- **Perception drift between frames.** The same untouched box measured 2.4 cm apart in y between two frames on
  November24; re-run perception from the pose you will grasp from rather than reusing an old frame.
- **Not covered yet:** finding the basket automatically, the cause of the −y grasp correction, choosing grasps by the
  SO101 jaw geometry (Contact-GraspNet assumes a Panda gripper), loads heavier than a juice box, a live side pull.
