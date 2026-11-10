#!/usr/bin/env python
"""Teleoperate an SO-101 follower arm with an iOS/Android phone.

Adapted from lerobot's examples/phone_to_so100/teleoperate.py for the SO-101,
with the robot port, phone OS and URDF path exposed as command-line options.

Setup (once):
    pip install -e ".[phone,kinematics]"     # in the lerobot repo
    # URDF + meshes already downloaded to ./SO101/

Usage:
    python teleop_phone_so101.py --port /dev/cu.usbmodemXXXX --id my_follower --phone ios
"""

import argparse
import math
import time
from dataclasses import dataclass, field

import numpy as np
from pathlib import Path

from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import (
    RobotActionProcessorStep,
    TransitionKey,
    RobotProcessorPipeline,
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.teleoperators.phone import Phone, PhoneConfig
from lerobot.teleoperators.phone.config_phone import PhoneOS
from lerobot.teleoperators.phone.phone_processor import MapPhoneActionToRobotAction
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.rotation import Rotation
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

DEFAULT_URDF = Path(__file__).parent / "SO101" / "so101_new_calib.urdf"


@dataclass
class PhoneRotationRelative(RobotActionProcessorStep):
    """Measure phone rotation from the Move press, not from the AR session origin.

    lerobot passes the phone's absolute orientation ("Absolute orientation as rotvec",
    phone_processor.py) and composes it onto the gripper's reference pose. So whatever
    tilt the phone already had when the AR session started is commanded the instant you
    press Move - typically a large, constant roll/yaw the arm must adopt before you have
    moved at all.

    Latching the phone's rotation on the rising edge of `enabled` and sending
    `R_now @ R_latched^T` makes "however I am holding it right now" the neutral pose.
    """

    _ref_rot: object = field(default=None, init=False, repr=False)
    _prev_enabled: bool = field(default=False, init=False, repr=False)

    def action(self, action):
        enabled = bool(action.get("enabled", False))
        if not enabled:
            self._prev_enabled = False
            return action

        r_now = Rotation.from_rotvec(
            [float(action.get("target_wx", 0.0)),
             float(action.get("target_wy", 0.0)),
             float(action.get("target_wz", 0.0))]
        ).as_matrix()

        if not self._prev_enabled or self._ref_rot is None:
            self._ref_rot = r_now
        self._prev_enabled = True

        delta = r_now @ self._ref_rot.T
        wx, wy, wz = Rotation.from_matrix(delta).as_rotvec()
        action["target_wx"], action["target_wy"], action["target_wz"] = float(wx), float(wy), float(wz)
        return action

    def transform_features(self, features):
        return features

    def reset(self):
        self._ref_rot = None
        self._prev_enabled = False


@dataclass
class RotationInBaseFrame(RobotActionProcessorStep):
    """Interpret the phone's rotation in the robot's base frame, not the gripper's.

    lerobot composes `desired = reference_rot @ phone_rot`, i.e. the rotation is
    applied in the gripper's own frame, so which joint a given phone tilt drives
    changes as wrist_roll turns. Conjugating by the reference rotation,
    `phone_rot' = ref^T @ phone_rot @ ref`, makes that same composition come out as
    `desired = phone_rot @ reference_rot`: a fixed, pose-independent base-frame
    rotation.

    The reference is latched on the rising edge of `enabled`, matching
    EEReferenceAndDelta's own latching so both steps use the same frame.
    """

    kinematics: object
    _ref_rot: object = field(default=None, init=False, repr=False)
    _prev_enabled: bool = field(default=False, init=False, repr=False)

    def action(self, action):
        enabled = bool(action.get("enabled", False))
        if not enabled:
            self._prev_enabled = False
            return action

        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            return action
        q = np.array(
            [float(v) for k, v in observation.items() if isinstance(k, str) and k.endswith(".pos")],
            dtype=float,
        )
        if not self._prev_enabled or self._ref_rot is None:
            self._ref_rot = self.kinematics.forward_kinematics(q)[:3, :3]
        self._prev_enabled = True

        r_world = Rotation.from_rotvec(
            [float(action.get("target_wx", 0.0)),
             float(action.get("target_wy", 0.0)),
             float(action.get("target_wz", 0.0))]
        ).as_matrix()
        r_local = self._ref_rot.T @ r_world @ self._ref_rot
        wx, wy, wz = Rotation.from_matrix(r_local).as_rotvec()
        action["target_wx"], action["target_wy"], action["target_wz"] = float(wx), float(wy), float(wz)
        return action

    def transform_features(self, features):
        return features

    def reset(self):
        self._ref_rot = None
        self._prev_enabled = False


@dataclass
class RemapAxes(RobotActionProcessorStep):
    """Permute and/or negate a triple of phone commands.

    `spec` is three comma-separated terms drawn from x, y, z with an optional
    leading minus, e.g. "y,x,-z" means: new first = old y, new second = old x,
    new third = -old z. An empty spec is a no-op.

    Needed because lerobot's MapPhoneActionToRobotAction hardcodes one particular
    phone-to-robot axis convention (phone_processor.py:84-88). If the phone is held
    differently, moving or tilting it drives the wrong robot axis, and a sign flip
    cannot correct a permutation.

    `prefix` selects which triple: "target_w" for rotation, "target_" for translation.
    """

    spec: str = ""
    prefix: str = "target_w"

    def __post_init__(self):
        self.terms = []
        for term in (t.strip().lower() for t in self.spec.split(",") if t.strip()):
            sign = -1.0 if term.startswith("-") else 1.0
            axis = term.lstrip("+-")
            if axis not in ("x", "y", "z"):
                raise SystemExit(f"axis map terms must be x, y or z (optionally negated); got {term!r}")
            self.terms.append((sign, f"{self.prefix}{axis}"))
        if self.terms and len(self.terms) != 3:
            raise SystemExit(f"an axis map needs exactly 3 terms, got {len(self.terms)}")

    def action(self, action):
        if not self.terms:
            return action
        keys = tuple(f"{self.prefix}{a}" for a in ("x", "y", "z"))
        source = {k: float(action.get(k, 0.0)) for k in keys}
        for out_key, (sign, in_key) in zip(keys, self.terms, strict=True):
            action[out_key] = sign * source[in_key]
        return action

    def transform_features(self, features):
        return features


def build_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="serial port of the follower arm")
    p.add_argument("--id", default="my_follower", help="calibration id used by lerobot-calibrate")
    p.add_argument("--phone", choices=["ios", "android"], default="ios")
    p.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--ee-step", type=float, default=0.5,
                   help="phone-motion to end-effector scaling (lower = less sensitive)")
    p.add_argument("--rot-reference", choices=["press", "session"], default="press",
                   help="what counts as neutral orientation: 'press' measures tilt from how you "
                        "hold the phone when you press Move (recommended); 'session' is lerobot's "
                        "behaviour, measuring from the AR session origin.")
    p.add_argument("--rot-frame", choices=["gripper", "base"], default="gripper",
                   help="frame the phone's rotation is applied in. 'gripper' (lerobot's behaviour) "
                        "means the axis a tilt drives changes as wrist_roll turns; 'base' keeps it "
                        "fixed regardless of arm pose.")
    p.add_argument("--trans-map", default="", metavar="SPEC",
                   help="remap the phone's translation axes, same syntax as --rot-map. Use when "
                        "moving the phone sideways drives the arm forward/back. Applied before "
                        "--invert and --ee-step.")
    p.add_argument("--rot-map", default="", metavar="SPEC",
                   help="remap the phone's rotation axes, e.g. 'y,x,-z' means new wx = old wy, "
                        "new wy = old wx, new wz = -old wz. Use when tilting the phone drives the "
                        "wrong robot axis. Terms may be negated, e.g. 'y,x,-z'.")
    p.add_argument("--invert", default="", metavar="AXES",
                   help="comma-separated axes to flip, e.g. 'x' or 'x,z'. Use when the arm "
                        "moves opposite to the phone. Axes are in the robot's frame: "
                        "x forward/back, y left/right, z up/down.")
    p.add_argument("--orientation-weight", type=float, default=0.01,
                   help="how hard IK tries to match the commanded gripper orientation, against a "
                        "position weight of 1.0. lerobot's default 0.01 means position dominates and "
                        "wrist_flex barely tracks tilt. Try 0.3-1.0 to point the gripper down; on a "
                        "5-DOF arm this trades away position accuracy.")
    p.add_argument("--max-ee-step-m", type=float, default=0.10,
                   help="largest end-effector jump allowed per control step, in metres")
    p.add_argument("--gripper-speed", type=float, default=20.0,
                   help="gripper units moved per control step while A/B is held (default 20, "
                        "range is 0-100 so the default saturates in ~5 frames)")
    p.add_argument("--debug-inputs", action="store_true",
                   help="print the raw phone inputs and the resulting gripper command")
    p.add_argument("--rest-pose", default="",
                   help="comma-separated joint targets in degrees to return to on exit, in bus "
                        "order (shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper). "
                        "Default: the pose the arm was in when the script started.")
    p.add_argument("--return-seconds", type=float, default=3.0,
                   help="how long the return-to-rest move takes (0 disables it)")
    p.add_argument("--settle-seconds", type=float, default=2.0,
                   help="after the return ramp, keep holding the rest pose up to this long until "
                        "every joint has actually arrived (default 2.0)")
    p.add_argument("--settle-tolerance", type=float, default=2.0,
                   help="degrees of error per joint counted as arrived (default 2.0)")
    p.add_argument("--keep-torque", action="store_true",
                   help="leave torque enabled on exit so the arm holds its rest pose")
    return p


def parse_args():
    return build_parser().parse_args()


def print_settings(args):
    """Echo every command-line value, flagging the ones left at their default."""
    defaults = {a.dest: a.default for a in build_parser()._actions}
    print("Settings:")
    for name, value in sorted(vars(args).items()):
        flag = "--" + name.replace("_", "-")
        marker = "" if value != defaults.get(name) else "   (default)"
        print(f"  {flag:<20} {value!r}{marker}")


def return_to_rest(robot, rest_pose, seconds, fps, settle_s=2.0, tol_deg=2.0):
    """Interpolate from the current joints to `rest_pose` so torque-off doesn't drop the arm.

    Servos lag behind their goal, especially joints loaded by gravity, so after the
    ramp we keep holding the final target until every joint has actually arrived
    (or `settle_s` elapses). Disconnecting before that cuts torque mid-move and the
    arm stops short of the rest pose.
    """
    if seconds <= 0 or not rest_pose:
        return
    obs = robot.get_observation()
    start = {k: float(v) for k, v in obs.items() if k.endswith(".pos")}
    if not start:
        return
    keys = [k for k in start if k in rest_pose]
    steps = max(1, int(seconds * fps))
    print(f"Returning to rest pose over {seconds:.1f}s...")
    for i in range(1, steps + 1):
        # Cosine ease-in-out: no jerk at either end of the move.
        alpha = 0.5 - 0.5 * math.cos(math.pi * i / steps)
        target = {key: start[key] + alpha * (rest_pose[key] - start[key]) for key in keys}
        robot.send_action(target)
        precise_sleep(1.0 / fps)

    # Hold the final target until the joints converge.
    final = {key: rest_pose[key] for key in keys}
    deadline = time.perf_counter() + settle_s
    errors = {}
    while time.perf_counter() < deadline:
        robot.send_action(final)
        precise_sleep(1.0 / fps)
        now = {k: float(v) for k, v in robot.get_observation().items() if k.endswith(".pos")}
        errors = {k: abs(now[k] - final[k]) for k in keys if k in now}
        if errors and max(errors.values()) <= tol_deg:
            print("Rest pose reached.")
            return
    stuck = {k.removesuffix(".pos"): round(v, 1) for k, v in errors.items() if v > tol_deg}
    if stuck:
        print(f"Joints short of rest pose after {settle_s:.1f}s (degrees off): {stuck}")
        print("They may be blocked, gravity-loaded, or clamped by their position limits.")


def main():
    args = parse_args()

    # placo resolves the URDF's mesh paths relative to the process, not the file,
    # so the URDF must be passed as an absolute path.
    print_settings(args)

    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.is_file():
        raise SystemExit(f"URDF not found: {urdf_path}")

    # A negative step size flips that axis: EEReferenceAndDelta multiplies the
    # phone's delta by these before adding it to the reference pose.
    inverted = {a.strip().lower() for a in args.invert.split(",") if a.strip()}
    if not inverted <= {"x", "y", "z"}:
        raise SystemExit(f"--invert accepts x, y, z; got {sorted(inverted)}")
    step_sizes = {a: (-args.ee_step if a in inverted else args.ee_step) for a in "xyz"}
    print("Resolved:")
    print(f"  urdf                 {urdf_path}")
    print(f"  ee step sizes        {step_sizes}")
    print(f"  orientation weight   {args.orientation_weight} (position weight is 1.0)")
    print(f"  translation remap    {args.trans_map or 'none (x,y,z unchanged)'}")
    print(f"  rotation remap       {args.rot_map or 'none (wx,wy,wz unchanged)'}")
    print(f"  rotation frame       {args.rot_frame}")
    print(f"  rotation reference   {args.rot_reference}")

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id, use_degrees=True))
    phone_os = PhoneOS.IOS if args.phone == "ios" else PhoneOS.ANDROID
    teleop_device = Phone(PhoneConfig(phone_os=phone_os))

    motor_names = list(robot.bus.motors.keys())
    kinematics_solver = RobotKinematics(
        urdf_path=str(urdf_path),
        target_frame_name="gripper_frame_link",
        joint_names=motor_names,
    )

    # phone pose -> end-effector target -> joint targets
    phone_to_robot_joints_processor = RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ](
        steps=[
            MapPhoneActionToRobotAction(platform=phone_os),
            RemapAxes(spec=args.trans_map, prefix="target_"),
            RemapAxes(spec=args.rot_map, prefix="target_w"),
            *([PhoneRotationRelative()] if args.rot_reference == "press" else []),
            *([RotationInBaseFrame(kinematics=kinematics_solver)] if args.rot_frame == "base" else []),
            EEReferenceAndDelta(
                kinematics=kinematics_solver,
                end_effector_step_sizes=step_sizes,
                motor_names=motor_names,
                use_latched_reference=True,
            ),
            EEBoundsAndSafety(
                end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
                max_ee_step_m=args.max_ee_step_m,
            ),
            GripperVelocityToJoint(speed_factor=args.gripper_speed),
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=motor_names,
                initial_guess_current_joints=True,
                orientation_weight=args.orientation_weight,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    robot.connect()

    # Capture the startup pose before anything moves: wherever the arm was parked is,
    # by definition, a pose it can be left in safely.
    startup = {k: float(v) for k, v in robot.get_observation().items() if k.endswith(".pos")}
    if args.rest_pose:
        values = [float(v) for v in args.rest_pose.split(",")]
        if len(values) != len(motor_names):
            raise SystemExit(
                f"--rest-pose needs {len(motor_names)} values ({', '.join(motor_names)}), got {len(values)}"
            )
        rest_pose = {f"{name}.pos": v for name, v in zip(motor_names, values, strict=True)}
    else:
        rest_pose = startup
    source = "--rest-pose" if args.rest_pose else "captured at startup"
    print(f"  rest pose ({source}): "
          + ", ".join(f"{k.removesuffix('.pos')}={v:.1f}" for k, v in rest_pose.items()))

    teleop_device.connect()
    if not robot.is_connected or not teleop_device.is_connected:
        raise SystemExit("Robot or phone is not connected.")

    init_rerun(session_name="phone_so101_teleop")
    print("Hold the enable button on the phone and move it to drive the arm. Ctrl+C to stop.")

    try:
        while True:
            t0 = time.perf_counter()
            robot_obs = robot.get_observation()
            phone_obs = teleop_device.get_action()
            if args.debug_inputs:
                rot = phone_obs.get("phone.rot")
                rotvec = rot.as_rotvec() if rot is not None else None
                raw = phone_obs.get("phone.raw_inputs", {})
                if rotvec is not None:
                    print(f"phone rotvec x={rotvec[0]:+.2f} y={rotvec[1]:+.2f} z={rotvec[2]:+.2f}  "
                          f"enabled={phone_obs.get('phone.enabled')} buttons={raw}")
            joint_action = phone_to_robot_joints_processor((phone_obs, robot_obs))
            if args.debug_inputs:
                print(f"    -> gripper.pos={joint_action.get('gripper.pos')} "
                      f"(now {robot_obs.get('gripper.pos')})")
            robot.send_action(joint_action)
            log_rerun_data(observation=phone_obs, action=joint_action)
            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - t0), 0.0))
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        try:
            return_to_rest(robot, rest_pose, args.return_seconds, args.fps,
                           settle_s=args.settle_seconds, tol_deg=args.settle_tolerance)
        except Exception as e:  # noqa: BLE001 - never block disconnect on a failed return
            print(f"Return to rest failed: {e}")
        if args.keep_torque:
            robot.config.disable_torque_on_disconnect = False
            print("Leaving torque enabled; the arm will hold its pose.")
        robot.disconnect()
        teleop_device.disconnect()


if __name__ == "__main__":
    main()
