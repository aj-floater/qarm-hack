"""Launch a gripper-focused test simulation using PyBullet's built-in GUI controls."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from sim.env import QArmSimEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the test sim (gripper-focused) with PyBullet debug sliders.")
    parser.add_argument("--real-time", action="store_true", help="Let PyBullet step in real time.")
    parser.add_argument("--time-step", type=float, default=1.0 / 120.0, help="Physics timestep (seconds).")
    parser.add_argument(
        "--gripper-urdf",
        type=Path,
        help="Override path to the gripper URDF (useful when editing the file).",
    )
    parser.add_argument(
        "--new-gripper",
        action="store_true",
        help="Use the new gripper package at qarm_gripper_new/urdf/robot.urdf.",
    )
    parser.add_argument("--light-mode", action="store_true", help="Use a light PyBullet background.")
    parser.add_argument(
        "--with-arm",
        action="store_true",
        help="Include the arm instead of running gripper-only.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    gripper_only = not args.with_arm
    gripper_urdf = args.gripper_urdf
    if args.new_gripper:
        gripper_urdf = Path(__file__).resolve().parent.parent / "qarm_gripper_new" / "urdf" / "robot.urdf"
    env = QArmSimEnv(
        gui=True,
        real_time=args.real_time,
        time_step=args.time_step,
        dark_mode=not args.light_mode,
        enable_joint_sliders=not gripper_only,
        enable_gripper_sliders=True,
        show_debug_gui=True,
        gripper_only=gripper_only,
        gripper_urdf_path=gripper_urdf,
        add_ground=True,
        attach_gripper=not gripper_only,
    )
    env.reset()

    mode_label = "gripper-only" if gripper_only else "arm + gripper"
    print(f"Running {mode_label} with PyBullet debug sliders. Close the PyBullet window or Ctrl+C to exit.")
    try:
        while True:
            if env.enable_joint_sliders or env.enable_gripper_sliders:
                env.apply_joint_slider_targets()
            if not args.real_time:
                env.step()
            time.sleep(env.time_step)
    except KeyboardInterrupt:
        pass
    finally:
        env.disconnect()


if __name__ == "__main__":
    main()
