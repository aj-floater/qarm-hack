"""
Convenience entry point for launching the QArm simulation environment (VSCode-friendly).

Run as:
    python -m sim.run_gui --gui --real-time --sliders
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from sim.env import QArmSimEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch the QArm simulation environment.")
    parser.add_argument("--gui", action="store_true", help="Enable PyBullet GUI (default headless).")
    parser.add_argument("--real-time", action="store_true", help="Let PyBullet step in real time.")
    parser.add_argument("--sliders", action="store_true", help="Expose arm joint sliders in the GUI.")
    parser.add_argument(
        "--attach-gripper",
        action="store_true",
        help="Load qarm_gripper as a second body and constrain it to the end effector.",
    )
    parser.add_argument(
        "--gripper-sliders",
        action="store_true",
        help="Show sliders for gripper joints (requires --attach-gripper or --gripper-only).",
    )
    parser.add_argument(
        "--gripper-only",
        action="store_true",
        help="Load only the gripper URDF, fixed to the floor (no arm).",
    )
    parser.add_argument(
        "--gripper-urdf",
        type=Path,
        help="Override path to the gripper URDF (useful while editing).",
    )
    parser.add_argument("--headless-steps", type=int, default=600, help="Steps to run when headless.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.gripper_only and not args.gui:
        print("Enabling --gui for gripper-only mode.")
        args.gui = True
    attach_gripper = False if args.gripper_only else args.attach_gripper
    if args.gripper_sliders and not (attach_gripper or args.gripper_only):
        print("Hint: enable --attach-gripper or --gripper-only to use --gripper-sliders.")
    env = QArmSimEnv(
        gui=args.gui,
        real_time=args.real_time,
        enable_joint_sliders=args.sliders,
        attach_gripper=attach_gripper,
        enable_gripper_sliders=args.gripper_sliders,
        gripper_only=args.gripper_only,
        gripper_urdf_path=args.gripper_urdf,
        add_ground=args.gripper_only,
    )
    env.reset()

    try:
        if args.gui:
            print("GUI running. Close window or Ctrl+C to exit.")
            while True:
                if args.sliders or args.gripper_sliders:
                    env.apply_joint_slider_targets()
                if not args.real_time:
                    env.step()
                time.sleep(env.time_step)
        else:
            print(f"Running headless for {args.headless_steps} steps...")
            for _ in range(args.headless_steps):
                env.step()
            print("Headless run complete.")
    except KeyboardInterrupt:
        print("Stopping simulation...")
    finally:
        env.disconnect()


if __name__ == "__main__":
    main()
