"""
Launch a gripper-focused test simulation with a Qt control panel (no PyBullet UI widgets).

Usage:
    python -m sim.test_sim --gripper-only   # default gripper-only view
"""

from __future__ import annotations

import argparse
from pathlib import Path

from sim.env import QArmSimEnv
from sim.qt_panel import launch_qt_control_panel


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the test sim (gripper-focused) with a Qt control panel.")
    parser.add_argument("--real-time", action="store_true", help="Let PyBullet step in real time.")
    parser.add_argument("--time-step", type=float, default=1.0 / 120.0, help="Physics timestep (seconds).")
    parser.add_argument(
        "--gripper-urdf",
        type=Path,
        help="Override path to the gripper URDF (useful when editing the file).",
    )
    parser.add_argument("--light-mode", action="store_true", help="Use a light PyBullet background.")
    parser.add_argument(
        "--with-arm",
        action="store_true",
        help="Include the arm instead of running gripper-only (still controlled via Qt).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    gripper_only = not args.with_arm
    env = QArmSimEnv(
        gui=True,
        real_time=args.real_time,
        time_step=args.time_step,
        dark_mode=not args.light_mode,
        enable_joint_sliders=False,
        enable_gripper_sliders=False,
        show_debug_gui=False,
        gripper_only=gripper_only,
        gripper_urdf_path=args.gripper_urdf,
        add_ground=True,
        attach_gripper=not gripper_only,
    )
    env.reset()
    window_title = "QArm Test Sim (Qt controls)"
    launch_qt_control_panel(env, include_gripper=True, window_title=window_title)


if __name__ == "__main__":
    main()
