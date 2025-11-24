"""
Launch the full QArm simulation with a Qt control panel (no PyBullet UI widgets).

Usage:
    python -m sim.actual_sim               # GUI + Qt controls
    python -m sim.actual_sim --attach-gripper
"""

from __future__ import annotations

import argparse
from pathlib import Path

from sim.env import QArmSimEnv
from sim.qt_panel import launch_qt_control_panel


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the main QArm sim with a Qt control panel.")
    parser.add_argument("--real-time", action="store_true", help="Let PyBullet step in real time.")
    parser.add_argument("--attach-gripper", action="store_true", help="Attach the qarm_gripper to the end effector.")
    parser.add_argument(
        "--gripper-urdf",
        type=Path,
        help="Override path to the gripper URDF (useful when iterating on the file).",
    )
    parser.add_argument("--time-step", type=float, default=1.0 / 120.0, help="Physics timestep (seconds).")
    parser.add_argument("--light-mode", action="store_true", help="Use a light PyBullet background.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    env = QArmSimEnv(
        gui=True,
        real_time=args.real_time,
        time_step=args.time_step,
        dark_mode=not args.light_mode,
        enable_joint_sliders=False,
        enable_gripper_sliders=False,
        show_debug_gui=False,
        attach_gripper=args.attach_gripper,
        gripper_urdf_path=args.gripper_urdf,
        add_ground=True,
    )
    env.reset()
    launch_qt_control_panel(env, include_gripper=args.attach_gripper, window_title="QArm Actual Sim (Qt controls)")


if __name__ == "__main__":
    main()
