"""
Convenience entry point for launching the QArm simulation environment (VSCode-friendly).

Run as:
    python -m sim.run_gui --gui --real-time --sliders
"""

from __future__ import annotations

import argparse
import time

from sim.env import QArmSimEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch the QArm simulation environment.")
    parser.add_argument("--gui", action="store_true", help="Enable PyBullet GUI (default headless).")
    parser.add_argument("--real-time", action="store_true", help="Let PyBullet step in real time.")
    parser.add_argument("--sliders", action="store_true", help="Expose joint sliders in the GUI.")
    parser.add_argument("--headless-steps", type=int, default=600, help="Steps to run when headless.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    env = QArmSimEnv(
        gui=args.gui,
        real_time=args.real_time,
        enable_joint_sliders=args.sliders,
    )
    env.reset()

    try:
        if args.gui:
            print("GUI running. Close window or Ctrl+C to exit.")
            while True:
                if args.sliders:
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
