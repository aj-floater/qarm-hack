"""Launch the full QArm simulation using PyBullet's built-in GUI controls."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from sim.env import QArmSimEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the main QArm sim with PyBullet debug sliders.")
    parser.add_argument("--real-time", action="store_true", help="Let PyBullet step in real time.")
    parser.add_argument("--time-step", type=float, default=1.0 / 120.0, help="Physics timestep (seconds).")
    parser.add_argument("--light-mode", action="store_true", help="Use a light PyBullet background.")
    parser.add_argument("--base-mesh", type=str, default=None, help="Path to pine base mesh for physics/visuals.")
    parser.add_argument("--base-collision-mesh", type=str, default=None, help="Collision mesh for the base (optional).")
    parser.add_argument(
        "--base-scale",
        type=float,
        nargs="+",
        default=[1.0],
        help="Uniform or XYZ scale for the base mesh (one value or three values).",
    )
    parser.add_argument("--base-friction", type=float, default=0.8, help="Lateral friction for the base mesh.")
    parser.add_argument("--base-restitution", type=float, default=0.0, help="Restitution for the base mesh.")
    parser.add_argument("--base-yaw", type=float, default=180.0, help="Rotation (degrees about Z) to apply to the base.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    models_dir = Path(__file__).resolve().parent / "models"
    default_base = models_dir / "pinebase.stl"
    default_base_collision = models_dir / "pinebase_collision.stl"
    scale_arg = args.base_scale
    if len(scale_arg) == 1:
        base_scale: float | list[float] = scale_arg[0]
    elif len(scale_arg) == 3:
        base_scale = scale_arg
    else:
        raise SystemExit("Base scale must be one value (uniform) or three values (XYZ).")

    if args.base_mesh:
        base_mesh = Path(args.base_mesh).expanduser().resolve()
    elif default_base.exists():
        base_mesh = default_base
    else:
        base_mesh = None
    if args.base_collision_mesh:
        base_collision = Path(args.base_collision_mesh).expanduser().resolve()
    elif base_mesh and default_base_collision.exists():
        base_collision = default_base_collision
    else:
        base_collision = base_mesh

    env = QArmSimEnv(
        gui=True,
        real_time=args.real_time,
        time_step=args.time_step,
        dark_mode=not args.light_mode,
        enable_joint_sliders=True,
        show_debug_gui=True,
        add_ground=base_mesh is None,
        base_mesh_path=base_mesh,
        base_collision_mesh_path=base_collision,
        base_mesh_scale=base_scale,
        base_yaw_deg=args.base_yaw,
        base_friction=args.base_friction,
        base_restitution=args.base_restitution,
    )
    env.reset()

    print("Running with PyBullet debug sliders. Close the PyBullet window or Ctrl+C to exit.")
    try:
        while True:
            if env.enable_joint_sliders:
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
