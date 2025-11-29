"""
Multi-hoop demo that reuses the hoop segment mesh for collisions.

- Spawns ten hoop meshes arranged around a 68 mm diameter circle.
- Each hoop builds a collision ring by tiling the hoop-segment mesh
  at 29.9 degree intervals along a 68 mm diameter circle (matching the model).
- Panda3D viewer opens by default so you can see all hoops at once.

Run it:
    python -m demos.multi_hoop_segments
"""

from __future__ import annotations

import time
from pathlib import Path
from types import SimpleNamespace
import math

from api.factory import make_qarm
from common.qarm_base import QArmBase

MODE = "sim"
USE_PANDA_VIEWER = True
USE_PYBULLET_GUI = False
TIME_STEP = 1.0 / 240.0
MODEL_DIR = Path(__file__).resolve().parent / "models"

HOOP_MESH = MODEL_DIR / "hoop.stl"
HOOP_SEGMENT = MODEL_DIR / "hoop-segment.stl"
RING_DIAMETER_MM = 68.0
SEGMENT_SPACING_DEG = 29.9
HOOP_COUNT = 10
HOOP_PLACEMENT_STEP_DEG = 29.9
HOOP_CIRCLE_CENTER = (0.0, -0.35)
HOOP_BASE_HEIGHT = 0.08
HOOP_CIRCLE_RADIUS_M = (RING_DIAMETER_MM / 2.0) / 1000.0
COLOR_PALETTE = [
    (0.1, 0.8, 0.2, 1.0),
    (0.9, 0.4, 0.1, 1.0),
    (0.2, 0.4, 0.9, 1.0),
    (0.8, 0.2, 0.8, 1.0),
    (0.9, 0.8, 0.2, 1.0),
]


def build_hoop_layout() -> list[dict[str, object]]:
    layout: list[dict[str, object]] = []
    for idx in range(HOOP_COUNT):
        angle_deg = idx * HOOP_PLACEMENT_STEP_DEG
        angle_rad = math.radians(angle_deg)
        x = HOOP_CIRCLE_CENTER[0] + HOOP_CIRCLE_RADIUS_M * math.cos(angle_rad)
        y = HOOP_CIRCLE_CENTER[1] + HOOP_CIRCLE_RADIUS_M * math.sin(angle_rad)
        layout.append(
            {
                "position": (x, y, HOOP_BASE_HEIGHT),
                "orientation_euler_deg": (0.0, 0.0, angle_deg),
                "rgba": COLOR_PALETTE[idx % len(COLOR_PALETTE)],
            }
        )
    return layout


HOOP_LAYOUT = build_hoop_layout()


def collision_segments() -> dict[str, object]:
    """Build a collision ring definition using the hoop-segment mesh."""
    segment_count = int(round(360.0 / SEGMENT_SPACING_DEG))
    return {
        "mesh_path": HOOP_SEGMENT,
        "radius": RING_DIAMETER_MM / 2.0,
        "yaw_step_deg": SEGMENT_SPACING_DEG,
        "count": segment_count,
    }


def add_multiple_hoops(arm: QArmBase) -> None:
    env = getattr(arm, "env", None)
    if env is None or not hasattr(env, "add_kinematic_object"):
        print("[MultiHoop] Simulator backend not available; skipping hoop spawn.")
        return
    if not HOOP_MESH.exists() or not HOOP_SEGMENT.exists():
        print(f"[MultiHoop] Missing hoop meshes under {MODEL_DIR}.")
        return

    for idx, spec in enumerate(HOOP_LAYOUT, start=1):
        env.add_kinematic_object(
            mesh_path=HOOP_MESH,
            position=spec["position"],
            orientation_euler_deg=spec["orientation_euler_deg"],
            scale=0.001,
            collision_segments=collision_segments(),
            mass=0.1,
            force_convex_for_dynamic=True,
            rgba=spec["rgba"],
            lateral_friction=1.0,
            rolling_friction=0.05,
            spinning_friction=0.05,
            restitution=0.0,
        )
        print(f"[MultiHoop] Spawned hoop #{idx} with collision segments.")


def run_viewer(arm: QArmBase) -> None:
    env = getattr(arm, "env", None)
    if env is None:
        print("[MultiHoop] No env attached; viewer unavailable.")
        return

    from sim.panda_viewer import PandaArmViewer, PhysicsBridge

    viewer_args = SimpleNamespace(
        time_step=env.time_step,
        hide_base=False,
        hide_accents=False,
        probe_base_collision=False,
        show_sliders=False,
        reload_meshes=False,
    )
    bridge = PhysicsBridge(time_step=env.time_step, env=env, reset=False)
    PandaArmViewer(bridge, viewer_args).run()


def main() -> None:
    auto_step = not USE_PANDA_VIEWER
    arm = make_qarm(
        mode=MODE,
        gui=USE_PYBULLET_GUI,
        real_time=False,
        time_step=TIME_STEP,
        auto_step=auto_step,
    )
    arm.home()
    add_multiple_hoops(arm)

    try:
        if USE_PANDA_VIEWER:
            run_viewer(arm)
        else:
            print("[MultiHoop] Hoops loaded. Press Ctrl+C to quit.")
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[MultiHoop] Exiting demo.")
    finally:
        try:
            arm.home()
        except Exception:
            pass


if __name__ == "__main__":
    main()
