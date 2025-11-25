"""
Copy of student_main focused on dropping a kinematic mesh into the sim.

Run with:
    python -m student_template.student_object
"""

from __future__ import annotations

import time
from pathlib import Path
from types import SimpleNamespace

from api.factory import make_qarm
from common.qarm_base import QArmBase

MODEL_DIR = Path(__file__).parent / "models"

# Viewer toggles match student_main defaults.
USE_PANDA_VIEWER = True
USE_PYBULLET_GUI = False
# Set True to show joint sliders inside the Panda viewer.
SHOW_JOINT_SLIDERS = True

# Preload the bundled Blender monkey so students see a static mesh instantly.
KINEMATIC_OBJECTS: list[dict[str, object]] = [
    {
        "mesh_path": MODEL_DIR / "hoop.stl",
        "position": (0.0, -0.3, 0.08),
        "euler_deg": (0.0, 0.0, 45.0),  # roll, pitch, yaw in degrees
        "scale": 0.001,
        "mass": 0.1,
        "force_convex_for_dynamic": True,
        "rgba": (0.1, 0.9, 0.1, 1.0),  # bright green hoop
    },
    {
        "mesh_path": MODEL_DIR / "blender_monkey.stl",
        "position": (0.2, -0.3, 0.08),
        "euler_deg": (0.0, 0.0, 45.0),
        "scale": 0.05,
        "mass": 0.5,
        "force_convex_for_dynamic": True,
        "rgba": (0.85, 0.25, 0.25, 1.0),  # red monkey
    },
    {
        "mesh_path": MODEL_DIR / "dog.STL",
        "position": (0.2, -0.25, 0.08),
        "euler_deg": (0.0, 0.0, -15.0),
        "scale": 0.001,
        "mass": 0.5,
        "force_convex_for_dynamic": True,
        "rgba": (0.25, 0.5, 0.95, 1.0),  # blue dog
    },
    {
        "mesh_path": MODEL_DIR / "head.stl",
        "position": (0.0, -0.5, 0.08),
        "euler_deg": (0.0, 0.0, 90.0),
        "scale": 0.003,
        "mass": 0.5,
        "force_convex_for_dynamic": True,
        "rgba": (0.95, 0.8, 0.2, 1.0),  # yellow head
    },
]


def add_kinematic_objects(arm: QArmBase, objects: list[dict[str, object]]) -> None:
    """
    Convenience wrapper around the simulator's kinematic mesh helper.
    Pass a list of dicts shaped like KINEMATIC_OBJECTS above.
    """
    if not objects:
        return
    env = getattr(arm, "env", None)
    if env is None or not hasattr(env, "add_kinematic_object"):
        print("[Student] Current QArm backend does not support kinematic objects.")
        return
    for obj in objects:
        # Push student-provided values with safe defaults for everything else.
        body_id = env.add_kinematic_object(
            mesh_path=obj["mesh_path"],
            position=obj.get("position", (0.0, 0.0, 0.0)),
            scale=obj.get("scale", 1.0),
            collision_scale=obj.get("collision_scale"),
            rgba=obj.get("rgba"),
            mass=obj.get("mass", 0.0),
            force_convex_for_dynamic=obj.get("force_convex_for_dynamic", True),
            orientation_quat_xyzw=obj.get("quat_xyzw"),
            orientation_euler_deg=obj.get("euler_deg"),
        )
        print(f"[Student] Added kinematic mesh {obj['mesh_path']} (body_id={body_id})")


def main() -> None:
    use_panda_viewer = USE_PANDA_VIEWER
    use_pybullet_gui = USE_PYBULLET_GUI

    # Keep PyBullet stepping driven by the Panda viewer.
    real_time = False
    auto_step = not use_panda_viewer

    arm = make_qarm(mode="sim", gui=use_pybullet_gui, real_time=real_time, auto_step=auto_step)
    arm.home()
    add_kinematic_objects(arm, KINEMATIC_OBJECTS)
    time.sleep(0.1)

    def launch_viewer() -> None:
        from sim.panda_viewer import PandaArmViewer, PhysicsBridge

        viewer_args = SimpleNamespace(
            time_step=arm.env.time_step,
            hide_base=False,
            hide_accents=False,
            probe_base_collision=False,
            show_sliders=SHOW_JOINT_SLIDERS,
        )
        physics = PhysicsBridge(
            time_step=arm.env.time_step,
            env=getattr(arm, "env", None),
            reset=False,
        )
        app = PandaArmViewer(physics, viewer_args)
        app.run()

    try:
        if use_panda_viewer:
            launch_viewer()  # blocks until closed
        else:
            print("[Student] Kinematic objects loaded. Press Ctrl+C to exit.")
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping kinematic object demo...")
    finally:
        try:
            arm.home()
        except Exception:
            pass


if __name__ == "__main__":
    main()
