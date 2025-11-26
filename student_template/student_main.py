"""
Minimal student-facing entry point: drive the QArm in simulation via joint commands.

Quick API reminder (common to sim + hardware):
- `get_joint_positions()` -> list[float] ordered (yaw, shoulder, elbow, wrist)
- `set_joint_positions(q)` sends that list/tuple to the robot
- `home()` moves to a safe default pose

The sim is meant to be viewed in Panda3D (`python -m sim.panda_viewer`). The PyBullet
GUI is optional and best used for debugging joint values; this script launches Panda3D
by default (edit the toggles near the top to change that). Populate KINEMATIC_OBJECTS
to drop static meshes (position/orientation + mesh path) into the scene.

Run with:
    python -m student_template.student_main                      # opens Panda3D viewer
    # edit the defaults near the top of this file to change duration/step or disable the viewer
"""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from pathlib import Path

from api.factory import make_qarm
from common.qarm_base import DEFAULT_JOINT_ORDER, QArmBase

# Tweak these if you want different defaults.
DEFAULT_DURATION_S = 0.0
DEFAULT_STEP_S = 0.02
USE_PANDA_VIEWER = True
USE_PYBULLET_GUI = False
# Set True to show joint sliders inside the Panda viewer.
SHOW_JOINT_SLIDERS = False
# Set True to force Panda3D to reload STL meshes each launch (bypasses cache).
RELOAD_MESHES = False
# Fill this list with entries to drop static meshes into the scene.
# Example shape of an entry:
# {
#     "mesh_path": Path(__file__).parent / "blender_monkey.stl",
#     "position": (0.2, 0.0, 0.08),
#     "euler_deg": (0.0, 0.0, 45.0),  # roll, pitch, yaw in degrees
#     "scale": 0.5,
#     "mass": 0.1,  # optional: set >0 to let gravity act on the mesh
# }
KINEMATIC_OBJECTS: list[dict[str, object]] = []


def add_kinematic_objects(arm: QArmBase, objects: list[dict[str, object]]) -> None:
    """
    Convenience wrapper around the simulator's kinematic mesh helper.
    Pass a list of dicts shaped like the KINEMATIC_OBJECTS example above.
    """
    if not objects:
        return
    env = getattr(arm, "env", None)
    if env is None or not hasattr(env, "add_kinematic_object"):
        print("[Student] Current QArm backend does not support kinematic objects.")
        return
    for obj in objects:
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


def demo_motion(arm: QArmBase, duration: float, step_s: float, stop_event: threading.Event | None = None) -> None:
    """
    Walk through a repeating set of waypoints in joint space.

    Each waypoint is a list/tuple in the documented order:
    (yaw, shoulder, elbow, wrist) in radians.

    The sequence loops until `duration` elapses (<=0 means run forever) or a stop event is set.
    """

    joint_order = DEFAULT_JOINT_ORDER  # handy alias for printing / editing below
    waypoints = [
        ("Home (current pose)", arm.get_joint_positions()),
        ("Stretch forward", [0.0, -0.5, 0.7, 0.0]),
        ("Move over", [-1.0, -0.1, 0.7, 0.0]),
        ("Lift up", [0.0, -0.2, 0.3, 0.0]),
        ("Rotate wrist", [0.0, -0.2, 0.3, 0.8]),
    ]

    start = time.time()
    end_time = None if duration <= 0 else start + duration

    while True:
        for name, pose in waypoints:
            if stop_event is not None and stop_event.is_set():
                return
            if end_time is not None and time.time() >= end_time:
                return
            print(f"[Demo] Moving to: {name}  (order={joint_order})")
            arm.set_joint_positions(pose)
            time.sleep(1.0)
            time.sleep(step_s)  # brief dwell between waypoints


def main() -> None:
    # -------- Simple defaults (edit the constants above to change behavior) --------
    duration = DEFAULT_DURATION_S
    step = DEFAULT_STEP_S
    use_panda_viewer = USE_PANDA_VIEWER
    use_pybullet_gui = USE_PYBULLET_GUI

    # Use manual stepping; when Panda viewer is active it drives the simulation clock.
    real_time = False
    auto_step = not use_panda_viewer
    # Build the sim arm; turn on PyBullet's own GUI only if you flip the flag above.
    arm = make_qarm(mode="sim", gui=use_pybullet_gui, real_time=real_time, auto_step=auto_step)
    arm.home()
    add_kinematic_objects(arm, KINEMATIC_OBJECTS)
    time.sleep(0.1)

    # -------- Viewer wiring (optional) --------
    # stop_event lets the background motion thread bail out when the viewer closes.
    stop_event = threading.Event()
    motion_thread: threading.Thread | None = None

    def launch_viewer() -> None:
        from sim.panda_viewer import PandaArmViewer, PhysicsBridge

        # Reuse the same PyBullet world inside Panda3D so visuals and physics stay in sync.
        viewer_args = SimpleNamespace(
            time_step=arm.env.time_step,
            hide_base=False,
            hide_accents=False,
            probe_base_collision=False,
            show_sliders=SHOW_JOINT_SLIDERS,
            reload_meshes=RELOAD_MESHES,
        )
        physics = PhysicsBridge(
            time_step=arm.env.time_step,
            env=getattr(arm, "env", None),
            reset=False,
        )
        app = PandaArmViewer(physics, viewer_args)
        app.run()

    def run_with_viewer() -> threading.Thread:
        """Start motion in the background and block on the Panda viewer window."""
        thread = threading.Thread(target=demo_motion, args=(arm, duration, step, stop_event), daemon=True)
        thread.start()
        launch_viewer()  # blocks until the window is closed
        stop_event.set()
        return thread

    try:
        # -------- Motion loop --------
        if use_panda_viewer:
            motion_thread = run_with_viewer()
        else:
            demo_motion(arm, duration=duration, step_s=step, stop_event=stop_event)
    except KeyboardInterrupt:
        print("\nStopping demo...")
        stop_event.set()
    finally:
        if motion_thread is not None and motion_thread.is_alive():
            motion_thread.join(timeout=1.0)
        # Park the arm back at home for a clean exit.
        try:
            arm.home()
        except Exception:
            pass


if __name__ == "__main__":
    main()
