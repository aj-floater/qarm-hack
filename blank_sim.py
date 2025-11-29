"""
Friendly starter template for the QArm simulator.

Use this file as your sandbox: edit one section at a time, read the comments,
and try things out as you learn.

Run it:
    python -m demos.blank_sim
"""

from __future__ import annotations

import time
from types import SimpleNamespace

from api.factory import make_qarm
from common.qarm_base import DEFAULT_JOINT_ORDER, QArmBase
from demos._shared import run_with_viewer

# ---------------------------------------------------------------------------
# Quick knobs to tweak the experience
# ---------------------------------------------------------------------------
MODE = "mirror"  # keep "sim" until you have hardware; use "mirror" for hardware+sim
USE_PANDA_VIEWER = True  # Panda3D window that shows the arm; set False for console only
USE_PYBULLET_GUI = False  # Bullet's debug sliders (rarely needed)
TIME_STEP = 1.0 / 240.0
HEADLESS_SECONDS = 10.0  # how long to run when no viewer is open
SHOW_VIEWER_SLIDERS = False
RELOAD_MESHES = False


def _launch_viewer(arm: QArmBase) -> None:
    """Open the Panda3D viewer. Beginners: just leave this alone."""
    env = getattr(arm, "env", None)
    if env is None:
        print("[Blank] No simulator attached; viewer cannot start.")
        return

    from sim.panda_viewer import PandaArmViewer, PhysicsBridge

    viewer_args = SimpleNamespace(
        time_step=env.time_step,
        hide_base=False,
        hide_accents=False,
        probe_base_collision=False,
        show_sliders=SHOW_VIEWER_SLIDERS,
        reload_meshes=RELOAD_MESHES,
    )
    bridge = PhysicsBridge(time_step=env.time_step, env=env, reset=False)
    PandaArmViewer(bridge, viewer_args).run()


def _headless_spin(env: object) -> None:
    """Keep physics running without graphics (useful for scripts/tests)."""
    if not hasattr(env, "step") or not hasattr(env, "time_step"):
        print("[Blank] No environment to step. Enable the simulator first.")
        return
    steps = int(HEADLESS_SECONDS / env.time_step)
    print(f"[Blank] Stepping headless for {HEADLESS_SECONDS:.1f}s ({steps} steps)...")
    for _ in range(steps):
        env.step()
        time.sleep(env.time_step)
    print("[Blank] Done stepping; press Ctrl+C to exit or start scripting moves.")


def main() -> None:
    """Entry point. Follow the comments below to add your own code."""
    mode = MODE.lower()
    mirror_mode = mode == "mirror"
    effective_mode = "hardware" if mirror_mode else mode
    auto_step = not USE_PANDA_VIEWER  # viewer updates the sim when it's open
    print(f"[Blank] Connecting to QArm in {mode} mode with joint order {DEFAULT_JOINT_ORDER}")
    arm = make_qarm(
        mode=effective_mode,
        gui=USE_PYBULLET_GUI,
        real_time=False,
        time_step=TIME_STEP,
        auto_step=auto_step,
        mirror_sim=mirror_mode,
    )

    env = getattr(arm, "env", None)
    if env is not None:
        env.reset()  # start from a zeroed pose

    try:
        if USE_PANDA_VIEWER:
            print("[Blank] Viewer opening. Running your script while the window is visible.")
            # Use a worker thread so `student_script` runs at the same time as the viewer.
            # Threads let one part of your program keep working (running commands) while
            # another part (the Panda window) stays responsive on the main thread.
            import threading

            worker = threading.Thread(target=student_script, args=(arm,), daemon=True)
            worker.start()
            _launch_viewer(arm)
            worker.join()
        elif env is not None:
            student_script(arm)
            _headless_spin(env)
        else:
            print("[Blank] No simulator to run; check MODE.")
    except KeyboardInterrupt:
        print("\n[Blank] Stopping minimal sim.")
    finally:
        try:
            arm.home()
        except Exception:
            pass
        disconnect = getattr(arm, "disconnect", None)
        if callable(disconnect):
            try:
                disconnect()
            except Exception:
                pass


def student_script(arm: QArmBase) -> None:
    """
    BEGINNER PLAYGROUND: put your experiments here.

    This function runs after the viewer is on-screen (when enabled), so you can see everything.
    Uncomment one idea at a time or replace them with your own.
    """

    time.sleep(4.0)  # wait a moment for things to settle

    # --- Example 1: print the current joint angles (yaw, shoulder, elbow, wrist)
    # print("Current joints:", arm.get_joint_positions())

    # --- Example 2: nudge the elbow slightly, then print the new values
    # joints = arm.get_joint_positions()
    # joints[2] += 0.5  # index 2 is the elbow
    # arm.set_joint_positions(joints)
    # print("Moved joints:", arm.get_joint_positions())

    # --- Example 3: alternate between two poses a few times
    # pose_a = [0.0, 0.3, -0.3, 0.0]
    # pose_b = [0.0, 0.6, -0.6, 0.0]
    # for i in range(5):
    #     arm.set_joint_positions(pose_a)
    #     time.sleep(0.5)
    #     arm.set_joint_positions(pose_b)
    #     time.sleep(0.5)

    # Remove the `pass` when you start adding your own code.
    pass


if __name__ == "__main__":
    main()
