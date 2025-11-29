"""
Scripted pick-and-place in plain Python.

What it shows:
- A small dictionary of poses you can edit without hunting through code.
- A simple sequence that opens/closes the gripper between waypoints.
- Prints every move so you see which joint set is being sent.

Run it (opens Panda3D viewer by default):
    python -m demos.pick_and_place
Tweak `POSES` and `SEQUENCE` below to match where your hoop and stand sit.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

from api.factory import make_qarm
from common.qarm_base import DEFAULT_JOINT_ORDER, QArmBase

MODE = "mirror"
USE_MIRROR_MODE = False  # set True to drive hardware while mirroring into the simulator
USE_PANDA_VIEWER = True
USE_PYBULLET_GUI = False
STEP_DELAY_S = 1.0

# Joint targets in `(yaw, shoulder, elbow, wrist)` order.
POSES: dict[str, tuple[float, float, float, float]] = {
    "home": (0.0, 0.55, 0.0, 0.0),
    "above_pick": (0.25, 0.35, -0.1, 0.0),
    "at_pick": (0.25, 0.6, -0.35, 0.0),
    "lift": (0.25, 0.35, -0.1, 0.0),
    "above_place": (-0.75, 0.45, 0.05, 0.2),
    "at_place": (-0.75, 0.65, -0.25, 0.2),
}

SEQUENCE: list[tuple[str, str]] = [
    ("Move above the hoop", "above_pick"),
    ("Lower to grab height", "at_pick"),
    ("Close gripper", "close"),
    ("Lift the hoop", "lift"),
    ("Slide over to the stand", "above_place"),
    ("Lower onto the stand", "at_place"),
    ("Open gripper", "open"),
    ("Return home", "home"),
]

GRIPPER_OPEN_ANGLE = 0.0
GRIPPER_CLOSED_ANGLE = 0.55


def go_to(arm: QArmBase, pose_name: str, wait: float) -> None:
    pose = POSES[pose_name]
    print(f"[PickPlace] {pose_name}: {pose}")
    arm.set_joint_positions(pose)
    time.sleep(wait)


def run_sequence(arm: QArmBase) -> None:
    print("[PickPlace] Joint order:", ", ".join(DEFAULT_JOINT_ORDER))
    for action, target in SEQUENCE:
        if target == "open":
            print("[PickPlace] Opening gripper")
            arm.set_gripper_position(GRIPPER_OPEN_ANGLE)
        elif target == "close":
            print("[PickPlace] Closing gripper")
            arm.set_gripper_position(GRIPPER_CLOSED_ANGLE)
        else:
            go_to(arm, target, STEP_DELAY_S)
        time.sleep(STEP_DELAY_S)


def main() -> None:
    auto_step = not USE_PANDA_VIEWER
    mode = MODE.lower()
    mirror_mode = USE_MIRROR_MODE and mode == "hardware"
    effective_mode = "hardware" if mirror_mode else mode
    arm = make_qarm(
        mode=effective_mode,
        gui=USE_PYBULLET_GUI,
        real_time=False,
        auto_step=auto_step,
        mirror_sim=mirror_mode,
    )
    mirror_status = "on" if mirror_mode else "off"
    print(f"[PickPlace] Connected (mode={mode}, mirror={mirror_status}); moving to home.")
    arm.home()
    time.sleep(STEP_DELAY_S)

    try:
        if USE_PANDA_VIEWER:
            import threading
            from sim.panda_viewer import PandaArmViewer, PhysicsBridge

            def launch_viewer() -> None:
                env = getattr(arm, "env", None)
                if env is None:
                    print("[PickPlace] Viewer unavailable (no sim env).")
                    return
                args = SimpleNamespace(
                    time_step=env.time_step,
                    hide_base=False,
                    hide_accents=False,
                    probe_base_collision=False,
                    show_sliders=False,
                    reload_meshes=False,
                )
                bridge = PhysicsBridge(time_step=env.time_step, env=env, reset=False)
                PandaArmViewer(bridge, args).run()

            motion = threading.Thread(target=run_sequence, args=(arm,), daemon=True)
            motion.start()
            launch_viewer()  # blocks until window closes
        else:
            run_sequence(arm)
    except KeyboardInterrupt:
        print("\n[PickPlace] Sequence interrupted.")
    finally:
        try:
            print("[PickPlace] Returning to home...")
            arm.home()
        except Exception:
            pass


if __name__ == "__main__":
    main()
