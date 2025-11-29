"""
Tiny REPL to nudge the arm by hand.

What it shows:
- How to read the current joint angles and send small nudges.
- Quick buttons for opening/closing the gripper and returning home.
- No extra dependencies beyond the base simulator.

Run it (opens Panda3D viewer by default):
    python -m demos.keyboard_control
Press Enter on an empty line to quit.
"""

from __future__ import annotations

import time

from types import SimpleNamespace

from api.factory import make_qarm
from common.qarm_base import DEFAULT_JOINT_ORDER, QArmBase
from demos._shared import run_with_viewer

MODE = "mirror"
USE_PANDA_VIEWER = True
USE_PYBULLET_GUI = False
DELTA_RAD = 0.1  # ~5.7 degrees per tap

JOINT_INDEX = {name: i for i, name in enumerate(DEFAULT_JOINT_ORDER)}
GRIPPER_OPEN_ANGLE = 0.0
GRIPPER_CLOSED_ANGLE = 0.55

COMMANDS: dict[str, tuple[str, float]] = {
    "q": ("yaw", +DELTA_RAD),
    "a": ("yaw", -DELTA_RAD),
    "w": ("shoulder", +DELTA_RAD),
    "s": ("shoulder", -DELTA_RAD),
    "e": ("elbow", +DELTA_RAD),
    "d": ("elbow", -DELTA_RAD),
    "r": ("wrist", +DELTA_RAD),
    "f": ("wrist", -DELTA_RAD),
    "o": ("gripper", +1.0),  # open
    "c": ("gripper", -1.0),  # close
    "h": ("home", 0.0),
    "p": ("print", 0.0),
}


def print_help() -> None:
    print("\n[Keyboard] Controls (joint order: %s)" % ", ".join(DEFAULT_JOINT_ORDER))
    print("  q/a : yaw +/-")
    print("  w/s : shoulder +/-")
    print("  e/d : elbow +/-")
    print("  r/f : wrist +/-")
    print("  o/c : open / close gripper")
    print("  h   : return to home")
    print("  p   : print joint angles")
    print("  Enter with no input quits\n")


def nudge_joint(arm: QArmBase, joint: str, delta: float) -> None:
    q = arm.get_joint_positions()
    q[JOINT_INDEX[joint]] += delta
    arm.set_joint_positions(q)


def main() -> None:
    auto_step = not USE_PANDA_VIEWER
    arm = make_qarm(
        mode=MODE,
        gui=USE_PYBULLET_GUI,
        real_time=False,
        auto_step=auto_step,
    )
    arm.home()
    print_help()

    stop_event = None
    input_thread = None

    def control_loop() -> None:
        try:
            while True:
                cmd = input("[Keyboard] Command: ").strip().lower()
                if not cmd:
                    break
                action = COMMANDS.get(cmd)
                if action is None:
                    print("[Keyboard] Unknown key. Try again.")
                    print_help()
                    continue
                target, value = action
                if target == "gripper":
                    if value > 0:
                        arm.set_gripper_position(GRIPPER_OPEN_ANGLE)
                    else:
                        arm.set_gripper_position(GRIPPER_CLOSED_ANGLE)
                elif target == "home":
                    arm.home()
                elif target == "print":
                    print("[Keyboard] Current joints:", arm.get_joint_positions())
                else:
                    nudge_joint(arm, target, value)
                time.sleep(0.05)
        except KeyboardInterrupt:
            return

    if USE_PANDA_VIEWER:
        from sim.panda_viewer import PandaArmViewer, PhysicsBridge

        def launch_viewer() -> None:
            env = getattr(arm, "env", None)
            if env is None:
                print("[Keyboard] Viewer unavailable (no sim env).")
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

        run_with_viewer(launch_viewer, control_loop)
    else:
        control_loop()
    try:
        arm.home()
    except Exception:
        pass


if __name__ == "__main__":
    main()
