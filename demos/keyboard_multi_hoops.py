"""Keyboard teleop with multiple segmented hoops (sliders + labels).

Run: ``python -m demos.gamepad_multi_hoops``
Set ``MODE = "hardware"`` + ``MIRROR_SIM_WHEN_HARDWARE = True`` to drive the
real arm while keeping the simulation/viewer in sync.
"""

from __future__ import annotations

import argparse
import math
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import NamedTuple

from api.factory import make_qarm
from common.qarm_base import QArmBase
from demos._shared import run_with_viewer

# ---- user settings ----
MODE = "hardware"  # change to "hardware" to drive the real arm
MIRROR_SIM_WHEN_HARDWARE = True  # keep the simulation/viewer running alongside hardware
USE_PANDA_VIEWER = True
USE_PYBULLET_GUI = False
STEP_S = 0.02
SHOW_SLIDERS = True
MODEL_DIR = Path(__file__).resolve().parent / "models"

HOOP_SEGMENT = MODEL_DIR / "hoop-segment.stl"
HOOP_COLLISION_SEGMENTS = {
    "mesh_path": HOOP_SEGMENT,
    "radius": 68.0 / 2.0,  # mm ring diameter -> 34 mm radius before scaling
    "yaw_step_deg": 29.9,
    "count": 12,
}
HOOP_MATERIAL = {
    # Very low frictions + softer contacts so gravity can tip the hoops over.
    "lateral_friction": 0.12,
    "rolling_friction": 0.0,
    "spinning_friction": 0.0,
    "restitution": 0.0,
    "contact_stiffness": 8.0e3,
    "contact_damping": 3.0e2,
}
HOOP_POSITIONS = [(0.0, -0.30, 0.08), (0.1, -0.35, 0.08), (-0.1, -0.40, 0.08), (0.23, -0.25, 0.08)]
HOOP_TILT_DEG = 3.0  # small roll to break perfect balance so gravity makes them fall
HOOP_COLOR = (0.15, 0.85, 0.3, 1.0)

JOINT_SPEEDS = {"yaw": 1.5, "shoulder": 1.5, "elbow": 1.5, "wrist": 1.5}
GRIPPER_OPEN_ANGLE = 0.0
GRIPPER_CLOSED_ANGLE = 0.55
GRIPPER_SPEED = 1.5  # rad/s when a bumper is held
GRIPPER_LOCKS = {"GRIPPER_JOINT1B": 0.8, "GRIPPER_JOINT2B": -0.8}


class AxisBinding(NamedTuple):
    axis: str
    negative_key: str
    positive_key: str
    label: str


class ButtonBinding(NamedTuple):
    key: str
    name: str
    label: str


KEYBOARD_AXIS_BINDINGS = [
    AxisBinding("yaw", "j", "l", "Base yaw"),
    AxisBinding("shoulder", "i", "k", "Shoulder pitch"),
    AxisBinding("elbow", "u", "o", "Elbow pitch"),
    AxisBinding("wrist", "n", "m", "Wrist pitch"),
]
KEYBOARD_BUTTON_BINDINGS = [
    ButtonBinding("z", "close", "Close gripper"),
    ButtonBinding("x", "open", "Open gripper"),
]


class KeyboardInputState:
    """Thread-safe storage for the current keyboard teleop inputs."""

    def __init__(self) -> None:
        self._axes: dict[str, float] = {name: 0.0 for name in JOINT_SPEEDS}
        self._axis_keys: dict[str, dict[str, float]] = {name: {} for name in JOINT_SPEEDS}
        self._buttons: dict[str, bool] = {"close": False, "open": False}
        self._lock = threading.Lock()

    def set_axis_key(self, axis: str, key: str, direction: float) -> None:
        with self._lock:
            axis_state = self._axis_keys.setdefault(axis, {})
            if direction == 0.0:
                axis_state.pop(key, None)
            else:
                axis_state[key] = direction
            total = sum(axis_state.values())
            self._axes[axis] = clamp(total, (-1.0, 1.0))

    def set_button(self, name: str, pressed: bool) -> None:
        with self._lock:
            self._buttons[name] = bool(pressed)

    def snapshot(self) -> tuple[dict[str, float], dict[str, bool]]:
        with self._lock:
            return dict(self._axes), dict(self._buttons)


def keyboard_help_text() -> str:
    lines = ["[KeyboardHoops] Keyboard controls (focus the Panda viewer window):"]
    for binding in KEYBOARD_AXIS_BINDINGS:
        left = binding.negative_key.upper()
        right = binding.positive_key.upper()
        lines.append(f"  {binding.label:<18}: {left} / {right}")
    close_key = next((b.key for b in KEYBOARD_BUTTON_BINDINGS if b.name == "close"), None)
    open_key = next((b.key for b in KEYBOARD_BUTTON_BINDINGS if b.name == "open"), None)
    if close_key and open_key:
        lines.append(f"  Gripper close/open : {close_key.upper()} / {open_key.upper()}")
    else:
        for button in KEYBOARD_BUTTON_BINDINGS:
            lines.append(f"  {button.label:<18}: {button.key.upper()}")
    lines.append("  Hold a key to keep the joint moving; release to stop.")
    return "\n".join(lines)


def bind_keyboard_teleop(viewer: object, keyboard_state: KeyboardInputState) -> None:
    """Attach Panda3D key events so we can drive the teleop thread."""

    def bind_axis(key_name: str, axis_name: str, direction: float) -> None:
        def _press() -> None:
            keyboard_state.set_axis_key(axis_name, key_name, direction)

        def _release() -> None:
            keyboard_state.set_axis_key(axis_name, key_name, 0.0)

        viewer.accept(key_name, _press)
        viewer.accept(f"{key_name}-repeat", _press)
        viewer.accept(f"{key_name}-up", _release)

    def bind_button(key_name: str, button_name: str) -> None:
        def _press() -> None:
            keyboard_state.set_button(button_name, True)

        def _release() -> None:
            keyboard_state.set_button(button_name, False)

        viewer.accept(key_name, _press)
        viewer.accept(f"{key_name}-repeat", _press)
        viewer.accept(f"{key_name}-up", _release)

    for axis in KEYBOARD_AXIS_BINDINGS:
        bind_axis(axis.negative_key, axis.axis, -1.0)
        bind_axis(axis.positive_key, axis.axis, 1.0)
    for button in KEYBOARD_BUTTON_BINDINGS:
        bind_button(button.key, button.name)


def clamp(val: float, bounds: tuple[float, float]) -> float:
    lo, hi = bounds
    return max(lo, min(hi, val))

def add_hoops(arm: QArmBase) -> list[int]:
    env = getattr(arm, "env", None)
    if env is None or not hasattr(env, "add_kinematic_object"):
        print("[KeyboardHoops] Simulator backend not available; skipping hoop spawn.")
        return []
    hoop_mesh = MODEL_DIR / "hoop.stl"
    if not hoop_mesh.exists() or not HOOP_SEGMENT.exists():
        print(f"[KeyboardHoops] Missing hoop meshes under {MODEL_DIR}.")
        return []
    ids: list[int] = []
    for pos in HOOP_POSITIONS:
        hoop_id = env.add_kinematic_object(
            mesh_path=hoop_mesh,
            position=pos,
            # Add a small roll tilt so gravity will tip the hoop off its edge.
            orientation_euler_deg=(HOOP_TILT_DEG, 0.0, 20.0),
            scale=0.001,
            collision_segments=HOOP_COLLISION_SEGMENTS,
            mass=0.1,
            force_convex_for_dynamic=True,
            rgba=HOOP_COLOR,
            **HOOP_MATERIAL,
        )
        ids.append(hoop_id)
    print(f"[KeyboardHoops] Spawned {len(ids)} hoops with collision segments.")
    return ids


def teleop_loop(arm: QArmBase, stop_event: threading.Event, keyboard_state: KeyboardInputState) -> None:
    print(keyboard_help_text())
    limits = [(-math.pi, math.pi)] * 4
    grip_target = GRIPPER_OPEN_ANGLE
    grip_bounds = (GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)
    while not stop_event.is_set():
        axes, buttons = keyboard_state.snapshot()
        q = arm.get_joint_positions()
        dt = STEP_S
        q[0] = clamp(q[0] + axes.get("yaw", 0.0) * JOINT_SPEEDS["yaw"] * dt, limits[0])
        q[1] = clamp(q[1] + axes.get("shoulder", 0.0) * JOINT_SPEEDS["shoulder"] * dt, limits[1])
        q[2] = clamp(q[2] + axes.get("elbow", 0.0) * JOINT_SPEEDS["elbow"] * dt, limits[2])
        q[3] = clamp(q[3] + axes.get("wrist", 0.0) * JOINT_SPEEDS["wrist"] * dt, limits[3])
        try:
            arm.set_joint_positions(q)
            closing = buttons.get("close", False)
            opening = buttons.get("open", False)
            if closing and not opening:
                grip_target = clamp(grip_target + GRIPPER_SPEED * dt, grip_bounds)
            elif opening and not closing:
                grip_target = clamp(grip_target - GRIPPER_SPEED * dt, grip_bounds)
            arm.set_gripper_positions([grip_target])
        except Exception as exc:
            print(f"[KeyboardHoops] Failed to send commands: {exc}")
            stop_event.set()
            break
        if stop_event.wait(STEP_S):
            break


def launch_viewer(arm: QArmBase, keyboard_state: KeyboardInputState | None = None) -> None:
    env = getattr(arm, "env", None)
    if env is None:
        print("[KeyboardHoops] No env attached; viewer unavailable.")
        return
    from sim.panda_viewer import PandaArmViewer, PhysicsBridge

    viewer_args = SimpleNamespace(
        time_step=env.time_step,
        hide_base=False,
        hide_accents=False,
        probe_base_collision=False,
        show_sliders=SHOW_SLIDERS,
        reload_meshes=False,
    )
    bridge = PhysicsBridge(time_step=env.time_step, env=env, reset=False)
    viewer = PandaArmViewer(bridge, viewer_args)
    if keyboard_state is not None:
        bind_keyboard_teleop(viewer, keyboard_state)
    viewer.run()


def lock_gripper_b_joints(env: object) -> None:
    """Force gripper B joints to the desired locked angles."""
    if env is None:
        return
    try:
        # Prefer the env helper if available (handles bookkeeping).
        locker = getattr(env, "_lock_joint_by_name", None)
        if callable(locker):
            locker(GRIPPER_LOCKS)
            return
    except Exception:
        pass
    try:
        import pybullet as p  # type: ignore
    except Exception:
        return
    try:
        for idx, name in enumerate(getattr(env, "joint_names", [])):
            if name not in GRIPPER_LOCKS:
                continue
            p.resetJointState(env.robot_id, idx, GRIPPER_LOCKS[name], physicsClientId=env.client)
    except Exception:
        return


def start_live_hoop_labels(arm: QArmBase, hoop_ids: list[int], stop_event: threading.Event) -> threading.Thread | None:
    env = getattr(arm, "env", None)
    if env is None or not hoop_ids:
        return None
    try:
        import pybullet as p  # type: ignore
    except Exception:
        return None

    label_ids: dict[int, int] = {}
    for i, hoop_id in enumerate(hoop_ids):
        try:
            label_ids[hoop_id] = env.add_point_label(
                name=f"Hoop {i+1}",
                position=(0.0, 0.0, 0.0),
                color=(0.9, 0.4, 0.1, 1.0),
                text_scale=0.028,
                marker_scale=0.05,
                show_coords=True,
            )
        except Exception:
            continue

    def _loop() -> None:
        while not stop_event.wait(0.1):
            for hoop_id, label_id in list(label_ids.items()):
                try:
                    pos, _ = p.getBasePositionAndOrientation(hoop_id, physicsClientId=env.client)
                    env.update_point_label(label_id, position=pos)
                except Exception:
                    continue

    t = threading.Thread(target=_loop, daemon=True)
    t.start()
    return t


def main() -> None:
    parser = argparse.ArgumentParser(description="QArm keyboard hoops demo")
    parser.add_argument("--mode", choices=["sim", "hardware"], default=MODE, help="QArm control backend")
    parser.add_argument("--no-viewer", action="store_true", help="Disable Panda viewer (headless)")
    parser.add_argument("--pybullet-gui", action="store_true", help="Force PyBullet GUI on")
    parser.add_argument("--mirror-sim", action="store_true", default=MIRROR_SIM_WHEN_HARDWARE, help="Mirror hardware commands into sim")
    args = parser.parse_args()

    mode = args.mode.lower()
    use_viewer = USE_PANDA_VIEWER and not args.no_viewer
    auto_step = not use_viewer
    mirror_sim = args.mirror_sim and mode == "hardware"
    gui = args.pybullet_gui or USE_PYBULLET_GUI

    arm = make_qarm(
        mode=mode,
        gui=gui,
        real_time=False,
        auto_step=auto_step,
        mirror_sim=mirror_sim,
    )
    print(f"[KeyboardHoops] Mode: {mode} (mirror sim: {'on' if mirror_sim else 'off'}) viewer={'on' if use_viewer else 'off'} gui={'pybullet' if gui else 'none'}")
    arm.home()
    lock_gripper_b_joints(getattr(arm, "env", None))
    hoop_ids = add_hoops(arm)
    stop_event = threading.Event()
    keyboard_state = KeyboardInputState()
    worker = threading.Thread(target=teleop_loop, args=(arm, stop_event, keyboard_state), daemon=True)
    worker.start()
    label_thread = start_live_hoop_labels(arm, hoop_ids, stop_event)
    try:
        if use_viewer:
            run_with_viewer(lambda: launch_viewer(arm, keyboard_state), lambda: stop_event.wait())
        else:
            print("[KeyboardHoops] Running headless teleop (keyboard input unavailable). Press Ctrl+C to stop.")
            while not stop_event.wait(1.0):
                pass
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        worker.join(timeout=1.0)
        if label_thread is not None:
            label_thread.join(timeout=1.0)
        try:
            arm.home()
        except Exception:
            pass


if __name__ == "__main__":
    main()
