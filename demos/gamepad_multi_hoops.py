"""Gamepad teleop with multiple segmented hoops (sliders + labels). Run: python -m demos.gamepad_multi_hoops"""

from __future__ import annotations

import math
import os
import threading
import time
from pathlib import Path
from types import SimpleNamespace

from api.factory import make_qarm
from common.qarm_base import QArmBase
from demos._shared import run_with_viewer

# ---- user settings ----
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
    # Higher frictions and damping help hoops settle upright instead of rolling to odd angles.
    "lateral_friction": 0.8,
    "rolling_friction": 0.2,
    "spinning_friction": 0.2,
    "restitution": 0.0,
    "contact_stiffness": 1.2e5,
    "contact_damping": 1.0e4,
}
HOOP_POSITIONS = [(0.0, -0.30, 0.08), (0.1, -0.35, 0.08), (-0.1, -0.40, 0.08), (0.23, -0.25, 0.08)]
HOOP_COLOR = (0.15, 0.85, 0.3, 1.0)

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
JOYSTICK_AXES = {"left_x": 0, "left_y": 1, "right_x": 2, "right_y": 3}
JOYSTICK_BUTTONS = {"left_bumper": 9, "right_bumper": 10}
GAMEPAD_DEADZONE = 0.1
JOINT_SPEEDS = {"yaw": 1.4, "shoulder": 1.0, "elbow": 1.2, "wrist": 1.0}
GRIPPER_OPEN_ANGLE = 0.0
GRIPPER_CLOSED_ANGLE = 0.55
GRIPPER_SPEED = 1.5  # rad/s when a bumper is held
GRIPPER_LOCKS = {"GRIPPER_JOINT1B": 0.8, "GRIPPER_JOINT2B": -0.8}

def clamp(val: float, bounds: tuple[float, float]) -> float:
    lo, hi = bounds
    return max(lo, min(hi, val))

def square_stick(x: float, y: float, deadzone: float) -> tuple[float, float]:
    def dz(v: float) -> float:
        return 0.0 if abs(v) < deadzone else v
    x, y = dz(x), dz(y)
    if x == 0.0 and y == 0.0:
        return 0.0, 0.0
    radius = min(1.0, math.hypot(x, y))
    max_component = max(abs(x), abs(y), 1e-6)
    scale = radius / max_component
    return clamp(x * scale, (-1.0, 1.0)), clamp(y * scale, (-1.0, 1.0))


class XboxController:
    def __init__(self, deadzone: float) -> None:
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError("pygame is required for gamepad control (pip install pygame).") from exc
        self.pygame = pygame
        self.deadzone = float(deadzone)
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick detected. Plug in your controller and try again.")
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self._zero = {name: 0.0 for name in JOYSTICK_AXES}
        self._last_buttons: dict[str, int] = {}
        self._calibrate_zero()

    def _calibrate_zero(self) -> None:
        samples = {name: [] for name in JOYSTICK_AXES}
        for _ in range(20):
            self.pygame.event.pump()
            for name, idx in JOYSTICK_AXES.items():
                samples[name].append(float(self.joy.get_axis(idx)))
            self.pygame.time.wait(5)
        for name, vals in samples.items():
            self._zero[name] = sum(vals) / len(vals) if vals else 0.0

    def read(self) -> tuple[float, float, float, float, dict[str, int]]:
        self.pygame.event.pump()
        lx = float(self.joy.get_axis(JOYSTICK_AXES["left_x"])) - self._zero["left_x"]
        ly = float(self.joy.get_axis(JOYSTICK_AXES["left_y"])) - self._zero["left_y"]
        rx = float(self.joy.get_axis(JOYSTICK_AXES["right_x"])) - self._zero["right_x"]
        ry = float(self.joy.get_axis(JOYSTICK_AXES["right_y"])) - self._zero["right_y"]
        lx, ly = square_stick(lx, -ly, GAMEPAD_DEADZONE)  # invert Y so forward is positive
        rx, ry = square_stick(rx, -ry, GAMEPAD_DEADZONE)
        buttons = {
            "left_bumper": int(self.joy.get_button(JOYSTICK_BUTTONS["left_bumper"])),
            "right_bumper": int(self.joy.get_button(JOYSTICK_BUTTONS["right_bumper"])),
        }
        self._last_buttons = buttons
        return lx, ly, rx, ry, buttons


def add_hoops(arm: QArmBase) -> list[int]:
    env = getattr(arm, "env", None)
    if env is None or not hasattr(env, "add_kinematic_object"):
        print("[GamepadHoops] Simulator backend not available; skipping hoop spawn.")
        return []
    hoop_mesh = MODEL_DIR / "hoop.stl"
    if not hoop_mesh.exists() or not HOOP_SEGMENT.exists():
        print(f"[GamepadHoops] Missing hoop meshes under {MODEL_DIR}.")
        return []
    ids: list[int] = []
    for pos in HOOP_POSITIONS:
        hoop_id = env.add_kinematic_object(
            mesh_path=hoop_mesh,
            position=pos,
            orientation_euler_deg=(0.0, 0.0, 20.0),
            scale=0.001,
            collision_segments=HOOP_COLLISION_SEGMENTS,
            mass=0.1,
            force_convex_for_dynamic=True,
            rgba=HOOP_COLOR,
            **HOOP_MATERIAL,
        )
        ids.append(hoop_id)
    print(f"[GamepadHoops] Spawned {len(ids)} hoops with collision segments.")
    return ids


def teleop_loop(arm: QArmBase, stop_event: threading.Event) -> None:
    try:
        pad = XboxController(deadzone=GAMEPAD_DEADZONE)
    except RuntimeError as exc:
        print(f"[GamepadHoops] {exc}")
        return
    print("[GamepadHoops] Left stick yaw/shoulder, right stick elbow/wrist, bumpers close/open gripper.")
    limits = [(-math.pi, math.pi)] * 4
    grip_target = GRIPPER_OPEN_ANGLE
    grip_bounds = (GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)
    while not stop_event.is_set():
        lx, ly, rx, ry, buttons = pad.read()
        q = arm.get_joint_positions()
        dt = STEP_S
        q[0] = clamp(q[0] + (-lx) * JOINT_SPEEDS["yaw"] * dt, limits[0])
        q[1] = clamp(q[1] + (-ly) * JOINT_SPEEDS["shoulder"] * dt, limits[1])
        q[2] = clamp(q[2] + (-ry) * JOINT_SPEEDS["elbow"] * dt, limits[2])
        q[3] = clamp(q[3] + (rx) * JOINT_SPEEDS["wrist"] * dt, limits[3])
        try:
            arm.set_joint_positions(q)
            closing = buttons.get("left_bumper")
            opening = buttons.get("right_bumper")
            if closing and not opening:
                grip_target = clamp(grip_target + GRIPPER_SPEED * dt, grip_bounds)
            elif opening and not closing:
                grip_target = clamp(grip_target - GRIPPER_SPEED * dt, grip_bounds)
            arm.set_gripper_positions([grip_target])
        except Exception as exc:
            print(f"[GamepadHoops] Failed to send commands: {exc}")
            stop_event.set()
            break
        if stop_event.wait(STEP_S):
            break


def launch_viewer(arm: QArmBase) -> None:
    env = getattr(arm, "env", None)
    if env is None:
        print("[GamepadHoops] No env attached; viewer unavailable.")
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
    PandaArmViewer(bridge, viewer_args).run()


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
    auto_step = not USE_PANDA_VIEWER
    arm = make_qarm(mode="sim", gui=USE_PYBULLET_GUI, real_time=False, auto_step=auto_step)
    arm.home()
    lock_gripper_b_joints(getattr(arm, "env", None))
    hoop_ids = add_hoops(arm)
    stop_event = threading.Event()
    worker = threading.Thread(target=teleop_loop, args=(arm, stop_event), daemon=True)
    worker.start()
    label_thread = start_live_hoop_labels(arm, hoop_ids, stop_event)
    try:
        if USE_PANDA_VIEWER:
            run_with_viewer(lambda: launch_viewer(arm), lambda: stop_event.wait())
        else:
            print("[GamepadHoops] Running headless teleop. Press Ctrl+C to stop.")
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
