"""
Copy of student_object.py with Xbox gamepad teleop added.

Run with:
    python -m student_template.student_gamepad_object

Left stick: yaw (X) + shoulder (Y)
Right stick: gripper 1A/2A (X) + elbow (Y)
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

from api.factory import make_qarm
from common.qarm_base import QArmBase

MODEL_DIR = Path(__file__).parent / "models"

# Viewer toggles match student_object defaults.
USE_PANDA_VIEWER = True
USE_PYBULLET_GUI = False
# Set True to show joint sliders inside the Panda viewer.
SHOW_JOINT_SLIDERS = True
# Set True to force Panda3D to reload STL meshes each launch (bypasses cache).
RELOAD_MESHES = False
# Step size for gamepad polling / position updates when not using Panda.
STEP_S = 0.02

# Preload multiple hoops (rings) so you have several to pick/place.
KINEMATIC_OBJECTS: list[dict[str, object]] = []
HOOP_COLLISION = MODEL_DIR / "hoop-collision.stl"
for i, offset in enumerate(
    [
        (0.0, -0.30, 0.08),
        (0.1, -0.35, 0.08),
        (-0.1, -0.40, 0.08),
        (0.05, -0.25, 0.08),
        (-0.05, -0.32, 0.08),
    ]
):
    hue = 0.1 + 0.15 * i
    color = (0.1 + 0.1 * i, 0.8 - 0.1 * i, 0.3 + 0.05 * i, 1.0)
    KINEMATIC_OBJECTS.append(
        {
            "mesh_path": MODEL_DIR / "hoop.stl",
            "position": offset,
            "euler_deg": (0.0, 0.0, 20.0 * i),  # slight spin per hoop
            "scale": 0.001,
            "mass": 0.1,
            "force_convex_for_dynamic": True,
            "collision_mesh_path": HOOP_COLLISION,
            "rgba": color,
        }
    )

# Controller + teleop defaults.
USE_GAMEPAD_CONTROL = True
GAMEPAD_DEADZONE = 0.1
# Enable to log raw/processed stick/button values and commanded targets periodically.
GAMEPAD_DEBUG = False
GAMEPAD_DEBUG_INTERVAL_S = 0.5
# When True, dump every button state each debug interval.
GAMEPAD_DEBUG_ALL_BUTTONS = False
# SDL can run headless for joystick polling so we do not clash with Panda3D's window.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
# Axis indices follow the common Xbox layout seen by pygame/SDL (adjust if yours differs).
JOYSTICK_AXES = {
    "left_x": 0,
    "left_y": 1,
    # On this controller: axis 2 = right stick X, axis 3 = right stick Y (from debug).
    "right_x": 2,
    "right_y": 3,
}
# Button indices (default Xbox layout: LB=4, RB=5).
JOYSTICK_BUTTONS = {
    "left_bumper": 9,
    "right_bumper": 10,
}
# Map stick extremes directly to joint angles (±360 deg here; clamped to URDF limits).
JOINT_TARGET_RANGE_RAD = {
    "yaw": math.radians(360.0),
    "shoulder": math.radians(360.0),
    "elbow": math.radians(360.0),
}
# Optional per-joint overrides for min/max angles (radians). Leave empty to use URDF limits.
JOINT_LIMIT_OVERRIDES: dict[str, tuple[float, float]] = {
    "yaw": (math.radians(-120.0), math.radians(30.0)),  # -120° to +30°
    "shoulder": (math.radians(20), math.radians(90.0)),  # 0° to +90°
    "gripper_joint1a": (math.radians(10.0), math.radians(-60.0)),  # -20° to +60°
    "gripper_joint2a": (math.radians(-10.0), math.radians(60.0)),  # -20° to +60°
}
JOINT_SPEED_RAD_PER_S = {
    "yaw": 1.4,
    "shoulder": 1.0,
    "elbow": 1.2,
}
GRIPPER_TARGET_RANGE_RAD = math.radians(120.0)  # symmetric +/- clamp, inverted on one finger to close
MAX_GRIPPER_RANGE_RAD = 1.2  # safety clamp if limits unavailable.
GRIPPER_SPEED_RAD_PER_S = 2.0  # for bumper-based incremental control


def clamp(value: float, limits: tuple[float, float] | None) -> float:
    """Clamp value to optional (low, high) bounds."""
    if limits is None:
        return value
    low, high = limits
    return max(low, min(high, value))


def square_stick(x: float, y: float, deadzone: float) -> tuple[float, float]:
    """
    Map a circular stick to a square so diagonals still reach full ±1 per axis.
    Keeps radial magnitude while expanding toward the square boundary.
    """

    def apply_deadzone(v: float) -> float:
        return 0.0 if abs(v) < deadzone else v

    x = apply_deadzone(x)
    y = apply_deadzone(y)
    if x == 0.0 and y == 0.0:
        return 0.0, 0.0
    radius = min(1.0, math.hypot(x, y))
    max_component = max(abs(x), abs(y), 1e-6)
    scale = radius / max_component
    x_sq = clamp(x * scale, (-1.0, 1.0))
    y_sq = clamp(y * scale, (-1.0, 1.0))
    return x_sq, y_sq


@dataclass
class StickAxes:
    left_x: float
    left_y: float
    right_x: float
    right_y: float


class XboxController:
    """Lightweight pygame-backed reader for an Xbox-style gamepad."""

    def __init__(self, deadzone: float, axis_map: dict[str, int]) -> None:
        try:
            import pygame
        except ImportError as exc:  # pragma: no cover - import guard
            raise RuntimeError("pygame is required for gamepad control (pip install pygame).") from exc

        self.pygame = pygame
        self.deadzone = float(deadzone)
        self.axis_map = dict(axis_map)
        self.button_map = dict(JOYSTICK_BUTTONS)
        self.zero_offsets: dict[str, float] = {}
        self._last_buttons: dict[str, int] = {}
        self._all_buttons: list[int] = []
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick detected. Plug in your Xbox controller and try again.")
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"[Gamepad] Connected to: {self.joystick.get_name()}")
        self.axis_count = self.joystick.get_numaxes()
        self.button_count = self.joystick.get_numbuttons()
        self._calibrate_zero()

    def _calibrate_zero(self) -> None:
        """Capture a few samples to center any stick bias so elbow/yaw start near 0."""
        samples: dict[str, list[float]] = {name: [] for name in self.axis_map}
        for _ in range(20):
            self.pygame.event.pump()
            for name in samples:
                samples[name].append(self._raw_axis(name))
            self.pygame.time.wait(5)
        for name, vals in samples.items():
            self.zero_offsets[name] = sum(vals) / len(vals) if vals else 0.0
        print("[Gamepad] Zero offsets:", {k: round(v, 3) for k, v in self.zero_offsets.items()})

    def _raw_axis(self, name: str) -> float:
        idx = self.axis_map.get(name, -1)
        if idx < 0 or idx >= self.joystick.get_numaxes():
            return 0.0
        return float(self.joystick.get_axis(idx))

    def _axis(self, name: str) -> float:
        raw = self._raw_axis(name)
        centered = raw - self.zero_offsets.get(name, 0.0)
        return clamp(centered, (-1.0, 1.0))

    def _button(self, name: str) -> int:
        idx = self.button_map.get(name, -1)
        if idx < 0 or idx >= self.button_count:
            return 0
        try:
            return int(self.joystick.get_button(idx))
        except Exception:
            return 0

    def read(self) -> StickAxes:
        """Return square-bounded axes in the range [-1, 1]."""
        self.pygame.event.pump()
        self._all_buttons = [int(self.joystick.get_button(i)) for i in range(self.button_count)]
        raw = {
            "left_x": self._raw_axis("left_x"),
            "left_y": self._raw_axis("left_y"),
            "right_x": self._raw_axis("right_x"),
            "right_y": self._raw_axis("right_y"),
        }
        centered = {
            name: raw[name] - self.zero_offsets.get(name, 0.0)
            for name in raw
        }
        lx, ly = centered["left_x"], -centered["left_y"]  # invert Y so forward is positive
        rx, ry = centered["right_x"], -centered["right_y"]
        lx_sq, ly_sq = square_stick(lx, ly, self.deadzone)
        rx_sq, ry_sq = square_stick(rx, ry, self.deadzone)
        self._last_buttons = {
            "left_bumper": self._button("left_bumper"),
            "right_bumper": self._button("right_bumper"),
        }
        return StickAxes(left_x=lx_sq, left_y=ly_sq, right_x=rx_sq, right_y=ry_sq)

    def gripper_input(self) -> float | None:
        """Return gripper input from bumpers: left=-1 (close), right=+1 (open)."""
        lb = self._last_buttons.get("left_bumper", 0)
        rb = self._last_buttons.get("right_bumper", 0)
        if lb and not rb:
            return -1.0
        if rb and not lb:
            return 1.0
        return 0.0


class GamepadTeleop:
    """Map stick motion to joint velocity commands."""

    def __init__(self, arm: QArmBase, controller: XboxController, step_s: float) -> None:
        self.arm = arm
        self.controller = controller
        self.step_s = float(step_s)
        self.name_to_idx = {name.lower(): i for i, name in enumerate(getattr(arm, "joint_names", []))}
        self.limits = self._fetch_joint_limits()
        self.gripper_indices = self._select_gripper_indices()
        self.gripper_signs = self._gripper_signs()

    def _fetch_joint_limits(self) -> dict[int, tuple[float, float]]:
        """Pull joint limits from the sim if available."""
        env = getattr(self.arm, "env", None)
        order = list(getattr(self.arm, "joint_order", []))
        limits: dict[int, tuple[float, float]] = {}
        if env is None:
            return limits
        try:
            import pybullet as p  # type: ignore
        except Exception:
            return limits
        for pos, idx in enumerate(order):
            try:
                info = p.getJointInfo(env.robot_id, idx, physicsClientId=env.client)
            except Exception:
                continue
            lower, upper = info[8], info[9]
            if lower >= upper:
                lower, upper = -math.pi, math.pi
            limits[pos] = (lower, upper)
        # Apply any user overrides keyed by joint name (case-insensitive).
        for name, bounds in JOINT_LIMIT_OVERRIDES.items():
            if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
                continue
            pos = self.name_to_idx.get(name.lower())
            if pos is None:
                continue
            lo, hi = float(bounds[0]), float(bounds[1])
            if lo > hi:
                lo, hi = hi, lo
            limits[pos] = (lo, hi)
        return limits

    def _select_gripper_indices(self) -> list[int]:
        """Prefer joints named GRIPPER_JOINT1A/2A; fall back to any detected gripper joints."""
        primary: list[int] = []
        for name in ("gripper_joint1a", "gripper_joint2a"):
            if name in self.name_to_idx:
                primary.append(self.name_to_idx[name])
        if primary:
            return primary
        guessed = getattr(self.arm, "_gripper_joint_indices", [])
        pos_map = {idx: pos for pos, idx in enumerate(getattr(self.arm, "joint_order", []))}
        return [pos_map[idx] for idx in guessed if idx in pos_map]

    def _gripper_signs(self) -> dict[int, int]:
        """Invert one side of the gripper so positive values close the jaws."""
        signs: dict[int, int] = {}
        for name, idx in self.name_to_idx.items():
            if idx not in self.gripper_indices:
                continue
            signs[idx] = -1 if any(tag in name for tag in ("2a", "2b", "right")) else 1
        return signs

    def _set_axis_target(self, joints: list[float], joint_name: str, axis_value: float) -> None:
        idx = self.name_to_idx.get(joint_name)
        if idx is None:
            return
        span = JOINT_TARGET_RANGE_RAD.get(joint_name, math.pi)
        limits = self.limits.get(idx)
        if limits is not None:
            low, high = limits
            mid = 0.5 * (low + high)
            amp = min(high - mid, mid - low)
            if amp <= 0:
                amp = min(abs(low), abs(high), span)
            target = mid + axis_value * amp
            joints[idx] = clamp(target, limits)
        else:
            target = axis_value * span
            joints[idx] = clamp(target, (-span, span))

    def _apply_gripper(self, joints: list[float], axis_value: float, dt: float) -> None:
        if not self.gripper_indices:
            return
        delta = axis_value * GRIPPER_SPEED_RAD_PER_S * dt
        for i, idx in enumerate(self.gripper_indices):
            sign = -self.gripper_signs.get(idx, -1 if i % 2 else 1)
            limits = self.limits.get(idx, (-MAX_GRIPPER_RANGE_RAD, MAX_GRIPPER_RANGE_RAD))
            joints[idx] = clamp(joints[idx] + delta * sign, limits)
        self._sync_gripper_pairs(joints)

    def _apply_joint_velocity(self, joints: list[float], joint_name: str, axis_value: float, dt: float) -> None:
        """Integrate joint motion based on stick deflection (speed scales with |axis|)."""
        idx = self.name_to_idx.get(joint_name)
        if idx is None:
            return
        speed = JOINT_SPEED_RAD_PER_S.get(joint_name, 1.0)
        gain = 0.4 + 0.6 * abs(axis_value)
        delta = axis_value * speed * gain * dt
        limits = self.limits.get(idx, (-math.pi, math.pi))
        joints[idx] = clamp(joints[idx] + delta, limits)

    def _sync_gripper_pairs(self, joints: list[float]) -> None:
        """Keep 1B matched to 1A and 2B matched to 2A so those joints move as a unit."""
        pairs = [("gripper_joint1a", "gripper_joint1b"), ("gripper_joint2a", "gripper_joint2b")]
        for master, follower in pairs:
            idx_master = self.name_to_idx.get(master)
            idx_follower = self.name_to_idx.get(follower)
            if idx_master is None or idx_follower is None:
                continue
            if idx_master < len(joints) and idx_follower < len(joints):
                joints[idx_follower] = joints[idx_master]

    def run(self, stop_event: threading.Event) -> None:
        print(
            "[Gamepad] Left stick: yaw (X) + shoulder (Y). "
            "Bumpers: gripper close/open. Right stick Y: elbow. "
            "Stick extremes map directly to target angles."
        )
        last_debug_log = 0.0
        last_time = time.time()
        while not stop_event.is_set():
            axes = self.controller.read()
            now = time.time()
            dt = max(1e-4, now - last_time)
            last_time = now
            targets = list(self.arm.get_joint_positions())
            self._apply_joint_velocity(targets, "yaw", -axes.left_x, dt)
            self._apply_joint_velocity(targets, "shoulder", -axes.left_y, dt)
            self._apply_joint_velocity(targets, "elbow", -axes.right_y, dt)
            grip_input = self.controller.gripper_input()
            self._apply_gripper(targets, grip_input if grip_input is not None else axes.right_x, dt)
            now = time.time()
            if GAMEPAD_DEBUG and (now - last_debug_log) >= GAMEPAD_DEBUG_INTERVAL_S:
                dbg_axes = {
                    "lx": round(axes.left_x, 3),
                    "ly": round(axes.left_y, 3),
                    "rx": round(axes.right_x, 3),
                    "ry": round(axes.right_y, 3),
                }
                dbg_buttons = {k: v for k, v in getattr(self.controller, "_last_buttons", {}).items()}
                dbg_all = getattr(self.controller, "_all_buttons", [])
                msg = "[Gamepad] dbg axes {} buttons {} targets {}".format(
                    dbg_axes, dbg_buttons, [round(v, 3) for v in targets]
                )
                if GAMEPAD_DEBUG_ALL_BUTTONS:
                    msg += f" all_buttons={dbg_all}"
                print(msg)
                last_debug_log = now
            try:
                self.arm.set_joint_positions(targets)
            except Exception as exc:
                print(f"[Gamepad] Failed to send joint targets: {exc}")
                stop_event.set()
                break

            if stop_event.wait(self.step_s):
                break


def gamepad_teleop_loop(arm: QArmBase, step_s: float, stop_event: threading.Event) -> None:
    """Spin until stop_event is set, commanding joints from the Xbox controller."""
    try:
        controller = XboxController(deadzone=GAMEPAD_DEADZONE, axis_map=JOYSTICK_AXES)
    except RuntimeError as exc:
        print(f"[Gamepad] {exc}")
        return
    teleop = GamepadTeleop(arm, controller, step_s=step_s)
    teleop.run(stop_event)


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

    stop_event = threading.Event()
    motion_thread: threading.Thread | None = None

    def launch_viewer() -> None:
        from sim.panda_viewer import PandaArmViewer, PhysicsBridge

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

    try:
        if use_panda_viewer:
            if USE_GAMEPAD_CONTROL:
                motion_thread = threading.Thread(
                    target=gamepad_teleop_loop,
                    args=(arm, STEP_S, stop_event),
                    daemon=True,
                )
                motion_thread.start()
            try:
                launch_viewer()  # blocks until closed
            except KeyboardInterrupt:
                stop_event.set()
            stop_event.set()
        else:
            if USE_GAMEPAD_CONTROL:
                gamepad_teleop_loop(arm, step_s=STEP_S, stop_event=stop_event)
            else:
                print("[Student] Kinematic objects loaded. Press Ctrl+C to exit.")
                while True:
                    time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping kinematic object demo...")
        stop_event.set()
    finally:
        try:
            if motion_thread is not None and motion_thread.is_alive():
                stop_event.set()
                motion_thread.join(timeout=1.0)
            arm.home()
        except Exception:
            pass
        try:
            import pygame

            pygame.quit()
        except Exception:
            pass


if __name__ == "__main__":
    main()
