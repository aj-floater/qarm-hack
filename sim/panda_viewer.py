"""
Panda3D-based viewer for the QArm with PyBullet physics.

Run with:
    python -m sim.panda_viewer

Controls:
    - Mouse1 drag or arrow keys: orbit camera (yaw/pitch)
    - Mouse2 drag or WASD: pan target on the ground plane
    - Mouse wheel or +/- : zoom
    - R: reset arm to home
    - Space: pause/resume physics stepping
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Tuple

try:
    import pybullet as p
except ImportError as exc:  # pragma: no cover - runtime guard
    raise SystemExit("pybullet is not installed. Run `pip install -e .`.") from exc

try:
    from direct.gui.DirectGui import DirectLabel, DirectSlider
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import (
        AmbientLight,
        DirectionalLight,
        Filename,
        LQuaternionf,
        LineSegs,
        NodePath,
        TextNode,
        Vec3,
        Vec4,
    )
except ImportError as exc:  # pragma: no cover - runtime guard
    raise SystemExit("Panda3D is not installed. Try `pip install panda3d`.") from exc

from sim.env import QArmSimEnv


class PhysicsBridge:
    """Thin wrapper around QArmSimEnv for stepping and reading link poses."""

    def __init__(self, time_step: float) -> None:
        self.env = QArmSimEnv(gui=False, add_ground=False, enable_joint_sliders=False, time_step=time_step)
        self.env.reset()
        self.client = self.env.client
        self.robot_id = self.env.robot_id
        self.link_name_to_index = dict(self.env.link_name_to_index)

        self.joint_order: List[int] = list(self.env.movable_joint_indices)
        self.joint_meta: List[Tuple[int, str, float, float]] = []
        for idx in self.joint_order:
            info = p.getJointInfo(self.robot_id, idx, physicsClientId=self.client)
            name = info[1].decode("utf-8")
            lower, upper = info[8], info[9]
            # If limits are invalid/zero, fall back to a sensible range.
            if lower >= upper:
                lower, upper = -math.pi, math.pi
            self.joint_meta.append((idx, name, lower, upper))

    def step(self) -> None:
        self.env.step()

    def home(self) -> None:
        self.env.reset()

    def set_joints(self, values: List[float]) -> None:
        if len(values) != len(self.joint_order):
            raise ValueError(f"Expected {len(self.joint_order)} joint values, got {len(values)}")
        self.env.set_joint_positions(values)

    def get_link_poses(self) -> Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
        poses: Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]] = {}
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.client)
        poses["base_link"] = (base_pos, base_orn)
        for name, idx in self.link_name_to_index.items():
            state = p.getLinkState(self.robot_id, idx, computeForwardKinematics=True, physicsClientId=self.client)
            pos = state[4]
            orn = state[5]
            poses[name] = (pos, orn)
        return poses


class PandaArmViewer(ShowBase):
    """Render the PyBullet-simulated arm using Panda3D."""

    def __init__(self, physics: PhysicsBridge, args: argparse.Namespace) -> None:
        super().__init__()
        self.disableMouse()
        self.physics = physics
        self.paused = False
        self.time_step = args.time_step

        self.cam_target = Vec3(0, 0, 0.05)
        self.cam_distance = 0.5
        self.cam_yaw = 45.0
        self.cam_pitch = -30.0
        self._orbit_drag = False
        self._pan_drag = False
        self._last_mouse: Tuple[float, float] | None = None

        self.link_nodes: Dict[str, NodePath] = {}
        self.joint_sliders: List[DirectSlider] = []
        self.fps_label: DirectLabel | None = None

        self._setup_scene()
        self._setup_models()
        self._setup_ui()
        self._bind_controls()

        self.taskMgr.add(self._update_task, "update-task")

    def _setup_scene(self) -> None:
        self.setBackgroundColor(0, 0, 0, 1)
        self.camLens.setNearFar(0.01, 10.0)
        amb = AmbientLight("ambient")
        amb.setColor(Vec4(0.3, 0.3, 0.3, 1))
        amb_np = self.render.attachNewNode(amb)
        self.render.setLight(amb_np)

        sun = DirectionalLight("sun")
        sun.setColor(Vec4(0.8, 0.8, 0.8, 1))
        sun_np = self.render.attachNewNode(sun)
        sun_np.setHpr(45, -45, 0)
        self.render.setLight(sun_np)

        self._create_grid()
        self._update_camera()

    def _setup_models(self) -> None:
        mesh_dir = Path(__file__).resolve().parent.parent / "qarm" / "meshes"
        mesh_map: Dict[str, List[Path]] = {
            "base_link": [mesh_dir / "base_link.STL"],
            "YAW": [mesh_dir / "YAW.STL"],
            "BICEP": [mesh_dir / "BICEP.STL"],
            "FOREARM": [mesh_dir / "FOREARM.STL"],
            "END-EFFECTOR": [mesh_dir / "END-EFFECTOR.STL", mesh_dir / "Gripper.stl"],
        }

        for link, paths in mesh_map.items():
            parent = self.render.attachNewNode(f"{link}_node")
            for path in paths:
                node = self._load_mesh(path)
                node.reparentTo(parent)
            parent.setColor(Vec4(0.2, 0.2, 0.22, 1))
            parent.setTwoSided(True)
            self.link_nodes[link] = parent

    def _setup_ui(self) -> None:
        y = 0.9
        for _, name, lower, upper in self.physics.joint_meta:
            lbl = DirectLabel(
                text=name,
                scale=0.05,
                pos=(-1.3, 0, y),
                frameColor=(0, 0, 0, 0),
                text_align=TextNode.ALeft,
            )
            slider = DirectSlider(
                range=(lower, upper),
                value=0.0,
                pageSize=(upper - lower) / 100.0,
                scale=0.3,
                pos=(-0.2, 0, y),
                command=self._on_slider_change,
            )
            self.joint_sliders.append(slider)
            y -= 0.12

        self.fps_label = DirectLabel(
            text="",
            scale=0.05,
            pos=(1.0, 0, 0.9),
            frameColor=(0, 0, 0, 0),
            text_align=TextNode.ARight,
        )

    def _bind_controls(self) -> None:
        self.accept("escape", self._quit)
        self.accept("r", self._reset)
        self.accept("space", self._toggle_pause)
        self.accept("+", self._zoom, [-0.05])
        self.accept("=", self._zoom, [-0.05])
        self.accept("-", self._zoom, [0.05])
        self.accept("arrow_left", self._orbit, [-5, 0])
        self.accept("arrow_right", self._orbit, [5, 0])
        self.accept("arrow_up", self._orbit, [0, 5])
        self.accept("arrow_down", self._orbit, [0, -5])
        # Mouse controls
        self.accept("mouse1", self._start_orbit_drag)
        self.accept("mouse1-up", self._stop_drag)
        self.accept("mouse3", self._start_pan_drag)  # right click
        self.accept("mouse3-up", self._stop_drag)
        self.accept("wheel_up", self._zoom, [-0.05])
        self.accept("wheel_down", self._zoom, [0.05])
        # Keyboard pan
        pan_step = 0.02
        self.accept("w", self._pan_target, [0, pan_step])
        self.accept("w-repeat", self._pan_target, [0, pan_step])
        self.accept("s", self._pan_target, [0, -pan_step])
        self.accept("s-repeat", self._pan_target, [0, -pan_step])
        self.accept("a", self._pan_target, [-pan_step, 0])
        self.accept("a-repeat", self._pan_target, [-pan_step, 0])
        self.accept("d", self._pan_target, [pan_step, 0])
        self.accept("d-repeat", self._pan_target, [pan_step, 0])

    def _update_task(self, task):
        if not self.paused:
            self.physics.step()
        self._sync_models()
        self._handle_mouse()
        self._update_camera()
        self._update_fps(task.dt)
        return task.cont

    def _sync_models(self) -> None:
        poses = self.physics.get_link_poses()
        for name, node in self.link_nodes.items():
            if name not in poses:
                continue
            pos, orn = poses[name]
            node.setPos(pos[0], pos[1], pos[2])
            node.setQuat(LQuaternionf(orn[3], orn[0], orn[1], orn[2]))

    def _update_camera(self) -> None:
        cam_pos = self._spherical_to_cartesian(self.cam_distance, math.radians(self.cam_yaw), math.radians(self.cam_pitch))
        self.camera.setPos(self.cam_target + cam_pos)
        self.camera.lookAt(self.cam_target)

    def _update_fps(self, dt: float) -> None:
        if not self.fps_label:
            return
        if dt <= 1e-6:
            return
        fps = 1.0 / dt
        self.fps_label["text"] = f"FPS: {fps:5.1f}"

    def _create_grid(self, size: float = 0.6, step: float = 0.1) -> None:
        ls = LineSegs()
        ls.setColor(0.35, 0.35, 0.35, 0.4)
        for x in frange(-size, size + 1e-6, step):
            ls.moveTo(x, -size, 0)
            ls.drawTo(x, size, 0)
        for y in frange(-size, size + 1e-6, step):
            ls.moveTo(-size, y, 0)
            ls.drawTo(size, y, 0)
        grid = self.render.attachNewNode(ls.create())
        grid.setTransparency(True)

    def _load_mesh(self, path: Path) -> NodePath:
        try:
            return self.loader.loadModel(Filename.fromOsSpecific(str(path)))
        except Exception:
            # Fallback to a simple box if the mesh cannot be loaded.
            try:
                return self.loader.loadModel("models/box")
            except Exception:
                cm_node = self.render.attachNewNode("placeholder")
                cm_node.setScale(0.02)
                return cm_node

    def _on_slider_change(self) -> None:
        values = [slider["value"] for slider in self.joint_sliders]
        self.physics.set_joints(values)

    def _zoom(self, delta: float) -> None:
        self.cam_distance = max(0.1, self.cam_distance + delta)

    def _orbit(self, dyaw: float, dpitch: float) -> None:
        self.cam_yaw = (self.cam_yaw + dyaw) % 360
        self.cam_pitch = max(-89.0, min(89.0, self.cam_pitch + dpitch))

    def _pan_target(self, dx: float, dy: float) -> None:
        self.cam_target += Vec3(dx, dy, 0)

    def _reset(self) -> None:
        self.physics.home()
        for slider in self.joint_sliders:
            slider["value"] = 0.0

    def _toggle_pause(self) -> None:
        self.paused = not self.paused

    def _quit(self) -> None:
        self.userExit()

    def _start_orbit_drag(self) -> None:
        self._orbit_drag = True
        if self.mouseWatcherNode.hasMouse():
            m = self.mouseWatcherNode.getMouse()
            self._last_mouse = (m.getX(), m.getY())

    def _start_pan_drag(self) -> None:
        self._pan_drag = True
        if self.mouseWatcherNode.hasMouse():
            m = self.mouseWatcherNode.getMouse()
            self._last_mouse = (m.getX(), m.getY())

    def _stop_drag(self) -> None:
        self._orbit_drag = False
        self._pan_drag = False
        self._last_mouse = None

    def _handle_mouse(self) -> None:
        if not self.mouseWatcherNode.hasMouse():
            return
        current = self.mouseWatcherNode.getMouse()
        x, y = current.getX(), current.getY()
        if self._last_mouse is None:
            self._last_mouse = (x, y)
            return
        dx = x - self._last_mouse[0]
        dy = y - self._last_mouse[1]
        self._last_mouse = (x, y)
        if self._orbit_drag:
            self._orbit(dx * -200, dy * -200)  # invert horizontal for orbit
        elif self._pan_drag:
            # Pan in the view plane (camera right/up).
            cam_quat = self.camera.getQuat(self.render)
            right = cam_quat.getRight()
            up = cam_quat.getUp()
            pan_scale = self.cam_distance * 0.5
            move = (right * dx * -pan_scale) + (up * dy * -pan_scale)  # invert horizontal and vertical for pan
            self.cam_target += move

    @staticmethod
    def _spherical_to_cartesian(radius: float, yaw: float, pitch: float) -> Vec3:
        x = radius * math.cos(pitch) * math.cos(yaw)
        y = radius * math.cos(pitch) * math.sin(yaw)
        z = radius * math.sin(pitch)
        return Vec3(x, y, z)


def frange(start: float, stop: float, step: float):
    val = start
    while val <= stop + 1e-9:
        yield round(val, 6)
        val += step


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Panda3D viewer for the QArm with PyBullet physics.")
    parser.add_argument("--time-step", type=float, default=1.0 / 120.0, help="Physics timestep (seconds).")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    physics = PhysicsBridge(time_step=args.time_step)
    app = PandaArmViewer(physics, args)
    app.run()


if __name__ == "__main__":
    main()
