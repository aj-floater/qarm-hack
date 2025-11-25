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
    from direct.filter.CommonFilters import CommonFilters
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
        Material,
        TransparencyAttrib,
        AntialiasAttrib,
    )
except ImportError as exc:  # pragma: no cover - runtime guard
    raise SystemExit("Panda3D is not installed. Try `pip install panda3d`.") from exc

from sim.env import QArmSimEnv


class PhysicsBridge:
    """
    Thin wrapper around QArmSimEnv for stepping and reading link poses.

    Can either create its own environment (default) or attach to an existing
    one, which is useful when embedding the viewer inside another script.
    """

    def __init__(
        self,
        time_step: float,
        base_mesh: Path | None,
        base_collision_mesh: Path | None,
        base_mesh_scale: float | List[float],
        base_yaw_deg: float,
        base_friction: float,
        base_restitution: float,
        env: QArmSimEnv | None = None,
        reset: bool = True,
    ) -> None:
        if env is None:
            self.env = QArmSimEnv(
                gui=False,
                add_ground=base_mesh is None,
                enable_joint_sliders=False,
                time_step=time_step,
                base_mesh_path=base_mesh,
                base_collision_mesh_path=base_collision_mesh,
                base_mesh_scale=base_mesh_scale,
                base_yaw_deg=base_yaw_deg,
                base_friction=base_friction,
                base_restitution=base_restitution,
            )
            if reset:
                self.env.reset()
        else:
            self.env = env
            if reset:
                self.env.reset()

        self.base_mesh_scale = base_mesh_scale
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

    def probe_base_collision(self) -> None:
        """Spawn a small probe and report contacts with the floor to verify collision is active."""
        if self.env.floor_id is None:
            print("[PandaViewer] No floor present; skipping collision probe.")
            return

        # Log the collision shape type/path Bullet sees.
        shape_data = p.getCollisionShapeData(self.env.floor_id, -1, physicsClientId=self.client)
        for sd in shape_data:
            geom_type = sd[2]
            file_name = sd[4]
            mesh_scale = sd[3]
            type_name = {
                getattr(p, "GEOM_BOX", -1): "BOX",
                getattr(p, "GEOM_SPHERE", -1): "SPHERE",
                getattr(p, "GEOM_CAPSULE", -1): "CAPSULE",
                getattr(p, "GEOM_CYLINDER", -1): "CYLINDER",
                getattr(p, "GEOM_MESH", -1): "MESH",
                getattr(p, "GEOM_PLANE", -1): "PLANE",
                getattr(p, "GEOM_HEIGHTFIELD", -1): "HEIGHTFIELD",
            }.get(geom_type, str(geom_type))
            print(
                "[PandaViewer] Floor collision shape:",
                f"type={type_name} ({geom_type})",
                f"file={file_name}",
                f"scale={mesh_scale}",
            )

        aabb_min, aabb_max = p.getAABB(self.env.floor_id, -1, physicsClientId=self.client)
        print("[PandaViewer] Floor AABB:", aabb_min, aabb_max)
        center = [(a + b) * 0.5 for a, b in zip(aabb_min, aabb_max)]
        start = [center[0], center[1], aabb_max[2] + 0.2]
        end = [center[0], center[1], aabb_min[2] - 0.2]
        ray = p.rayTest(start, end, physicsClientId=self.client)
        print("[PandaViewer] Ray test hit:", ray)

        col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.015, physicsClientId=self.client)
        probe = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=col,
            basePosition=[center[0], center[1], aabb_max[2] + 0.05],
            physicsClientId=self.client,
        )
        # Let it fall onto the base.
        for _ in range(90):
            p.stepSimulation(physicsClientId=self.client)
        contacts = p.getContactPoints(bodyA=probe, bodyB=self.env.floor_id, physicsClientId=self.client)
        print(f"[PandaViewer] Probe contact count with floor: {len(contacts)}")
        if contacts:
            c = contacts[0]
            print(
                "[PandaViewer] First contact:",
                f"position={c[5]}",
                f"normal={c[7]}",
                f"distance={c[8]}",
                f"normal_force={c[9]}",
            )
        p.removeBody(probe, physicsClientId=self.client)

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
        self.render.setShaderAuto()
        self.render.setAntialias(AntialiasAttrib.MMultisample)
        self.render2d.setAntialias(AntialiasAttrib.MMultisample)
        self.physics = physics
        self.paused = False
        self.time_step = args.time_step
        self.base_mesh_path = self._resolve_path(args.base_mesh)
        self.green_accent_path = self._resolve_path(args.green_accent)
        self.blue_accent_path = self._resolve_path(args.blue_accent)
        self.show_base = not args.hide_base
        self.show_accents = not args.hide_accents
        self.base_yaw_deg = args.base_yaw
        self.base_mesh_scale = getattr(args, "base_scale", getattr(physics, "base_mesh_scale", 1.0))
        # Grid sizing (edit here if you want different spans).
        self.grid_step = 0.1
        self.grid_x_neg_cells = 4
        self.grid_x_pos_cells = 6
        self.grid_y_neg_cells = 8
        self.grid_y_pos_cells = 4

        # Default camera: more top-down, looking at the board center.
        self.cam_target = Vec3(0, 0, 0.02)
        self.cam_distance = 2.5
        self.cam_yaw = -65.0
        self.cam_pitch = 40.0
        self._orbit_drag = False
        self._pan_drag = False
        self._last_mouse: Tuple[float, float] | None = None

        self.link_nodes: Dict[str, NodePath] = {}
        self.joint_sliders: List[DirectSlider] = []
        self.fps_label: DirectLabel | None = None
        self.filters: CommonFilters | None = None

        self._setup_scene()
        self._setup_static_meshes()
        self._setup_models()
        self._setup_ui()
        self._bind_controls()

        self.taskMgr.add(self._update_task, "update-task")

    def _setup_scene(self) -> None:
        # Slightly lifted dark background with a hint of blue.
        self.setBackgroundColor(0.03, 0.03, 0.03, 1)
        self.camLens.setNearFar(0.01, 10.0)
        # Ambient + key/fill/rim lights for depth and highlights.
        amb = AmbientLight("ambient")
        # Warmer ambient for overall lift.
        amb.setColor(Vec4(0.22, 0.2, 0.18, 1))
        amb_np = self.render.attachNewNode(amb)
        self.render.setLight(amb_np)

        key = DirectionalLight("key")
        # Brighter key with a warm tint + shadows.
        key.setColor(Vec4(1.25, 1.05, 0.9, 1))
        key.setShadowCaster(True, 4096, 4096)
        key_np = self.render.attachNewNode(key)
        key_np.setHpr(-35, -45, 0)
        self.render.setLight(key_np)

        # Soft fill to lift shadows slightly.
        fill = DirectionalLight("fill")
        fill.setColor(Vec4(0.4, 0.45, 0.6, 1))
        fill.setShadowCaster(False)
        fill_np = self.render.attachNewNode(fill)
        fill_np.setHpr(60, -10, 0)
        self.render.setLight(fill_np)

        # Emissive geometry to visualize light sources.
        self._add_light_markers()
        self._setup_postprocess()

        # Optional subtle horizon tint via a translucent card.
        try:
            from panda3d.core import CardMaker, TransparencyAttrib

            cm = CardMaker("horizon")
            cm.setFrame(-1, 1, -1, 1)
            card = self.render2d.attachNewNode(cm.generate())
            card.setPos(0, 0, -0.2)
            card.setScale(1.5)
            card.setColor(0.08, 0.1, 0.14, 0.25)
            card.setTransparency(TransparencyAttrib.MAlpha)
            card.setBin("background", 0)
        except Exception:
            pass

        self._create_grid()
        self._update_camera()

    def _add_light_markers(self) -> None:
        """Add small emissive cubes to visualize light sources in the scene."""
        try:
            marker_model = self.loader.loadModel("models/box")
        except Exception:
            return

        markers = [
            {"pos": Vec3(-1.2, 2.0, 1.6), "color": Vec4(1.0, 1.0, 1.0, 1.0), "tint": Vec4(1.1, 1.05, 1.0, 1.0), "scale": 0.08},
            {"pos": Vec3(1.4, -2.0, 1.2), "color": Vec4(1.0, 1.0, 1.0, 1.0), "tint": Vec4(0.6, 0.9, 1.6, 1.0), "scale": 0.06},
        ]
        for cfg in markers:
            cube = marker_model.copyTo(self.render)
            cube.setPos(cfg["pos"])
            cube.setScale(cfg["scale"])
            # Base white emissive look with a subtle color tint for bloom.
            cube.setColor(cfg["color"])
            cube.setColorScale(cfg.get("tint", cfg["color"]) * 8.0)
            cube.setLightOff()
            cube.setShaderAuto(False)
            cube.setTransparency(TransparencyAttrib.MAlpha)

    def _setup_postprocess(self) -> None:
        """Enable a light bloom to make emissive markers pop."""
        if self.win is None or self.cam is None:
            self.filters = None
            return
        try:
            self.filters = CommonFilters(self.win, self.cam)
            ok = self.filters.setBloom(
                blend=(0.25, 0.25, 0.25, 0.0),
                desat=0.2,
                intensity=2.2,
                size="large",
                mintrigger=0.6,
                maxtrigger=1.0,
            )
            if not ok:
                self.filters = None
            else:
                # Add ambient occlusion for extra depth if available.
                try:
                    self.filters.setAmbientOcclusion(
                        strength=0.4,
                        radius=0.35,
                        min_samples=8,
                        max_samples=16,
                    )
                except Exception:
                    pass
        except Exception:
            self.filters = None

    def _setup_models(self) -> None:
        mesh_dir = Path(__file__).resolve().parent / "qarm" / "meshes"
        mesh_map: Dict[str, List[Path]] = {
            "base_link": [mesh_dir / "base_link.STL"],
            "YAW": [mesh_dir / "YAW.STL"],
            "BICEP": [mesh_dir / "BICEP.STL"],
            "FOREARM": [mesh_dir / "FOREARM.STL"],
            "END-EFFECTOR": [mesh_dir / "END-EFFECTOR.STL", mesh_dir / "Gripper.stl"],
        }

        for link, paths in mesh_map.items():
            parent = self.render.attachNewNode(f"{link}_node")
            parent.setShaderAuto(True)
            for path in paths:
                node = self._load_mesh(path)
                node.reparentTo(parent)
            # Set per-link colors (base dark grey, red accent on YAW) via a material.
            mat = Material()
            if link == "YAW":
                diffuse = Vec4(0.6, 0.14, 0.14, 1)
            elif link == "END-EFFECTOR":
                diffuse = Vec4(0.6, 0.14, 0.14, 1)  # make gripper red
            else:
                diffuse = Vec4(0.12, 0.12, 0.14, 1)
            mat.setDiffuse(diffuse)
            mat.setAmbient(diffuse * 0.8)
            mat.setSpecular(Vec4(0.08, 0.08, 0.08, 1))
            mat.setShininess(5.0)
            parent.setMaterial(mat, 1)
            parent.setColor(Vec4(1, 1, 1, 1))
            parent.setTwoSided(True)
            self.link_nodes[link] = parent

    def _setup_ui(self) -> None:
        # Position sliders in the top-right corner.
        x = 0.8
        y = 0.85
        for _, name, lower, upper in self.physics.joint_meta:
            slider = DirectSlider(
                range=(lower, upper),
                value=0.0,
                pageSize=(upper - lower) / 100.0,
                scale=0.3,
                pos=(x, 0, y),
                command=self._on_slider_change,
            )
            label = DirectLabel(
                text=name,
                scale=0.05,
                pos=(x - 0.45, 0, y + 0.02),
                frameColor=(0, 0, 0, 0),
                text_fg=(1, 1, 1, 1),
                text_align=TextNode.ALeft,
            )
            label.reparentTo(self.aspect2d)
            self.joint_sliders.append(slider)
            y -= 0.12

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

    def _create_grid(self) -> None:
        """Grid sized by per-axis cell counts and step."""
        step = self.grid_step
        size_x_neg = self.grid_x_neg_cells * step
        size_x_pos = self.grid_x_pos_cells * step
        size_y_neg = self.grid_y_neg_cells * step
        size_y_pos = self.grid_y_pos_cells * step
        ls = LineSegs()
        ls.setColor(0.65, 0.65, 0.7, 0.8)
        for x in frange(-size_x_neg, size_x_pos + 1e-6, step):
            ls.moveTo(x, -size_y_neg, 0)
            ls.drawTo(x, size_y_pos, 0)
        for y in frange(-size_y_neg, size_y_pos + 1e-6, step):
            ls.moveTo(-size_x_neg, y, 0)
            ls.drawTo(size_x_pos, y, 0)
        grid = self.render.attachNewNode(ls.create())
        grid.setTransparency(True)
        grid.setLightOff()
        grid.setBin("background", 5)
        grid.setDepthOffset(1)

    def _setup_static_meshes(self) -> None:
        """Load the pine base and colored accent meshes (visual-only)."""
        models_dir = Path(__file__).resolve().parent / "models"

        if self.show_base:
            base_path = Path(self.base_mesh_path) if self.base_mesh_path else models_dir / "pinebase.stl"
            base_node = self._load_mesh(base_path)
            base_node.reparentTo(self.render)
            base_node.setH(self.base_yaw_deg)  # rotate around Z at the origin
            base_node.setTwoSided(True)
            base_node.setScale(self._as_vec3(self.base_mesh_scale))
            base_node.setShaderAuto(True)
            mat = Material()
            # Soft pine-like tint with subtle sheen.
            diffuse = Vec4(0.82, 0.72, 0.55, 1)
            mat.setDiffuse(diffuse)
            mat.setAmbient(diffuse * 0.85)
            mat.setSpecular(Vec4(0.12, 0.1, 0.08, 1))
            mat.setShininess(8.0)
            base_node.setMaterial(mat, 1)
            base_node.setColor(Vec4(1, 1, 1, 1))

        if self.show_accents:
            accent_defs = [
                ("green", self.green_accent_path, Vec4(0.25, 0.75, 0.35, 1)),
                ("blue", self.blue_accent_path, Vec4(0.2, 0.45, 0.92, 1)),
            ]
            for name, override, color in accent_defs:
                path = Path(override) if override else models_dir / f"{name}accent.stl"
                node = self._load_mesh(path)
                node.reparentTo(self.render)
                node.setH(self.base_yaw_deg)
                node.setTwoSided(True)
                node.setScale(self._as_vec3(self.base_mesh_scale))
                node.setShaderAuto(True)
                mat = Material()
                mat.setDiffuse(color)
                mat.setAmbient(color * 0.8)
                mat.setSpecular(Vec4(0.1, 0.1, 0.1, 1))
                mat.setShininess(6.0)
                node.setMaterial(mat, 1)
                node.setColor(Vec4(1, 1, 1, 1))

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

    @staticmethod
    def _as_vec3(scale) -> Vec3:
        """Coerce a scale value (scalar/iterable/string) to a Vec3."""
        if scale is None:
            return Vec3(1.0, 1.0, 1.0)
        if isinstance(scale, (str, bytes)):
            try:
                val = float(scale)
            except Exception:
                val = 1.0
            return Vec3(val, val, val)
        if isinstance(scale, (int, float)):
            val = float(scale)
            return Vec3(val, val, val)
        try:
            vals = list(scale)
        except Exception:
            return Vec3(1.0, 1.0, 1.0)
        if len(vals) == 1:
            v = float(vals[0])
            return Vec3(v, v, v)
        if len(vals) >= 3:
            return Vec3(float(vals[0]), float(vals[1]), float(vals[2]))
        return Vec3(1.0, 1.0, 1.0)

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

    @staticmethod
    def _resolve_path(value: str | None) -> Path | None:
        """Return an absolute Path or None for empty values."""
        if not value:
            return None
        return Path(value).expanduser().resolve()


def frange(start: float, stop: float, step: float):
    val = start
    while val <= stop + 1e-9:
        yield round(val, 6)
        val += step


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Panda3D viewer for the QArm with PyBullet physics.")
    parser.add_argument("--time-step", type=float, default=1.0 / 120.0, help="Physics timestep (seconds).")
    parser.add_argument("--base-mesh", type=str, default=None, help="Path to pine base mesh (visual).")
    parser.add_argument("--base-collision-mesh", type=str, default=None, help="Path to collision mesh for the base.")
    parser.add_argument(
        "--base-scale",
        type=float,
        nargs="+",
        default=[1.0],
        help="Uniform or XYZ scale for the base mesh (one value or three values).",
    )
    parser.add_argument("--base-yaw", type=float, default=180.0, help="Rotation (degrees about Z) to apply to the base.")
    parser.add_argument("--base-friction", type=float, default=0.8, help="Lateral friction for the base collision.")
    parser.add_argument("--base-restitution", type=float, default=0.0, help="Restitution (bounciness) for the base.")
    parser.add_argument("--green-accent", type=str, default=None, help="Path to green accent mesh (visual).")
    parser.add_argument("--blue-accent", type=str, default=None, help="Path to blue accent mesh (visual).")
    parser.add_argument("--hide-base", action="store_true", help="Do not load the pine base mesh.")
    parser.add_argument("--hide-accents", action="store_true", help="Do not load the accent meshes.")
    parser.add_argument(
        "--probe-base-collision",
        action="store_true",
        help="Drop a small probe to verify the base collision mesh is active (prints contact info).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    models_dir = Path(__file__).resolve().parent / "models"
    default_base = models_dir / "pinebase.stl"
    default_base_collision = models_dir / "pinebase_collision.stl"
    scale_arg = args.base_scale
    if len(scale_arg) == 1:
        base_scale: float | List[float] = scale_arg[0]
    elif len(scale_arg) == 3:
        base_scale = scale_arg
    else:
        raise SystemExit("Base scale must be one value (uniform) or three values (XYZ).")

    if args.base_mesh:
        base_mesh_path = Path(args.base_mesh).expanduser().resolve()
    elif not args.hide_base and default_base.exists():
        base_mesh_path = default_base
    else:
        base_mesh_path = None

    if args.base_collision_mesh:
        base_collision_path = Path(args.base_collision_mesh).expanduser().resolve()
    elif base_mesh_path and default_base_collision.exists():
        base_collision_path = default_base_collision
    else:
        base_collision_path = base_mesh_path

    # Log the chosen assets so it's obvious which collision is active.
    print("[PandaViewer] base visual mesh:", base_mesh_path)
    print("[PandaViewer] base collision mesh:", base_collision_path)
    print("[PandaViewer] base scale:", base_scale)
    print("[PandaViewer] base yaw:", args.base_yaw)
    print("[PandaViewer] base friction/restitution:", args.base_friction, args.base_restitution)
    physics = PhysicsBridge(
        time_step=args.time_step,
        base_mesh=base_mesh_path,
        base_collision_mesh=base_collision_path,
        base_mesh_scale=base_scale,
        base_yaw_deg=args.base_yaw,
        base_friction=args.base_friction,
        base_restitution=args.base_restitution,
    )
    if args.probe_base_collision:
        physics.probe_base_collision()
    app = PandaArmViewer(physics, args)
    app.run()


if __name__ == "__main__":
    main()
