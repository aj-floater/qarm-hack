"""
Lightweight PyBullet environment wrapper for the QArm simulation.

PyBullet provides the physics; Panda3D (see :mod:`sim.panda_viewer`) is the
primary viewport. The PyBullet GUI can be enabled for debugging but defaults
to headless usage. Keep the joint ordering consistent with the URDF:
- 0: world_base_joint (fixed, not driven)
- 1: YAW
- 2: SHOULDER
- 3: ELBOW
- 4: WRIST
- URDF expected at ``sim/qarm/urdf/QARM.urdf`` by default.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

try:
    import pybullet as p
except ImportError as exc:  # pragma: no cover - runtime guard
    raise SystemExit("pybullet is not installed. Run `pip install -e .` first.") from exc

from sim.assets import BaseMeshAssets, DEFAULT_BASE_ASSETS


@dataclass
class KinematicObject:
    """Metadata for a static/kinematic mesh spawned in the scene."""

    body_id: int
    visual_path: Path
    collision_path: Path | None
    mass: float
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    scale: tuple[float, float, float]
    rgba: tuple[float, float, float, float] | None


class QArmSimEnv:
    """
    Container for PyBullet state and helper utilities.

    Minimal responsibilities:
    - start a PyBullet client (GUI or DIRECT),
    - load the QArm URDF from sim/qarm/urdf,
    - record joint indices/names and expose controllable joints,
    - provide reset/step and joint position helpers.
    """

    DARK_LINK_COLOR = (0.15, 0.15, 0.18, 1.0)
    ACCENT_LINK_COLOR = (0.78, 0.12, 0.12, 1.0)
    DARK_FLOOR_COLOR = (0.1, 0.1, 0.1, 1.0)
    LIGHT_FLOOR_COLOR = (0.8, 0.8, 0.8, 1.0)
    BACKDROP_COLOR = (0.85, 0.85, 0.85, 1.0)
    HOLD_FORCE = 1  # small holding torque for light joints.
    HOLD_FORCE_STRONG = 4  # higher holding torque for primary arm joints.

    def __init__(
        self,
        gui: bool = False,
        urdf_path: Path | None = None,
        time_step: float = 1.0 / 120.0,
        add_ground: bool = True,
        use_mesh_floor: bool = True,
        base_assets: BaseMeshAssets | None = DEFAULT_BASE_ASSETS,
        real_time: bool = False,
        dark_mode: bool = True,
        show_debug_gui: bool = False,
        show_camera_previews: bool = False,
        enable_joint_sliders: bool = False,
    ) -> None:
        mode = p.GUI if gui else p.DIRECT
        connect_options = ""
        if gui and dark_mode:
            connect_options = (
                f"--background_color_red={self.BACKDROP_COLOR[0]} "
                f"--background_color_green={self.BACKDROP_COLOR[1]} "
                f"--background_color_blue={self.BACKDROP_COLOR[2]}"
            )
        self.client = p.connect(mode, options=connect_options)
        p.setPhysicsEngineParameter(
            numSolverIterations=120,
            numSubSteps=4,
            physicsClientId=self.client,
        )
        self.time_step = time_step
        self.real_time = real_time
        self.gui_enabled = gui
        self.dark_mode = dark_mode
        self.show_debug_gui = show_debug_gui
        self.show_camera_previews = show_camera_previews
        self.enable_joint_sliders = enable_joint_sliders
        self.floor_id: int | None = None
        self.robot_id: int | None = None
        self._joint_slider_ids: list[int] = []
        self.joint_indices: list[int] = []
        self.joint_names: list[str] = []
        self.link_name_to_index: dict[str, int] = {}
        self.movable_joint_indices: list[int] = []
        self._joint_index_to_pos: dict[int, int] = {}
        # Joints we want to hold fixed even if commanded (e.g., gripper 1B/2B).
        self._locked_joint_positions: dict[int, float] = {}
        self.locked_joint_indices: list[int] = []
        self._locked_slider_ids: dict[int, int] = {}
        self.kinematic_objects: list[KinematicObject] = []

        p.setTimeStep(self.time_step, physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setRealTimeSimulation(1 if self.real_time else 0, physicsClientId=self.client)
        # Improve grasp behavior by enabling cone friction for contacts.
        p.setPhysicsEngineParameter(enableConeFriction=1, physicsClientId=self.client)

        if self.gui_enabled:
            self._configure_gui()

        self.base_assets = base_assets or DEFAULT_BASE_ASSETS
        self.use_mesh_floor = bool(use_mesh_floor)
        self.base_mesh_path: Path | None = None
        self.base_collision_mesh_path: Path | None = None
        self.base_mesh_scale: float | Sequence[float] | None = None
        self.base_collision_mesh_scale: float | Sequence[float] | None = None
        self.base_friction: float = 0.0
        self.base_restitution: float = 0.0
        self.base_yaw_deg: float = 0.0

        if add_ground and self.use_mesh_floor:
            self.base_mesh_path = Path(self.base_assets.visual_mesh)
            self.base_collision_mesh_path = Path(self.base_assets.collision_mesh)
            self.base_mesh_scale = self.base_assets.visual_scale
            self.base_collision_mesh_scale = self.base_assets.collision_scale
            self.base_friction = float(self.base_assets.friction)
            self.base_restitution = float(self.base_assets.restitution)
            self.base_yaw_deg = float(self.base_assets.yaw_deg)
            print(
                "[QArmSimEnv] Using mesh floor:",
                f"visual={self.base_mesh_path}",
                f"collision={self.base_collision_mesh_path}",
                f"visual_scale={self.base_mesh_scale}",
                f"collision_scale={self.base_collision_mesh_scale}",
                f"friction={self.base_friction}",
                f"restitution={self.base_restitution}",
            )
            self.floor_id = self._create_mesh_floor(
                visual_path=self.base_mesh_path,
                collision_path=self.base_collision_mesh_path,
                visual_scale=self.base_mesh_scale,
                collision_scale=self.base_collision_mesh_scale,
                friction=self.base_friction,
                restitution=self.base_restitution,
                yaw_deg=self.base_yaw_deg,
            )
            p.changeDynamics(
                self.floor_id,
                -1,
                contactProcessingThreshold=0.0,
                physicsClientId=self.client,
            )
        elif add_ground:
            self.floor_id = self._create_floor(enable_collision=True)

        urdf_path = urdf_path or Path(__file__).resolve().parent / "qarm" / "urdf" / "QARM.urdf"
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF not found at {urdf_path}")

        self.robot_id = p.loadURDF(str(urdf_path), useFixedBase=True, physicsClientId=self.client)
        if self.dark_mode:
            self._apply_robot_palette()

        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client)
        self.joint_indices = list(range(num_joints))
        for j in self.joint_indices:
            info = p.getJointInfo(self.robot_id, j, physicsClientId=self.client)
            self.joint_names.append(info[1].decode("utf-8"))
            self.link_name_to_index[info[12].decode("utf-8")] = j
        # Filter out fixed joints for control commands.
        self.movable_joint_indices = [
            j
            for j in self.joint_indices
            if p.getJointInfo(self.robot_id, j, physicsClientId=self.client)[2] != p.JOINT_FIXED
        ]
        self._joint_index_to_pos = {idx: i for i, idx in enumerate(self.movable_joint_indices)}
        # Lock gripper joints 1B/2B so they stay static in the simulation.
        self._lock_joint_by_name({"GRIPPER_JOINT1B", "GRIPPER_JOINT2B"})

        # Disable default motor torques so we can drive positions explicitly.
        hold_forces: list[float] = []
        strong_joints = {"YAW", "SHOULDER", "ELBOW", "WRIST"}
        for j in self.movable_joint_indices:
            name = self.joint_names[j]
            hold_forces.append(self.HOLD_FORCE_STRONG if name in strong_joints else self.HOLD_FORCE)

        p.setJointMotorControlArray(
            self.robot_id,
            jointIndices=self.movable_joint_indices,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[0.0] * len(self.movable_joint_indices),
            forces=hold_forces,
            physicsClientId=self.client,
        )
        self._configure_contact_friction()

        if self.gui_enabled and self.enable_joint_sliders:
            self._create_joint_sliders()

    def reset(self, home: Sequence[float] | None = None) -> None:
        """
        Reset joints to a home pose (defaults to zeros for movable joints).

        The length of `home` must match the number of movable joints.
        """
        if self.robot_id is not None:
            target = list(home) if home is not None else [0.0] * len(self.movable_joint_indices)
            if len(target) != len(self.movable_joint_indices):
                raise ValueError(f"Home pose length {len(target)} != movable joints {len(self.movable_joint_indices)}")
            for joint_id, angle in zip(self.movable_joint_indices, target):
                p.resetJointState(self.robot_id, joint_id, angle, physicsClientId=self.client)

    def step(self, n: int = 1) -> None:
        """Advance the simulation by n steps (no-op if running in real-time mode)."""
        if self.real_time:
            return
        for _ in range(n):
            p.stepSimulation(physicsClientId=self.client)

    def set_joint_positions(self, q: Sequence[float], max_force: float = 5.0) -> None:
        """
        Drive movable joints to target positions using POSITION_CONTROL.

        `q` must align with `movable_joint_indices` (currently YAW, SHOULDER, ELBOW, WRIST).
        """
        if self.robot_id is None:
            raise RuntimeError("No arm loaded in the simulation.")
        if len(q) != len(self.movable_joint_indices):
            raise ValueError(f"Expected {len(self.movable_joint_indices)} joints, got {len(q)}")
        targets = list(q)
        for idx, lock_val in self._locked_joint_positions.items():
            pos = self._joint_index_to_pos.get(idx)
            if pos is None or pos >= len(targets):
                continue
            targets[pos] = lock_val
        p.setJointMotorControlArray(
            self.robot_id,
            jointIndices=self.movable_joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=targets,
            forces=[max_force] * len(self.movable_joint_indices),
            physicsClientId=self.client,
        )

    def get_joint_positions(self, indices: Iterable[int] | None = None) -> list[float]:
        """
        Read joint angles (radians) for the requested indices (defaults to movable joints).
        """
        if self.robot_id is None:
            raise RuntimeError("No arm loaded in the simulation.")
        selected = list(indices) if indices is not None else self.movable_joint_indices
        states = p.getJointStates(self.robot_id, selected, physicsClientId=self.client)
        return [s[0] for s in states]

    def disconnect(self) -> None:
        """Disconnect the PyBullet client."""
        if p.isConnected(self.client):
            p.disconnect(self.client)

    def _configure_gui(self) -> None:
        """Apply GUI defaults: dark mode and hide camera preview windows."""
        if not self.show_camera_previews:
            for flag in (
                p.COV_ENABLE_RGB_BUFFER_PREVIEW,
                p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
            ):
                p.configureDebugVisualizer(flag, 0, physicsClientId=self.client)

        # Show Bullet's on-screen GUI so the debug panels/sliders are available.
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=self.client)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1, physicsClientId=self.client)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1, physicsClientId=self.client)

    def _create_joint_sliders(self) -> None:
        """Expose joint sliders in the PyBullet GUI for manual manipulation."""
        self._joint_slider_ids.clear()
        for idx in self.movable_joint_indices:
            name = self.joint_names[idx]
            slider_id = p.addUserDebugParameter(
                paramName=f"{name} (rad)",
                rangeMin=-3.14159,
                rangeMax=3.14159,
                startValue=0.0,
            )
            self._joint_slider_ids.append(slider_id)

    def _create_locked_joint_sliders(self) -> None:
        """Expose sliders for locked joints so their fixed angle can be adjusted."""
        if not self._locked_joint_positions:
            return
        for idx, locked_val in self._locked_joint_positions.items():
            name = self.joint_names[idx]
            slider_id = p.addUserDebugParameter(
                paramName=f"{name} (locked rad)",
                rangeMin=-3.14159,
                rangeMax=3.14159,
                startValue=float(locked_val),
            )
            self._locked_slider_ids[idx] = slider_id

    def _create_floor(self, enable_collision: bool) -> int:
        """Create a translucent base plane to provide spatial reference."""
        base_collision = (
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[5, 5, 0.02], physicsClientId=self.client)
            if enable_collision
            else -1
        )
        base_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[5, 5, 0.002],
            rgbaColor=(*self.DARK_FLOOR_COLOR[:3], 0.6),
            specularColor=[0.0, 0.0, 0.0],
            physicsClientId=self.client,
        )
        return p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=base_collision,
            baseVisualShapeIndex=base_visual,
            basePosition=[0.0, 0.0, -0.01],
            physicsClientId=self.client,
        )

    def _create_mesh_floor(
        self,
        visual_path: Path,
        collision_path: Path,
        visual_scale: float | Sequence[float],
        collision_scale: float | Sequence[float],
        friction: float,
        restitution: float,
        yaw_deg: float,
    ) -> int:
        """Create a static mesh floor that matches the Panda3D base model."""
        visual_scale_vec = self._as_vec3(visual_scale)
        collision_scale_vec = self._as_vec3(collision_scale)
        if not visual_path.exists():
            raise FileNotFoundError(f"Base visual mesh not found: {visual_path}")
        if not collision_path.exists():
            raise FileNotFoundError(f"Base collision mesh not found: {collision_path}")

        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=str(visual_path),
            meshScale=visual_scale_vec,
            rgbaColor=(0.82, 0.72, 0.55, 1.0),  # pine-like tint
            specularColor=[0.08, 0.08, 0.08],
            physicsClientId=self.client,
        )
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=str(collision_path),
            meshScale=collision_scale_vec,
            flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
            physicsClientId=self.client,
        )
        if collision_shape < 0:
            raise RuntimeError(f"Failed to create collision shape from {collision_path}")
        yaw_rad = math.radians(yaw_deg)
        base_orientation = p.getQuaternionFromEuler([0.0, 0.0, yaw_rad])
        body_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[0.0, 0.0, 0.0],
            baseOrientation=base_orientation,
            physicsClientId=self.client,
        )
        p.changeDynamics(
            body_id,
            -1,
            lateralFriction=friction,
            restitution=restitution,
            rollingFriction=0.001,
            spinningFriction=0.001,
            physicsClientId=self.client,
        )
        return body_id

    def _apply_robot_palette(self) -> None:
        """Tint the robot to the default dark grey and red colorway."""
        accent_links = {1, 3, 7, 9}  # YAW, ELBOW, gripper 1B and 2B get red highlights.
        visual_data = p.getVisualShapeData(self.robot_id, physicsClientId=self.client)
        for shape in visual_data:
            link_index = shape[1]
            if link_index in accent_links:
                color = self.ACCENT_LINK_COLOR
            else:
                color = self.DARK_LINK_COLOR
            p.changeVisualShape(
                self.robot_id,
                link_index,
                rgbaColor=color,
                specularColor=[0.1, 0.1, 0.1],
                physicsClientId=self.client,
            )

    def _configure_contact_friction(self) -> None:
        """Apply higher friction to gripper surfaces to help objects stick when grasped."""
        if self.robot_id is None:
            return
        gripper_links = {
            "END-EFFECTOR",
            "GRIPPER_BASE",
            "GRIPPER_LINK1A",
            "GRIPPER_LINK1B",
            "GRIPPER_LINK2A",
            "GRIPPER_LINK2B",
        }
        default_friction = (0.9, 0.01, 0.01)  # lateral, rolling, spinning
        gripper_friction = (1.4, 0.04, 0.04)  # slightly softer to reduce jitter
        # Base link is index -1 in Bullet; give it the default friction.
        p.changeDynamics(
            self.robot_id,
            -1,
            lateralFriction=default_friction[0],
            rollingFriction=default_friction[1],
            spinningFriction=default_friction[2],
            restitution=0.0,
            contactProcessingThreshold=0.0,
            contactStiffness=8e4,
            contactDamping=6e3,
            physicsClientId=self.client,
        )
        for name, idx in self.link_name_to_index.items():
            fric = gripper_friction if name in gripper_links else default_friction
            p.changeDynamics(
                self.robot_id,
                idx,
                lateralFriction=fric[0],
                rollingFriction=fric[1],
                spinningFriction=fric[2],
                restitution=0.0,
                contactProcessingThreshold=0.0,
                contactStiffness=8e4,
                contactDamping=6e3,
                ccdSweptSphereRadius=0.02 if name in gripper_links else 0.0,
                physicsClientId=self.client,
            )

    def apply_joint_slider_targets(self) -> None:
        """Read joint slider values (if enabled) and command the arm accordingly."""
        if self._joint_slider_ids:
            targets = [p.readUserDebugParameter(slider_id) for slider_id in self._joint_slider_ids]
            self.set_joint_positions(targets)

    def get_camera_image(
        self,
        width: int = 640,
        height: int = 480,
        fov: float = 60.0,
        near: float = 0.01,
        far: float = 5.0,
        distance: float = 0.5,
        yaw: float = 45.0,
        pitch: float = -30.0,
        target: Sequence[float] = (0.0, 0.0, 0.05),
        return_depth: bool = False,
    ) -> tuple[int, int, bytes] | tuple[int, int, bytes, list[float]]:
        """
        Render an RGB frame from a virtual camera (works in DIRECT/headless mode).

        Returns (width, height, rgba_bytes[, depth_map]). Caller can convert to an image using
        libraries like Qt, PIL, or OpenCV. If `return_depth` is True, a depth map (list of floats)
        is also returned to allow background masking.
        """
        # Clamp values to avoid invalid camera setups.
        width = max(4, int(width))
        height = max(4, int(height))
        fov = max(1e-3, float(fov))
        near = max(1e-4, float(near))
        far = max(near + 1e-3, float(far))
        distance = max(1e-3, float(distance))
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=list(target),
            distance=distance,
            yaw=yaw,
            pitch=pitch,
            roll=0.0,
            upAxisIndex=2,
        )
        proj = p.computeProjectionMatrixFOV(fov=fov, aspect=width / float(height), nearVal=near, farVal=far)
        _, _, rgba, depth, _ = p.getCameraImage(
            width,
            height,
            viewMatrix=view,
            projectionMatrix=proj,
            renderer=p.ER_TINY_RENDERER,
            physicsClientId=self.client,
        )
        if return_depth:
            return width, height, bytes(rgba), depth
        return width, height, bytes(rgba)

    def add_kinematic_object(
        self,
        mesh_path: str | Path,
        *,
        position: Sequence[float] = (0.0, 0.0, 0.0),
        orientation_euler_deg: Sequence[float] | None = None,
        orientation_quat_xyzw: Sequence[float] | None = None,
        scale: float | Sequence[float] = 1.0,
        collision_scale: float | Sequence[float] | None = None,
        rgba: Sequence[float] | None = None,
        collision_mesh_path: str | Path | None = None,
        enable_collision: bool = True,
        mass: float = 0.0,
        force_convex_for_dynamic: bool = True,
        align_aabb_center: bool = False,
    ) -> int:
        """
        Spawn a static (mass=0) mesh in the scene and return its PyBullet body id.

        Students can pass Euler angles in degrees (roll, pitch, yaw) or a quaternion
        (x, y, z, w). Collision defaults to the same mesh unless a separate collision
        mesh is provided or collision is disabled. Set mass>0 to let gravity act on
        the mesh. If `force_convex_for_dynamic` is True, dynamic bodies will use a
        convex collision shape (PyBullet ignores concave collision on dynamic bodies).
        Set `align_aabb_center=True` to reposition the body so its AABB center matches
        the requested position (helpful for off-center mesh origins). Defaults to False.
        """

        def _coerce_xyz(values: Sequence[float]) -> list[float]:
            coords = list(values)
            if len(coords) != 3:
                raise ValueError(f"Expected a length-3 position, got {coords}")
            return [float(v) for v in coords]

        mesh = Path(mesh_path).expanduser()
        if not mesh.exists():
            raise FileNotFoundError(f"Kinematic mesh not found: {mesh}")

        collision_mesh: Path | None = None
        if collision_mesh_path is not None:
            collision_mesh = Path(collision_mesh_path).expanduser()
            if not collision_mesh.exists():
                raise FileNotFoundError(f"Collision mesh not found: {collision_mesh}")

        pos_xyz = _coerce_xyz(position)
        scale_vec = self._as_vec3(scale)
        collision_scale_vec = self._as_vec3(collision_scale) if collision_scale is not None else scale_vec

        if orientation_quat_xyzw is not None:
            quat_vals = list(orientation_quat_xyzw)
            if len(quat_vals) != 4:
                raise ValueError(f"Quaternion must have 4 values (x, y, z, w), got {quat_vals}")
            orientation_xyzw = tuple(float(v) for v in quat_vals)
        elif orientation_euler_deg is not None:
            euler_vals = list(orientation_euler_deg)
            if len(euler_vals) != 3:
                raise ValueError(f"Euler angles must be length 3 (roll, pitch, yaw), got {euler_vals}")
            orientation_xyzw = tuple(
                p.getQuaternionFromEuler([math.radians(float(v)) for v in euler_vals])
            )  # returns (x, y, z, w)
        else:
            orientation_xyzw = (0.0, 0.0, 0.0, 1.0)

        color_rgba: tuple[float, float, float, float] | None = None
        if rgba is not None:
            color_vals = list(rgba)
            if len(color_vals) == 3:
                color_vals.append(1.0)
            if len(color_vals) != 4:
                raise ValueError(f"RGBA must have 3 or 4 values, got {color_vals}")
            color_rgba = tuple(float(v) for v in color_vals)
        if color_rgba is None:
            color_rgba = (0.92, 0.92, 0.92, 1.0)

        mass = max(0.0, float(mass))

        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=str(mesh),
            meshScale=scale_vec,
            rgbaColor=color_rgba,
            specularColor=[0.05, 0.05, 0.05],
            physicsClientId=self.client,
        )

        collision_shape = -1
        used_collision_path: Path | None = None
        if enable_collision:
            used_collision_path = collision_mesh or mesh
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_MESH,
                fileName=str(used_collision_path),
                meshScale=collision_scale_vec,
                flags=(
                    0
                    if (mass > 0.0 and force_convex_for_dynamic)
                    else p.GEOM_FORCE_CONCAVE_TRIMESH
                ),
                physicsClientId=self.client,
            )
            if collision_shape < 0:
                raise RuntimeError(f"Failed to create collision shape from {used_collision_path}")

        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=pos_xyz,
            baseOrientation=orientation_xyzw,
            physicsClientId=self.client,
        )

        if align_aabb_center:
            try:
                aabb_min, aabb_max = p.getAABB(body_id, -1, physicsClientId=self.client)
                center = [(a + b) * 0.5 for a, b in zip(aabb_min, aabb_max)]
                delta = [pos_xyz[i] - center[i] for i in range(3)]
                if any(abs(d) > 1e-8 for d in delta):
                    curr_pos, curr_orn = p.getBasePositionAndOrientation(body_id, physicsClientId=self.client)
                    new_pos = [curr_pos[i] + delta[i] for i in range(3)]
                    p.resetBasePositionAndOrientation(
                        body_id,
                        new_pos,
                        curr_orn,
                        physicsClientId=self.client,
                    )
            except Exception:
                pass
        friction_kwargs = {
            "lateralFriction": 1.4,
            "restitution": 0.0,
            "rollingFriction": 0.04,
            "spinningFriction": 0.04,
            "contactProcessingThreshold": 0.0,
            "contactStiffness": 8e4,
            "contactDamping": 6e3,
        }
        if mass > 0.0:
            ccd_radius = 0.015  # higher minimum to help very small scaled meshes
            try:
                aabb_min, aabb_max = p.getAABB(body_id, -1, physicsClientId=self.client)
                max_dim = max((aabb_max[i] - aabb_min[i]) for i in range(3))
                ccd_radius = max(ccd_radius, max_dim * 0.7)
            except Exception:
                pass
            friction_kwargs["ccdSweptSphereRadius"] = ccd_radius
        p.changeDynamics(
            body_id,
            -1,
            physicsClientId=self.client,
            **friction_kwargs,
        )
        self.kinematic_objects.append(
            KinematicObject(
                body_id=body_id,
                visual_path=mesh,
                collision_path=used_collision_path,
                mass=mass,
                position=tuple(pos_xyz),
                orientation_xyzw=tuple(orientation_xyzw),
                scale=tuple(scale_vec),
                rgba=color_rgba,
            )
        )
        return body_id

    def list_kinematic_objects(self) -> list[KinematicObject]:
        """Return a shallow copy of the currently spawned kinematic meshes."""
        return list(self.kinematic_objects)

    def _lock_joint_by_name(self, names: set[str]) -> None:
        """Record the current position of any movable joints whose names appear in `names`."""
        if not names or self.robot_id is None:
            return
        for idx in self.movable_joint_indices:
            name = self.joint_names[idx]
            if name not in names:
                continue
            try:
                state = p.getJointState(self.robot_id, idx, physicsClientId=self.client)
                lock_val = state[0]
            except Exception:
                lock_val = 0.0
            self._locked_joint_positions[idx] = lock_val
            if idx not in self.locked_joint_indices:
                self.locked_joint_indices.append(idx)

    # -------- Locked joint helpers (used by GUI overlays/viewers) --------
    def locked_joint_info(self) -> list[tuple[int, str, float, float, float]]:
        """Return locked joint metadata: (index, name, lower, upper, value)."""
        info_list: list[tuple[int, str, float, float, float]] = []
        if self.robot_id is None:
            return info_list
        for idx in self.locked_joint_indices:
            try:
                info = p.getJointInfo(self.robot_id, idx, physicsClientId=self.client)
                lower, upper = info[8], info[9]
                if lower >= upper:
                    lower, upper = -math.pi, math.pi
                state = p.getJointState(self.robot_id, idx, physicsClientId=self.client)
                val = state[0]
            except Exception:
                lower, upper, val = -math.pi, math.pi, 0.0
            info_list.append((idx, self.joint_names[idx], lower, upper, val))
        return info_list

    def set_locked_joint_value(self, joint_idx: int, value: float) -> None:
        """Update a locked joint's hold position."""
        if joint_idx not in self.locked_joint_indices:
            return
        val = float(value)
        self._locked_joint_positions[joint_idx] = val
        try:
            p.resetJointState(self.robot_id, joint_idx, val, physicsClientId=self.client)
        except Exception:
            pass

    @staticmethod
    def _as_vec3(scale: float | Sequence[float]) -> list[float]:
        """Coerce a float or XYZ sequence to a 3-element scale vector."""
        if isinstance(scale, (int, float)):
            return [float(scale)] * 3
        values = list(scale)
        if len(values) != 3:
            raise ValueError(f"Mesh scale must be a scalar or length-3 sequence, got {values}")
        return [float(v) for v in values]
