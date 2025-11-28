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


@dataclass
class PointLabel:
    """Metadata for a user-defined point annotation in the scene."""

    label_id: int
    name: str
    position: tuple[float, float, float]
    color_rgba: tuple[float, float, float, float]
    text_scale: float
    marker_scale: float
    show_coords: bool


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
    # Revert to tighter release detection; rely on lock grace period to ignore initial jitter.
    GRIPPER_RELEASE_THRESHOLD = 0.0001  # radians of motion before starting release timer
    GRIPPER_RELEASE_TIME = 0.005  # seconds gripper must exceed threshold to unlock hoop
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
        self._robot_link_indices: list[int] = []
        # Joints we want to hold fixed even if commanded (e.g., gripper 1B/2B).
        self._locked_joint_positions: dict[int, float] = {}
        self.locked_joint_indices: list[int] = []
        self._locked_slider_ids: dict[int, int] = {}
        self.kinematic_objects: list[KinematicObject] = []
        self.point_labels: list[PointLabel] = []
        self._point_label_counter: int = 0
        # Grasp/locking helpers
        self._gripper_b_links: list[int] = []
        self._gripper_control_indices: list[int] = []
        self._hoop_body_ids: set[int] = set()
        self._hoop_contact_timer: dict[int, float] = {}
        self._active_hoop_constraint: int | None = None
        self._active_hoop_body: int | None = None
        self._last_gripper_joint_positions: list[float] | None = None
        self._hoop_collision_disabled: set[int] = set()
        self._active_hoop_info: dict[str, object] | None = None
        self._gripper_move_timer: float = 0.0
        self._hoop_release_cooldown: dict[int, float] = {}
        self._hoop_lock_grace: float = 0.0

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
        self._robot_link_indices = [-1] + self.joint_indices
        # Filter out fixed joints for control commands.
        self.movable_joint_indices = [
            j
            for j in self.joint_indices
            if p.getJointInfo(self.robot_id, j, physicsClientId=self.client)[2] != p.JOINT_FIXED
        ]
        self._joint_index_to_pos = {idx: i for i, idx in enumerate(self.movable_joint_indices)}
        # Lock gripper joints 1B/2B so they stay static in the simulation.
        self._lock_joint_by_name({"GRIPPER_JOINT1B", "GRIPPER_JOINT2B"})
        self._gripper_b_links = [
            self.link_name_to_index[name]
            for name in ("GRIPPER_LINK1B", "GRIPPER_LINK2B")
            if name in self.link_name_to_index
        ]
        self._gripper_control_indices = [
            idx
            for idx in self.movable_joint_indices
            if self.joint_names[idx] in {"GRIPPER_JOINT1A", "GRIPPER_JOINT2A"}
        ]

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
            self._create_locked_joint_sliders()

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

    def step(self, n: int = 1, dt: float | None = None) -> None:
        """Advance the simulation by n steps (no-op if running in real-time mode).

        If dt is provided, it is split across the steps and passed to contact/grasp
        helpers so timing stays in wall-clock seconds even if render FPS varies.
        """
        if self.real_time:
            return
        n_steps = max(1, int(n))
        dt_per_step = self.time_step if dt is None else float(dt) / n_steps
        for _ in range(n_steps):
            p.stepSimulation(physicsClientId=self.client)
            self._update_hoop_grasp(dt_per_step)

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
        pad_links = {"GRIPPER_LINK1B", "GRIPPER_LINK2B"}
        default_friction = (0.9, 0.01, 0.01)  # lateral, rolling, spinning
        gripper_friction = (1.6, 0.04, 0.04)  # main gripper surfaces
        pad_friction = (2.5, 0.1, 0.1)  # foam-like pads on inner B faces
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
            if name in pad_links:
                fric = pad_friction
                stiffness = 6e4
                damping = 1e4
                ccd = 0.025
            elif name in gripper_links:
                fric = gripper_friction
                stiffness = 6e4
                damping = 8e3
                ccd = 0.02
            else:
                fric = default_friction
                stiffness = 8e4
                damping = 6e3
                ccd = 0.0
            p.changeDynamics(
                self.robot_id,
                idx,
                lateralFriction=fric[0],
                rollingFriction=fric[1],
                spinningFriction=fric[2],
                restitution=0.0,
                contactProcessingThreshold=0.0,
                contactStiffness=stiffness,
                contactDamping=damping,
                ccdSweptSphereRadius=ccd,
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
        collision_segments: dict[str, object] | None = None,
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
        Provide `collision_segments` to build a compound collision shape from repeated
        mesh segments (e.g., a hoop made of convex slices arranged in a ring).
        """

        mesh = Path(mesh_path).expanduser()
        if not mesh.exists():
            raise FileNotFoundError(f"Kinematic mesh not found: {mesh}")

        collision_mesh: Path | None = None
        if collision_mesh_path is not None:
            collision_mesh = Path(collision_mesh_path).expanduser()
            if not collision_mesh.exists():
                raise FileNotFoundError(f"Collision mesh not found: {collision_mesh}")

        pos_xyz = self._coerce_xyz(position)
        scale_vec = self._as_vec3(scale)
        collision_scale_vec = self._as_vec3(collision_scale) if collision_scale is not None else scale_vec

        def _create_segment_ring_collision(
            segment_mesh: Path,
            *,
            radius_m: float,
            yaw_step_deg: float,
            count_override: int | None = None,
        ) -> int:
            if yaw_step_deg <= 0.0:
                raise ValueError(f"yaw_step_deg must be > 0, got {yaw_step_deg}")
            count = count_override if count_override is not None else int(round(360.0 / yaw_step_deg))
            count = max(1, count)
            yaw_step_rad = math.radians(yaw_step_deg)
            positions = []
            orientations = []
            for i in range(count):
                yaw = yaw_step_rad * i
                c, s = math.cos(yaw), math.sin(yaw)
                positions.append([radius_m * c, radius_m * s, 0.0])
                orientations.append(p.getQuaternionFromEuler([0.0, 0.0, yaw]))
            segment_flags = (
                0
                if (mass > 0.0 and force_convex_for_dynamic)
                else p.GEOM_FORCE_CONCAVE_TRIMESH
            )
            collision_shape = p.createCollisionShapeArray(
                shapeTypes=[p.GEOM_MESH] * count,
                fileNames=[str(segment_mesh)] * count,
                meshScales=[list(collision_scale_vec)] * count,
                flags=[segment_flags] * count,
                collisionFramePositions=positions,
                collisionFrameOrientations=orientations,
                physicsClientId=self.client,
            )
            if collision_shape < 0:
                raise RuntimeError(f"Failed to create compound collision from {segment_mesh}")
            return collision_shape

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
            if collision_segments:
                segment_mesh = Path(collision_segments.get("mesh_path", collision_mesh or mesh))
                if not segment_mesh.exists():
                    raise FileNotFoundError(f"Collision segment mesh not found: {segment_mesh}")
                yaw_step = float(collision_segments.get("yaw_step_deg", 29.9))
                count_override = collision_segments.get("count")
                count_override = int(count_override) if count_override is not None else None
                segment_radius_units = float(collision_segments.get("radius", 68.0 / 2.0))
                # Assume uniform scaling; fall back to X scale if not uniform.
                radius_scale = collision_scale_vec[0]
                if not all(abs(radius_scale - v) < 1e-8 for v in collision_scale_vec):
                    radius_scale = collision_scale_vec[0]
                collision_shape = _create_segment_ring_collision(
                    segment_mesh,
                    radius_m=segment_radius_units * radius_scale,
                    yaw_step_deg=yaw_step,
                    count_override=count_override,
                )
                used_collision_path = segment_mesh
            else:
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
        is_hoop = "hoop" in mesh.name.lower()
        friction_kwargs = {
            # Hoops get higher friction to avoid sliding off stands too easily.
            "lateralFriction": 0.8 if is_hoop else 1.4,
            "restitution": 0.0,
            "rollingFriction": 0.03 if is_hoop else 0.04,
            "spinningFriction": 0.03 if is_hoop else 0.04,
            "contactProcessingThreshold": 0.0,
            "contactStiffness": 8e4 if is_hoop else 8e4,
            "contactDamping": 6e3 if is_hoop else 6e3,
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
        if "hoop" in mesh.name.lower():
            self._hoop_body_ids.add(body_id)
        return body_id

    def list_kinematic_objects(self) -> list[KinematicObject]:
        """Return a shallow copy of the currently spawned kinematic meshes."""
        return list(self.kinematic_objects)

    # ---------- Point labels (for Panda3D annotations) ----------
    def add_point_label(
        self,
        name: str,
        position: Sequence[float],
        *,
        color: Sequence[float] | None = None,
        text_scale: float = 0.025,
        marker_scale: float = 0.1,
        show_coords: bool = True,
    ) -> int:
        """
        Store a labeled 3D point for visualization in the Panda3D viewer.

        Returns a unique label id that can be used to identify/remove it later.
        `text_scale` controls the rendered text size; `marker_scale` controls the
        crosshair size. Defaults keep labels small and unobtrusive.
        """
        pos_xyz = self._coerce_xyz(position)
        rgba = self._coerce_rgba(color, default=(0.9, 0.2, 0.2, 1.0))
        label_id = self._point_label_counter
        self._point_label_counter += 1
        label = PointLabel(
            label_id=label_id,
            name=str(name),
            position=tuple(pos_xyz),
            color_rgba=rgba,
            text_scale=max(0.001, float(text_scale)),
            marker_scale=max(0.001, float(marker_scale)),
            show_coords=bool(show_coords),
        )
        self.point_labels.append(label)
        return label_id

    def clear_point_labels(self) -> None:
        """Remove all stored point labels."""
        self.point_labels.clear()
        self._point_label_counter = 0

    def update_point_label(
        self,
        label_id: int,
        *,
        position: Sequence[float] | None = None,
        color: Sequence[float] | None = None,
        name: str | None = None,
        text_scale: float | None = None,
        marker_scale: float | None = None,
        show_coords: bool | None = None,
    ) -> bool:
        """
        Mutate an existing point label (position, color, name, scales, show_coords).

        Returns True if the label was found and updated.
        """
        for lbl in self.point_labels:
            if getattr(lbl, "label_id", None) != label_id:
                continue
            if position is not None:
                lbl.position = tuple(self._coerce_xyz(position))
            if color is not None:
                lbl.color_rgba = self._coerce_rgba(color, default=lbl.color_rgba)
            if name is not None:
                lbl.name = str(name)
            if text_scale is not None:
                lbl.text_scale = max(0.001, float(text_scale))
            if marker_scale is not None:
                lbl.marker_scale = max(0.001, float(marker_scale))
            if show_coords is not None:
                lbl.show_coords = bool(show_coords)
            return True
        return False

    def list_point_labels(self) -> list[PointLabel]:
        """Return a shallow copy of stored point labels."""
        return list(self.point_labels)

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

    # ---------- Hoop reset helper (used by Panda viewer button) ----------
    def reset_hoops(self) -> None:
        """Reset hoop-like kinematic objects to their original pose if they came from add_kinematic_object."""
        if not getattr(self, "kinematic_objects", None):
            return
        self._release_hoop_constraint()
        self._hoop_contact_timer.clear()
        for obj in self.kinematic_objects:
            try:
                p.resetBasePositionAndOrientation(
                    obj.body_id,
                    obj.position,
                    obj.orientation_xyzw,
                    physicsClientId=self.client,
                )
            except Exception:
                continue
        self._active_hoop_info = None
        self._gripper_move_timer = 0.0

    @staticmethod
    def _as_vec3(scale: float | Sequence[float]) -> list[float]:
        """Coerce a float or XYZ sequence to a 3-element scale vector."""
        if isinstance(scale, (int, float)):
            return [float(scale)] * 3
        values = list(scale)
        if len(values) != 3:
            raise ValueError(f"Mesh scale must be a scalar or length-3 sequence, got {values}")
        return [float(v) for v in values]

    @staticmethod
    def _coerce_xyz(values: Sequence[float]) -> list[float]:
        """Ensure a length-3 position tuple."""
        coords = list(values)
        if len(coords) != 3:
            raise ValueError(f"Expected a length-3 position, got {coords}")
        return [float(v) for v in coords]

    @staticmethod
    def _coerce_rgba(values: Sequence[float] | None, default: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        """Normalize RGB/RGBA input to a 4-tuple."""
        if values is None:
            return default
        color_vals = list(values)
        if len(color_vals) == 3:
            color_vals.append(1.0)
        if len(color_vals) != 4:
            raise ValueError(f"RGBA must have 3 or 4 values, got {color_vals}")
        return tuple(float(v) for v in color_vals)

    # ---------- Hoop grasp/lock helpers ----------
    def _release_hoop_constraint(self) -> None:
        pose: tuple[tuple[float, float, float], tuple[float, float, float, float]] | None = None
        if self._active_hoop_body is not None:
            try:
                pose = p.getBasePositionAndOrientation(self._active_hoop_body, physicsClientId=self.client)
            except Exception:
                pose = None
        if self._active_hoop_constraint is not None:
            try:
                p.removeConstraint(self._active_hoop_constraint, physicsClientId=self.client)
            except Exception:
                pass
        if self._active_hoop_body is not None:
            hoop_id = self._active_hoop_body
            # Keep collisions disabled during a short cooldown to avoid immediate re-collision.
            self._disable_hoop_collision(hoop_id)
            self._hoop_release_cooldown[hoop_id] = 0.2
            if pose is not None:
                try:
                    p.resetBasePositionAndOrientation(hoop_id, pose[0], pose[1], physicsClientId=self.client)
                except Exception:
                    pass
        self._active_hoop_constraint = None
        self._active_hoop_body = None
        self._active_hoop_info = None

    def _gripper_joint_positions(self) -> list[float]:
        if not self._gripper_control_indices or self.robot_id is None:
            return []
        try:
            states = p.getJointStates(self.robot_id, self._gripper_control_indices, physicsClientId=self.client)
            return [s[0] for s in states]
        except Exception:
            return []

    def _update_hoop_grasp(self, dt: float) -> None:
        """Lock a hoop to the gripper if any B link maintains contact; release on gripper motion."""
        if not self._hoop_body_ids or not self._gripper_b_links:
            return
        # Handle release grace periods: re-enable collisions once cooldown elapses.
        for hoop_id in list(self._hoop_release_cooldown.keys()):
            remaining = self._hoop_release_cooldown[hoop_id] - dt
            if remaining <= 0.0:
                self._enable_hoop_collision(hoop_id)
                self._hoop_release_cooldown.pop(hoop_id, None)
            else:
                self._hoop_release_cooldown[hoop_id] = remaining
        # If locked, drop when gripper joints move.
        current_grip = self._gripper_joint_positions()
        if self._active_hoop_constraint is not None:
            # Ignore small gripper motion immediately after lock to avoid accidental drops.
            if self._hoop_lock_grace > 0.0:
                self._hoop_lock_grace = max(0.0, self._hoop_lock_grace - dt)
                self._last_gripper_joint_positions = current_grip
                return
            if self._last_gripper_joint_positions and current_grip:
                delta = max(abs(a - b) for a, b in zip(current_grip, self._last_gripper_joint_positions))
                if delta > self.GRIPPER_RELEASE_THRESHOLD:
                    self._gripper_move_timer += dt
                    if self._gripper_move_timer >= self.GRIPPER_RELEASE_TIME:
                        self._release_hoop_constraint()
                        self._hoop_contact_timer.clear()
                        self._gripper_move_timer = 0.0
                        self._last_gripper_joint_positions = current_grip
                        return
                else:
                    self._gripper_move_timer = 0.0
            else:
                self._gripper_move_timer = 0.0
            self._last_gripper_joint_positions = current_grip
            return
        self._last_gripper_joint_positions = current_grip
        self._gripper_move_timer = 0.0

        for hoop_id in list(self._hoop_body_ids):
            contacts = p.getContactPoints(bodyA=self.robot_id, bodyB=hoop_id, physicsClientId=self.client)
            b_link_contacts = [c for c in contacts if c[3] in self._gripper_b_links]
            links_hit = {c[3] for c in b_link_contacts}
            if set(self._gripper_b_links).issubset(links_hit) and b_link_contacts:
                self._hoop_contact_timer[hoop_id] = self._hoop_contact_timer.get(hoop_id, 0.0) + dt
                if self._hoop_contact_timer[hoop_id] >= 0.05:
                    c = b_link_contacts[0]
                    parent_link = c[3]
                    child_link = c[4]
                    pos_on_a = c[5]
                    pos_on_b = c[6]

                    def body_pose(body_id: int, link_idx: int) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
                        if link_idx == -1:
                            return p.getBasePositionAndOrientation(body_id, physicsClientId=self.client)
                        state = p.getLinkState(body_id, link_idx, computeForwardKinematics=True, physicsClientId=self.client)
                        return state[4], state[5]

                    parent_world_pos, parent_world_orn = body_pose(self.robot_id, parent_link)
                    child_world_pos, child_world_orn = body_pose(hoop_id, child_link)
                    inv_parent_pos, inv_parent_orn = p.invertTransform(parent_world_pos, parent_world_orn)
                    parent_local_pos, parent_local_orn = p.multiplyTransforms(
                        inv_parent_pos, inv_parent_orn, child_world_pos, child_world_orn
                    )
                    parent_local = parent_local_pos
                    child_local = (0.0, 0.0, 0.0)
                    try:
                        cid = p.createConstraint(
                            parentBodyUniqueId=self.robot_id,
                            parentLinkIndex=parent_link,
                            childBodyUniqueId=hoop_id,
                            childLinkIndex=child_link,
                            jointType=p.JOINT_FIXED,
                            jointAxis=[0.0, 0.0, 0.0],
                            parentFramePosition=parent_local,
                            parentFrameOrientation=parent_local_orn,
                            childFramePosition=child_local,
                            childFrameOrientation=[0.0, 0.0, 0.0, 1.0],
                            physicsClientId=self.client,
                        )
                        p.changeConstraint(
                            cid,
                            maxForce=400.0,
                            erp=0.2,
                            physicsClientId=self.client,
                        )
                        self._active_hoop_constraint = cid
                        self._active_hoop_body = hoop_id
                        self._active_hoop_info = {
                            "parent_link": parent_link,
                            "parent_local": parent_local,
                            "child_link": child_link,
                            "child_local": child_local,
                            "contact_normal": c[7] if len(c) > 7 else (0.0, 0.0, 1.0),
                        }
                        self._disable_hoop_collision(hoop_id)
                        # Short grace window to ignore immediate gripper jitter after lock.
                        self._hoop_lock_grace = 0.2
                    except Exception:
                        pass
                    self._hoop_contact_timer[hoop_id] = 0.0
                    break
            else:
                self._hoop_contact_timer[hoop_id] = 0.0

    def get_active_hoop_info(self) -> dict[str, object] | None:
        """Return info about the currently locked hoop (if any)."""
        if self._active_hoop_body is None or self._active_hoop_info is None:
            return None
        info = dict(self._active_hoop_info)
        info["hoop_body_id"] = self._active_hoop_body
        return info

    def _disable_hoop_collision(self, hoop_id: int) -> None:
        """Disable collisions between the hoop and the robot while locked."""
        if hoop_id in self._hoop_collision_disabled:
            return
        for link_idx in self._robot_link_indices:
            try:
                p.setCollisionFilterPair(self.robot_id, hoop_id, link_idx, -1, 0, physicsClientId=self.client)
            except Exception:
                continue
        self._hoop_collision_disabled.add(hoop_id)

    def _enable_hoop_collision(self, hoop_id: int) -> None:
        """Re-enable collisions once unlocked."""
        if hoop_id not in self._hoop_collision_disabled:
            return
        for link_idx in self._robot_link_indices:
            try:
                p.setCollisionFilterPair(self.robot_id, hoop_id, link_idx, -1, 1, physicsClientId=self.client)
            except Exception:
                continue
        self._hoop_collision_disabled.discard(hoop_id)
