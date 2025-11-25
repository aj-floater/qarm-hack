"""
Lightweight PyBullet environment wrapper for the QArm simulation.

This sets up a physics client, loads the URDF, captures joint metadata, and
offers helpers for stepping and basic joint control. Keep the joint ordering
consistent with the URDF:
- 0: world_base_joint (fixed, not driven)
- 1: YAW
- 2: SHOULDER
- 3: ELBOW
- 4: WRIST
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Sequence

try:
    import pybullet as p
except ImportError as exc:  # pragma: no cover - runtime guard
    raise SystemExit("pybullet is not installed. Run `pip install -e .` first.") from exc


class QArmSimEnv:
    """
    Container for PyBullet state and helper utilities.

    Minimal responsibilities:
    - start a PyBullet client (GUI or DIRECT),
    - load the QArm URDF from qarm/urdf,
    - record joint indices/names and expose controllable joints,
    - provide reset/step and joint position helpers.
    """

    DARK_LINK_COLOR = (0.15, 0.15, 0.18, 1.0)
    ACCENT_LINK_COLOR = (0.78, 0.12, 0.12, 1.0)
    DARK_FLOOR_COLOR = (0.1, 0.1, 0.1, 1.0)
    LIGHT_FLOOR_COLOR = (0.8, 0.8, 0.8, 1.0)
    BACKDROP_COLOR = (0.85, 0.85, 0.85, 1.0)
    HOLD_FORCE = 0.2  # small holding torque to keep joints from flopping when idle.

    def __init__(
        self,
        gui: bool = False,
        urdf_path: Path | None = None,
        time_step: float = 1.0 / 120.0,
        add_ground: bool = False,
        real_time: bool = False,
        dark_mode: bool = True,
        show_debug_gui: bool = False,
        show_camera_previews: bool = False,
        enable_joint_sliders: bool = False,
        base_mesh_path: Path | None = None,
        base_collision_mesh_path: Path | None = None,
        base_mesh_scale: float | Sequence[float] = 1.0,
        base_friction: float = 0.8,
        base_restitution: float = 0.0,
        base_yaw_deg: float = 180.0,
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

        p.setTimeStep(self.time_step, physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setRealTimeSimulation(1 if self.real_time else 0, physicsClientId=self.client)

        if self.gui_enabled:
            self._configure_gui()

        self.base_mesh_path = Path(base_mesh_path) if base_mesh_path else None
        self.base_collision_mesh_path = Path(base_collision_mesh_path) if base_collision_mesh_path else None
        self.base_mesh_scale = base_mesh_scale
        self.base_friction = float(base_friction)
        self.base_restitution = float(base_restitution)
        self.base_yaw_deg = float(base_yaw_deg)

        if self.base_mesh_path is not None:
            collision_path = self.base_collision_mesh_path or self.base_mesh_path
            print(
                "[QArmSimEnv] Using mesh floor:",
                f"visual={self.base_mesh_path}",
                f"collision={collision_path}",
                f"scale={self.base_mesh_scale}",
                f"friction={self.base_friction}",
                f"restitution={self.base_restitution}",
            )
            self.floor_id = self._create_mesh_floor(
                visual_path=self.base_mesh_path,
                collision_path=collision_path,
                mesh_scale=self.base_mesh_scale,
                friction=self.base_friction,
                restitution=self.base_restitution,
                yaw_deg=self.base_yaw_deg,
            )
        elif add_ground:
            self.floor_id = self._create_floor(enable_collision=True)

        urdf_path = urdf_path or Path(__file__).resolve().parent.parent / "qarm" / "urdf" / "QARM.urdf"
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

        # Disable default motor torques so we can drive positions explicitly.
        p.setJointMotorControlArray(
            self.robot_id,
            jointIndices=self.movable_joint_indices,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[0.0] * len(self.movable_joint_indices),
            forces=[self.HOLD_FORCE] * len(self.movable_joint_indices),
            physicsClientId=self.client,
        )

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
        p.setJointMotorControlArray(
            self.robot_id,
            jointIndices=self.movable_joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=list(q),
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
        mesh_scale: float | Sequence[float],
        friction: float,
        restitution: float,
        yaw_deg: float,
    ) -> int:
        """Create a static mesh floor that matches the Panda3D base model."""
        scale_vec = self._as_vec3(mesh_scale)
        if not visual_path.exists():
            raise FileNotFoundError(f"Base visual mesh not found: {visual_path}")
        if not collision_path.exists():
            raise FileNotFoundError(f"Base collision mesh not found: {collision_path}")

        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=str(visual_path),
            meshScale=scale_vec,
            rgbaColor=(0.82, 0.72, 0.55, 1.0),  # pine-like tint
            specularColor=[0.08, 0.08, 0.08],
            physicsClientId=self.client,
        )
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=str(collision_path),
            meshScale=scale_vec,
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
        accent_links = {1, 3}  # YAW and ELBOW joints look good with red highlights.
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

    @staticmethod
    def _as_vec3(scale: float | Sequence[float]) -> list[float]:
        """Coerce a float or XYZ sequence to a 3-element scale vector."""
        if isinstance(scale, (int, float)):
            return [float(scale)] * 3
        values = list(scale)
        if len(values) != 3:
            raise ValueError(f"Mesh scale must be a scalar or length-3 sequence, got {values}")
        return [float(v) for v in values]
