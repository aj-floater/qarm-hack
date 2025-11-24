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
        attach_gripper: bool = False,
        enable_gripper_sliders: bool = False,
        gripper_only: bool = False,
        gripper_urdf_path: Path | None = None,
        gripper_search_paths: Sequence[Path] | None = None,
        disable_gripper_self_collisions: bool = True,
        static_gripper: bool = False,
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
        self.attach_gripper = attach_gripper and not gripper_only
        self.enable_gripper_sliders = enable_gripper_sliders
        self.gripper_only = gripper_only
        self.floor_id: int | None = None
        self.robot_id: int | None = None
        self._joint_slider_ids: list[int] = []
        self._gripper_slider_ids: list[int] = []
        self.gripper_id: int | None = None
        self.gripper_joint_indices: list[int] = []
        self.gripper_joint_names: list[str] = []
        self.joint_indices: list[int] = []
        self.joint_names: list[str] = []
        self.link_name_to_index: dict[str, int] = {}
        self.movable_joint_indices: list[int] = []
        self.disable_gripper_self_collisions = disable_gripper_self_collisions
        self.static_gripper = static_gripper

        default_gripper_urdf = Path(__file__).resolve().parent.parent / "qarm_gripper" / "urdf" / "qarm_gripper.urdf"
        self.gripper_urdf = Path(gripper_urdf_path) if gripper_urdf_path is not None else default_gripper_urdf

        default_search_paths = [
            Path(__file__).resolve().parent.parent / "qarm_gripper",
            Path(__file__).resolve().parent.parent / "qarm_gripper_new",
        ]
        self.gripper_search_paths = list(gripper_search_paths) if gripper_search_paths is not None else default_search_paths

        p.setTimeStep(self.time_step, physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setRealTimeSimulation(1 if self.real_time else 0, physicsClientId=self.client)

        if self.gui_enabled:
            self._configure_gui()

        for path in self.gripper_search_paths:
            if path.exists():
                p.setAdditionalSearchPath(str(path), physicsClientId=self.client)

        if add_ground or self.gripper_only:
            self.floor_id = self._create_floor(enable_collision=True)

        if self.gripper_only:
            if not self.gripper_urdf.exists():
                raise FileNotFoundError(f"Gripper URDF not found at {self.gripper_urdf}")
            self.gripper_id = self._load_gripper_body(
                gripper_urdf=self.gripper_urdf,
                base_pos=[0.0, 0.0, 0.0],
                base_orn=[0.0, 0.0, 0.0, 1.0],
                use_fixed_base=True,
            )
            if self.gui_enabled:
                self._focus_camera_on_gripper()
            if self.gui_enabled and self.enable_gripper_sliders:
                self._create_gripper_sliders()
            return

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

        if self.attach_gripper:
            self._attach_gripper()

        if self.gui_enabled and self.enable_joint_sliders:
            self._create_joint_sliders()
        if self.gui_enabled and self.enable_gripper_sliders and self.gripper_id is not None:
            self._create_gripper_sliders()

    def reset(self, home: Sequence[float] | None = None, gripper_home: Sequence[float] | None = None) -> None:
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

        if self.gripper_id is not None and self.gripper_joint_indices:
            g_target = list(gripper_home) if gripper_home is not None else [0.0] * len(self.gripper_joint_indices)
            if len(g_target) != len(self.gripper_joint_indices):
                raise ValueError(
                    f"Gripper home length {len(g_target)} != gripper joints {len(self.gripper_joint_indices)}"
                )
            for joint_id, angle in zip(self.gripper_joint_indices, g_target):
                p.resetJointState(self.gripper_id, joint_id, angle, physicsClientId=self.client)

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

    def set_gripper_joint_positions(self, q: Sequence[float], max_force: float = 2.0) -> None:
        """
        Command the attached gripper joints (if present).

        q order follows the joint order defined in qarm_gripper/urdf/qarm_gripper.urdf.
        """
        if self.gripper_id is None or not self.gripper_joint_indices:
            raise RuntimeError("No gripper loaded. Initialize with gripper_only=True or attach_gripper=True.")
        if len(q) != len(self.gripper_joint_indices):
            raise ValueError(f"Expected {len(self.gripper_joint_indices)} gripper joints, got {len(q)}")
        p.setJointMotorControlArray(
            self.gripper_id,
            jointIndices=self.gripper_joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=list(q),
            forces=[max_force] * len(self.gripper_joint_indices),
            physicsClientId=self.client,
        )

    def get_gripper_joint_positions(self) -> list[float]:
        """Return current joint positions for the attached gripper."""
        if self.gripper_id is None or not self.gripper_joint_indices:
            raise RuntimeError("No gripper loaded. Initialize with gripper_only=True or attach_gripper=True.")
        states = p.getJointStates(self.gripper_id, self.gripper_joint_indices, physicsClientId=self.client)
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

    def _create_gripper_sliders(self) -> None:
        """Create sliders for the attached gripper joints."""
        self._gripper_slider_ids.clear()
        for idx, name in zip(self.gripper_joint_indices, self.gripper_joint_names):
            slider_id = p.addUserDebugParameter(
                paramName=f"{name} (rad)",
                rangeMin=-1.0,
                rangeMax=1.0,
                startValue=0.0,
            )
            self._gripper_slider_ids.append(slider_id)

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
            rgbaColor=(*self.DARK_FLOOR_COLOR[:3], 0.05),
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

    def _focus_camera_on_gripper(self) -> None:
        """Pull the camera toward the origin where the gripper sits."""
        p.resetDebugVisualizerCamera(
            cameraDistance=0.3,
            cameraYaw=45,
            cameraPitch=-35,
            cameraTargetPosition=[0.0, 0.0, 0.05],
            physicsClientId=self.client,
        )

    def _load_gripper_body(
        self,
        gripper_urdf: Path,
        base_pos: Sequence[float],
        base_orn: Sequence[float],
        use_fixed_base: bool,
    ) -> int:
        """Load the gripper URDF and collect joint metadata."""
        gripper_id = p.loadURDF(
            str(gripper_urdf),
            basePosition=base_pos,
            baseOrientation=base_orn,
            useFixedBase=use_fixed_base or self.static_gripper,
            physicsClientId=self.client,
        )

        if self.disable_gripper_self_collisions:
            num_links = p.getNumJoints(gripper_id, physicsClientId=self.client)
            # Disable collisions between all link pairs (including base -1).
            link_indices = list(range(num_links))
            for i in [-1] + link_indices:
                for j in link_indices:
                    if i == j:
                        continue
                    p.setCollisionFilterPair(gripper_id, gripper_id, i, j, enableCollision=0, physicsClientId=self.client)

        num_joints = p.getNumJoints(gripper_id, physicsClientId=self.client)
        self.gripper_joint_indices = list(range(num_joints))
        self.gripper_joint_names = [
            p.getJointInfo(gripper_id, j, physicsClientId=self.client)[1].decode("utf-8")
            for j in self.gripper_joint_indices
        ]

        if self.static_gripper:
            # Lock joints in place with strong position holds and extra damping to prevent flailing.
            current_positions = [p.getJointState(gripper_id, j, physicsClientId=self.client)[0] for j in self.gripper_joint_indices]
            p.setJointMotorControlArray(
                gripper_id,
                jointIndices=self.gripper_joint_indices,
                controlMode=p.POSITION_CONTROL,
                targetPositions=current_positions,
                forces=[5.0] * len(self.gripper_joint_indices),
                positionGains=[1.0] * len(self.gripper_joint_indices),
                physicsClientId=self.client,
            )
            for j in self.gripper_joint_indices:
                p.changeDynamics(
                    gripper_id,
                    j,
                    linearDamping=1.0,
                    angularDamping=1.0,
                    physicsClientId=self.client,
                )
        else:
            # Disable default motors so we can drive the gripper manually.
            p.setJointMotorControlArray(
                gripper_id,
                jointIndices=self.gripper_joint_indices,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[0.0] * len(self.gripper_joint_indices),
                forces=[self.HOLD_FORCE] * len(self.gripper_joint_indices),
                physicsClientId=self.client,
            )
        return gripper_id

    def _attach_gripper(self) -> None:
        """Load qarm_gripper as a separate PyBullet body and constrain it to the end effector."""
        if not self.gripper_urdf.exists():
            raise FileNotFoundError(f"Gripper URDF not found at {self.gripper_urdf}")

        ee_index = self.link_name_to_index.get("END-EFFECTOR")
        if ee_index is None:
            raise RuntimeError("END-EFFECTOR link not found; cannot attach gripper.")

        ee_state = p.getLinkState(
            self.robot_id,
            ee_index,
            computeForwardKinematics=True,
            physicsClientId=self.client,
        )
        base_pos = ee_state[4]
        base_orn = ee_state[5]

        self.gripper_id = self._load_gripper_body(
            gripper_urdf=self.gripper_urdf,
            base_pos=base_pos,
            base_orn=base_orn,
            use_fixed_base=False,
        )

        constraint_id = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=ee_index,
            childBodyUniqueId=self.gripper_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
            parentFrameOrientation=[0, 0, 0, 1],
            childFrameOrientation=[0, 0, 0, 1],
            physicsClientId=self.client,
        )
        if constraint_id < 0:
            raise RuntimeError("Failed to create constraint between arm and gripper.")

    def apply_joint_slider_targets(self) -> None:
        """Read joint slider values (if enabled) and command the arm accordingly."""
        if self._joint_slider_ids:
            targets = [p.readUserDebugParameter(slider_id) for slider_id in self._joint_slider_ids]
            self.set_joint_positions(targets)
        if self._gripper_slider_ids and self.gripper_id is not None:
            g_targets = [p.readUserDebugParameter(slider_id) for slider_id in self._gripper_slider_ids]
            self.set_gripper_joint_positions(g_targets)
