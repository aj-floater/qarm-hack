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
import math

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
        add_base_fixture: bool = True,
        base_fixture_path: Path | None = None,
        base_fixture_position: Sequence[float] = (0.0, 0.0, -0.02),
        base_fixture_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        base_fixture_scale: Sequence[float] = (0.001, 0.001, 0.001),
        base_fixture_color: Sequence[float] = (0.85, 0.85, 0.85, 1.0),
        add_darm_hoop: bool = True,
        darm_hoop_path: Path | None = None,
        darm_hoop_scale: Sequence[float] = (0.001, 0.001, 0.001),
        darm_hoop_color: Sequence[float] = (0.0, 1.0, 0.0, 1.0),  # Bright green for visibility
        darm_hoop_clearance: float = 0.0005,
        add_target_markers: bool = True,
        target_coordinates: Sequence[Sequence[float]] | None = None,
        target_marker_radius: float = 0.01,
        target_marker_color: Sequence[float] = (1.0, 0.0, 0.0, 1.0),  # Red markers
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
        self._joint_slider_ids: list[int] = []

        p.setTimeStep(self.time_step, physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setRealTimeSimulation(1 if self.real_time else 0, physicsClientId=self.client)

        if self.gui_enabled:
            self._configure_gui()

        urdf_path = urdf_path or Path(__file__).resolve().parent.parent / "qarm" / "urdf" / "QARM.urdf"
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF not found at {urdf_path}")

        self.robot_id = p.loadURDF(str(urdf_path), useFixedBase=True, physicsClientId=self.client)
        if self.dark_mode:
            self._apply_robot_palette()

        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client)
        self.joint_indices = list(range(num_joints))
        self.joint_names = [
            p.getJointInfo(self.robot_id, j, physicsClientId=self.client)[1].decode("utf-8") for j in self.joint_indices
        ]
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
            forces=[0.0] * len(self.movable_joint_indices),
            physicsClientId=self.client,
        )

        if self.gui_enabled and self.enable_joint_sliders:
            self._create_joint_sliders()

        # Initialize with default home pose instead of zeros
        default_home = [0.0, 0.5, -0.5, 0.0] if len(self.movable_joint_indices) == 4 else [0.0] * len(self.movable_joint_indices)
        self._last_joint_command = default_home
        # Set initial pose immediately so arm doesn't flop around
        self.set_joint_positions(default_home)
        
        self.base_fixture_id: int | None = None
        if add_base_fixture:
            fixture_mesh = base_fixture_path or Path(__file__).resolve().parent.parent / "qarm" / "meshes" / "QarmBase.stl"
            if not fixture_mesh.exists():
                raise FileNotFoundError(f"Base fixture mesh not found at {fixture_mesh}")
            self.base_fixture_id = self._load_static_mesh(
                mesh_path=fixture_mesh,
                position=base_fixture_position,
                rpy=base_fixture_rpy,
                scale=base_fixture_scale,
                color=base_fixture_color,
                enable_collision=False,
            )

        self.darm_hoop_id: int | None = None
        self.darm_hoop_center: tuple[float, float, float] | None = None
        if add_darm_hoop:
            hoop_mesh = darm_hoop_path or Path(__file__).resolve().parent.parent / "qarm" / "meshes" / "QarmHoop.stl"
            if not hoop_mesh.exists():
                raise FileNotFoundError(f"DarmHoop mesh not found at {hoop_mesh}")
            self.darm_hoop_id, self.darm_hoop_center = self._load_darm_hoop(
                hoop_mesh=hoop_mesh,
                hoop_scale=darm_hoop_scale,
                hoop_color=darm_hoop_color,
                hoop_clearance=darm_hoop_clearance,
                base_fixture_id=self.base_fixture_id,
                base_fixture_position=base_fixture_position,
                base_fixture_rpy=base_fixture_rpy,
            )
            print(f"[QArmSimEnv] DarmHoop center (world): {self.darm_hoop_center}")

        # Create target coordinate markers for inverse kinematics testing
        self.target_coordinates: list[tuple[float, float, float]] = []
        self.target_marker_ids: list[int] = []
        if add_target_markers:
            # Default target coordinates if none provided
            if target_coordinates is None:
                # Three reachable coordinates: spread out more for better visibility
                # Coordinates in (x, y, z) format, relative to arm base
                default_targets = [
                    (0.10, 0.20, 0.12),   # Target 1: Left side, forward, higher (red)
                    (0.25, 0.30, 0.40),   # Target 2: Right side, further forward, much higher (blue)
                    (0.15, 0.40, 0.06),   # Target 3: Center, far forward (orange)
                ]
                target_coordinates = default_targets
            
            # Different colors for each marker
            marker_colors = [
                (1.0, 0.0, 0.0, 1.0),  # Red for Target 1
                (0.0, 0.0, 1.0, 1.0),  # Blue for Target 2
                (1.0, 0.65, 0.0, 1.0), # Orange for Target 3
            ]
            
            self.target_coordinates = [tuple(coord) for coord in target_coordinates]
            print(f"[QArmSimEnv] Creating {len(self.target_coordinates)} target coordinate markers")
            
            for i, coord in enumerate(self.target_coordinates):
                marker_color = marker_colors[i] if i < len(marker_colors) else target_marker_color
                marker_id = self._create_target_marker(
                    position=coord,
                    radius=target_marker_radius,
                    color=marker_color,
                    marker_id=i + 1,
                )
                self.target_marker_ids.append(marker_id)
                print(f"[QArmSimEnv] Target {i+1} marker at: {coord} (color: {marker_color[:3]})")

    def reset(self, home: Sequence[float] | None = None) -> None:
        """
        Reset joints to a home pose (defaults to a safe upright pose for movable joints).

        The length of `home` must match the number of movable joints.
        Default home pose: [0.0, 0.5, -0.5, 0.0] for [YAW, SHOULDER, ELBOW, WRIST]
        """
        # Default home pose: arm extended upward/outward instead of all zeros
        default_home = [0.0, 0.5, -0.5, 0.0] if len(self.movable_joint_indices) == 4 else [0.0] * len(self.movable_joint_indices)
        target = list(home) if home is not None else default_home
        if len(target) != len(self.movable_joint_indices):
            raise ValueError(f"Home pose length {len(target)} != movable joints {len(self.movable_joint_indices)}")
        for joint_id, angle in zip(self.movable_joint_indices, target):
            p.resetJointState(self.robot_id, joint_id, angle, physicsClientId=self.client)
        self.set_joint_positions(target)

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
        self._last_joint_command = list(q)

    def get_joint_positions(self, indices: Iterable[int] | None = None) -> list[float]:
        """
        Read joint angles (radians) for the requested indices (defaults to movable joints).
        """
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

        if self.enable_joint_sliders or self.show_debug_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=self.client)
        elif self.dark_mode:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)

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
                physicsClientId=self.client,
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
            rgbaColor=(*self.DARK_FLOOR_COLOR[:3], 0.2),
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

    def _load_static_mesh(
        self,
        mesh_path: Path,
        position: Sequence[float],
        rpy: Sequence[float],
        scale: Sequence[float],
        color: Sequence[float],
        enable_collision: bool,
    ) -> int:
        """Load a static mesh (e.g., game board) at a fixed pose."""
        orientation = p.getQuaternionFromEuler(rpy)
        collision_shape = (
            p.createCollisionShape(
                shapeType=p.GEOM_MESH,
                fileName=str(mesh_path),
                meshScale=list(scale),
                physicsClientId=self.client,
            )
            if enable_collision
            else -1
        )
        # Try loading the mesh - PyBullet may silently fail, so we'll verify it loaded
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=str(mesh_path),
            meshScale=list(scale),
            rgbaColor=list(color),
            specularColor=[0.1, 0.1, 0.1],
            physicsClientId=self.client,
        )
        
        # Verify the visual shape was created
        if visual_shape < 0:
            print(f"[QArmSimEnv] Warning: Failed to create visual shape from {mesh_path}, using fallback box")
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[0.05, 0.05, 0.01],
                rgbaColor=list(color),
                physicsClientId=self.client,
            )
        return p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=list(position),
            baseOrientation=orientation,
            physicsClientId=self.client,
        )

    def _load_darm_hoop(
        self,
        hoop_mesh: Path,
        hoop_scale: Sequence[float],
        hoop_color: Sequence[float],
        hoop_clearance: float,
        base_fixture_id: int | None,
        base_fixture_position: Sequence[float],
        base_fixture_rpy: Sequence[float],
    ) -> tuple[int, tuple[float, float, float]]:
        """
        Place the DarmHoop so that it rests on top of the base fixture (if present) and report its center.
        """
        fixture_top_z = base_fixture_position[2]
        if base_fixture_id is not None:
            _, aabb_max = p.getAABB(base_fixture_id, physicsClientId=self.client)
            fixture_top_z = aabb_max[2]

        # Offset hoop position to be visible on platform (not directly under Q arm base)
        hoop_offset_x = 0.0  # Keep centered in X
        hoop_offset_y = 0.3  # Move forward 30cm in Y direction to be visible
        # Load hoop at a provisional position
        provisional_position = (base_fixture_position[0] + hoop_offset_x, base_fixture_position[1] + hoop_offset_y, fixture_top_z + 0.01)
        hoop_id = self._load_static_mesh(
            mesh_path=hoop_mesh,
            position=provisional_position,
            rpy=base_fixture_rpy,
            scale=hoop_scale,
            color=hoop_color,
            enable_collision=False,
        )

        # Step simulation multiple times to ensure mesh is fully loaded and AABB is calculated
        for _ in range(20):
            p.stepSimulation(physicsClientId=self.client)

        # Try to get visual shape data to verify mesh loaded
        visual_data = p.getVisualShapeData(hoop_id, physicsClientId=self.client)
        if visual_data:
            print(f"[QArmSimEnv] DarmHoop visual shape data: {len(visual_data)} shape(s)")

        hoop_aabb_min, hoop_aabb_max = p.getAABB(hoop_id, physicsClientId=self.client)
        aabb_size = tuple(mx - mn for mn, mx in zip(hoop_aabb_min, hoop_aabb_max))
        
        # If AABB is invalid (zero size), create a fallback visual representation
        if aabb_size[0] < 0.001 or aabb_size[1] < 0.001 or aabb_size[2] < 0.001:
            print(f"[QArmSimEnv] Warning: DarmHoop STL mesh failed to load (AABB: {aabb_size})")
            print(f"[QArmSimEnv] Creating fallback hoop representation using simple geometry")
            
            # Remove the failed mesh body
            p.removeBody(hoop_id, physicsClientId=self.client)
            
            # Create a torus/ring shape as fallback (hollow in the middle)
            estimated_hoop_outer_radius = 0.05  # 50mm outer radius (100mm outer diameter)
            estimated_hoop_inner_radius = 0.04  # 40mm inner radius (80mm inner diameter) - creates hollow center
            estimated_hoop_thickness = 0.01  # 10mm thick
            hoop_bottom_z = fixture_top_z + hoop_clearance
            hoop_center_z = hoop_bottom_z + estimated_hoop_thickness / 2
            
            # Offset hoop position to be visible on platform (not directly under Q arm base)
            hoop_offset_x = 0.0  # Keep centered in X
            hoop_offset_y = 0.3  # Move forward 30cm in Y direction to be visible
            
            # Create a torus/ring shape using segments arranged in a circle
            num_segments = 16  # Number of segments to form the ring (fewer for simplicity)
            segment_angle = 2 * math.pi / num_segments
            torus_major_radius = (estimated_hoop_outer_radius + estimated_hoop_inner_radius) / 2
            torus_minor_radius = (estimated_hoop_outer_radius - estimated_hoop_inner_radius) / 2
            
            # Create visual shapes for torus segments arranged in a circle
            visual_shapes = []
            for i in range(num_segments):
                angle = i * segment_angle
                # Position of segment center on the torus circle
                x = torus_major_radius * math.cos(angle)
                y = torus_major_radius * math.sin(angle)
                z = 0
                
                # Create a small box segment for the ring
                segment_shape = p.createVisualShape(
                    shapeType=p.GEOM_BOX,
                    halfExtents=[torus_minor_radius, torus_minor_radius, estimated_hoop_thickness / 2],
                    rgbaColor=list(hoop_color),
                    physicsClientId=self.client,
                )
                visual_shapes.append((segment_shape, (x, y, z)))
            
            # Create the hoop as a compound body with all segments
            # Use the first segment as the base, others will be positioned relative to it
            if visual_shapes:
                first_shape, first_pos = visual_shapes[0]
                hoop_id = p.createMultiBody(
                    baseMass=0.0,
                    baseVisualShapeIndex=first_shape,
                    basePosition=(
                        base_fixture_position[0] + hoop_offset_x + first_pos[0],
                        base_fixture_position[1] + hoop_offset_y + first_pos[1],
                        hoop_center_z + first_pos[2]
                    ),
                    baseOrientation=p.getQuaternionFromEuler(base_fixture_rpy),
                    physicsClientId=self.client,
                )
                
                # Add remaining segments as separate bodies at their positions
                for seg_shape, seg_pos in visual_shapes[1:]:
                    p.createMultiBody(
                        baseMass=0.0,
                        baseVisualShapeIndex=seg_shape,
                        basePosition=(
                            base_fixture_position[0] + hoop_offset_x + seg_pos[0],
                            base_fixture_position[1] + hoop_offset_y + seg_pos[1],
                            hoop_center_z + seg_pos[2]
                        ),
                        baseOrientation=p.getQuaternionFromEuler(base_fixture_rpy),
                        physicsClientId=self.client,
                    )
            
            center_world = (base_fixture_position[0] + hoop_offset_x, base_fixture_position[1] + hoop_offset_y, hoop_center_z)
            final_position = (base_fixture_position[0] + hoop_offset_x, base_fixture_position[1] + hoop_offset_y, hoop_center_z)
        else:
            # Use calculated AABB
            hoop_height = hoop_aabb_max[2] - hoop_aabb_min[2]
            desired_bottom_z = fixture_top_z + hoop_clearance
            translation_z = desired_bottom_z - hoop_aabb_min[2]
            
            # Offset hoop position to be visible on platform (not directly under Q arm base)
            hoop_offset_x = 0.0  # Keep centered in X
            hoop_offset_y = 0.3  # Move forward 30cm in Y direction to be visible
            
            current_position, current_orientation = p.getBasePositionAndOrientation(hoop_id, physicsClientId=self.client)
            aligned_position = (
                base_fixture_position[0] + hoop_offset_x,
                base_fixture_position[1] + hoop_offset_y,
                current_position[2] + translation_z,
            )
            p.resetBasePositionAndOrientation(
                hoop_id,
                aligned_position,
                current_orientation,
                physicsClientId=self.client,
            )
            # Recalculate AABB after repositioning
            for _ in range(5):
                p.stepSimulation(physicsClientId=self.client)
            hoop_aabb_min, hoop_aabb_max = p.getAABB(hoop_id, physicsClientId=self.client)
            hoop_center = tuple(
                (mn + mx) * 0.5 for mn, mx in zip(hoop_aabb_min, hoop_aabb_max)
            )
            # hoop_center is already in world coordinates after repositioning
            center_world = hoop_center
            final_position = aligned_position

        # Debug output
        print(f"[QArmSimEnv] DarmHoop AABB size: {aabb_size}")
        print(f"[QArmSimEnv] DarmHoop final position: {final_position}")
        print(f"[QArmSimEnv] DarmHoop center (world): {center_world}")
        
        return hoop_id, center_world

    def _create_target_marker(
        self,
        position: Sequence[float],
        radius: float,
        color: Sequence[float],
        marker_id: int,
    ) -> int:
        """
        Create a visual sphere marker at a target coordinate for inverse kinematics testing.
        
        Args:
            position: (x, y, z) coordinate in world space
            radius: Radius of the marker sphere
            color: RGBA color tuple
            marker_id: Identifier number for the marker
            
        Returns:
            Body ID of the created marker
        """
        marker_shape = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=radius,
            rgbaColor=list(color),
            specularColor=[0.1, 0.1, 0.1],
            physicsClientId=self.client,
        )
        
        # Create a small text label above the marker (optional, using debug line)
        # Add a vertical line to make it more visible
        marker_body = p.createMultiBody(
            baseMass=0.0,
            baseVisualShapeIndex=marker_shape,
            basePosition=list(position),
            physicsClientId=self.client,
        )
        
        # Add a small vertical line above the marker for better visibility
        line_end = (position[0], position[1], position[2] + radius * 3)
        p.addUserDebugLine(
            position,
            line_end,
            lineColorRGB=color[:3],
            lineWidth=2,
            lifeTime=0,  # Permanent
            physicsClientId=self.client,
        )
        
        # Add floating coordinate text label above the marker
        text_position = (position[0], position[1], position[2] + radius * 4)
        coord_text = f"({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"
        p.addUserDebugText(
            coord_text,
            textPosition=text_position,
            textColorRGB=color[:3],
            textSize=1.2,
            lifeTime=0,  # Permanent
            physicsClientId=self.client,
        )
        
        return marker_body

    def apply_joint_slider_targets(self) -> None:
        """Read joint slider values (if enabled) and command the arm accordingly."""
        if not self._joint_slider_ids:
            return
        try:
            targets = [
                p.readUserDebugParameter(slider_id, physicsClientId=self.client) for slider_id in self._joint_slider_ids
            ]
        except p.error as exc:  # pragma: no cover - GUI timing guard
            # The GUI sometimes lags behind the simulation thread on startup; if sliders
            # are not ready yet just skip this frame and try again next tick.
            if "Failed to read parameter" in str(exc):
                return
            raise
        self.set_joint_positions(targets)
