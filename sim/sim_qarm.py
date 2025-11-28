"""
Simulation-backed QArm controller that wraps :class:`sim.env.QArmSimEnv`.

The physics backend is PyBullet; the primary viewport is Panda3D
(:mod:`sim.panda_viewer`). The PyBullet GUI can be enabled for debugging, but
the default is headless to avoid clashing with Panda3D.
"""

from __future__ import annotations

import math
from typing import Any, Sequence

try:
    import pybullet as p
except ImportError as exc:  # pragma: no cover - runtime guard
    raise SystemExit("pybullet is not installed. Run `pip install -e .` first.") from exc

from common.qarm_base import DEFAULT_JOINT_ORDER, QArmBase
from sim.env import QArmSimEnv


class SimQArm(QArmBase):
    """
    Wrap the PyBullet simulation so student code only sees the QArmBase API.

    Parameters mirror the underlying :class:`sim.env.QArmSimEnv` where relevant;
    pass an existing ``env`` if you already have one, otherwise SimQArm will
    create its own.
    """

    def __init__(
        self,
        env: QArmSimEnv | None = None,
        *,
        gui: bool = False,
        real_time: bool = True,
        time_step: float = 1.0 / 120.0,
        home_pose: Sequence[float] | None = None,
        auto_step: bool = True,
        **env_kwargs: Any,
    ) -> None:
        self.env = env or QArmSimEnv(
            gui=gui,
            real_time=real_time,
            time_step=time_step,
            add_ground=True,
            **env_kwargs,
        )
        if self.env.robot_id is None:
            raise RuntimeError("Simulation did not load a robot; cannot control QArm.")

        self._env_index_to_pos: dict[int, int] = {
            idx: i for i, idx in enumerate(self.env.movable_joint_indices)
        }
        self._gripper_joint_indices = (
            list(getattr(self.env, "_gripper_control_indices", [])) or self._detect_gripper_joints()
        )
        self._arm_joint_indices: list[int] = [
            idx for idx in self.env.movable_joint_indices if idx not in self._gripper_joint_indices
        ]
        if not self._arm_joint_indices:
            raise RuntimeError("No movable joints found in the simulation.")
        self.joint_order: list[int] = list(self._arm_joint_indices)
        self.joint_names: list[str] = [self.env.joint_names[idx] for idx in self.joint_order]
        self.joint_name_hint: tuple[str, ...] = DEFAULT_JOINT_ORDER  # yaw, shoulder, elbow, wrist
        self._home_pose = self._validate_home_pose(home_pose)
        self._auto_step = bool(auto_step)
        self._gripper_limits = self._query_gripper_limits()
        self._gripper_angle_limits = self._compute_gripper_angle_limits()
        self._open_angle = 0.0
        self._closed_angle = min(0.55, self._gripper_angle_limits[1])
        self._gripper_motion_state: str = "idle"

    def home(self) -> None:
        """Reset the arm to its configured home pose."""
        full_home = [0.0] * len(self.env.movable_joint_indices)
        for idx, angle in zip(self._arm_joint_indices, self._home_pose):
            pos = self._env_index_to_pos[idx]
            full_home[pos] = angle
        self.env.reset(full_home)
        self._maybe_step()

    def set_joint_positions(self, q: Sequence[float]) -> None:
        """
        Command the arm joints in the same order as :attr:`joint_order`.
        """
        targets = list(q)
        expected = len(self.joint_order)
        if len(targets) != expected:
            raise ValueError(f"Expected {expected} joint targets, got {len(targets)}")
        full_targets = self.env.get_joint_positions()  # includes gripper joints
        for idx, angle in zip(self.joint_order, targets):
            pos = self._env_index_to_pos[idx]
            full_targets[pos] = angle
        self.env.set_joint_positions(full_targets)
        self._maybe_step()

    def get_joint_positions(self) -> list[float]:
        """Return current arm joint angles (radians) in the default order."""
        return self.env.get_joint_positions(self.joint_order)

    def open_gripper(self) -> None:
        """Open the gripper to the default open angle."""
        self.set_gripper_position(self._open_angle)

    def close_gripper(self) -> None:
        """Close the gripper to the default closed angle."""
        self.set_gripper_position(self._closed_angle)

    def set_gripper_position(self, angle: float) -> None:
        """Drive the gripper symmetrically using a single angle target."""
        if not self._gripper_joint_indices:
            raise NotImplementedError(
                "Gripper control is not available in this simulation (no gripper joints found in the URDF)."
            )
        clamped = self._clamp_gripper_angle(angle)
        self._gripper_motion_state = self.env.record_gripper_command(clamped)
        targets = self._gripper_targets_for_angle(clamped)
        self._apply_gripper_targets(targets)

    def move_ee_to(self, target_pos: Sequence[float]) -> None:  # pragma: no cover - placeholder for student IK
        """
        Hook for future end-effector control once IK is wired.

        Student-provided IK (or PyBullet's IK solver) can be integrated here.
        """
        raise NotImplementedError("SimQArm.move_ee_to will be implemented once IK is available.")

    def disconnect(self) -> None:
        """Disconnect the underlying PyBullet client."""
        self.env.disconnect()

    # Internal helpers
    def _validate_home_pose(self, home_pose: Sequence[float] | None) -> list[float]:
        if home_pose is None:
            return [0.0] * len(self.joint_order)
        values = list(home_pose)
        if len(values) != len(self.joint_order):
            raise ValueError(f"Home pose length {len(values)} does not match movable joints {len(self.joint_order)}")
        return values

    def _detect_gripper_joints(self) -> list[int]:
        """Heuristic: look for movable joints with 'grip' or 'finger' in their names."""
        candidates: list[int] = []
        for idx in self.env.movable_joint_indices:
            name = self.env.joint_names[idx].lower()
            if "grip" in name or "finger" in name:
                candidates.append(idx)
        return candidates

    def _query_gripper_limits(self) -> dict[int, tuple[float, float]]:
        limits: dict[int, tuple[float, float]] = {}
        if not self._gripper_joint_indices:
            return limits
        for idx in self._gripper_joint_indices:
            info = p.getJointInfo(self.env.robot_id, idx, physicsClientId=self.env.client)
            lower, upper = info[8], info[9]
            if lower >= upper:
                lower, upper = 0.0, 0.0
            limits[idx] = (lower, upper)
        return limits

    def _compute_gripper_angle_limits(self) -> tuple[float, float]:
        """Derive a safe symmetric range for the single gripper angle."""
        lower_bound = -math.inf
        upper_bound = math.inf
        for idx, (j_lower, j_upper) in self._gripper_limits.items():
            name = self.env.joint_names[idx].upper()
            if name == "GRIPPER_JOINT1A":
                lower_bound = max(lower_bound, -j_upper)
                upper_bound = min(upper_bound, -j_lower)
            elif name == "GRIPPER_JOINT2A":
                lower_bound = max(lower_bound, j_lower)
                upper_bound = min(upper_bound, j_upper)
        if not math.isfinite(lower_bound) or not math.isfinite(upper_bound) or lower_bound >= upper_bound:
            lower_bound, upper_bound = -1.0, 1.0
        # Keep within a reasonable travel to avoid unrealistic extremes.
        lower_bound = max(lower_bound, -1.2)
        upper_bound = min(upper_bound, 1.2)
        return (lower_bound, upper_bound)

    def _clamp_gripper_angle(self, angle: float) -> float:
        lo, hi = self._gripper_angle_limits
        return max(lo, min(hi, float(angle)))

    def _gripper_targets_for_angle(self, angle: float) -> dict[int, float]:
        """Map a scalar gripper angle to per-joint targets."""
        targets: dict[int, float] = {}
        for idx in self._gripper_joint_indices:
            name = self.env.joint_names[idx].upper()
            if name == "GRIPPER_JOINT1A":
                targets[idx] = -angle
            elif name == "GRIPPER_JOINT2A":
                targets[idx] = angle
            else:
                targets[idx] = angle
        return targets

    def _apply_gripper_targets(self, desired: dict[int, float]) -> None:
        targets = self.env.get_joint_positions()  # includes gripper joints
        for joint_idx, target_value in desired.items():
            pos = self._env_index_to_pos[joint_idx]
            targets[pos] = target_value
        self.env.set_joint_positions(targets)
        self._maybe_step()

    def _maybe_step(self) -> None:
        """Advance the sim if we're not running PyBullet in real-time mode."""
        if self._auto_step and not self.env.real_time:
            self.env.step()
