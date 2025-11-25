"""
Simulation-backed QArm controller that wraps :class:`sim.env.QArmSimEnv`.

The physics backend is PyBullet; the primary viewport is Panda3D
(:mod:`sim.panda_viewer`). The PyBullet GUI can be enabled for debugging, but
the default is headless to avoid clashing with Panda3D.
"""

from __future__ import annotations

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

        self.joint_order: list[int] = list(self.env.movable_joint_indices)
        if not self.joint_order:
            raise RuntimeError("No movable joints found in the simulation.")
        self.joint_names: list[str] = [self.env.joint_names[idx] for idx in self.joint_order]
        self.joint_name_hint: tuple[str, ...] = DEFAULT_JOINT_ORDER
        self._home_pose = self._validate_home_pose(home_pose)
        self._auto_step = bool(auto_step)
        self._joint_index_to_pos = {idx: i for i, idx in enumerate(self.joint_order)}
        self._gripper_joint_indices = self._detect_gripper_joints()
        self._gripper_limits = self._query_gripper_limits()

    def home(self) -> None:
        """Reset the arm to its configured home pose."""
        self.env.reset(self._home_pose)
        self._maybe_step()

    def set_joint_positions(self, q: Sequence[float]) -> None:
        """
        Command the arm joints in the same order as :attr:`joint_order`.
        """
        targets = list(q)
        if len(targets) != len(self.joint_order):
            raise ValueError(f"Expected {len(self.joint_order)} joint targets, got {len(targets)}")
        self.env.set_joint_positions(targets)
        self._maybe_step()

    def get_joint_positions(self) -> list[float]:
        """Return current joint angles (radians) in simulation order."""
        return self.env.get_joint_positions(self.joint_order)

    def open_gripper(self) -> None:
        """Open the gripper if the current URDF exposes gripper joints."""
        self._set_gripper(open_state=True)

    def close_gripper(self) -> None:
        """Close the gripper if the current URDF exposes gripper joints."""
        self._set_gripper(open_state=False)

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
        for idx in self.joint_order:
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

    def _set_gripper(self, *, open_state: bool) -> None:
        if not self._gripper_joint_indices:
            raise NotImplementedError(
                "Gripper control is not available in this simulation (no gripper joints found in the URDF)."
            )
        current = self.get_joint_positions()
        targets = list(current)
        desired = self._gripper_targets(open_state=open_state)
        for joint_idx, target_value in desired.items():
            pos = self._joint_index_to_pos[joint_idx]
            targets[pos] = target_value
        self.env.set_joint_positions(targets)
        self._maybe_step()

    def _gripper_targets(self, *, open_state: bool) -> dict[int, float]:
        """Compute per-joint targets for opening/closing the gripper."""
        targets: dict[int, float] = {}
        for idx in self._gripper_joint_indices:
            lower, upper = self._gripper_limits.get(idx, (0.0, 0.0))
            if upper > lower:
                targets[idx] = upper if open_state else lower
            else:
                targets[idx] = 0.04 if open_state else 0.0
        return targets

    def _maybe_step(self) -> None:
        """Advance the sim if we're not running PyBullet in real-time mode."""
        if self._auto_step and not self.env.real_time:
            self.env.step()
