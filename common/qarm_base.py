"""
Base interface for controlling the Quanser QArm in joint space.

Conventions:
- Joint order must be fixed and consistent between simulation and hardware
  implementations; document the chosen order when wiring up a concrete class.
- The default joint naming/ordering used across this repo is:
  ("yaw", "shoulder", "elbow", "wrist")
- All joint angles are expressed in radians.
- Students are expected to build their own forward and inverse kinematics on
  top of the joint-level commands defined here.
"""

from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from typing import Sequence


DEFAULT_JOINT_ORDER: tuple[str, ...] = ("yaw", "shoulder", "elbow", "wrist")


class QArmBase(ABC):
    """
    Abstract base class for controlling a Quanser QArm.

    The public ``set_joint_positions`` method is rate-limited and blocking so
    motion more closely matches the physical hardware. Concrete backends only
    need to implement the instantaneous variant.
    """

    DEFAULT_JOINT_SPEED = 0.8  # rad/s
    CONTROL_STEP_S = 0.05

    @abstractmethod
    def home(self) -> None:
        """Move the arm to a safe 'home' configuration."""
        raise NotImplementedError

    def set_joint_positions(self, q: Sequence[float], *, speed: float | None = None) -> None:
        """
        Command the arm to the specified joint configuration, blocking until the move is complete.

        speed:
            Maximum joint speed (rad/s). When None, ``DEFAULT_JOINT_SPEED`` is used.
        """
        targets = list(q)
        if not targets:
            return
        max_speed = self.DEFAULT_JOINT_SPEED if speed is None else max(1e-6, float(speed))
        current = self.get_joint_positions()
        if len(current) != len(targets):
            self._set_joint_positions_instant(targets)
            return
        deltas = [abs(t - c) for t, c in zip(targets, current)]
        max_delta = max(deltas, default=0.0)
        if max_delta == 0.0:
            return
        duration = max_delta / max_speed
        steps = max(1, int(math.ceil(duration / self.CONTROL_STEP_S)))
        dt = duration / steps if steps > 0 else 0.0
        for step in range(1, steps + 1):
            alpha = step / steps
            intermediate = [c + (t - c) * alpha for c, t in zip(current, targets)]
            self._set_joint_positions_instant(intermediate)
            if step < steps and dt > 0.0:
                time.sleep(dt)

    def set_joint_positions_instant(self, q: Sequence[float]) -> None:
        """Bypass rate limiting and command the joints immediately."""
        self._set_joint_positions_instant(q)

    @abstractmethod
    def _set_joint_positions_instant(self, q: Sequence[float]) -> None:
        """Backend-specific implementation for immediate joint commands."""
        raise NotImplementedError

    @abstractmethod
    def get_joint_positions(self) -> list[float]:
        """Return the current joint angles (radians) in the same order as set_joint_positions."""
        raise NotImplementedError

    @abstractmethod
    def set_gripper_position(self, angle: float) -> None:
        """
        Command the gripper to a target angle (radians).

        Implementations decide how this maps to individual gripper joints but
        should treat a single scalar target as the user-facing control.
        """
        raise NotImplementedError

    @abstractmethod
    def set_gripper_positions(self, angles: Sequence[float]) -> None:
        """
        Command the gripper using explicit per-joint targets.

        Implementations should clamp to safe limits and map these values to the
        underlying gripper joints in a consistent, documented order.
        """
        raise NotImplementedError

    def move_ee_to(self, target_pos: Sequence[float]) -> None:
        """
        Optional helper: move the end-effector to a target position.

        Default implementation raises NotImplementedError. In future, this may
        be implemented using user-provided IK or higher-level motion planning.
        """
        raise NotImplementedError("move_ee_to is not implemented by default")
