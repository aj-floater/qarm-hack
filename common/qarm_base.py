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

from abc import ABC, abstractmethod
from typing import Sequence


DEFAULT_JOINT_ORDER: tuple[str, ...] = ("yaw", "shoulder", "elbow", "wrist")


class QArmBase(ABC):
    """
    Abstract base class for controlling a Quanser QArm.

    This interface is intentionally joint-space oriented. Students will use
    this API to implement their own kinematics and motion logic.
    """

    @abstractmethod
    def home(self) -> None:
        """Move the arm to a safe 'home' configuration."""
        raise NotImplementedError

    @abstractmethod
    def set_joint_positions(self, q: Sequence[float]) -> None:
        """
        Command the arm to the specified joint configuration.

        q: Iterable of joint angles (radians) in a fixed, documented order.
        """
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
    def open_gripper(self) -> None:
        """Open the gripper."""
        raise NotImplementedError

    @abstractmethod
    def close_gripper(self) -> None:
        """Close the gripper."""
        raise NotImplementedError

    def move_ee_to(self, target_pos: Sequence[float]) -> None:
        """
        Optional helper: move the end-effector to a target position.

        Default implementation raises NotImplementedError. In future, this may
        be implemented using user-provided IK or higher-level motion planning.
        """
        raise NotImplementedError("move_ee_to is not implemented by default")
