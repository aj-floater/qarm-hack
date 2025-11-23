"""
Simulation-based QArm controller that will wrap a PyBullet environment.

This class is a stub; a human will connect it to a PyBullet client that loads
the URDF from the qarm/ directory and drives joint positions.
"""

from __future__ import annotations

from typing import Sequence

from common.qarm_base import QArmBase


class SimQArm(QArmBase):
    """
    Simulation-based QArm controller using PyBullet.

    This class will be connected to a PyBullet environment that loads the QArm
    URDF from the `qarm/` directory. For now, methods are stubs to be filled in
    by a human after the repository is created.
    """

    def __init__(self) -> None:
        # TODO: accept a PyBullet client / environment handle.
        # e.g. client_id, body_id, joint_indices, etc.
        raise NotImplementedError("SimQArm __init__ to be implemented later")

    def home(self) -> None:
        # TODO: implement home pose for sim.
        raise NotImplementedError

    def set_joint_positions(self, q: Sequence[float]) -> None:
        # TODO: use pybullet.setJointMotorControlArray / similar.
        raise NotImplementedError

    def get_joint_positions(self) -> list[float]:
        # TODO: read from pybullet.getJointState(s).
        raise NotImplementedError

    def open_gripper(self) -> None:
        # TODO: command gripper joint(s) to open position.
        raise NotImplementedError

    def close_gripper(self) -> None:
        # TODO: command gripper joint(s) to closed position.
        raise NotImplementedError

    def move_ee_to(self, target_pos: Sequence[float]) -> None:
        """
        Move the end effector to the requested position once IK is available.

        Implementers can plug in student-provided inverse kinematics or a
        PyBullet IK solver here.
        """
        raise NotImplementedError("SimQArm.move_ee_to to be implemented with IK")
