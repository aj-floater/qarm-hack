"""
Hardware-based QArm controller that will wrap the Quanser Python API.

Quanser-specific imports and calls are intentionally omitted; fill them in when
the hardware SDK is available in the hackathon environment.
"""

from __future__ import annotations

from typing import Any, Sequence, List

from common.qarm_base import DEFAULT_JOINT_ORDER, QArmBase

import numpy as np
from atexit import register
from contextlib import ExitStack
from pal.products.qarm import QArm
from contextlib import contextmanager, redirect_stdout
from os import devnull

@contextmanager
def suppress_stdout():
    """Context manager to suppress stdout by redirecting it to os.devnull."""
    with open(devnull, "w") as fnull:
        with redirect_stdout(fnull):
            yield

class RealQArm(QArmBase):
    """
    Hardware-based QArm controller using the Quanser Python API.

    This class will wrap the vendor-provided SDK to control the physical QArm.
    For now, it only defines the shape of the interface and where future code
    should be added.
    """

    def __init__(self, client: Any | None = None) -> None:
        # Placeholder for the Quanser hardware connection. Keep this import-free
        # so the package is importable without the vendor SDK present.
        self.client = client
        self.joint_name_hint: tuple[str, ...] = DEFAULT_JOINT_ORDER
        
        self._current_joint_positions = [0.0, 0.0, 0.0, 0.0]

        self.exitStack = ExitStack()
        register(self.exitStack.close)

        with suppress_stdout():
            self.arm = self.exitStack.enter_context(QArm(hardware=1))
        self.home()

    def home(self) -> None:
        # self._not_ready("home")
        self.set_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.set_gripper_position(0.0)

    def set_joint_positions(self, joint_angles: List[float]) -> None:
        # self._not_ready("set_joint_positions")
        self._current_joint_positions = joint_angles

        self.arm.read_write_std(phiCMD=joint_angles, baseLED=np.array([0, 1, 0], dtype=np.float64))

    def get_joint_positions(self) -> list[float]:
        # self._not_ready("get_joint_positions")
        return self._current_joint_positions

    def set_gripper_position(self, closure: float) -> None:
        # self._not_ready("set_gripper_position")
        self.arm.read_write_std(gprCMD=closure, baseLED=np.array([0, 1, 0], dtype=np.float64))

    def set_gripper_positions(self, angles: Sequence[float]) -> None:
        self._not_ready("set_gripper_positions")

    def open_gripper(self) -> None:
        self.set_gripper_position(0.0)

    def close_gripper(self) -> None:
        self.set_gripper_position(0.55)

    def _not_ready(self, call: str) -> None:
        raise NotImplementedError(
            f"RealQArm.{call} requires the Quanser hardware SDK and a connected robot. "
            "Use mode='sim' during the hackathon, or supply a hardware client when available."
        )
