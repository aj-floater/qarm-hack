"""
Hardware-based QArm controller that will wrap the Quanser Python API.

Quanser-specific imports and calls are intentionally omitted; fill them in when
the hardware SDK is available in the hackathon environment.
"""

from __future__ import annotations

from typing import Sequence

from common.qarm_base import QArmBase


class RealQArm(QArmBase):
    """
    Hardware-based QArm controller using the Quanser Python API.

    This class will wrap the vendor-provided SDK to control the physical QArm.
    For now, it only defines the shape of the interface and where future code
    should be added.
    """

    def __init__(self) -> None:
        # TODO: establish connection to QArm via Quanser API.
        # e.g. self.client = quanser_sdk.connect(...)
        raise NotImplementedError(
            "RealQArm __init__ to be implemented when hardware API is available"
        )

    def home(self) -> None:
        # TODO: send joints to a predefined safe home configuration.
        raise NotImplementedError

    def set_joint_positions(self, q: Sequence[float]) -> None:
        # TODO: send desired joint positions to hardware.
        raise NotImplementedError

    def get_joint_positions(self) -> list[float]:
        # TODO: query actual joint positions from hardware.
        raise NotImplementedError

    def open_gripper(self) -> None:
        # TODO: open gripper via hardware command.
        raise NotImplementedError

    def close_gripper(self) -> None:
        # TODO: close gripper via hardware command.
        raise NotImplementedError
