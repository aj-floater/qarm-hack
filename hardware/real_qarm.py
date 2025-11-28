"""
Hardware-based QArm controller that will wrap the Quanser Python API.

Quanser-specific imports and calls are intentionally omitted; fill them in when
the hardware SDK is available in the hackathon environment.
"""

from __future__ import annotations

from typing import Any, Sequence

from common.qarm_base import DEFAULT_JOINT_ORDER, QArmBase


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

    def home(self) -> None:
        self._not_ready("home")

    def set_joint_positions(self, q: Sequence[float]) -> None:
        self._not_ready("set_joint_positions")

    def get_joint_positions(self) -> list[float]:
        self._not_ready("get_joint_positions")

    def set_gripper_position(self, angle: float) -> None:
        self._not_ready("set_gripper_position")

    def open_gripper(self) -> None:
        self._not_ready("open_gripper")

    def close_gripper(self) -> None:
        self._not_ready("close_gripper")

    def _not_ready(self, call: str) -> None:
        raise NotImplementedError(
            f"RealQArm.{call} requires the Quanser hardware SDK and a connected robot. "
            "Use mode='sim' during the hackathon, or supply a hardware client when available."
        )
