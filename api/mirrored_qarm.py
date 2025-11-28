"""
Composite QArm controller that mirrors commands into a secondary backend.

Typical use: drive real hardware while echoing the same joint/gripper
commands into a simulation so the motion stays visible in the viewer.
"""

from __future__ import annotations

from typing import Sequence

from common.qarm_base import DEFAULT_JOINT_ORDER, QArmBase


class MirroredQArm(QArmBase):
    """Fan out commands to a primary controller and a mirror (e.g., simulation)."""

    def __init__(self, primary: QArmBase, mirror: QArmBase, *, mirror_name: str = "mirror") -> None:
        self.primary = primary
        self.mirror = mirror
        self.mirror_name = mirror_name
        self.env = getattr(mirror, "env", None)
        self.joint_order = getattr(primary, "joint_order", getattr(mirror, "joint_order", list(DEFAULT_JOINT_ORDER)))
        self.joint_names = getattr(primary, "joint_names", getattr(mirror, "joint_names", list(DEFAULT_JOINT_ORDER)))
        self.joint_name_hint = getattr(primary, "joint_name_hint", getattr(mirror, "joint_name_hint", DEFAULT_JOINT_ORDER))
        self._mirror_error_logged: set[str] = set()

    def _fan_out(self, method: str, *args, **kwargs) -> None:
        primary_exc: Exception | None = None
        try:
            getattr(self.primary, method)(*args, **kwargs)
        except Exception as exc:  # pragma: no cover - thin wrapper
            primary_exc = exc
        try:
            getattr(self.mirror, method)(*args, **kwargs)
        except Exception as exc:  # pragma: no cover - thin wrapper
            key = f"{method}:{type(exc).__name__}"
            if key not in self._mirror_error_logged:
                print(f"[MirroredQArm] Mirror '{self.mirror_name}' failed during {method}: {exc}")
                self._mirror_error_logged.add(key)
        if primary_exc:
            raise primary_exc

    def home(self) -> None:
        self._fan_out("home")

    def set_joint_positions(self, q: Sequence[float]) -> None:
        self._fan_out("set_joint_positions", q)

    def get_joint_positions(self) -> list[float]:
        return self.primary.get_joint_positions()

    def set_gripper_position(self, angle: float) -> None:
        self._fan_out("set_gripper_position", angle)

    def set_gripper_positions(self, angles: Sequence[float] | float) -> None:
        self._fan_out("set_gripper_positions", angles)

    def terminate(self) -> None:
        for target in (self.primary, self.mirror):
            closer = getattr(target, "terminate", None)
            if callable(closer):
                try:
                    closer()
                except Exception:
                    pass
        # Disconnect simulation if present.
        disconnect = getattr(self.mirror, "disconnect", None)
        if callable(disconnect):
            try:
                disconnect()
            except Exception:
                pass
