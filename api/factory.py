"""Factory for creating either a simulated or hardware-backed QArm controller."""

from __future__ import annotations

from typing import Any, Sequence

from common.qarm_base import QArmBase
from .mirrored_qarm import MirroredQArm
from hardware.real_qarm import RealQArm
from sim.env import QArmSimEnv
from sim.sim_qarm import SimQArm


def make_qarm(
    mode: str = "sim",
    *,
    gui: bool = False,
    real_time: bool = True,
    time_step: float = 1.0 / 120.0,
    home_pose: Sequence[float] | None = None,
    auto_step: bool = True,
    sim_env: QArmSimEnv | None = None,
    hardware_client: Any | None = None,
    mirror_sim: bool = False,
    **sim_kwargs: Any,
) -> QArmBase:
    """
    Create a QArm controller instance.

    mode:
        - "sim" / "simulation" (default): returns :class:`SimQArm` wired to PyBullet.
        - "hardware": returns :class:`RealQArm` stub ready for the Quanser SDK.

    Any additional keyword arguments are forwarded to :class:`SimQArm` /
    :class:`sim.env.QArmSimEnv` when in simulation mode.

    mirror_sim:
        - When True and ``mode`` is ``"hardware"``, also spin up a simulation
          and mirror all commands to it so you can watch the motion in the
          viewer while driving the real arm. Ignored in simulation mode.
    """
    mode_norm = mode.strip().lower()

    def _make_sim() -> SimQArm:
        return SimQArm(
            env=sim_env,
            gui=gui,
            real_time=real_time,
            time_step=time_step,
            home_pose=home_pose,
            auto_step=auto_step,
            **sim_kwargs,
        )

    if mode_norm in {"sim", "simulation"}:
        return _make_sim()
    if mode_norm == "hardware":
        hardware = RealQArm(client=hardware_client)
        if mirror_sim:
            sim_arm = _make_sim()
            return MirroredQArm(primary=hardware, mirror=sim_arm, mirror_name="simulation")
        return hardware
    raise ValueError(f"Unknown QArm mode '{mode}'. Use 'sim' (default) or 'hardware'.")
