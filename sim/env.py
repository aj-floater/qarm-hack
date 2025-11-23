"""
Placeholder for the PyBullet environment that will host the QArm simulation.

Future work should:
- start a PyBullet physics client (GUI or DIRECT),
- load the URDF from the qarm/ directory,
- expose body and joint identifiers for SimQArm,
- provide helper methods for resetting, stepping, and rendering.
"""

from __future__ import annotations


class QArmSimEnv:
    """
    Container for PyBullet state and helper utilities.

    Replace the stubbed constructor and methods with real PyBullet wiring once
    the simulation loop is implemented.
    """

    def __init__(self) -> None:
        # TODO: connect to PyBullet and load the QArm URDF.
        raise NotImplementedError("QArmSimEnv to be implemented with PyBullet setup")

    def step(self) -> None:
        # TODO: step the physics simulation forward.
        raise NotImplementedError

    def reset(self) -> None:
        # TODO: reset the simulation and return to a known state.
        raise NotImplementedError
