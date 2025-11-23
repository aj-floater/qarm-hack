"""
Example script showing how a higher-level strategy might interact with the QArm API.

This is a stub – the actual motion logic and simulation wiring will be implemented later.
"""

from common.qarm_base import QArmBase


def run_demo(arm: QArmBase) -> None:
    """
    Simple placeholder flow:

    - Home the arm
    - (In future) move to a test position
    - (In future) pick and place a hoop
    """
    arm.home()
    # TODO: add simple test motions once SimQArm is wired to PyBullet.


if __name__ == "__main__":
    # TODO: in future we will instantiate either SimQArm or RealQArm here.
    raise SystemExit("Instantiate SimQArm or RealQArm and pass it to run_demo()")
