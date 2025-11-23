# QArm Hackathon Framework (WIP)

This repository provides the core structure for a hackathon centred around the
Quanser QArm robotic manipulator.

Students will control a QArm in simulation (and later on real hardware) by
commanding joint angles and implementing their own kinematics and strategies
for picking and placing coloured hoops onto stands.

## Current status

- `qarm/` contains the URDF and mesh resources for the QArm.
- `common.QArmBase` defines a joint-space control interface.
- `sim.SimQArm` and `hardware.RealQArm` are skeleton implementations that will
  be wired to PyBullet and the Quanser API respectively.

Next steps (to be done by a human):

- Implement the PyBullet environment in `sim/env.py`.
- Connect `SimQArm` to the PyBullet simulation.
- Connect `RealQArm` to the physical QArm via the Quanser Python SDK.
- Build student-facing templates and milestone scripts on top of `QArmBase`.
