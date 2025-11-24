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

## Local setup (for devs and students)

```bash
cd /Users/archiejames/coding/qarm-hack      # or clone destination
python3 -m venv .venv
source .venv/bin/activate                   # Windows: .venv\Scripts\activate
pip install --upgrade pip
pip install -e .                            # installs pybullet + numpy
```

- On macOS, you may need Xcode Command Line Tools once: `xcode-select --install`.
- PyBullet builds a native wheel; it can take a minute on first install.
- **Every new shell**: reactivate the venv (`source .venv/bin/activate`). If you forget,
  `python/pip` will fall back to the system interpreter and you may see PEP 668 errors
  about an "externally managed environment."

Quick smoke test (no GUI):
```bash
python - <<'PY'
import pybullet as p
cid = p.connect(p.DIRECT)
robot = p.loadURDF("qarm/urdf/QARM.urdf")
print("client:", cid, "robot:", robot)
p.disconnect()
PY
```

### VSCode run/debug helpers

- Open `sim/run_gui.py` and use "Run Python File" or drop this into `.vscode/launch.json`:

```jsonc
{
  "name": "QArm GUI",
  "type": "python",
  "request": "launch",
  "module": "sim.run_gui",
  "justMyCode": true,
  "args": ["--gui", "--real-time", "--sliders"]
}
```

- Script flags:
  - `--gui` to show the PyBullet window (omit for headless smoke tests).
  - `--real-time` to let PyBullet handle stepping internally.
  - `--sliders` to expose only the "Joint Angles" control panel.
  - `--headless-steps N` to limit DIRECT-mode test duration.
