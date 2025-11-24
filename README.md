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
cd /qarm-hack      # or clone destination
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

## New to Python/VSCode? Start here

- Install VSCode from code.visualstudio.com (Windows/macOS/Linux).
- Install Python 3.11+:
  - Windows: grab the installer from python.org, check "Add python.exe to PATH".
  - macOS: `brew install python` or use the python.org installer.
  - Ubuntu/Debian: `sudo apt-get install python3 python3-venv python3-pip`.
- Launch VSCode and install the "Python" extension (ms-python.python). Pylance is recommended too.
- Create/select the project venv in VSCode: `Ctrl+Shift+P` → start typing **Python: Create Environment** → select it → choose `Venv` and the Python you installed. VSCode will wire the workspace to that interpreter.
- Open a new VSCode terminal; it should auto-activate `.venv` (or run the `source .venv/bin/activate` / `.venv\Scripts\activate` command shown above).
- Run `pip install -e .` in that terminal to pull dependencies into the venv. If the interpreter shown in VSCode's status bar is not your venv, click it and pick the `.venv` interpreter.

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

## Running the sims (PyBullet GUI)

- **Actual sim (full arm, optional gripper):**
  ```bash
  python -m sim.actual_sim --real-time            # arm only (PyBullet sliders)
  python -m sim.actual_sim --real-time --attach-gripper            # attach default gripper
  python -m sim.actual_sim --real-time --attach-gripper --new-gripper  # attach new gripper package
  ```
- **Test sim (gripper-focused):**
  ```bash
  python -m sim.test_sim --real-time              # gripper-only by default (PyBullet sliders)
  python -m sim.test_sim --real-time --with-arm   # include arm + gripper
  python -m sim.test_sim --real-time --with-arm --new-gripper  # include new gripper package
  ```
- Use `--gripper-urdf PATH` in either launch to point at a URDF you're editing. `--light-mode` switches PyBullet to a light background.

### VSCode run/debug helpers

- The repo tracks `.vscode/launch.json` so you get the launch targets on clone. The file contents:

```jsonc
{
  // Use IntelliSense to learn about possible attributes.
  // Hover to view descriptions of existing attributes.
  // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Actual Sim (PyBullet GUI)",
      "type": "debugpy",
      "request": "launch",
      "module": "sim.actual_sim",
      "justMyCode": true,
      "args": ["--real-time"]
    },
    {
      "name": "Test Sim (PyBullet GUI)",
      "type": "debugpy",
      "request": "launch",
      "module": "sim.test_sim",
      "justMyCode": true,
      "args": ["--real-time"]
    }
  ]
}
```

- Launch configs use PyBullet's own GUI and debug sliders. `--real-time` is pre-wired; add more args as needed.
