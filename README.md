# QArm Hackathon Framework (WIP)

This repository provides the core structure for a hackathon centred around the
Quanser QArm robotic manipulator.

You will control a QArm in simulation (and later on real hardware) by
commanding joint angles and implementing their own kinematics and strategies
for picking and placing coloured hoops onto stands.

## Hackathon context

We only have one physical arm for up to 70 students, so this kit ships with a
bespoke simulation environment where you can run, debug, and iterate on your
code without queueing for hardware time. The brief is to pick and place hoops
from fixed locations on the board to their colour-matched stands. That can be
approached in many ways, and the simulator exposes helper features including:

1. Importing kinematic objects into the scene.
2. Adding labels in the viewer (handy while debugging inverse kinematics).
3. Accurate board geometry plus stand positions to match the real setup.

The only control interface you have is direct joint-angle commands. While you
could brute-force a sequence of angles, that is tedious and unrepresentative of
industrial pick-and-place workflows. We strongly recommend deriving a simple
inverse kinematics solution for this arm; it is a valuable robotics skill and
straightforward to implement for this mechanism.

## Current status

- `sim/qarm/` contains the URDF and mesh resources for the QArm.
- `common.QArmBase` defines the joint-space API and default joint ordering.
- `sim.env.QArmSimEnv` runs the PyBullet backend; `sim.SimQArm` wraps it to match `QArmBase`.
- `api.make_qarm` switches between simulation (default, headless) and the stubbed `hardware.RealQArm`.
- `demos/` holds simplified student demos (quickstart, pick/place, keyboard control, gamepad hoops, scene helpers).
- `hardware.RealQArm` remains a stub until the Quanser SDK is available.

## Dependencies

- **Windows:** install [Visual C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/).
- **macOS:** install the Xcode Command Line Tools (`xcode-select --install`).
- **Python:** use a version between 3.0 and 3.14 so both Panda3D and PyBullet supply wheels. Follow the [Python install guide](docs/python-install.md) if you need help picking an interpreter.
- **Editor:** VSCode is strongly recommended (any similar IDE works if you already have one configured).

## Environment setup

```bash
cd /qarm-hack      # or clone destination
python -m venv .venv
.venv\Scripts\activate                  # macOS/Linux: source .venv/bin/activate
pip install -e .                        # installs the Python libraries for this project
```

PyBullet builds a native wheel; the first install can take a minute. **Every new shell**: reactivate the venv (`source .venv/bin/activate` or `.venv\Scripts\activate`). Using a virtual environment keeps this project's packages (PyBullet, Panda3D, etc.) isolated from your system Python so you avoid polluting other projects and everyone on your team shares the same dependency versions.

## Start coding

Kick off the sandbox:
```bash
python -m blank_sim
```
You should see the simulator window with the QArm standing on its platform. `blank_sim.py` introduces the API and viewer flow, so treat it as the jumping-off point for your own solution.

When you are ready to explore other behaviours, run the demos:
```bash
python -m demos.name_of_demo
```
Replace `name_of_demo` with any file under `demos/` (for example `student_main`, `pick_and_place`, or `keyboard_control`). Each script highlights different workflows and helpers you can adapt to the hackathon challenge.

Students should only need to modify the demo scripts inside `demos/` and the `blank_sim.py` skeleton; changing other files risks breaking the shared setup and is strongly discouraged.

## VSCode launch shortcuts

The workspace tracks `.vscode/launch.json`, which now lists `blank_sim` as the default run/debug target alongside quick-launch entries for each demo (`demos.pick_and_place`, `demos.scene_objects`, `demos.label_demo`, `demos.hoop_segments`). Open VSCode's Run & Debug view to select the scenario you want to run without retyping the module path every time.
