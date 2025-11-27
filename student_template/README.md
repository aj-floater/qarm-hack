# Student Template

Use this folder as your safe sandbox to learn the QArm API. The entry script drives the
arm in joint space so you can plug in your own kinematics later.

## Fast path (see something move)

```bash
python -m student_template.student_main
```

This launches the simulation and opens the Panda3D viewer (Panda3D comes from `pip install -e .`).
Close the viewer window or press Ctrl+C to stop.
Edit the defaults near the top of `student_main.py` if you want a longer run, different step size, or to disable the viewer / enable the PyBullet GUI.

## What you just ran

- Creates a QArm in simulation mode via `api.make_qarm()`.
- Cycles through a handful of joint-space waypoints in the order `(yaw, shoulder, elbow, wrist)`.
- Keeps looping until the default duration expires.

## Use an Xbox controller

- Install deps with `pip install -e .` (pulls in `pygame` for joystick polling).
- Plug in/pair your controller and run `python -m student_template.student_main`.
- Default mapping: left stick X → yaw, left stick Y → shoulder, right stick Y → elbow, right trigger → gripper (1A/2A). Stick inputs are square-bounded so diagonals still hit full range.
- Stick extremes map to absolute angles (center = 0 rad; edges ≈ ±360° for arm joints, ≈ ±120° for the gripper, clamped to URDF limits). One gripper side is inverted so positive X closes the jaws symmetrically.
- Current overrides: yaw -120° to +30°, shoulder 0° to +90°, gripper (1A/2A) -20° to +60°. Tweak `JOINT_LIMIT_OVERRIDES` near the top of `student_main.py` if you want different ranges/limits (leave elbow untouched to use URDF).
- Tweak `USE_GAMEPAD_CONTROL`, `JOYSTICK_AXES`, `JOINT_TARGET_RANGE_RAD`, `GRIPPER_TARGET_RANGE_RAD`, and `GAMEPAD_DEADZONE` near the top of `student_main.py` if your axis order differs or you want different ranges/limits.
- Want the same teleop plus preloaded meshes? Try `python -m student_template.student_gamepad_object`, a gamepad-enabled clone of `student_object.py` that spawns the hoop/monkey/dog/head meshes.

## API

- `get_joint_positions()` → list of joint angles `(yaw, shoulder, elbow, wrist)`
- `set_joint_positions(q)` → command those joints
- `home()` → return to a safe pose

Feel free to copy/extend `student_main.py` and drop in your own FK/IK while keeping this API.

## Why is there an `__init__.py` here?

It marks this folder as a Python package so `python -m student_template.student_main` works
and the template can be imported cleanly elsewhere if needed.

## Extra info

No CLI flags are needed. Modify the constants at the top of `student_main.py` to change duration, step, or viewer/GUI options.

## Drop a kinematic mesh

- Add entries to `KINEMATIC_OBJECTS` in `student_template/student_main.py` to spawn meshes (mesh path + position + orientation in degrees) while running the joint-space demo. Include `"mass": 0.1` (or any positive value) to let gravity act on the mesh.
- Run `python -m student_template.student_object` for a copy of `student_main` that focuses on kinematic/dynamic meshes only (no motion loop) and preloads the included `blender_monkey.stl`. Keep `"force_convex_for_dynamic": True` if you give it mass so collisions with the base stay reliable.
