# Demo Pack

This folder holds short, beginner-friendly demos (each under ~200 lines) that show how
to drive the QArm in simulation or hardware. Open any file to see the constants you
can tweak: viewer toggles, joint waypoints, or small helper functions. Students should
only need to modify these demos (and `blank_sim.py` in the repo root); touching other
framework files can break the shared setup.

- `pick_and_place.py` - scripted sequence built from a small `POSES` dictionary so you can edit joint targets without touching logic; uses `set_joint_positions` directly (no IK).
- `scene_objects.py` - drops the dog, head, and monkey meshes at preset offsets.
- `hoop_segments.py` - spawns a single hoop that uses collision segments for a more realistic ring shape.
- `multi_hoop_segments.py` - spawns ten hoops around a 68 mm diameter circle and builds their collision rings by tiling the hoop-segment mesh at 29.9 degree intervals.
- `label_demo.py` - shows how to add and animate your own point labels in the viewer.

Run any demo with:
```bash
python -m demos.<module_name>
```
Keep `MODE = "sim"` in these demos; hardware requires the shared kit setup and is not
needed for coursework. Leave `USE_PYBULLET_GUI = False` unless instructed otherwise.
