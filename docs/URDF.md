# URDF Primer (QArm Gripper Example)

URDF (Unified Robot Description Format) is the XML-based format PyBullet, ROS, and similar tools use to describe robot kinematics, visuals, and physical properties. This repository includes two relevant URDF packages:

- `qarm/urdf/QARM.urdf` – the arm itself.
- `qarm_gripper/urdf/qarm_gripper.urdf` – the gripper assembly that can be loaded separately or attached to the arm.

You can learn the structure by inspecting the gripper URDF, which demonstrates each URDF element in a compact package.

## File structure overview

```xml
<robot name="qarm_gripper">
  <link name="base_link"> … </link>
  <link name="Link1A"> … </link>
  …
  <joint name="Joint1A" type="revolute"> … </joint>
  <joint name="Joint2A" type="revolute"> … </joint>
  …
</robot>
```

### `<robot>` root

Defines the robot/model name. Everything else sits inside this tag.

### `<link>` elements (rigid bodies)

Each link represents a rigid body and usually contains:

- `<inertial>` – physical properties for the physics engine.
  - `<origin xyz="..." rpy="...">` – frame for the inertial tensor and mass.
  - `<mass value="...">`
  - `<inertia ixx="..." ixy="..." ...>`
- `<visual>` – how to render the link.
  - `<origin>` – visual mesh pose relative to the link frame (often matches the inertial origin).
  - `<geometry>` – a mesh (`package://.../meshes/*.STL`) or primitive.
  - `<material>` – optional colour/texture.
- `<collision>` – shapes used for collision detection (can share the visual geometry or use simplified primitives).

In `qarm_gripper.urdf`, links like `Link1A`, `Link2A`, `Link1B`, etc., all follow this pattern. We aligned their visual/collision origins with the inertial origin so meshes render consistently without modifying the original CAD files.

### `<joint>` elements (kinematic connections)

Joints connect two links and define the allowed motion.

- `<parent link="...">` / `<child link="...">` define which links the joint connects.
- `<origin xyz="..." rpy="...">` describes the child link’s frame relative to the parent when the joint value is zero.
- `<axis xyz="...">` sets the motion axis (for revolute/prismatic joints).
- `<limit lower="..." upper="..." effort="..." velocity="...">` constrains motion range and defines basic dynamics parameters.
- Optional `<dynamics>`, `<mimic>`, `<safety_controller>` elements further customise behaviour.

Example from the gripper URDF:

```xml
<joint name="Joint1A" type="revolute">
  <origin xyz="-0.03075 0.0345 0.016987" rpy="0 0 0.59622" />
  <parent link="base_link" />
  <child link="Link1A" />
  <axis xyz="0 0 1" />
  <limit lower="0" upper="0.9" effort="5" velocity="1" />
</joint>
```

This specifies a revolute joint whose pivot sits at the given origin relative to `base_link`, rotates about the Z-axis, and has a positive range of 0–0.9 radians (approx 0–52°). When we exposed gripper sliders in PyBullet, we simply commanded these joint angles within those limits.

## Practical tips

- **Reuse packages**: Keep arm and gripper URDFs separate if you expect to swap variants. In this project the standalone gripper mesh (`Gripper.stl`) is mounted directly on the arm's END-EFFECTOR in `qarm/urdf/QARM.urdf` for simplicity.
- **Consistent origins**: Align `<visual>` and `<collision>` origins with `<inertial>` where possible to avoid mesh offsets from third-party CAD exports.
- **Unit conventions**: URDF assumes metres and radians. If the CAD export used millimetres, apply `scale="0.001 0.001 0.001"` on the `<mesh>` or re-export in metres.
- **Debugging**: Load the URDF standalone (`python -m sim.bootstrap --urdf qarm/urdf/QARM.urdf --gui`) to verify joint axes and ranges before driving it.
- **Live editing**: Use `python -m sim.run_gui --gui --sliders` to load the arm in PyBullet with debug sliders while you tweak URDF visuals.

For additional references, see:

- ROS URDF XML format: https://wiki.ros.org/urdf/XML
- PyBullet quickstart (URDF section): https://pybullet.org/wordpress/index.php/urdf-quickstart/
