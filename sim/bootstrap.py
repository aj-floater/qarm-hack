"""
One-off bootstrap script to load the QArm URDF in PyBullet for quick inspection.

Run with:
    python -m sim.bootstrap          # headless (DIRECT)
    python -m sim.bootstrap --gui    # opens PyBullet GUI
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

try:
    import pybullet as p
except ImportError as exc:  # pragma: no cover - runtime guard
    raise SystemExit("pybullet is not installed. Run `pip install -e .` first.") from exc


def list_joints(body_id: int) -> None:
    """Print joint indices and names for the loaded robot."""
    num_joints = p.getNumJoints(body_id)
    print(f"Found {num_joints} joints:")
    for j in range(num_joints):
        info = p.getJointInfo(body_id, j)
        name = info[1].decode("utf-8") if isinstance(info[1], (bytes, bytearray)) else str(info[1])
        print(f"  {j}: {name}")


def print_joint_states(body_id: int, joint_indices: Iterable[int]) -> None:
    """Print current positions for selected joints."""
    states = p.getJointStates(body_id, list(joint_indices))
    positions = [s[0] for s in states]
    print("Joint positions (rad):", positions)


def main() -> None:
    parser = argparse.ArgumentParser(description="Quickly load and inspect the QArm URDF in PyBullet.")
    parser.add_argument("--gui", action="store_true", help="Use PyBullet GUI instead of DIRECT.")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "qarm" / "urdf" / "QARM.urdf",
        help="Path to the QArm URDF.",
    )
    args = parser.parse_args()

    connection_mode = p.GUI if args.gui else p.DIRECT
    client = p.connect(connection_mode)
    p.setGravity(0, 0, -9.81, physicsClientId=client)

    urdf_path = args.urdf
    if not urdf_path.exists():
        raise SystemExit(f"URDF not found at {urdf_path}")

    print(f"Connecting in {'GUI' if args.gui else 'DIRECT'} mode...")
    print(f"Loading URDF from: {urdf_path}")
    robot_id = p.loadURDF(str(urdf_path), useFixedBase=True, physicsClientId=client)
    print(f"Client id: {client}, Robot id: {robot_id}")

    list_joints(robot_id)
    all_joint_indices = range(p.getNumJoints(robot_id))
    print_joint_states(robot_id, all_joint_indices)

    if args.gui:
        print("Interact in the GUI window. Close the window or press Ctrl+C here to exit.")
        try:
            while p.isConnected(client):
                p.stepSimulation(physicsClientId=client)
        except KeyboardInterrupt:
            pass

    p.disconnect(client)


if __name__ == "__main__":
    main()
