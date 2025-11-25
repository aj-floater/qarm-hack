"""Shared asset paths and scales for the QArm simulation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

Vec3Tuple = tuple[float, float, float]


@dataclass(frozen=True)
class BaseMeshAssets:
    """Bundle of asset paths and scales for the pine base and accents."""

    visual_mesh: Path
    collision_mesh: Path
    visual_scale: Vec3Tuple
    collision_scale: Vec3Tuple
    yaw_deg: float
    friction: float
    restitution: float
    green_accent_mesh: Path
    blue_accent_mesh: Path


MODELS_DIR = Path(__file__).resolve().parent / "models"

# Keep base meshes tiny relative to the STL dimensions (matches prior CLI defaults) and allow
# independent tuning of visual vs collision scales.
DEFAULT_BASE_ASSETS = BaseMeshAssets(
    visual_mesh=MODELS_DIR / "pinebase.stl",
    collision_mesh=MODELS_DIR / "pinebase_collision.stl",
    visual_scale=(1.00, 1.00, 1.00),
    # Collision mesh is authored 1000x smaller than the visual; keep it at 0.001 scale.
    collision_scale=(0.001, 0.001, 0.001),
    yaw_deg=180.0,
    friction=0.8,
    restitution=0.0,
    green_accent_mesh=MODELS_DIR / "greenaccent.stl",
    blue_accent_mesh=MODELS_DIR / "blueaccent.stl",
)
