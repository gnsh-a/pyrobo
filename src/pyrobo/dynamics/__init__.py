"""Dynamics module for manipulator dynamics matrices."""

try:
    from ._generated import compute_B, compute_C, compute_D, compute_G
except ModuleNotFoundError as exc:
    if exc.name != "pyrobo.dynamics._generated":
        raise
    raise ModuleNotFoundError(
        "pyrobo.dynamics._generated is missing. "
        "Regenerate it with: uv run python scripts/generate_dynamics.py"
    ) from exc

__all__ = ["compute_D", "compute_B", "compute_C", "compute_G"]
