# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PyRobo is a Python library for simulating a 3-DOF revolute-revolute-revolute (RRR) manipulator. It covers kinematics, dynamics, trajectory generation, PD/inverse-dynamics control, RK4 simulation, and 3D visualization. Intended for robotics education and experimentation.

## Commands

```bash
# Install (creates .venv, installs editable)
uv sync --extra dev

# Run tests
uv run python tests/test_kinematics.py
uv run python tests/test_dynamics.py
uv run python tests/test_trajectory.py

# Run examples
uv run python examples/example_1_forward_kinematics.py
uv run python examples/example_2_trajectory.py
uv run python examples/example_3_dynamics.py
uv run python examples/example_4_control.py

# Regenerate dynamics code from symbolic derivation (~30-90s)
uv run python scripts/generate_dynamics.py

# Verify regenerated dynamics
uv run python scripts/verify_generated_dynamics.py
```

## Architecture

The `pyrobo` package (under `src/pyrobo/`) implements a rigid-body RRR manipulator pipeline. Imports look like `from pyrobo.dynamics import compute_D`. Tests, examples, and code-generation scripts live at the repo root and invoke the package.

### RRR pipeline — data flow

Model parameters (`src/pyrobo/models/three_dof_rrr.py`) define a single dict `{g, m, L, I}` (gravity, mass, link length, inertia — all links identical). This dict is threaded through dynamics, control, and simulation functions.

The state vector convention throughout is `z = [q1, q2, q3, qd1, qd2, qd3]` (3 joint positions + 3 joint velocities).

### Module dependency chain

```
models → kinematics → dynamics → control → simulation → visualization
                                    ↑
                              scripts/ (code generation)
```

- **`src/pyrobo/kinematics/`** — Forward kinematics and Jacobians using DH convention. Returns 4x4 homogeneous transforms (`T10`, `T20`, `T30`). Also computes COM transforms.
- **`src/pyrobo/dynamics/`** — **Auto-generated code.** `compute_D`, `compute_B`, `compute_C`, `compute_G` take signature `(q, L, m, g, I)`. Do not hand-edit these files; regenerate via `uv run python scripts/generate_dynamics.py`.
- **`src/pyrobo/control/`** — PD control and computed-torque (inverse dynamics) control. Supports gravity compensation and optional full nonlinear decoupling.
- **`src/pyrobo/simulation/`** — RK4 integrator. `dynamics_openloop.py` for uncontrolled, `dynamics_closedloop.py` for controlled simulation. Both produce `zdot` vectors.
- **`src/pyrobo/trajectory/`** — Cubic spline trajectory generation with via-points.
- **`src/pyrobo/visualization/`** — `RobotRenderer` (3D matplotlib animation) and plotting utilities for joint tracking, energy, errors.
- **`src/pyrobo/utils/`** — Rotation matrices and homogeneous transform helpers.

### Dynamics code generation pipeline

The files in `src/pyrobo/dynamics/` are generated, not handwritten. The pipeline:
1. `scripts/symbolic_derivation.py` — Derives D, B, C, G matrices symbolically via Lagrangian mechanics (SymPy)
2. `scripts/code_generator.py` — Converts symbolic expressions to optimized NumPy code (with CSE)
3. `scripts/generate_dynamics.py` — Orchestrates derivation + code gen, writes to `src/pyrobo/dynamics/`, creates backups in `src/pyrobo/dynamics/backup/` as `*.py.backup`
4. `scripts/verify_generated_dynamics.py` — Numerically compares the newly generated functions against the backup copies to confirm equivalence after a regeneration

To change dynamics equations, edit `scripts/symbolic_derivation.py` then regenerate. Requires `sympy>=1.12`. Run via `uv run python scripts/generate_dynamics.py`. See `scripts/GENERATION_GUIDE.md` for a short operator-oriented summary.

### Dynamics matrix conventions

The equation of motion is: `D(q) * qdd + B(q) * qdqd + C(q) * qd² + G(q) = tau`
- `D` — 3x3 mass/inertia matrix
- `B` — 3x3 Coriolis matrix (cross-velocity terms: `q̇ᵢq̇ⱼ`)
- `C` — 3x3 centrifugal matrix (squared-velocity terms: `q̇ᵢ²`)
- `G` — 3x1 gravity vector

### Testing

Tests are plain Python scripts using `assert` and `np.allclose`, run via `uv run python tests/<name>.py`. There is no pytest configuration. Each test file has a `__main__` block that runs all test functions. Tests cover `kinematics`, `dynamics`, and `trajectory` only — there is no automated test for `control`, `simulation`, or `visualization`; those are exercised via the examples.
