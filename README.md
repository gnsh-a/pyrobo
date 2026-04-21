## PyRobo: 3-DOF Manipulator Simulation

PyRobo is a small Python library for simulating a 3-DOF revolute-revolute-revolute (RRR) manipulator, including kinematics, dynamics, trajectory generation, basic control, and simple 3D visualization.

### Main capabilities

- **Kinematics**: Forward kinematics and Jacobians
- **Dynamics**: Mass, Coriolis/centrifugal, and gravity terms
- **Trajectory**: Cubic spline trajectories with via-points
- **Control**: Simple PD and inverse-dynamics control
- **Simulation & plots**: RK4 integration, joint/trajectory/energy plots

---

### Installation

Requirements (typical setup):
- Python ≥ 3.10
- NumPy, Matplotlib

Install in editable/development mode from the repo root:

```bash
uv sync --extra dev
```

---

### Running the examples

From the repository root:

```bash
# Example 1: Forward kinematics + animation
uv run python examples/example_1_forward_kinematics.py

# Example 2: Trajectory generation
uv run python examples/example_2_trajectory.py

# Example 3: Dynamics + energy verification
uv run python examples/example_3_dynamics.py

# Example 4: Trajectory tracking control
uv run python examples/example_4_control.py
```

---

### Tests

Run basic unit tests from the repo root:

```bash
uv run python tests/test_kinematics.py
uv run python tests/test_dynamics.py
uv run python tests/test_trajectory.py
```

---

### Notes

- Default robot parameters (masses, lengths, inertia, gravity) are defined in the model code under `src/pyrobo/models/`.
- Advanced symbolic derivation and code-generation scripts live in `scripts/`, but are not needed for normal use.

This project is intended for robotics education and experimentation.
