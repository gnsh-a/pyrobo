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
- Python ≥ 3.8
- NumPy, Matplotlib (see `requirements.txt` for exact versions)

Install in editable/development mode from the repo root:

```bash
pip install -e .
```

or just:

```bash
pip install -r requirements.txt
```

---

### Running the examples

From the repository root:

```bash
# Example 1: Forward kinematics + animation
python -m examples.example_1_forward_kinematics

# Example 2: Trajectory generation
python -m examples.example_2_trajectory

# Example 3: Dynamics + energy verification
python -m examples.example_3_dynamics

# Example 4: Trajectory tracking control
python -m examples.example_4_control
```

---

### Tests

Run basic unit tests from the repo root:

```bash
python -m tests.test_kinematics
python -m tests.test_dynamics
python -m tests.test_trajectory
```

---

### Notes

- Default robot parameters (masses, lengths, inertia, gravity) are defined in the model code under `src/models/`.
- Advanced symbolic derivation and code-generation scripts live in `scripts/`, but are not needed for normal use.

This project is intended for robotics education and experimentation.
