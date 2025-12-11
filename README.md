# PyRobo: 3-DOF RRR Manipulator Simulation Library

A modular Python library for simulating a 3-degree-of-freedom (3-DOF) revolute-revolute-revolute (RRR) manipulator with kinematics, dynamics, trajectory planning, control, and visualization capabilities.

This is a Python version of my custom MATLAB robotics toolkit, providing a clean, extensible framework for robot simulation and control.

## Features

- **Forward Kinematics**: DH-convention based transformation matrices
- **Jacobian Computation**: Linear and angular velocity Jacobians for dynamics
- **Dynamics**: Full manipulator dynamics (mass, Coriolis, centrifugal, gravity)
- **Trajectory Planning**: Cubic spline generation with via-point interpolation
- **Numerical Integration**: 4th-order Runge-Kutta (RK4) integration
- **Control**: PD control with inverse dynamics (computed torque control)
- **Visualization**: 3D matplotlib animation and plotting utilities
- **Energy Verification**: Kinetic and potential energy tracking

## Installation

### Requirements

- Python >= 3.8
- NumPy >= 1.24
- Matplotlib >= 3.7
- SymPy >= 1.12 (optional, only needed for regenerating dynamics code)

### Setup

```bash
# Clone the repository
cd /Users/gnsh/work/repo/pyrobo

# Install in development mode
pip install -e .

# Or install requirements directly
pip install -r requirements.txt
```

## Running Examples

The library includes four comprehensive examples:

```bash
# Example 1: Forward kinematics with animation
python -m examples.example_1_forward_kinematics

# Example 2: Trajectory generation with via points
python -m examples.example_2_trajectory

# Example 3: Dynamic simulation with energy verification
python -m examples.example_3_dynamics

# Example 4: Trajectory tracking control
python -m examples.example_4_control
```

## Module Overview

### Kinematics (`src.kinematics`)

- `forward_kinematics(q, L)`: Compute transformation matrices T10, T20, T30
- `forward_kinematics_com(q, L)`: Transformations to link centers of mass
- `compute_jacobians(q, L)`: Linear and angular velocity Jacobians

### Dynamics (`src.dynamics`)

- `compute_D(q, L, m, g, I)`: Joint-space mass matrix
- `compute_B(q, L, m, g, I)`: Coriolis matrix
- `compute_C(q, L, m, g, I)`: Centrifugal matrix
- `compute_G(q, L, m, g, I)`: Gravity vector

The dynamics equation is:
```
D(q)·q̈ + B(q)·[q̇₁q̇₂, q̇₁q̇₃, q̇₂q̇₃]ᵀ + C(q)·[q̇₁², q̇₂², q̇₃²]ᵀ + G(q) = τ
```

### Trajectory (`src.trajectory`)

- `cubic_spline_coeffs(q0, qf, dq0, dqf, t0, tf)`: Cubic polynomial coefficients
- `multi_segment_trajectory(times, positions, velocities)`: Via-point trajectory
- `evaluate_trajectory(segments, t)`: Evaluate multi-segment trajectory

### Simulation (`src.simulation`)

- `rk4_step(zdot_func, z, t, dt, *args)`: Single RK4 integration step
- `simulate_rk4(zdot_func, z0, t_span, dt, *args)`: Full simulation
- `zdot_3dof(z, model_params)`: Open-loop dynamics
- `zdot_3dof_control(z, z_des, model_params, ...)`: Controlled dynamics

### Control (`src.control`)

- `pd_control(q, qd, q_des, qd_des, Kp, Kd)`: PD controller
- `compute_control_torques(...)`: Inverse dynamics control
- `compute_pd_gains(wn, zeta)`: Compute PD gains from specifications

### Visualization (`src.visualization`)

- `RobotRenderer`: 3D robot animation class
- `plot_trajectory(t, x, y, z)`: End-effector trajectory plots
- `plot_energy(t, T, V, E)`: Energy verification plots
- `plot_joint_tracking(t, q, q_des)`: Tracking performance plots

## Testing

Run unit tests to verify installation:

```bash
# Test all modules
python -m tests.test_kinematics
python -m tests.test_dynamics
python -m tests.test_trajectory
```

## Robot Model

The 3-DOF RRR manipulator has the following default parameters:

| Parameter | Symbol | Value | Units |
|-----------|--------|-------|-------|
| Gravitational constant | g | 9.81 | m/s² |
| Link mass | m | 10.0 | kg |
| Link length | L | 2.0 | m |
| Link inertia | I | 5.0 | kg·m² |

All three links are identical with:
- Center of mass at L/2
- Same mass and inertia properties

## Architecture

The library follows a functional architecture with modular design:

```
src/
├── kinematics/     # Forward kinematics and Jacobians
├── dynamics/       # D, B, C, G matrices
├── trajectory/     # Cubic spline generation
├── control/        # PD and inverse dynamics control
├── simulation/     # RK4 integration and dynamics
├── visualization/  # 3D rendering and plotting
├── models/         # Robot parameters
└── utils/          # Transformation utilities
```

## Extending the Library

The library is designed to be extensible:

1. **Add new robot models**: Create new parameter files in `src/models/`
2. **Implement custom controllers**: Extend `src/control/` with new control laws
3. **Add trajectory types**: Implement new trajectory generators in `src/trajectory/`
4. **Custom visualizations**: Extend `RobotRenderer` or create new plotting utilities

## Regenerating Dynamics (Advanced)

The dynamics functions in `src/dynamics/` are **auto-generated** from first principles using symbolic mathematics. This provides transparency and makes the derivation process repeatable.

### How It Works

The dynamics equations (D, B, C, G matrices) are derived symbolically using SymPy following these steps:

1. **Forward Kinematics** → Transformation matrices to each link and center of mass
2. **Jacobians** → Linear and angular velocity mappings (∂v/∂q̇, ∂ω/∂q̇)
3. **Lagrangian Mechanics** → Mass matrix from kinetic energy
4. **Christoffel Symbols** → Coriolis and centrifugal terms from ∂D/∂q
5. **Gravity Vector** → Potential energy derivatives
6. **Code Generation** → Optimized NumPy code with common subexpression elimination

### Regenerating the Code

To regenerate the dynamics functions (e.g., if you modify the robot geometry):

```bash
# Install SymPy if not already installed
pip install sympy>=1.12

# Run the code generator (~30-90 seconds)
python scripts/generate_dynamics.py
```

This will:
- Derive D, B, C, G matrices symbolically from scratch
- Apply optimizations (common subexpression elimination)
- Generate new Python files in `src/dynamics/`
- Backup old files to `src/dynamics/backup/`

The generated functions produce **identical results** to the hand-coded versions (within floating-point precision) but the derivation process is now transparent and modifiable.

### Generator Scripts

- `scripts/symbolic_derivation.py` - Core symbolic math (Lagrangian mechanics)
- `scripts/code_generator.py` - Converts SymPy expressions to optimized NumPy code  
- `scripts/generate_dynamics.py` - Main script to run the full generation process

## Performance Notes

- RK4 integration with custom implementation (no scipy dependency)
- Dynamics matrices use optimized NumPy operations
- Visualization uses matplotlib's interactive mode for animation
- Energy drift in long simulations is expected due to numerical integration

## License

This project is provided as-is for educational purposes.

## Acknowledgments

- Based on ME/ECE 441 robotics coursework
- Original MATLAB implementation by Ganesh Arivoli
- Python port maintains mathematical correctness of dynamics equations

## Contact

For questions or issues, please open an issue on the repository.
