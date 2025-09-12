# PyRobot - 3-DOF RRR Robot Analysis and Control

## Project Overview
This is a Python implementation of a robotics course project, originally developed in MATLAB. The project focuses on comprehensive analysis and control of a 3-Degrees of Freedom (3-DOF) Revolute-Revolute-Revolute (RRR) robot arm.

## Project Structure

```
pyrobot/
├── README.md                           # This file
├── requirements.txt                    # Python dependencies
├── src/                               # Source code directory
│   ├── __init__.py
│   ├── kinematics/                    # Kinematics analysis
│   │   ├── __init__.py
│   │   ├── forward_kinematics.py     # Forward kinematics (T10, T20, T30)
│   │   └── jacobian.py               # Jacobian calculations
│   ├── dynamics/                      # Robot dynamics
│   │   ├── __init__.py
│   │   ├── inertia_matrix.py         # D matrix calculation
│   │   ├── coriolis_matrix.py        # B matrix calculation
│   │   ├── centrifugal_matrix.py     # C matrix calculation
│   │   ├── gravity_vector.py         # G vector calculation
│   │   └── robot_model.py            # Robot parameter initialization
│   ├── trajectory/                    # Trajectory planning
│   │   ├── __init__.py
│   │   ├── cubic_spline.py           # Cubic spline trajectory generation
│   │   └── via_points.py             # Via point trajectory planning
│   ├── control/                       # Control systems
│   │   ├── __init__.py
│   │   ├── pd_controller.py          # PD control implementation
│   │   ├── simulation.py             # Dynamic simulation
│   │   └── tracking.py               # Trajectory tracking control
│   ├── visualization/                 # 3D visualization and plotting
│   │   ├── __init__.py
│   │   ├── robot_renderer.py         # 3D robot visualization
│   │   ├── trajectory_plotter.py     # Trajectory plotting
│   │   └── energy_analysis.py        # Energy balance plotting
│   └── utils/                         # Utility functions
│       ├── __init__.py
│       ├── math_utils.py             # Mathematical utilities
│       └── data_utils.py             # Data handling utilities
├── examples/                          # Example scripts and demos
│   ├── __init__.py
│   ├── forward_kinematics_demo.py    # Demo 1: Forward kinematics
│   ├── trajectory_planning_demo.py   # Demo 2: Trajectory planning
│   ├── dynamic_simulation_demo.py    # Demo 3: Dynamic simulation
│   ├── control_demo.py               # Demo 4: Control systems
│   └── complete_system_demo.py       # Demo 5: Complete system
├── tests/                            # Unit tests
│   ├── __init__.py
│   ├── test_kinematics.py
│   ├── test_dynamics.py
│   ├── test_trajectory.py
│   └── test_control.py
├── docs/                             # Documentation
│   ├── api_reference.md
│   ├── user_guide.md
│   └── theory_background.md
└── data/                             # Data files and results
    ├── trajectories/
    ├── simulations/
    └── plots/
```

## Key Features

### 1. **Robot Kinematics**
- **Forward Kinematics**: Complete transformation matrices (T10, T20, T30)
- **Jacobian Analysis**: Linear and angular velocity Jacobians for all links
- **End-effector Position**: Real-time position calculation

### 2. **Robot Dynamics**
- **Inertia Matrix (D)**: Mass and inertia properties
- **Coriolis Matrix (B)**: Coriolis forces
- **Centrifugal Matrix (C)**: Centrifugal forces  
- **Gravity Vector (G)**: Gravitational effects
- **Energy Analysis**: Kinetic and potential energy calculations

### 3. **Trajectory Planning**
- **Cubic Spline Interpolation**: Smooth trajectory generation
- **Via Point Planning**: Multi-segment trajectory planning
- **Velocity Profiles**: Continuous velocity and acceleration profiles

### 4. **Control Systems**
- **PD Control**: Proportional-derivative control
- **Nonlinear Decoupling**: Full dynamic decoupling
- **Trajectory Tracking**: Real-time trajectory following
- **Step Response**: System response analysis

### 5. **Visualization & Analysis**
- **3D Robot Visualization**: Real-time animated robot simulation
- **Trajectory Plotting**: Position, velocity, and acceleration plots
- **Energy Balance**: Conservation of energy verification
- **Error Analysis**: Tracking error visualization

## Technology Stack

### Core Libraries
- **NumPy**: Numerical computations and linear algebra
- **SciPy**: Scientific computing and optimization
- **SymPy**: Symbolic mathematics (for Jacobian derivation)
- **Matplotlib**: 2D plotting and visualization
- **Plotly**: Interactive 3D visualization
- **Pandas**: Data manipulation and analysis

### Additional Libraries
- **scikit-learn**: Machine learning utilities (if needed)
- **Jupyter**: Interactive notebooks for demos
- **pytest**: Unit testing framework
- **black**: Code formatting
- **flake8**: Code linting

## Implementation Plan

For detailed implementation phases and timeline, see [plan.md](plan.md).

---