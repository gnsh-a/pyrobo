"""
Example 1: Forward Kinematics Animation

This example demonstrates forward kinematics computation and animation
of the 3-DOF RRR manipulator following sinusoidal joint trajectories.

Run with: python -m examples.example_1_forward_kinematics
"""

import numpy as np
import matplotlib.pyplot as plt
from src.models import initialize_three_dof_rrr
from src.kinematics import forward_kinematics
from src.visualization import RobotRenderer, plot_trajectory


def main():
    """Run forward kinematics example."""
    # Initialize model parameters
    model_params = initialize_three_dof_rrr()
    L = model_params['L']
    
    # Time vector
    t = np.linspace(0, 2*np.pi, 100)
    
    # Joint trajectories (sinusoidal motions)
    q1 = (np.pi/2) * np.sin(t)
    q2 = (np.pi/2) * np.sin(2*t)
    q3 = np.sin(3*t)
    
    # Pre-allocate arrays for end-effector position
    x = np.zeros(len(t))
    y = np.zeros(len(t))
    z = np.zeros(len(t))
    
    # Compute end-effector trajectory
    q_trajectory = []
    for i in range(len(t)):
        q = np.array([q1[i], q2[i], q3[i]])
        q_trajectory.append(q)
        
        # Compute forward kinematics
        T10, T20, T30 = forward_kinematics(q, L)
        
        # Extract end-effector position
        pos = T30[:3, 3]
        x[i] = pos[0]
        y[i] = pos[1]
        z[i] = pos[2]
    
    q_trajectory = np.array(q_trajectory)
    
    # Plot end-effector trajectory vs time
    plot_trajectory(t, x, y, z, 
                   title="Operational Point Position vs Time")
    
    # Create 3D animation
    print("Creating 3D animation...")
    print("Close the plot window when done viewing.")
    
    renderer = RobotRenderer(L, axis_limits=[-4, 4, -4, 4, 0, 4])
    renderer.animate(q_trajectory, dt=0.05, draw_trajectory=True)
    
    plt.show()
    print("Animation complete!")


if __name__ == "__main__":
    main()

