"""
Example 3: Dynamic Simulation with Energy Verification

This example demonstrates open-loop dynamic simulation of the 3-DOF RRR
manipulator with energy conservation verification.

Run with: python -m examples.example_3_dynamics
"""

import numpy as np
import matplotlib.pyplot as plt
from src.models import initialize_three_dof_rrr
from src.kinematics import forward_kinematics, forward_kinematics_com
from src.simulation import rk4_step, zdot_3dof
from src.dynamics import compute_D
from src.visualization import RobotRenderer, plot_energy


def compute_energy(q, qd, model_params):
    """
    Compute kinetic and potential energy of the system.
    
    Parameters
    ----------
    q : ndarray (3,)
        Joint positions
    qd : ndarray (3,)
        Joint velocities
    model_params : dict
        Model parameters
        
    Returns
    -------
    T : float
        Kinetic energy
    V : float
        Potential energy
    """
    g = model_params['g']
    m = model_params['m']
    L = model_params['L']
    I = model_params['I']
    
    # Kinetic energy: T = 0.5 * qd^T * D(q) * qd
    D = compute_D(q, L, m, g, I)
    T = 0.5 * qd @ D @ qd
    
    # Potential energy: V = -m*g^T*rc for each link
    # Get COM positions
    Tc10, Tc20, Tc30 = forward_kinematics_com(q, L)
    rc1 = Tc10[:3, 3]
    rc2 = Tc20[:3, 3]
    rc3 = Tc30[:3, 3]
    
    g_vector = np.array([0, 0, -g])
    V = -(m * g_vector @ rc1 + m * g_vector @ rc2 + m * g_vector @ rc3)
    
    return T, V


def main():
    """Run dynamic simulation with energy verification."""
    print("=" * 60)
    print("Dynamic Simulation with Energy Verification")
    print("=" * 60)
    
    # Initialize model parameters
    model_params = initialize_three_dof_rrr()
    L = model_params['L']
    
    # Simulation parameters
    dt = 0.01  # integration time step [s]
    t_end = 10  # simulation duration [s]
    num_steps = int(t_end / dt)
    
    # Initial conditions
    q0 = np.array([0.0, 0.0, 0.0])  # initial positions
    qd0 = np.array([1.0, -2.0, 1.0])  # initial velocities
    z = np.concatenate([q0, qd0])
    
    print(f"\nSimulation Parameters:")
    print(f"  Time step:  {dt} s")
    print(f"  Duration:   {t_end} s")
    print(f"  Steps:      {num_steps}")
    print(f"\nInitial Conditions:")
    print(f"  q0  = {q0}")
    print(f"  qd0 = {qd0}")
    
    # Pre-allocate arrays
    t = np.zeros(num_steps)
    q = np.zeros((num_steps, 3))
    qd = np.zeros((num_steps, 3))
    T_energy = np.zeros(num_steps)
    V_energy = np.zeros(num_steps)
    E_energy = np.zeros(num_steps)
    
    # Store initial state
    q[0, :] = q0
    qd[0, :] = qd0
    T_energy[0], V_energy[0] = compute_energy(q0, qd0, model_params)
    E_energy[0] = T_energy[0] + V_energy[0]
    
    print(f"\nInitial Energy:")
    print(f"  Kinetic:    {T_energy[0]:.4f} J")
    print(f"  Potential:  {V_energy[0]:.4f} J")
    print(f"  Total:      {E_energy[0]:.4f} J")
    
    # Integrate equations of motion
    print("\nIntegrating equations of motion...")
    for i in range(num_steps - 1):
        # RK4 integration step
        z = rk4_step(zdot_3dof, z, t[i], dt, model_params)
        
        # Store results
        t[i+1] = t[i] + dt
        q[i+1, :] = z[0:3]
        qd[i+1, :] = z[3:6]
        
        # Compute energy
        T_energy[i+1], V_energy[i+1] = compute_energy(q[i+1, :], qd[i+1, :], model_params)
        E_energy[i+1] = T_energy[i+1] + V_energy[i+1]
        
        # Progress indicator
        if (i+1) % (num_steps // 10) == 0:
            progress = 100 * (i+1) / num_steps
            print(f"  Progress: {progress:.0f}%")
    
    print("Integration complete!")
    
    # Final energy
    print(f"\nFinal Energy:")
    print(f"  Kinetic:    {T_energy[-1]:.4f} J")
    print(f"  Potential:  {V_energy[-1]:.4f} J")
    print(f"  Total:      {E_energy[-1]:.4f} J")
    print(f"\nEnergy Drift: {E_energy[-1] - E_energy[0]:.6f} J ({100*(E_energy[-1] - E_energy[0])/E_energy[0]:.4f}%)")
    
    # Plot energy
    plot_energy(t, T_energy, V_energy, E_energy,
               title="System Energy vs Time")
    
    # Animate the motion
    print("\nCreating 3D animation...")
    print("Close the plot window when done viewing.")
    
    renderer = RobotRenderer(L, axis_limits=[-4, 4, -4, 4, 0, 4])
    
    # Subsample for animation (every 2nd frame)
    q_animation = q[::2, :]
    renderer.animate(q_animation, dt=dt*2, draw_trajectory=True)
    
    plt.show()
    print("Simulation complete!")


if __name__ == "__main__":
    main()

