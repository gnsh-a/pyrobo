"""
Example 4: Trajectory Tracking Control

This example demonstrates inverse dynamics control (computed torque control)
for trajectory tracking with the 3-DOF RRR manipulator.

Run with: python -m examples.example_4_control
"""

import numpy as np
import matplotlib.pyplot as plt
from src.models import initialize_three_dof_rrr
from src.trajectory import cubic_spline_coeffs, evaluate_spline
from src.simulation import rk4_step, zdot_3dof_control
from src.visualization import RobotRenderer, plot_joint_tracking, plot_joint_errors


def generate_desired_trajectory(times, positions, velocities, t_sim):
    """
    Generate desired trajectory from via points for simulation time vector.
    
    Parameters
    ----------
    times : array_like
        Via point times
    positions : array_like
        Via point positions
    velocities : array_like
        Via point velocities
    t_sim : ndarray
        Simulation time vector
        
    Returns
    -------
    q_des : ndarray (len(t_sim),)
        Desired positions
    qd_des : ndarray (len(t_sim),)
        Desired velocities
    """
    num_pts = len(t_sim)
    q_des = np.zeros(num_pts)
    qd_des = np.zeros(num_pts)
    
    # Generate trajectory for each segment
    t_idx = 0
    for i in range(len(times) - 1):
        t0 = times[i]
        tf = times[i+1]
        q0 = positions[i]
        qf = positions[i+1]
        dq0 = velocities[i]
        dqf = velocities[i+1]
        
        # Compute cubic spline coefficients
        a = cubic_spline_coeffs(q0, qf, dq0, dqf, t0, tf)
        
        # Find time indices for this segment
        if i < len(times) - 2:
            # Not the last segment
            seg_mask = (t_sim >= t0) & (t_sim < tf)
        else:
            # Last segment (include endpoint)
            seg_mask = (t_sim >= t0) & (t_sim <= tf)
        
        t_seg = t_sim[seg_mask]
        
        # Evaluate spline
        q_seg, qd_seg = evaluate_spline(a, t_seg)
        
        q_des[seg_mask] = q_seg
        qd_des[seg_mask] = qd_seg
    
    return q_des, qd_des


def main():
    """Run trajectory tracking control example."""
    print("=" * 60)
    print("Trajectory Tracking Control")
    print("=" * 60)
    
    # Initialize model parameters
    model_params = initialize_three_dof_rrr()
    L = model_params['L']
    
    # Simulation parameters
    dt = 0.01  # integration time step [s]
    t_end = 8  # simulation duration [s]
    num_steps = int(t_end / dt) + 1
    t_sim = np.linspace(0, t_end, num_steps)
    
    # Define via points (for joint 1 and joint 3)
    via_times = np.array([0, 2, 4, 6, 8])
    via_positions = np.array([0, np.pi/4, 3*np.pi/4, np.pi/2, 0])
    via_velocities = np.array([0, 3*np.pi/16, 0, -3*np.pi/16, 0])
    
    print(f"\nSimulation Parameters:")
    print(f"  Time step:  {dt} s")
    print(f"  Duration:   {t_end} s")
    print(f"  Steps:      {num_steps}")
    
    print(f"\nVia Points:")
    print(f"  Times:      {via_times}")
    print(f"  Positions:  {via_positions}")
    print(f"  Velocities: {via_velocities}")
    
    # Generate desired trajectories for all joints
    q1_des, qd1_des = generate_desired_trajectory(via_times, via_positions, 
                                                   via_velocities, t_sim)
    q2_des = np.zeros(num_steps)  # Joint 2 stays at zero
    qd2_des = np.zeros(num_steps)
    q3_des = -q1_des  # Joint 3 mirrors joint 1 (negated)
    qd3_des = -qd1_des
    
    # Assemble full desired state
    q_des = np.column_stack([q1_des, q2_des, q3_des])
    qd_des = np.column_stack([qd1_des, qd2_des, qd3_des])
    z_des = np.column_stack([q_des, qd_des])
    
    # Initial conditions (start at desired initial state)
    z = z_des[0, :]
    
    print(f"\nInitial Conditions:")
    print(f"  q0  = {z[0:3]}")
    print(f"  qd0 = {z[3:6]}")
    
    # Pre-allocate arrays
    q = np.zeros((num_steps, 3))
    qd = np.zeros((num_steps, 3))
    q[0, :] = z[0:3]
    qd[0, :] = z[3:6]
    
    # Controller parameters
    wn = 2 * np.pi * 1  # 1 Hz natural frequency
    zeta = 0.7  # damping ratio
    
    print(f"\nController Parameters:")
    print(f"  Natural frequency: {wn/(2*np.pi):.2f} Hz")
    print(f"  Damping ratio:     {zeta:.2f}")
    print(f"  Kp = {wn**2:.4f}")
    print(f"  Kd = {2*zeta*wn:.4f}")
    
    # Integrate with control
    print("\nIntegrating controlled dynamics...")
    for i in range(num_steps - 1):
        # RK4 integration step with control
        z = rk4_step(zdot_3dof_control, z, t_sim[i], dt, 
                    z_des[i, :], model_params, wn, zeta)
        
        # Store results
        q[i+1, :] = z[0:3]
        qd[i+1, :] = z[3:6]
        
        # Progress indicator
        if (i+1) % (num_steps // 10) == 0:
            progress = 100 * (i+1) / num_steps
            print(f"  Progress: {progress:.0f}%")
    
    print("Integration complete!")
    
    # Compute tracking errors
    errors = q_des - q
    max_errors = np.max(np.abs(errors), axis=0)
    
    print(f"\nMaximum Tracking Errors:")
    print(f"  Joint 1: {max_errors[0]:.6f} rad ({np.degrees(max_errors[0]):.4f} deg)")
    print(f"  Joint 2: {max_errors[1]:.6f} rad ({np.degrees(max_errors[1]):.4f} deg)")
    print(f"  Joint 3: {max_errors[2]:.6f} rad ({np.degrees(max_errors[2]):.4f} deg)")
    
    # Plot joint tracking
    plot_joint_tracking(t_sim, q, q_des, 
                       joint_names=['Joint 1', 'Joint 2', 'Joint 3'],
                       title="Joint Position Tracking")
    
    # Plot tracking errors for each joint
    figs = plot_joint_errors(t_sim, q, q_des,
                            joint_names=['Joint 1', 'Joint 2', 'Joint 3'],
                            title="Joint Displacement and Tracking Error")
    
    # Animate the motion
    print("\nCreating 3D animation...")
    print("Close the plot window when done viewing.")
    
    renderer = RobotRenderer(L, axis_limits=[-5, 5, -5, 5, 0, 5])
    
    # Subsample for animation (every 2nd frame)
    q_animation = q[::2, :]
    renderer.animate(q_animation, dt=dt*2, draw_trajectory=True)
    
    plt.show()
    print("Control simulation complete!")


if __name__ == "__main__":
    main()

