"""
Example 2: Trajectory Generation

This example demonstrates cubic spline trajectory generation with via points.

Run with: python -m examples.example_2_trajectory
"""

import numpy as np
import matplotlib.pyplot as plt
from src.trajectory import cubic_spline_coeffs, multi_segment_trajectory, evaluate_trajectory
from src.visualization.plotting_utils import plot_spline_trajectory


def part_2b():
    """
    Part 2B: Single cubic spline segment.
    
    Generate a cubic spline with specified initial and final conditions.
    """
    print("=" * 60)
    print("PART 2B: Single Cubic Spline Segment")
    print("=" * 60)
    
    # Define input parameters
    t0 = 0
    q0 = 0
    dq0 = 0
    tf = 10
    qf = np.pi
    dqf = 0
    
    # Calculate coefficients
    a = cubic_spline_coeffs(q0, qf, dq0, dqf, t0, tf)
    
    print(f"\nCubic Polynomial Coefficients:")
    print(f"  a0 = {a[0]:.6f}")
    print(f"  a1 = {a[1]:.6f}")
    print(f"  a2 = {a[2]:.6f}")
    print(f"  a3 = {a[3]:.6f}")
    
    # Create time vector for plotting
    t = np.linspace(t0, tf, 200)
    
    # Evaluate position and velocity
    q = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
    dq = a[1] + 2*a[2]*t + 3*a[3]*t**2
    
    # Verify boundary conditions
    print(f"\nBoundary Condition Verification:")
    print(f"  q(t0)  = {q[0]:.6f}  (expected: {q0:.6f})")
    print(f"  q(tf)  = {q[-1]:.6f}  (expected: {qf:.6f})")
    print(f"  dq(t0) = {dq[0]:.6f}  (expected: {dq0:.6f})")
    print(f"  dq(tf) = {dq[-1]:.6f}  (expected: {dqf:.6f})")
    
    # Plot
    plot_spline_trajectory(t, q, dq, 
                          title="Part 2B: Cubic Spline Trajectory")
    

def part_2c():
    """
    Part 2C: Multi-segment trajectory with via points.
    
    Generate a trajectory through 5 via points using cubic splines.
    """
    print("\n" + "=" * 60)
    print("PART 2C: Multi-Segment Trajectory with Via Points")
    print("=" * 60)
    
    # Define via points
    times = np.array([0, 2, 4, 6, 8])
    positions = np.array([0, np.pi/4, 3*np.pi/4, np.pi/2, 0])
    
    # Define velocities (start and end at zero, use heuristic for intermediate)
    velocities = np.zeros(5)
    velocities[0] = 0
    velocities[-1] = 0
    velocities[1] = 3*np.pi/16   # Slopes have same sign
    velocities[2] = 0              # Slopes have different signs
    velocities[3] = -3*np.pi/16    # Slopes have same sign
    
    print(f"\nVia Points:")
    print(f"  Times:      {times}")
    print(f"  Positions:  {positions}")
    print(f"  Velocities: {velocities}")
    
    # Generate multi-segment trajectory
    segments = multi_segment_trajectory(times, positions, velocities)
    
    # Create dense time vector for plotting
    t_plot = np.linspace(times[0], times[-1], 400)
    q_plot, dq_plot = evaluate_trajectory(segments, t_plot)
    
    # Plot trajectory
    fig = plot_spline_trajectory(t_plot, q_plot, dq_plot,
                                title="Part 2C: Multi-Segment Cubic Spline Trajectory")
    
    # Mark via points
    axes = fig.get_axes()
    axes[0].plot(times, positions, 'ro', markersize=8, label='Via Points')
    axes[0].legend()
    axes[1].plot(times, velocities, 'ro', markersize=8, label='Via Point Velocities')
    axes[1].legend()


def main():
    """Run trajectory generation examples."""
    # Part 2B: Single segment
    part_2b()
    
    # Part 2C: Multi-segment with via points
    part_2c()
    
    # Show all plots
    plt.show()
    print("\nTrajectory generation examples complete!")


if __name__ == "__main__":
    main()

