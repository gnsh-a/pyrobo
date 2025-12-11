"""
Unit tests for trajectory module.

Run with: python -m tests.test_trajectory
"""

import numpy as np
from src.trajectory import cubic_spline_coeffs, evaluate_spline, multi_segment_trajectory


def test_cubic_spline_boundary_conditions():
    """Test that cubic spline satisfies boundary conditions."""
    q0 = 0.0
    qf = np.pi
    dq0 = 0.0
    dqf = 0.0
    t0 = 0.0
    tf = 10.0
    
    a = cubic_spline_coeffs(q0, qf, dq0, dqf, t0, tf)
    
    # Evaluate at boundaries
    q_t0, dq_t0 = evaluate_spline(a, t0)
    q_tf, dq_tf = evaluate_spline(a, tf)
    
    # Check boundary conditions
    assert np.isclose(q_t0, q0), f"Position at t0: {q_t0} != {q0}"
    assert np.isclose(q_tf, qf), f"Position at tf: {q_tf} != {qf}"
    assert np.isclose(dq_t0, dq0), f"Velocity at t0: {dq_t0} != {dq0}"
    assert np.isclose(dq_tf, dqf), f"Velocity at tf: {dq_tf} != {dqf}"
    
    print("✓ Cubic spline boundary conditions test passed")


def test_cubic_spline_coefficients_shape():
    """Test that cubic spline returns correct number of coefficients."""
    a = cubic_spline_coeffs(0, 1, 0, 0, 0, 1)
    
    assert len(a) == 4, "Should return 4 coefficients"
    
    print("✓ Cubic spline coefficients shape test passed")


def test_multi_segment_trajectory():
    """Test multi-segment trajectory generation."""
    times = np.array([0, 2, 4])
    positions = np.array([0, np.pi/2, np.pi])
    velocities = np.array([0, 0, 0])
    
    segments = multi_segment_trajectory(times, positions, velocities)
    
    # Should have 2 segments for 3 via points
    assert len(segments) == 2
    
    # Each segment should have required keys
    for seg in segments:
        assert 't0' in seg
        assert 'tf' in seg
        assert 'a' in seg
        assert len(seg['a']) == 4
    
    print("✓ Multi-segment trajectory test passed")


def test_trajectory_continuity():
    """Test that multi-segment trajectory is continuous at via points."""
    times = np.array([0, 1, 2])
    positions = np.array([0, 1, 0])
    velocities = np.array([0, 0, 0])
    
    segments = multi_segment_trajectory(times, positions, velocities)
    
    # Check continuity at the middle via point
    t_via = times[1]
    
    # Evaluate first segment at t_via
    q1, dq1 = evaluate_spline(segments[0]['a'], t_via)
    
    # Evaluate second segment at t_via
    q2, dq2 = evaluate_spline(segments[1]['a'], t_via)
    
    # Should be continuous (same position)
    assert np.isclose(q1, positions[1])
    assert np.isclose(q2, positions[1])
    
    print("✓ Trajectory continuity test passed")


if __name__ == "__main__":
    test_cubic_spline_boundary_conditions()
    test_cubic_spline_coefficients_shape()
    test_multi_segment_trajectory()
    test_trajectory_continuity()
    print("\n" + "="*50)
    print("All trajectory tests passed!")
    print("="*50)

