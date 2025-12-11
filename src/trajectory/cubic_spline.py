"""
Cubic spline trajectory generation for smooth motion planning.

This module provides functions for generating cubic polynomial trajectories
with specified boundary conditions and multi-segment via-point trajectories.
"""

import numpy as np


def cubic_spline_coeffs(q0, qf, dq0, dqf, t0, tf):
    """
    Calculate coefficients of a cubic polynomial spline.
    
    The cubic spline is defined as:
        q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    
    Parameters
    ----------
    q0 : float
        Initial position
    qf : float
        Final position
    dq0 : float
        Initial velocity
    dqf : float
        Final velocity
    t0 : float
        Initial time
    tf : float
        Final time
        
    Returns
    -------
    a : ndarray (4,)
        Spline coefficients [a0, a1, a2, a3]
    """
    # Construct the coefficient matrix C
    C = np.array([
        [1,  t0,   t0**2,     t0**3],
        [0,   1,  2*t0,    3*t0**2],
        [1,  tf,   tf**2,     tf**3],
        [0,   1,  2*tf,    3*tf**2]
    ])
    
    # Construct the known values vector b
    b = np.array([q0, dq0, qf, dqf])
    
    # Solve the linear system C * a = b for coefficients a
    a = np.linalg.solve(C, b)
    
    return a


def evaluate_spline(a, t):
    """
    Evaluate cubic spline position and velocity at given time(s).
    
    Parameters
    ----------
    a : array_like (4,)
        Spline coefficients [a0, a1, a2, a3]
    t : float or array_like
        Time value(s) at which to evaluate the spline
        
    Returns
    -------
    q : float or ndarray
        Position at time t
    dq : float or ndarray
        Velocity at time t
    """
    a = np.asarray(a)
    t = np.asarray(t)
    
    # Position: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    q = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
    
    # Velocity: dq(t) = a1 + 2*a2*t + 3*a3*t^2
    dq = a[1] + 2*a[2]*t + 3*a[3]*t**2
    
    return q, dq


def multi_segment_trajectory(times, positions, velocities=None):
    """
    Generate a multi-segment cubic spline trajectory through via points.
    
    Parameters
    ----------
    times : array_like (n,)
        Time values for each via point (must be monotonically increasing)
    positions : array_like (n,)
        Position values at each via point
    velocities : array_like (n,), optional
        Velocity values at each via point. If None, velocities at intermediate
        points are computed using a heuristic (average of adjacent slopes).
        Start and end velocities are set to zero if not provided.
        
    Returns
    -------
    segments : list of dict
        List of segment information, each containing:
        - 't0': start time
        - 'tf': end time
        - 'a': coefficients [a0, a1, a2, a3]
        
    Notes
    -----
    If velocities are not provided, intermediate via point velocities are
    computed using the heuristic:
        - If adjacent slopes have same sign: v = (m1 + m2) / 2
        - If adjacent slopes have different signs: v = 0
    where m1 and m2 are the slopes of adjacent segments.
    """
    times = np.asarray(times)
    positions = np.asarray(positions)
    n = len(times)
    
    if len(positions) != n:
        raise ValueError("times and positions must have same length")
    
    if n < 2:
        raise ValueError("Need at least 2 via points")
    
    # Compute velocities if not provided
    if velocities is None:
        velocities = np.zeros(n)
        # Start and end velocities are zero
        velocities[0] = 0
        velocities[-1] = 0
        
        # Compute intermediate velocities using heuristic
        for i in range(1, n-1):
            # Compute slopes of adjacent segments
            m1 = (positions[i] - positions[i-1]) / (times[i] - times[i-1])
            m2 = (positions[i+1] - positions[i]) / (times[i+1] - times[i])
            
            # Apply heuristic
            if m1 * m2 > 0:  # Same sign
                velocities[i] = (m1 + m2) / 2
            else:  # Different signs or one is zero
                velocities[i] = 0
    else:
        velocities = np.asarray(velocities)
        if len(velocities) != n:
            raise ValueError("velocities must have same length as times/positions")
    
    # Generate segments
    segments = []
    for i in range(n-1):
        t0 = times[i]
        tf = times[i+1]
        q0 = positions[i]
        qf = positions[i+1]
        dq0 = velocities[i]
        dqf = velocities[i+1]
        
        # Compute coefficients for this segment
        a = cubic_spline_coeffs(q0, qf, dq0, dqf, t0, tf)
        
        segments.append({
            't0': t0,
            'tf': tf,
            'a': a
        })
    
    return segments


def evaluate_trajectory(segments, t):
    """
    Evaluate a multi-segment trajectory at given time(s).
    
    Parameters
    ----------
    segments : list of dict
        Segment information from multi_segment_trajectory()
    t : float or array_like
        Time value(s) at which to evaluate
        
    Returns
    -------
    q : float or ndarray
        Position at time t
    dq : float or ndarray
        Velocity at time t
        
    Notes
    -----
    Values outside the trajectory time range are extrapolated using the
    first or last segment.
    """
    t = np.asarray(t)
    scalar_input = t.ndim == 0
    t = np.atleast_1d(t)
    
    q = np.zeros_like(t)
    dq = np.zeros_like(t)
    
    for i, ti in enumerate(t):
        # Find appropriate segment
        seg_idx = 0
        for j, seg in enumerate(segments):
            if ti >= seg['t0']:
                seg_idx = j
            else:
                break
        
        # Evaluate spline for this segment
        q[i], dq[i] = evaluate_spline(segments[seg_idx]['a'], ti)
    
    if scalar_input:
        return q[0], dq[0]
    return q, dq

