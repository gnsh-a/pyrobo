"""
Numerical integration methods for dynamic simulation.

Provides 4th-order Runge-Kutta (RK4) integration for solving ODEs.
"""

import numpy as np


def rk4_step(zdot_func, z, t, dt, *args):
    """
    Perform a single 4th-order Runge-Kutta integration step.
    
    Parameters
    ----------
    zdot_func : callable
        Function that computes the state derivative: zdot = f(z, t, *args)
    z : ndarray
        Current state vector
    t : float
        Current time
    dt : float
        Time step
    *args : 
        Additional arguments to pass to zdot_func
        
    Returns
    -------
    z_next : ndarray
        State at time t + dt
        
    Notes
    -----
    Implements the classic 4th-order Runge-Kutta method:
        k1 = f(z, t)
        k2 = f(z + 0.5*k1*dt, t + 0.5*dt)
        k3 = f(z + 0.5*k2*dt, t + 0.5*dt)
        k4 = f(z + k3*dt, t + dt)
        z_next = z + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
    """
    k1 = zdot_func(z, *args)
    k2 = zdot_func(z + 0.5 * k1 * dt, *args)
    k3 = zdot_func(z + 0.5 * k2 * dt, *args)
    k4 = zdot_func(z + k3 * dt, *args)
    
    z_next = z + (k1 + 2*k2 + 2*k3 + k4) * dt / 6.0
    
    return z_next


def simulate_rk4(zdot_func, z0, t_span, dt, *args):
    """
    Simulate a dynamical system using 4th-order Runge-Kutta integration.
    
    Parameters
    ----------
    zdot_func : callable
        Function that computes state derivative: zdot = f(z, *args)
    z0 : array_like
        Initial state vector
    t_span : tuple (t0, tf)
        Start and end times for simulation
    dt : float
        Integration time step
    *args :
        Additional arguments to pass to zdot_func
        
    Returns
    -------
    t : ndarray (n,)
        Time points
    z : ndarray (n, len(z0))
        State history, where z[i, :] is the state at time t[i]
        
    Notes
    -----
    This is a convenience function that repeatedly calls rk4_step()
    to integrate over the entire time span.
    """
    z0 = np.asarray(z0)
    t0, tf = t_span
    
    # Create time array
    num_steps = int(np.ceil((tf - t0) / dt))
    t = np.linspace(t0, tf, num_steps + 1)
    
    # Pre-allocate state array
    z = np.zeros((len(t), len(z0)))
    z[0, :] = z0
    
    # Integrate
    for i in range(len(t) - 1):
        z[i+1, :] = rk4_step(zdot_func, z[i, :], t[i], dt, *args)
    
    return t, z

