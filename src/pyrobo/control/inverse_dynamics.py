"""
Control algorithms for the 3-DOF RRR manipulator.

Implements PD control and inverse dynamics (computed torque) control.
"""

import numpy as np
from ..dynamics import compute_D, compute_B, compute_C, compute_G


def pd_control(q, qd, q_des, qd_des, Kp, Kd):
    """
    Compute PD control torques.
    
    Parameters
    ----------
    q : array_like (3,)
        Current joint positions
    qd : array_like (3,)
        Current joint velocities
    q_des : array_like (3,)
        Desired joint positions
    qd_des : array_like (3,)
        Desired joint velocities
    Kp : float or array_like (3,)
        Proportional gain(s)
    Kd : float or array_like (3,)
        Derivative gain(s)
        
    Returns
    -------
    tau : ndarray (3,)
        Control torques
    """
    q = np.asarray(q)
    qd = np.asarray(qd)
    q_des = np.asarray(q_des)
    qd_des = np.asarray(qd_des)
    Kp = np.asarray(Kp)
    Kd = np.asarray(Kd)
    
    # Position and velocity errors
    e_pos = q_des - q
    e_vel = qd_des - qd
    
    # PD control law
    tau = Kp * e_pos + Kd * e_vel
    
    return tau


def compute_control_torques(q, qd, q_des, qd_des, qdd_des, model_params, 
                           Kp, Kd, gravity_compensation=True, full_decoupling=False):
    """
    Compute control torques using inverse dynamics (computed torque control).
    
    This implements nonlinear dynamic decoupling with PD control:
        tau = D(q) * tau' + G(q)  [with gravity compensation]
        tau = D(q) * tau' + V(q, qd) + G(q)  [with full decoupling]
    
    where tau' = -Kd*(qd - qd_des) - Kp*(q - q_des) + qdd_des
    
    Parameters
    ----------
    q : array_like (3,)
        Current joint positions
    qd : array_like (3,)
        Current joint velocities
    q_des : array_like (3,)
        Desired joint positions
    qd_des : array_like (3,)
        Desired joint velocities
    qdd_des : array_like (3,)
        Desired joint accelerations
    model_params : dict
        Model parameters (g, m, L, I)
    Kp : float or array_like (3,)
        Proportional gain(s)
    Kd : float or array_like (3,)
        Derivative gain(s)
    gravity_compensation : bool, default=True
        If True, includes gravity compensation (G term)
    full_decoupling : bool, default=False
        If True, includes full nonlinear terms (V = B*qdqd + C*qd2)
        
    Returns
    -------
    tau : ndarray (3,)
        Control torques
        
    Notes
    -----
    The decoupled system dynamics become:
        qdd = tau'
    
    For reference tracking, set tau' to include feedforward and feedback terms.
    """
    q = np.asarray(q)
    qd = np.asarray(qd)
    q_des = np.asarray(q_des)
    qd_des = np.asarray(qd_des)
    qdd_des = np.asarray(qdd_des)
    Kp = np.asarray(Kp)
    Kd = np.asarray(Kd)
    
    # Extract model parameters
    g = model_params['g']
    m = model_params['m']
    L = model_params['L']
    I = model_params['I']
    
    # Compute dynamics matrices
    D = compute_D(q, L, m, g, I)
    
    # Compute decoupled system control input
    e_pos = q_des - q
    e_vel = qd_des - qd
    tau_prime = -Kd * e_vel - Kp * e_pos + qdd_des
    
    # Compute control torques with inverse dynamics
    tau = D @ tau_prime
    
    if gravity_compensation:
        G = compute_G(q, L, m, g, I)
        tau = tau + G
    
    if full_decoupling:
        B = compute_B(q, L, m, g, I)
        C = compute_C(q, L, m, g, I)
        qdqd = np.array([qd[0]*qd[1], qd[0]*qd[2], qd[1]*qd[2]])
        qd2 = np.array([qd[0]**2, qd[1]**2, qd[2]**2])
        V = B @ qdqd + C @ qd2
        tau = tau + V
    
    return tau


def compute_pd_gains(wn, zeta):
    """
    Compute PD gains for a desired natural frequency and damping ratio.
    
    For a second-order system: qdd + 2*zeta*wn*qd + wn^2*q = 0
    The PD gains are:
        Kp = wn^2
        Kd = 2*zeta*wn
    
    Parameters
    ----------
    wn : float
        Desired natural frequency [rad/s]
    zeta : float
        Desired damping ratio (dimensionless)
        
    Returns
    -------
    Kp : float
        Proportional gain
    Kd : float
        Derivative gain
    """
    Kp = wn**2
    Kd = 2 * zeta * wn
    
    return Kp, Kd

