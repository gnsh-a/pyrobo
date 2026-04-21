"""
Closed-loop dynamics simulation for the 3-DOF RRR manipulator.

Computes state derivatives for numerical integration with control.
"""

import numpy as np
from ..dynamics import compute_D, compute_B, compute_C, compute_G
from ..control.inverse_dynamics import compute_pd_gains


def zdot_3dof_control(z, z_des, model_params, wn=2*np.pi, zeta=0.7, 
                      gravity_compensation=True, full_decoupling=False):
    """
    Compute state derivative for controlled 3-DOF manipulator dynamics.
    
    Uses inverse dynamics (computed torque) control with PD feedback.
    
    State vector z = [q1, q2, q3, qd1, qd2, qd3] where:
        q = [q1, q2, q3] are joint positions
        qd = [qd1, qd2, qd3] are joint velocities
    
    The control law is:
        tau = D(q) * tau' + G(q)  [with gravity compensation]
    where:
        tau' = -Kd*(qd - qd_des) - Kp*(q - q_des)
    
    Parameters
    ----------
    z : ndarray (6,)
        Current state vector [q1, q2, q3, qd1, qd2, qd3]
    z_des : ndarray (6,)
        Desired state vector [q1_des, q2_des, q3_des, qd1_des, qd2_des, qd3_des]
    model_params : dict
        Dictionary containing model parameters (g, m, L, I)
    wn : float, default=2*pi (1 Hz)
        Desired natural frequency for PD control [rad/s]
    zeta : float, default=0.7
        Desired damping ratio for PD control
    gravity_compensation : bool, default=True
        If True, includes gravity compensation
    full_decoupling : bool, default=False
        If True, includes full nonlinear decoupling (Coriolis/centrifugal)
        
    Returns
    -------
    zdot : ndarray (6,)
        State derivative [qd1, qd2, qd3, qdd1, qdd2, qdd3]
    """
    z = np.asarray(z)
    z_des = np.asarray(z_des)
    
    # Extract current positions and velocities
    q = z[0:3]
    qd = z[3:6]
    
    # Extract desired positions and velocities
    q_des = z_des[0:3]
    qd_des = z_des[3:6]
    
    # Extract model parameters
    g = model_params['g']
    m = model_params['m']
    L = model_params['L']
    I = model_params['I']
    
    # Compute dynamics matrices
    D = compute_D(q, L, m, g, I)
    B = compute_B(q, L, m, g, I)
    C = compute_C(q, L, m, g, I)
    G = compute_G(q, L, m, g, I)
    
    # Compute velocity product terms
    qdqd = np.array([qd[0]*qd[1], qd[0]*qd[2], qd[1]*qd[2]])
    qd2 = np.array([qd[0]**2, qd[1]**2, qd[2]**2])
    
    # Combined Coriolis and centrifugal vector
    V = B @ qdqd + C @ qd2
    
    # Compute PD gains
    Kp, Kd = compute_pd_gains(wn, zeta)
    
    # Decoupled system PD controller
    tau_prime = -Kd * (qd - qd_des) - Kp * (q - q_des)
    
    # Calculate total control torques
    tau = D @ tau_prime
    
    if gravity_compensation:
        tau = tau + G
    
    if full_decoupling:
        tau = tau + V
    
    # Compute joint acceleration from dynamics equation
    # D * qdd = tau - V - G
    qdd = np.linalg.solve(D, tau - V - G)
    
    # Assemble state derivative
    zdot = np.concatenate([qd, qdd])
    
    return zdot

