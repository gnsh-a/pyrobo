"""
Open-loop dynamics simulation for the 3-DOF RRR manipulator.

Computes state derivatives for numerical integration without control.
"""

import numpy as np
from ..dynamics import compute_D, compute_B, compute_C, compute_G


def zdot_3dof(z, model_params):
    """
    Compute state derivative for open-loop 3-DOF manipulator dynamics.
    
    State vector z = [q1, q2, q3, qd1, qd2, qd3] where:
        q = [q1, q2, q3] are joint positions
        qd = [qd1, qd2, qd3] are joint velocities
    
    The dynamics equation is:
        D(q) * qdd + B(q) * [qd1*qd2, qd1*qd3, qd2*qd3]^T 
                   + C(q) * [qd1^2, qd2^2, qd3^2]^T + G(q) = tau
    
    For open-loop simulation with no applied torques (tau = 0).
    
    Parameters
    ----------
    z : ndarray (6,)
        State vector [q1, q2, q3, qd1, qd2, qd3]
    model_params : dict
        Dictionary containing:
        - 'g': gravitational constant [m/s^2]
        - 'm': link mass [kg]
        - 'L': link length [m]
        - 'I': link inertia [kg*m^2]
        
    Returns
    -------
    zdot : ndarray (6,)
        State derivative [qd1, qd2, qd3, qdd1, qdd2, qdd3]
    """
    z = np.asarray(z)
    
    # Extract positions and velocities from state
    q = z[0:3]
    qd = z[3:6]
    
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
    
    # Viscous friction in joints (optional, set to zero by default)
    tau_friction = 0.0 * qd
    
    # Compute velocity product terms
    qdqd = np.array([qd[0]*qd[1], qd[0]*qd[2], qd[1]*qd[2]])
    qd2 = np.array([qd[0]**2, qd[1]**2, qd[2]**2])
    
    # Compute acceleration from dynamics equation
    # D * qdd = -B*qdqd - C*qd2 - G - tau_friction
    # For open-loop, no control torques (tau = 0)
    qdd = np.linalg.solve(D, -B @ qdqd - C @ qd2 - G - tau_friction)
    
    # Assemble state derivative
    zdot = np.concatenate([qd, qdd])
    
    return zdot

