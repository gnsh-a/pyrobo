"""
Jacobian computation for the 3-DOF RRR manipulator.

Computes linear and angular velocity Jacobians for each link's center of mass.
"""

import numpy as np
from .forward_kinematics import forward_kinematics, forward_kinematics_com


def compute_jacobians(q, L):
    """
    Compute linear and angular velocity Jacobians for all links.
    
    Parameters
    ----------
    q : array_like (3,)
        Joint angles [q1, q2, q3] in radians
    L : float
        Link length parameter
        
    Returns
    -------
    Jvc1 : ndarray (3, 3)
        Linear velocity Jacobian for center of mass of link 1
    Jvc2 : ndarray (3, 3)
        Linear velocity Jacobian for center of mass of link 2
    Jvc3 : ndarray (3, 3)
        Linear velocity Jacobian for center of mass of link 3
    Jw1 : ndarray (3, 3)
        Angular velocity Jacobian for link 1
    Jw2 : ndarray (3, 3)
        Angular velocity Jacobian for link 2
    Jw3 : ndarray (3, 3)
        Angular velocity Jacobian for link 3
    """
    q = np.asarray(q)
    
    # Get transformation matrices
    T10, T20, T30 = forward_kinematics(q, L)
    Tc10, Tc20, Tc30 = forward_kinematics_com(q, L)
    
    # Define axes of rotation in base frame coordinates
    z0 = np.array([0, 0, 1])
    z1 = T10[:3, 2]  # 3rd column of rotation part
    z2 = T20[:3, 2]
    
    # Define origins of frames in base frame coordinates
    o0 = np.array([0, 0, 0])
    o1 = T10[:3, 3]  # Position part
    o2 = T20[:3, 3]
    
    # Define positions of COM for each link in base frame
    oc1 = Tc10[:3, 3]
    oc2 = Tc20[:3, 3]
    oc3 = Tc30[:3, 3]
    
    # Linear velocity Jacobians (for COMs)
    # Jvc1: only joint 1 affects link 1's COM
    Jvc1 = np.column_stack([
        np.cross(z0, oc1 - o0),
        np.zeros(3),
        np.zeros(3)
    ])
    
    # Jvc2: joints 1 and 2 affect link 2's COM
    Jvc2 = np.column_stack([
        np.cross(z0, oc2 - o0),
        np.cross(z1, oc2 - o1),
        np.zeros(3)
    ])
    
    # Jvc3: all joints affect link 3's COM
    Jvc3 = np.column_stack([
        np.cross(z0, oc3 - o0),
        np.cross(z1, oc3 - o1),
        np.cross(z2, oc3 - o2)
    ])
    
    # Angular velocity Jacobians
    # Jw1: only joint 1 contributes to link 1's angular velocity
    Jw1 = np.column_stack([z0, np.zeros(3), np.zeros(3)])
    
    # Jw2: joints 1 and 2 contribute to link 2's angular velocity
    Jw2 = np.column_stack([z0, z1, np.zeros(3)])
    
    # Jw3: all joints contribute to link 3's angular velocity
    Jw3 = np.column_stack([z0, z1, z2])
    
    return Jvc1, Jvc2, Jvc3, Jw1, Jw2, Jw3

