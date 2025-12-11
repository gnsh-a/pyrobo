"""
Forward kinematics for the 3-DOF RRR manipulator.

This module implements the forward kinematics based on the DH convention.
"""

import numpy as np


def forward_kinematics(q, L):
    """
    Compute forward kinematics for the 3-DOF RRR manipulator.
    
    Returns homogeneous transformation matrices from each link frame to the base frame.
    
    Parameters
    ----------
    q : array_like (3,)
        Joint angles [q1, q2, q3] in radians
    L : float
        Link length parameter (all links have same length)
        
    Returns
    -------
    T10 : ndarray (4, 4)
        Homogeneous transform from frame {1} to frame {0} (base)
    T20 : ndarray (4, 4)
        Homogeneous transform from frame {2} to frame {0} (base)
    T30 : ndarray (4, 4)
        Homogeneous transform from frame {3} to frame {0} (base)
        
    Notes
    -----
    Frame definitions follow the convention required for basic Jacobian evaluation.
    Directly ported from FwdKin_G.m
    """
    q = np.asarray(q)
    q1, q2, q3 = q[0], q[1], q[2]
    
    # T1_0 from frame 1 to base
    c1 = np.cos(q1)
    s1 = np.sin(q1)
    T10 = np.array([[0,  -s1,  c1,  0],
                    [0,   c1,  s1,  0],
                    [-1,  0,   0,   L],
                    [0,   0,   0,   1]])
    
    # T2_1 from frame 2 to frame 1
    c2 = np.cos(q2)
    s2 = np.sin(q2)
    T21 = np.array([[0,  -c2,   s2,  0],
                    [0,  -s2,  -c2,  0],
                    [1,   0,    0,   L],
                    [0,   0,    0,   1]])
    
    # T3_2 from frame 3 to frame 2
    c3 = np.cos(q3)
    s3 = np.sin(q3)
    T32 = np.array([[s3,   0,   c3,  L*c3],
                    [-c3,  0,   s3,  L*s3],
                    [0,   -1,   0,   0],
                    [0,    0,   0,   1]])
    
    # Compute compound transformations
    T20 = T10 @ T21
    T30 = T20 @ T32
    
    return T10, T20, T30


def forward_kinematics_com(q, L):
    """
    Compute forward kinematics to the center of mass of each link.
    
    Parameters
    ----------
    q : array_like (3,)
        Joint angles [q1, q2, q3] in radians
    L : float
        Link length parameter
        
    Returns
    -------
    Tc10 : ndarray (4, 4)
        Transform from COM of link 1 to base frame
    Tc20 : ndarray (4, 4)
        Transform from COM of link 2 to base frame
    Tc30 : ndarray (4, 4)
        Transform from COM of link 3 to base frame
        
    Notes
    -----
    Assumes center of mass is at L/2 for each link.
    """
    q = np.asarray(q)
    q1, q2, q3 = q[0], q[1], q[2]
    
    # Link 1 COM frame
    c1 = np.cos(q1)
    s1 = np.sin(q1)
    Tc10 = np.array([[0,  -c1,   s1,  0],
                     [s1,  -s1,  -c1,  0],
                     [1,    0,    0,   L/2],
                     [0,    0,    0,   1]])
    
    # Frame {1} to base
    T10 = np.array([[0,  -s1,  c1,  0],
                    [0,   c1,  s1,  0],
                    [-1,  0,   0,   L],
                    [0,   0,   0,   1]])
    
    # Link 2 COM in frame {1}
    c2 = np.cos(q2)
    s2 = np.sin(q2)
    Tc21 = np.array([[0,  -c2,   s2,  0],
                     [0,  -s2,  -c2,  0],
                     [1,   0,    0,   L/2],
                     [0,   0,    0,   1]])
    
    # Link 2 frame {2} to frame {1}
    T21 = np.array([[0,  -c2,   s2,  0],
                    [0,  -s2,  -c2,  0],
                    [1,   0,    0,   L],
                    [0,   0,    0,   1]])
    
    # Link 3 COM in frame {2}
    c3 = np.cos(q3)
    s3 = np.sin(q3)
    Tc32 = np.array([[c3,  -s3,  0,  L*c3/2],
                     [s3,   c3,  0,  L*s3/2],
                     [0,    0,   1,  0],
                     [0,    0,   0,  1]])
    
    # Compute compound transformations
    T20 = T10 @ T21
    Tc20 = T10 @ Tc21
    Tc30 = T20 @ Tc32
    
    return Tc10, Tc20, Tc30

