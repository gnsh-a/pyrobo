"""
Transformation utilities for homogeneous transforms and rotation matrices.
"""

import numpy as np


def rotation_matrix(axis, angle):
    """
    Create a 3x3 rotation matrix for rotation about a principal axis.
    
    Parameters
    ----------
    axis : str
        Axis of rotation ('x', 'y', or 'z')
    angle : float
        Angle of rotation in radians
        
    Returns
    -------
    R : ndarray (3, 3)
        Rotation matrix
    """
    c = np.cos(angle)
    s = np.sin(angle)
    
    if axis == 'x':
        R = np.array([[1, 0, 0],
                      [0, c, -s],
                      [0, s, c]])
    elif axis == 'y':
        R = np.array([[c, 0, s],
                      [0, 1, 0],
                      [-s, 0, c]])
    elif axis == 'z':
        R = np.array([[c, -s, 0],
                      [s, c, 0],
                      [0, 0, 1]])
    else:
        raise ValueError(f"Invalid axis: {axis}. Must be 'x', 'y', or 'z'")
    
    return R


def homogeneous_transform(R=None, p=None):
    """
    Create a 4x4 homogeneous transformation matrix.
    
    Parameters
    ----------
    R : ndarray (3, 3), optional
        Rotation matrix. If None, uses identity rotation.
    p : ndarray (3,), optional
        Position vector. If None, uses zero vector.
        
    Returns
    -------
    T : ndarray (4, 4)
        Homogeneous transformation matrix
    """
    T = np.eye(4)
    
    if R is not None:
        T[:3, :3] = R
    
    if p is not None:
        T[:3, 3] = p
    
    return T


def extract_rotation(T):
    """
    Extract rotation matrix from homogeneous transform.
    
    Parameters
    ----------
    T : ndarray (4, 4)
        Homogeneous transformation matrix
        
    Returns
    -------
    R : ndarray (3, 3)
        Rotation matrix
    """
    return T[:3, :3]


def extract_position(T):
    """
    Extract position vector from homogeneous transform.
    
    Parameters
    ----------
    T : ndarray (4, 4)
        Homogeneous transformation matrix
        
    Returns
    -------
    p : ndarray (3,)
        Position vector
    """
    return T[:3, 3]

