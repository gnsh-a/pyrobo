"""
Symbolic derivation of robot dynamics using SymPy.

This module derives the dynamics equations for the 3-DOF RRR manipulator
from first principles using Lagrangian mechanics
"""

import sympy as sp
from sympy import symbols, cos, sin, Matrix, simplify, diff


def derive_forward_kinematics():
    """
    Symbolically derive forward kinematics transformation matrices.
    
    Returns
    -------
    T10, T20, T30 : Matrix
        Transformation matrices to each link frame
    Tc10, Tc20, Tc30 : Matrix
        Transformation matrices to each link's center of mass
    q : list
        Symbolic joint angle variables [q1, q2, q3]
    L, m, g, I : Symbol
        Symbolic parameters
    """
    # Define symbolic variables
    q1, q2, q3 = symbols('q1 q2 q3', real=True)
    L, m, g, I = symbols('L m g I', positive=True, real=True)
    
    q = [q1, q2, q3]
    
    print("Computing forward kinematics symbolically...")
    
    # Link 1: Frame {1} to base {0}
    c1 = cos(q1)
    s1 = sin(q1)
    T10 = Matrix([
        [0,  -s1,  c1,  0],
        [0,   c1,  s1,  0],
        [-1,  0,   0,   L],
        [0,   0,   0,   1]
    ])
    
    # Link 1 COM: Frame {c1} to base {0}
    Tc10 = Matrix([
        [0,  -c1,   s1,  0],
        [s1, -s1,  -c1,  0],
        [1,   0,    0,   L/2],
        [0,   0,    0,   1]
    ])
    
    # Link 2: Frame {2} to frame {1}
    c2 = cos(q2)
    s2 = sin(q2)
    T21 = Matrix([
        [0,  -c2,   s2,  0],
        [0,  -s2,  -c2,  0],
        [1,   0,    0,   L],
        [0,   0,    0,   1]
    ])
    
    # Link 2 COM: Frame {c2} to frame {1}
    Tc21 = Matrix([
        [0,  -c2,   s2,  0],
        [0,  -s2,  -c2,  0],
        [1,   0,    0,   L/2],
        [0,   0,    0,   1]
    ])
    
    # Link 3: Frame {3} to frame {2}
    c3 = cos(q3)
    s3 = sin(q3)
    T32 = Matrix([
        [s3,   0,   c3,  L*c3],
        [-c3,  0,   s3,  L*s3],
        [0,   -1,   0,   0],
        [0,    0,   0,   1]
    ])
    
    # Link 3 COM: Frame {c3} to frame {2}
    Tc32 = Matrix([
        [c3,  -s3,  0,  L*c3/2],
        [s3,   c3,  0,  L*s3/2],
        [0,    0,   1,  0],
        [0,    0,   0,  1]
    ])
    
    # Compound transformations
    print("  Computing compound transformations...")
    T20 = simplify(T10 * T21)
    T30 = simplify(T20 * T32)
    Tc20 = simplify(T10 * Tc21)
    Tc30 = simplify(T20 * Tc32)
    
    return T10, T20, T30, Tc10, Tc20, Tc30, q, L, m, g, I


def derive_jacobians(T10, T20, T30, Tc10, Tc20, Tc30):
    """
    Symbolically derive Jacobian matrices for each link.
    
    Parameters
    ----------
    T10, T20, T30 : Matrix
        Transformation matrices to link frames
    Tc10, Tc20, Tc30 : Matrix
        Transformation matrices to COM frames
        
    Returns
    -------
    Jvc1, Jvc2, Jvc3 : Matrix
        Linear velocity Jacobians for each link's COM
    Jw1, Jw2, Jw3 : Matrix
        Angular velocity Jacobians for each link
    """
    print("Computing Jacobians symbolically...")
    
    # Define axes of rotation in base frame
    z0 = Matrix([0, 0, 1])
    z1 = T10[0:3, 2]  # 3rd column (axis of rotation)
    z2 = T20[0:3, 2]
    
    # Define origins of frames in base frame
    o0 = Matrix([0, 0, 0])
    o1 = T10[0:3, 3]  # 4th column (position)
    o2 = T20[0:3, 3]
    
    # Define positions of COM for each link
    oc1 = Tc10[0:3, 3]
    oc2 = Tc20[0:3, 3]
    oc3 = Tc30[0:3, 3]
    
    # Linear velocity Jacobians (for COMs)
    print("  Computing Jvc1...")
    Jvc1 = Matrix.hstack(
        z0.cross(oc1 - o0),
        Matrix([0, 0, 0]),
        Matrix([0, 0, 0])
    )
    Jvc1 = simplify(Jvc1)
    
    print("  Computing Jvc2...")
    Jvc2 = Matrix.hstack(
        z0.cross(oc2 - o0),
        z1.cross(oc2 - o1),
        Matrix([0, 0, 0])
    )
    Jvc2 = simplify(Jvc2)
    
    print("  Computing Jvc3...")
    Jvc3 = Matrix.hstack(
        z0.cross(oc3 - o0),
        z1.cross(oc3 - o1),
        z2.cross(oc3 - o2)
    )
    Jvc3 = simplify(Jvc3)
    
    # Angular velocity Jacobians
    print("  Computing Jw1, Jw2, Jw3...")
    Jw1 = Matrix.hstack(z0, Matrix([0, 0, 0]), Matrix([0, 0, 0]))
    Jw2 = Matrix.hstack(z0, z1, Matrix([0, 0, 0]))
    Jw3 = Matrix.hstack(z0, z1, z2)
    
    return Jvc1, Jvc2, Jvc3, Jw1, Jw2, Jw3


def derive_mass_matrix(Jvc1, Jvc2, Jvc3, Jw1, Jw2, Jw3, 
                       Tc10, Tc20, Tc30, m, I, q):
    """
    Symbolically derive the mass matrix D(q) using Lagrangian mechanics.
    
    Parameters
    ----------
    Jvc1, Jvc2, Jvc3 : Matrix
        Linear velocity Jacobians
    Jw1, Jw2, Jw3 : Matrix
        Angular velocity Jacobians
    Tc10, Tc20, Tc30 : Matrix
        Transformation matrices to COM frames
    m : Symbol
        Mass
    I : Symbol
        Inertia
    q : list
        Joint variables [q1, q2, q3]
        
    Returns
    -------
    D : Matrix (3x3)
        Mass matrix
    """
    print("Computing mass matrix D symbolically...")
    
    # Define inertia tensor (simplified)
    Ixx = I/2
    Iyy = I
    Izz = I
    Ic = Matrix([
        [Ixx, 0, 0],
        [0, Iyy, 0],
        [0, 0, Izz]
    ])
    
    # Extract rotation matrices from transformation matrices
    R10 = Tc10[0:3, 0:3]
    R20 = Tc20[0:3, 0:3]
    R30 = Tc30[0:3, 0:3]
    
    # Translational kinetic energy contributions
    print("  Computing Dv1...")
    Dv1 = m * Jvc1.T * Jvc1
    Dv1 = simplify(Dv1)
    
    print("  Computing Dv2...")
    Dv2 = m * Jvc2.T * Jvc2
    Dv2 = simplify(Dv2)
    
    print("  Computing Dv3...")
    Dv3 = m * Jvc3.T * Jvc3
    Dv3 = simplify(Dv3)
    
    # Rotational kinetic energy contributions
    print("  Computing Dw1...")
    Dw1 = Jw1.T * R10 * Ic * R10.T * Jw1
    Dw1 = simplify(Dw1)
    
    print("  Computing Dw2...")
    Dw2 = Jw2.T * R20 * Ic * R20.T * Jw2
    Dw2 = simplify(Dw2)
    
    print("  Computing Dw3...")
    Dw3 = Jw3.T * R30 * Ic * R30.T * Jw3
    Dw3 = simplify(Dw3)
    
    # Total mass matrix
    print("  Assembling total mass matrix...")
    D = Dv1 + Dv2 + Dv3 + Dw1 + Dw2 + Dw3
    D = simplify(D)
    
    # Additional simplification (like the symbolic derivation process does)
    print("  Applying trigonometric simplifications...")
    D = D.subs({cos(q[1])**2 - 1: -sin(q[1])**2})
    D = D.subs({cos(q[1] + q[2])**2 - 1: -sin(q[1] + q[2])**2})
    D = simplify(D)
    
    return D


def derive_coriolis_centrifugal(D, q):
    """
    Symbolically derive Coriolis (B) and centrifugal (C) matrices.
    
    Uses Christoffel symbols of the first kind:
        b(i,j,k) = 0.5 * (dD[i,j]/dq[k] + dD[i,k]/dq[j] - dD[j,k]/dq[i])
    
    Parameters
    ----------
    D : Matrix (3x3)
        Mass matrix
    q : list
        Joint variables [q1, q2, q3]
        
    Returns
    -------
    B : Matrix (3x3)
        Coriolis matrix (multiplies velocity products)
    C : Matrix (3x3)
        Centrifugal matrix (multiplies squared velocities)
    """
    print("Computing Coriolis and centrifugal terms symbolically...")
    
    # Compute partial derivatives of D
    print("  Computing partial derivatives of D...")
    d = sp.MutableDenseNDimArray.zeros(3, 3, 3)
    for i in range(3):
        for j in range(3):
            for k in range(3):
                d[i, j, k] = diff(D[i, j], q[k])
    
    # Simplify partial derivatives
    print("  Simplifying partial derivatives...")
    for i in range(3):
        for j in range(3):
            for k in range(3):
                d[i, j, k] = simplify(d[i, j, k])
    
    # Compute Christoffel symbols
    print("  Computing Christoffel symbols...")
    b = sp.MutableDenseNDimArray.zeros(3, 3, 3)
    for i in range(3):
        for j in range(3):
            for k in range(3):
                b[i, j, k] = sp.Rational(1, 2) * (d[i, j, k] + d[i, k, j] - d[j, k, i])
                b[i, j, k] = simplify(b[i, j, k])
    
    # Assemble B matrix (Coriolis - velocity products)
    # B multiplies [q1_dot*q2_dot, q1_dot*q3_dot, q2_dot*q3_dot]^T
    print("  Assembling B matrix...")
    B = Matrix([
        [2*b[0, 0, 1],  2*b[0, 0, 2],  2*b[0, 1, 2]],
        [2*b[1, 0, 1],  2*b[1, 0, 2],  2*b[1, 1, 2]],
        [2*b[2, 0, 1],  2*b[2, 0, 2],  2*b[2, 1, 2]]
    ])
    B = simplify(B)
    
    # Assemble C matrix (Centrifugal - squared velocities)
    # C multiplies [q1_dot^2, q2_dot^2, q3_dot^2]^T
    print("  Assembling C matrix...")
    C = Matrix([
        [b[0, 0, 0],  b[0, 1, 1],  b[0, 2, 2]],
        [b[1, 0, 0],  b[1, 1, 1],  b[1, 2, 2]],
        [b[2, 0, 0],  b[2, 1, 1],  b[2, 2, 2]]
    ])
    C = simplify(C)
    
    return B, C


def derive_gravity_vector(Jvc1, Jvc2, Jvc3, m, g):
    """
    Symbolically derive the gravity vector G(q).
    
    Parameters
    ----------
    Jvc1, Jvc2, Jvc3 : Matrix
        Linear velocity Jacobians for each link's COM
    m : Symbol
        Mass
    g : Symbol
        Gravitational acceleration
        
    Returns
    -------
    G : Matrix (3x1)
        Gravity vector
    """
    print("Computing gravity vector G symbolically...")
    
    # Gravity vector in base frame (pointing down in z-direction)
    g_vec = Matrix([0, 0, -g])
    
    # G = -sum(Jvc_i^T * m_i * g_vector)
    G1 = -(Jvc1.T * (m * g_vec))
    G2 = -(Jvc2.T * (m * g_vec))
    G3 = -(Jvc3.T * (m * g_vec))
    
    G = G1 + G2 + G3
    G = simplify(G)
    
    return G


def derive_all_dynamics():
    """
    Derive all dynamics matrices from first principles.
    
    Returns
    -------
    dict
        Dictionary containing:
        - 'D': Mass matrix
        - 'B': Coriolis matrix
        - 'C': Centrifugal matrix
        - 'G': Gravity vector
        - 'q': Joint variables [q1, q2, q3]
        - 'params': Parameters [L, m, g, I]
    """
    print("="*60)
    print("SYMBOLIC DERIVATION OF 3-DOF RRR MANIPULATOR DYNAMICS")
    print("="*60)
    print()
    
    # Step 1: Forward kinematics
    T10, T20, T30, Tc10, Tc20, Tc30, q, L, m, g, I = derive_forward_kinematics()
    print()
    
    # Step 2: Jacobians
    Jvc1, Jvc2, Jvc3, Jw1, Jw2, Jw3 = derive_jacobians(
        T10, T20, T30, Tc10, Tc20, Tc30
    )
    print()
    
    # Step 3: Mass matrix
    D = derive_mass_matrix(Jvc1, Jvc2, Jvc3, Jw1, Jw2, Jw3, 
                           Tc10, Tc20, Tc30, m, I, q)
    print()
    
    # Step 4: Coriolis and centrifugal matrices
    B, C = derive_coriolis_centrifugal(D, q)
    print()
    
    # Step 5: Gravity vector
    G = derive_gravity_vector(Jvc1, Jvc2, Jvc3, m, g)
    print()
    
    print("="*60)
    print("DERIVATION COMPLETE")
    print("="*60)
    
    return {
        'D': D,
        'B': B,
        'C': C,
        'G': G,
        'q': q,
        'params': [L, m, g, I]
    }

