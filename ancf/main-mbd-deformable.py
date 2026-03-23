import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import ArtistAnimation
import os
import sys

# ============================================================
# INPUTS
# ============================================================
W = H = 0.003      # cross-section (m)
rho = 7700.0       # density (kg/m^3)

# Material parameters (steel)
E, nu = 2.0e11, 0.3
lam = E * nu / ((1 + nu) * (1 - 2 * nu))
mu  = E / (2 * (1 + nu))

# Debug flag for iteration print statements
DEBUG = False

# Multi-element parameters (will be updated by update_connectivity)
n_beam = None
N_coef = None
n_dofs = None
offset_start = None
offset_end = None

# Global storage for element properties (computed after mesh is defined)
B_INV = None
GAUSS_16 = None
L = None  # Element length (will be set to L_elem)


# ============================================================
# HELPER FUNCTIONS FOR B3-24
# ============================================================
# Gauss–Legendre nodes and weights for n = 2, 3, 4, and 5
GL_TABLE = {
    2: {
        "x": np.array([-0.5773502691896257,  0.5773502691896257]),
        "w": np.array([ 1.0000000000000000,  1.0000000000000000]),
    },
    3: {
        "x": np.array([-0.7745966692414834,  0.0,  0.7745966692414834]),
        "w": np.array([ 0.5555555555555556,  0.8888888888888888,  0.5555555555555556]),
    },
    4: {
        "x": np.array([-0.3399810435848563,  0.3399810435848563,
                        -0.8611363115940526,  0.8611363115940526]),
        "w": np.array([ 0.6521451548625461,  0.6521451548625461,
                         0.3478548451374538,  0.3478548451374538]),
    },
    5: {
        "x": np.array([-0.9061798459386640, -0.5384693101056831,  0.0,
                        0.5384693101056831,  0.9061798459386640]),
        "w": np.array([ 0.2369268850561891,  0.4786286704993665,  0.5688888888888889,
                        0.4786286704993665,  0.2369268850561891]),
    },
}

def gauss_legendre(n):
    data = GL_TABLE[n]
    return data["x"], data["w"]

# ============================================================
# PROGRAMMATIC SHAPE FUNCTION GENERATION
# ============================================================

def eval_basis(u, v, w):
    return np.array([1.0, u, v, w, u*v, u*w, u**2, u**3])

def eval_basis_derivatives(u, v, w):
    ddu = np.array([0.0, 1.0, 0.0, 0.0, v, w, 2*u, 3*u**2])
    ddv = np.array([0.0, 0.0, 1.0, 0.0, u, 0.0, 0.0, 0.0])
    ddw = np.array([0.0, 0.0, 0.0, 1.0, 0.0, u, 0.0, 0.0])
    return np.array([ddu, ddv, ddw])  # (3, 8)

def build_B_matrix(L_elem):
    """Builds B matrix for a specific element length."""
    B = np.zeros((8, 8))
    
    u1, v1, w1 = -L_elem/2, 0.0, 0.0
    basis1 = eval_basis(u1, v1, w1)
    basis_derivs1 = eval_basis_derivatives(u1, v1, w1)
    B[0, :] = basis1
    B[1, :] = basis_derivs1[0, :]
    B[2, :] = basis_derivs1[1, :]
    B[3, :] = basis_derivs1[2, :]
    
    u2, v2, w2 = L_elem/2, 0.0, 0.0
    basis2 = eval_basis(u2, v2, w2)
    basis_derivs2 = eval_basis_derivatives(u2, v2, w2)
    B[4, :] = basis2
    B[5, :] = basis_derivs2[0, :]
    B[6, :] = basis_derivs2[1, :]
    B[7, :] = basis_derivs2[2, :]
    
    return B

def shape_functions(u, v, w, B_inv=None):
    if B_inv is None:
        B_inv = B_INV
    
    basis = eval_basis(u, v, w)
    return B_inv @ basis  # (8,)

# ============================================================
# ELEMENT SETUP & SHAPE FUNCTION GENERATION
# ============================================================

def initialize_element_properties(L_elem):
    """
    Computes B_INV and GAUSS_16 for a specific element length.
    Updates global variables so solver functions can use them.
    """
    global B_INV, GAUSS_16, L
    
    L = L_elem  # Update global L for functions that rely on it
    
    # 1. Compute B_INV
    B_mat = build_B_matrix(L_elem)
    B_INV = np.linalg.inv(B_mat.T)
    
    # 2. Compute Gauss Points
    J_det = (L_elem * W * H) / 8.0
    
    gp_u, w_u = gauss_legendre(4)
    gp_v, w_v = gauss_legendre(2)
    gp_w, w_w = gauss_legendre(2)
    
    weights_list = []
    S_list = []
    H_list = []
    
    for i_u, xi in enumerate(gp_u):
        u = (L_elem/2) * xi
        for i_v, eta in enumerate(gp_v):
            v = (W/2) * eta
            for i_w, zeta in enumerate(gp_w):
                w = (H/2) * zeta
                weights_list.append(w_u[i_u] * w_v[i_v] * w_w[i_w] * J_det)
                basis = eval_basis(u, v, w)
                basis_derivs = eval_basis_derivatives(u, v, w)
                S_list.append(B_INV @ basis)
                H_list.append(B_INV @ basis_derivs.T)
    
    GAUSS_16 = {
        'weights': np.array(weights_list),
        'S': np.array(S_list),
        'H': np.array(H_list)
    }
    
    print(f"Element Initialized: Length={L_elem:.4f}m, J_det={J_det:.2e}")

# ============================================================
# UTILITY FUNCTIONS: DOF CONVERSION
# ============================================================

def Nmat_from_dofs(x_n):
    x_n = x_n.flatten()  # Ensure 1D
    n_nodes = len(x_n) // 3
    Nmat = np.zeros((3, n_nodes))
    for i in range(n_nodes):
        Nmat[:, i] = x_n[i*3:(i+1)*3]
    return Nmat

def dofs_from_Nmat(Nmat):
    return Nmat.flatten('F')  # Column-major order

# ============================================================
# POSITION COMPUTATION
# ============================================================
def beam_point_position(Nmat, u, v, w):
    s = shape_functions(u, v, w)
    return Nmat @ s   # (3x8) @ (8,) -> (3,)

# ============================================================
# EXTERNAL FORCES
# ============================================================
def point_load_vector(Nmat, uP, vP, wP, fP):
    s = shape_functions(uP, vP, wP)  # shape fn at that point
    F_ext = np.outer(s, fP)             # (8x3)
    return F_ext


def F_tip(t):
    if 0.0 <= t <= 0.05:
        force_z = -1.0 + np.cos(20.0 * np.pi * t)
        return np.array([0.0, 0.0, force_z])
    else:
        return np.array([0.0, 0.0, 0.0])

# ============================================================
# COMPUTATION FUNCTIONS
# ============================================================

def compute_mass_matrix():
    """
    Compute global mass matrix for multi-element beam.
    Assembles element mass matrices into global (N_coef, N_coef) matrix,
    then expands to (n_dofs, n_dofs) with kron(I3).
    """
    S = GAUSS_16['S']  # (16, 8)
    weights = GAUSS_16['weights']  # (16,)
    
    # Global mass matrix (N_coef × N_coef)
    m_global = np.zeros((N_coef, N_coef))
    
    # Loop over elements
    for elem in range(n_beam):
        # Compute element mass matrix (8 × 8)
        m_elem = np.zeros((8, 8))
        for gp_idx in range(16):
            s = S[gp_idx, :]
            weight = weights[gp_idx]
            m_elem += rho * np.outer(s, s) * weight
        
        m_elem = 0.5 * (m_elem + m_elem.T)  # Symmetrize
        
        # Assemble into global matrix
        idx = np.arange(offset_start[elem], offset_end[elem] + 1)
        for i_local, i_global in enumerate(idx):
            for j_local, j_global in enumerate(idx):
                m_global[i_global, j_global] += m_elem[i_local, j_local]
    
    # Expand to full DOF space (x, y, z per node)
    I3 = np.eye(3)
    M_e = np.kron(m_global, I3)  # (n_dofs, n_dofs)
    return M_e


def compute_gravity_force(g_vec):
    """
    Compute gravity force vector for multi-element beam.
    """
    S = GAUSS_16['S']  # (16, 8)
    weights = GAUSS_16['weights']  # (16,)
    
    # Global gravity force (N_coef × 3)
    G_global = np.zeros((N_coef, 3))
    
    # Loop over elements
    for elem in range(n_beam):
        # Compute element contribution
        v_i = np.zeros(8)
        for gp_idx in range(16):
            s = S[gp_idx, :]
            weight = weights[gp_idx]
            v_i += s * weight
        
        G_elem = rho * np.outer(v_i, g_vec)  # (8, 3)
        
        # Assemble into global matrix
        idx = np.arange(offset_start[elem], offset_end[elem] + 1)
        for i_local, i_global in enumerate(idx):
            G_global[i_global, :] += G_elem[i_local, :]
    
    G_f = G_global.reshape(-1, 1)  # (n_dofs, 1)
    return G_f


def compute_internal_force(x_n):
    """
    Compute internal force vector for multi-element beam.
    
    Args:
        x_n: (n_dofs,) full DOF vector (all nodes' x, y, z coordinates)
    
    Returns:
        f_int: (n_dofs, 1) global internal force vector
    """
    H = GAUSS_16['H']  # (16, 8, 3)
    weights = GAUSS_16['weights']  # (16,)
    
    # Global internal force vector
    f_int_global = np.zeros((n_dofs, 1))
    
    # Loop over elements
    for elem in range(n_beam):
        # Extract local DOFs for this element
        idx = np.arange(offset_start[elem], offset_end[elem] + 1)
        x_local = np.zeros(24)  # 8 nodes × 3 DOFs
        for i_local, i_global in enumerate(idx):
            x_local[3*i_local:3*(i_local+1)] = x_n[3*i_global:3*(i_global+1)]
        
        # Form local Nmat (3 × 8)
        Nmat = Nmat_from_dofs(x_local)
        
        # Vectorized computation over all Gauss points
        F_all = np.einsum('ij,gjk->gik', Nmat, H)  # (16, 3, 3)
        
        I3 = np.eye(3)
        E_all = 0.5 * (np.einsum('gji,gjk->gik', F_all, F_all) - I3[None, :, :])  # (16, 3, 3)
        trace_E_all = np.trace(E_all, axis1=1, axis2=2)  # (16,)
        F_FT_F_all = np.einsum('gil,gjl,gjk->gik', F_all, F_all, F_all)  # (16, 3, 3)
        
        P_all = (lam * trace_E_all[:, None, None] * F_all + 
                 mu * (F_FT_F_all - F_all))  # (16, 3, 3)
        
        f_int_elem = np.einsum('gij,gkj,g->ik', H, P_all, weights)  # (8, 3)
        
        # Assemble into global force vector
        for i_local, i_global in enumerate(idx):
            f_int_global[3*i_global:3*(i_global+1), 0] += f_int_elem[i_local, :]
    
    return f_int_global


# ============================================================
# MESH AND STATE INITIALIZATION
# ============================================================

def update_connectivity(n_elements):
    """Update global connectivity arrays based on number of elements."""
    global offset_start, offset_end, N_coef, n_dofs, n_beam
    n_beam = n_elements
    N_coef = 8 + 4 * (n_beam - 1)
    n_dofs = 3 * N_coef
    offset_start = np.array([i * 4 for i in range(n_beam)], dtype=int)
    offset_end = np.array([i * 4 + 7 for i in range(n_beam)], dtype=int)

def initialize_state_from_Nmat(Nmat_initial):
    """
    Create multi-element initial configuration.
    Element 0 uses Nmat_initial, subsequent elements are shifted in +x direction.
    """
    x0 = np.zeros(n_dofs)
    
    # Element 0: use the provided Nmat_initial
    x0_elem0 = dofs_from_Nmat(Nmat_initial)
    idx0 = np.arange(offset_start[0], offset_end[0] + 1)
    for i_local, i_global in enumerate(idx0):
        x0[3*i_global:3*(i_global+1)] = x0_elem0[3*i_local:3*(i_local+1)]
    
    # Elements 1 to n_beam-1: shift x-coordinate by L for each element
    for elem in range(1, n_beam):
        # Start with the last 4 nodes of previous element (shared nodes)
        prev_idx = np.arange(offset_start[elem-1], offset_end[elem-1] + 1)
        curr_idx = np.arange(offset_start[elem], offset_end[elem] + 1)
        
        # Copy shared nodes (last 4 of previous element = first 4 of current element)
        for i in range(4):
            prev_global = prev_idx[4 + i]  # Last 4 nodes of previous element
            curr_global = curr_idx[i]      # First 4 nodes of current element
            x0[3*curr_global:3*(curr_global+1)] = x0[3*prev_global:3*(prev_global+1)]
        
        # For the new nodes (last 4 of current element), copy from last 4 of previous
        # but shift x-coordinate of the first new node (local index 4) by L
        for i_local in range(4, 8):
            prev_local = i_local  # Same local index in previous element
            prev_global = prev_idx[prev_local]
            curr_global = curr_idx[i_local]
            if i_local == 4:
                # First new node: shift x by L
                x0[3*curr_global + 0] = x0[3*prev_global + 0] + L  # x
                x0[3*curr_global + 1] = x0[3*prev_global + 1]     # y
                x0[3*curr_global + 2] = x0[3*prev_global + 2]     # z
            else:
                # Other new nodes: copy as-is
                x0[3*curr_global:3*(curr_global+1)] = x0[3*prev_global:3*(prev_global+1)]
    
    v0 = np.zeros(n_dofs)
    return x0, v0


# ============================================================
# CONSTRAINT FUNCTIONS: FIX NODE 1 POSITION AND GRADIENTS
# ============================================================

def get_node1_fixed_state():
    """
    Return the fixed state for node 1 (initial position and gradients).
    
    Returns:
        target: (12,) array with [position, ∂/∂u, ∂/∂v, ∂/∂w] at Node 1
    """
    return np.array([
        0.0, 0.0, 0.0,  # Position (global position, node at local u=-L/2)
        1.0, 0.0, 0.0,  # Slope u (∂/∂u)
        0.0, 1.0, 0.0,  # Slope v (∂/∂v)
        0.0, 0.0, 1.0   # Slope w (∂/∂w)
    ])

def compute_constraint(x_n):
    """Compute constraint: c = x_node1 - x_fixed = 0 (clamped cantilever)."""
    x_current_root = x_n[0:12]
    target = get_node1_fixed_state()
    c = x_current_root - target
    return c.reshape(-1, 1)  # (12, 1)

def compute_constraint_derivative():
    """Constraint derivative matrix: c_e[0:12, 0:12] = I_12, rest zeros."""
    c_e = np.zeros((12, n_dofs))
    c_e[0:12, 0:12] = np.eye(12)
    return c_e

# ============================================================
# BDF-1 INTEGRATOR: RESIDUAL AND JACOBIAN
# ============================================================

def compute_residual(a_n, lambda_n, x_prev, v_prev, t, h, M_e, G_f):
    """
    Compute augmented residual with constraints.
    
    Args:
        a_n: (n_dofs, 1) acceleration vector
        lambda_n: (n_constraints, 1) Lagrange multiplier vector
        x_prev: (n_dofs,) previous position
        v_prev: (n_dofs,) previous velocity
        t: current time
        h: time step
        M_e: (n_dofs, n_dofs) mass matrix
        G_f: (n_dofs,) gravity force vector
    
    Returns:
        R: (n_dofs + n_constraints, 1) augmented residual [R_dynamics; R_constraint]
    """
    # Ensure column vectors
    a_n = np.asarray(a_n).reshape(-1, 1)
    lambda_n = np.asarray(lambda_n).reshape(-1, 1)
    x_prev = np.asarray(x_prev).flatten()
    v_prev = np.asarray(v_prev).flatten()
    G_f = np.asarray(G_f).reshape(-1, 1)
    
    # BDF-1: compute current position from acceleration
    x_n = x_prev + h * v_prev + h**2 * a_n.flatten()
    
    f_int = compute_internal_force(x_n)
    
    # External force applied to tip of last element
    elem_tip = n_beam - 1
    idx_tip = np.arange(offset_start[elem_tip], offset_end[elem_tip] + 1)
    x_tip_local = np.zeros(24)
    for i_local, i_global in enumerate(idx_tip):
        x_tip_local[3*i_local:3*(i_local+1)] = x_n[3*i_global:3*(i_global+1)]
    Nmat_tip = Nmat_from_dofs(x_tip_local)
    F_ext = point_load_vector(Nmat_tip, L/2, 0.0, 0.0, F_tip(t))  # Returns (8, 3)
    # Map to global DOFs
    F_ext_global = np.zeros((n_dofs, 1))
    for i_local, i_global in enumerate(idx_tip):
        F_ext_global[3*i_global:3*(i_global+1), 0] = F_ext[i_local, :]  # F_ext[i_local, :] is (3,)
    
    c_e = compute_constraint_derivative()
    n_constraints = c_e.shape[0]
    
    R_dynamics = M_e @ a_n + c_e.T @ lambda_n + f_int - G_f - F_ext_global
    R_constraint = compute_constraint(x_n) / h**2
    R = np.vstack([R_dynamics, R_constraint])
    
    return R

def compute_jacobian(x_n, M_e, h, n_constraints):
    """Compute augmented Jacobian matrix J = [M + h²K, c_e^T; c_e, 0]."""
    K_tangent = compute_tangent_stiffness_numerical(x_n)
    c_e = compute_constraint_derivative()
    
    size_J = n_dofs + n_constraints
    J_dynamics = M_e + h**2 * K_tangent
    
    J = np.zeros((size_J, size_J))
    J[0:n_dofs, 0:n_dofs] = J_dynamics
    J[0:n_dofs, n_dofs:size_J] = c_e.T
    J[n_dofs:size_J, 0:n_dofs] = c_e
    J[n_dofs:size_J, n_dofs:size_J] = np.zeros((n_constraints, n_constraints))
    
    return J

# ============================================================
# NUMERICAL TANGENT STIFFNESS (FINITE DIFFERENCE)
# ============================================================

def compute_tangent_stiffness_numerical(x_n, eps=1e-6):
    """
    Compute tangent stiffness matrix K_tangent = df_int/de using finite differences.
    
    Formula: K_tangent[i, j] ≈ (f_int(x + eps*e_j)[i] - f_int(x)[i]) / eps
    
    Args:
        x_n: (n_dofs,) DOF vector
        eps: perturbation size for finite differences
    
    Returns:
        K_tangent: (n_dofs, n_dofs) stiffness matrix
    """
    f_int_base = compute_internal_force(x_n).flatten()
    
    K_tangent = np.zeros((n_dofs, n_dofs))
    for j in range(n_dofs):
        x_perturbed = x_n.copy()
        x_perturbed[j] += eps
        f_int_perturbed = compute_internal_force(x_perturbed).flatten()
        K_tangent[:, j] = (f_int_perturbed - f_int_base) / eps
    
    return K_tangent


# ============================================================
# BDF-1 INTEGRATOR: NEWTON-RAPHSON SOLVER
# ============================================================

def solve_acceleration_newton(x_prev, v_prev, t, h, M_e, G_f, tol=1e-6, max_iter=10, a_init=None, lambda_init=None):
    """
    Solve for acceleration and Lagrange multipliers using Newton-Raphson.
    
    Args:
        x_prev: (n_dofs,) previous position
        v_prev: (n_dofs,) previous velocity
        t: current time
        h: time step
        M_e: (n_dofs, n_dofs) mass matrix
        G_f: (n_dofs,) gravity force vector
        tol: tolerance for convergence
        max_iter: maximum iterations
        a_init: (n_dofs,) initial guess for acceleration
        lambda_init: (n_constraints,) initial guess for Lagrange multipliers
    
    Returns:
        a_n: (n_dofs,) converged acceleration
        lambda_n: (n_constraints,) converged Lagrange multipliers
    """
    # Determine constraint size dynamically
    c_temp = compute_constraint(x_prev)
    n_constraints = c_temp.shape[0]
    
    a_n = np.zeros(n_dofs) if a_init is None else a_init.copy()
    lambda_n = np.zeros(n_constraints) if lambda_init is None else lambda_init.copy()
    
    for iter_num in range(max_iter):
        # Compute residual
        R = compute_residual(a_n.reshape(-1, 1), lambda_n.reshape(-1, 1), 
                            x_prev, v_prev, t, h, M_e, G_f)
        R_norm = np.linalg.norm(R)
        
        if R_norm < tol:
            if iter_num > 0 and DEBUG:  # Only print if it took more than 0 iterations
                print(f"Newton-Raphson converged in {iter_num} iterations (||R|| = {R_norm:.6e})")
            break
        
        x_n = x_prev + h * v_prev + h**2 * a_n
        J = compute_jacobian(x_n, M_e, h, n_constraints)
        
        size_J = n_dofs + n_constraints
        try:
            delta = np.linalg.solve(J, -R)
            delta_a = delta[0:n_dofs].flatten()
            delta_lambda = delta[n_dofs:size_J].flatten()
        except np.linalg.LinAlgError:
            if DEBUG:
                print(f"Warning: Singular Jacobian at iteration {iter_num}")
            break
        
        a_n += delta_a
        lambda_n += delta_lambda
    
    if iter_num == max_iter - 1 and R_norm >= tol:
        if DEBUG:
            print(f"Warning: Newton-Raphson did not converge: ||R|| = {R_norm:.6e} after {iter_num} iterations")
    
    return a_n, lambda_n


# ============================================================
# BDF-1 INTEGRATOR: TIME-STEPPING LOOP
# ============================================================

def simulate_bdf1(t0, tf, h, x0, v0, tol=1e-6, max_iter=10, save_states=True):
    # Precompute constant matrices
    print("Precomputing constant matrices...")
    M_e = compute_mass_matrix()
    g_vec = np.array([0.0, 0.0, -9.81])
    G_f = compute_gravity_force(g_vec).flatten()
    
    x_n = x0.copy()
    v_n = v0.copy()
    a_n = np.zeros(n_dofs)
    c_temp = compute_constraint(x0)
    n_constraints = c_temp.shape[0]
    lambda_n = np.zeros(n_constraints)
    t = t0
    n_steps = int((tf - t0) / h)
    
    # Storage for states at each time step
    if save_states:
        saved_states = []
        saved_times = []
        saved_lambdas = []
        # Save initial state
        saved_states.append(x_n.copy())
        saved_times.append(t0)
        saved_lambdas.append(lambda_n.copy())
    
    print(f"\nRunning BDF-1 integration with constraints:")
    print(f"  Time: {t0:.6f} to {tf:.6f} s")
    print(f"  Time step: {h:.6e} s")
    print(f"  Number of steps: {n_steps}")
    print(f"  Newton tolerance: {tol:.2e}")
    print(f"  Max Newton iterations: {max_iter}")
    print(f"  Constraint: Node 1 clamped (position + gradients) at {get_node1_fixed_state()}\n")
    
    for step in range(1, n_steps + 1):
        t = t0 + step * h
        if DEBUG:
            print(f"Step {step}/{n_steps}, t = {t:.6f} s")
        
        # Solve for acceleration and Lagrange multipliers using Newton-Raphson
        # (seeded with previous step's values)
        a_n, lambda_n = solve_acceleration_newton(x_n, v_n, t, h, M_e, G_f, 
                                                   tol, max_iter, 
                                                   a_init=a_n, lambda_init=lambda_n)
        
        x_n = x_n + h * v_n + h**2 * a_n
        v_n = v_n + h * a_n
        
        c = compute_constraint(x_n)
        c_norm = np.linalg.norm(c)
        if DEBUG:
            print(f"  Constraint violation: ||c|| = {c_norm:.6e}, ||λ|| = {np.linalg.norm(lambda_n):.6e}")
        
        if save_states:
            saved_states.append(x_n.copy())
            saved_times.append(t)
            saved_lambdas.append(lambda_n.copy())
    
    print(f"\nIntegration complete!")
    print(f"Final time: {t:.6f} s")
    
    if save_states:
        return x_n, v_n, saved_states, saved_times, saved_lambdas
    else:
        return x_n, v_n


# ============================================================
# VISUALIZATION FUNCTIONS
# ============================================================

def save_tip_positions_csv(saved_states, saved_times, h):
    """
    Save tip positions to CSV file with refine and step size in filename.
    Tip is the end of the last element.
    
    Args:
        saved_states: list of (n_dofs,) DOF vectors
        saved_times: list of time values
        h: time step
    """
    print("\nComputing tip positions for CSV...")
    tip_x = []
    tip_y = []
    tip_z = []
    F_z = []
    
    # Tip is at the end of the last element
    elem_tip = n_beam - 1
    idx_tip = np.arange(offset_start[elem_tip], offset_end[elem_tip] + 1)
    
    for i, state in enumerate(saved_states):
        x_tip_local = np.zeros(24)
        for i_local, i_global in enumerate(idx_tip):
            x_tip_local[3*i_local:3*(i_local+1)] = state[3*i_global:3*(i_global+1)]
        Nmat = Nmat_from_dofs(x_tip_local)
        tip_pos = beam_point_position(Nmat, L/2, 0.0, 0.0)
        tip_x.append(tip_pos[0])
        tip_y.append(tip_pos[1])
        tip_z.append(tip_pos[2])
        F_z.append(F_tip(saved_times[i])[2])
    
    # Prepare data for CSV
    data = np.column_stack([saved_times, tip_x, tip_y, tip_z, F_z])
    
    # Save in same folder as script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, f'tip_positions_refined_{n_beam}_h{h:.0e}.csv')
    
    # Save with header
    header = 'time(s),tip_x(m),tip_y(m),tip_z(m),force_z(N)'
    np.savetxt(output_path, data, delimiter=',', header=header, comments='', fmt='%.10e')
    print(f"CSV saved: {output_path}")


def plot_tip_positions(saved_states, saved_times, h):
    """
    Plot x, y, z positions of tip of beam's centerline vs time.
    Tip is the end of the last element.
    
    Args:
        saved_states: list of (n_dofs,) DOF vectors
        saved_times: list of time values
        h: time step
    """
    print("\nComputing tip positions...")
    tip_x = []
    tip_y = []
    tip_z = []
    
    # Tip is at the end of the last element
    elem_tip = n_beam - 1
    idx_tip = np.arange(offset_start[elem_tip], offset_end[elem_tip] + 1)
    
    for i, state in enumerate(saved_states):
        x_tip_local = np.zeros(24)
        for i_local, i_global in enumerate(idx_tip):
            x_tip_local[3*i_local:3*(i_local+1)] = state[3*i_global:3*(i_global+1)]
        Nmat = Nmat_from_dofs(x_tip_local)
        tip_pos = beam_point_position(Nmat, L/2, 0.0, 0.0)
        tip_x.append(tip_pos[0])
        tip_y.append(tip_pos[1])
        tip_z.append(tip_pos[2])
    
    # Create single figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Plot X position
    axes[0].plot(saved_times, tip_x, 'b-')
    axes[0].set_title("Tip X Position")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Global X Position (m)")
    axes[0].grid(True)
    
    # Plot Y position
    axes[1].plot(saved_times, tip_y, 'g-')
    axes[1].set_title("Tip Y Position")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Global Y Position (m)")
    axes[1].grid(True)
    
    # Plot Z position
    axes[2].plot(saved_times, tip_z, 'b-')
    axes[2].set_title("Tip Z Position")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Global Z Position (m)")
    axes[2].grid(True)
    
    plt.tight_layout()
    # Save in same folder as script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, f'tip_positions_refined_{n_beam}_h{h:.0e}.png')
    fig.savefig(output_path, dpi=300)
    print(f"Image saved: {output_path}")
    plt.close(fig)


def create_beam_animation(saved_states, saved_times, h, num_frames=300, num_points=15):
    """
    Create 3D animation of beam deformation.
    
    Args:
        saved_states: list of (n_dofs,) DOF vectors
        saved_times: list of time values
        h: time step
        num_frames: number of animation frames
        num_points: number of points along beam per element for visualization
    """
    print("\nPreparing animation data...")
    n_steps = len(saved_states)
    times = saved_times
    
    indices = np.round(np.linspace(0, n_steps - 1, num_frames)).astype(int)
    
    u_arr = np.linspace(-L / 2, L / 2, num_points)
    half_w = W / 2
    half_h = H / 2
    
    S_center = np.array([shape_functions(uu, 0, 0) for uu in u_arr])
    S_corners = []
    for (v, w) in [(half_w, half_h), (half_w, -half_h), (-half_w, half_h), (-half_w, -half_h)]:
        S_corners.append(np.array([shape_functions(uu, v, w) for uu in u_arr]))
    S_corners = np.array(S_corners)
    
    all_center_pos = []
    all_corners = []
    
    for frame_idx in range(num_frames):
        idx = indices[frame_idx]
        q_full = saved_states[idx]
        
        # Collect positions for all elements
        frame_center_pos = []
        frame_corners = []
        
        for elem in range(n_beam):
            elem_idx = np.arange(offset_start[elem], offset_end[elem] + 1)
            x_local = np.zeros(24)
            for i_local, i_global in enumerate(elem_idx):
                x_local[3*i_local:3*(i_local+1)] = q_full[3*i_global:3*(i_global+1)]
            Nmat = Nmat_from_dofs(x_local)
            
            center_pos = S_center @ Nmat.T
            frame_center_pos.append(center_pos)
            corners = np.einsum('fpu,ud->fpd', S_corners, Nmat.T)
            frame_corners.append(corners)
        
        all_center_pos.append(np.vstack(frame_center_pos))
        all_corners.append(np.array(frame_corners))  # (n_beam, 4, num_points, 3)
    
    all_center_pos = np.array(all_center_pos)
    all_corners = np.array(all_corners)
    
    print(f"Precomputed positions for {num_frames} frames. Creating animation...")

    fig_anim = plt.figure(figsize=(8, 6))
    ax_anim = fig_anim.add_subplot(111, projection='3d')
    # Update axis limits for multi-element beam
    # Use n_beam * L (total length) for axis limits
    total_length = n_beam * L
    ax_anim.set_xlim([-0.02, total_length + 0.02])
    ax_anim.set_ylim([-0.02, 0.02])
    ax_anim.set_zlim([-0.05, 0.02])
    ax_anim.set_xlabel('X')
    ax_anim.set_ylabel('Y')
    ax_anim.set_zlabel('Z')

    ims = []

    for frame in range(num_frames):
        idx = indices[frame]
        pos = all_center_pos[frame]
        corners_all = all_corners[frame]  # (n_beam, 4, num_points, 3)
        
        center_line = ax_anim.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'g-', linewidth=0.5)[0]
        corner_lines = []
        cross_lines = []

        # Plot corner lines for all elements
        for elem in range(n_beam):
            corners = corners_all[elem]  # (4, num_points, 3)
            for k in range(4):
                ls = '--' if k == 1 else '-'
                corner_lines.append(ax_anim.plot(corners[k][:, 0], corners[k][:, 1], corners[k][:, 2], ls, color='k', linewidth=0.5)[0])

        # Plot cross-sections for all elements (only at ends)
        cross_indices = np.array([0, num_points - 1], dtype=int)  # Only at ends
        for elem in range(n_beam):
            corners = corners_all[elem]  # (4, num_points, 3)
            for c_idx in cross_indices:
                pts = corners[:, c_idx, :]
                order = [0, 1, 3, 2, 0]
                for seg in range(4):
                    ls = '-'  # Solid lines for end cross-sections
                    p1 = order[seg]
                    p2 = order[seg + 1]
                    seg_x = pts[[p1, p2], 0]
                    seg_y = pts[[p1, p2], 1]
                    seg_z = pts[[p1, p2], 2]
                    cross_lines.append(ax_anim.plot(seg_x, seg_y, seg_z, ls, color='k', linewidth=0.5)[0])
        
        title_text = fig_anim.text(0.10, 0.95, f"Beam Animation (t = {times[idx]:.3f} s)", va='top', ha='left', fontsize=12)
        ims.append([center_line] + corner_lines + cross_lines + [title_text])

    print("Rendering animation frames...")
    ani = ArtistAnimation(fig_anim, ims, interval=1000/30, blit=False, repeat_delay=1000)
    print("Encoding video with libx264...")
    # Save in same folder as script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    video_path = os.path.join(script_dir, f'beam_refined_{n_beam}_h{h:.0e}.mp4')
    # Use libx264 codec for Linux compatibility
    # -threads 0: Use all available CPU cores for encoding
    ani.save(video_path, writer='ffmpeg', fps=30, dpi=150,
             extra_args=['-vcodec', 'libx264', '-b:v', '5M', '-threads', '0', '-pix_fmt', 'yuv420p'])
    print(f"Video saved: {video_path}")


if __name__ == "__main__":
    # ============================================================
    # 1. MESH DEFINITION
    # ============================================================
    L_total = 0.5            # FIXED Total beam length
    n_elements = 1
    n_elements_desired = int(sys.argv[1]) if len(sys.argv) > 1 else n_elements
    
    L_elem = L_total / n_elements_desired
    
    # Update global connectivity and element properties
    update_connectivity(n_elements_desired)
    initialize_element_properties(L_elem)
    
    # ============================================================
    # 2. INITIALIZATION
    # ============================================================
    # Node 1 at (0,0,0). Node 2 at (L_elem, 0, 0)
    Nmat_initial = np.array([
        [0.0, 1.0, 0.0, 0.0, L_elem, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0,    0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0,    0.0, 0.0, 1.0],
    ])
    
    # Initialize state
    x0, v0 = initialize_state_from_Nmat(Nmat_initial)
    
    # ============================================================
    # 3. SIMULATION
    # ============================================================
    print(f"SIMULATION SETUP: L_total={L_total}m, Elements={n_beam}, L_elem={L_elem:.4f}m")
    
    # Verify initial state
    f_int_initial = compute_internal_force(x0)
    g_vec = np.array([0.0, 0.0, -9.81])
    G_f_initial = compute_gravity_force(g_vec)
    print(f"Initial internal force norm: {np.linalg.norm(f_int_initial):.6e}")
    print(f"Initial gravity force norm: {np.linalg.norm(G_f_initial):.6e}")
    print(f"Initial total force norm: {np.linalg.norm(f_int_initial + G_f_initial):.6e}")
    
    c_initial = compute_constraint(x0)
    print(f"Initial constraint violation: {np.linalg.norm(c_initial):.6e}")
    print()
    
    # Time integration parameters
    t0 = 0.0
    h = 5e-4
    tf = 10.0
    
    # Run BDF-1 integration
    result = simulate_bdf1(t0, tf, h, x0, v0, 
                           tol=1e-6, max_iter=100, save_states=True)
    
    if len(result) == 5:
        x_final, v_final, saved_states, saved_times, saved_lambdas = result
        
        # Logging and Visualization
        save_tip_positions_csv(saved_states, saved_times, h)
        plot_tip_positions(saved_states, saved_times, h)
        # create_beam_animation(saved_states, saved_times, h)
    