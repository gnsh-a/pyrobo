"""
Unit tests for dynamics module.

Run with: python -m tests.test_dynamics
"""

import numpy as np
from src.dynamics import compute_D, compute_B, compute_C, compute_G


def test_mass_matrix_properties():
    """Test that mass matrix has correct properties."""
    q = np.array([0.1, 0.2, 0.3])
    L = 2.0
    m = 10.0
    g = 9.81
    I = 5.0
    
    D = compute_D(q, L, m, g, I)
    
    # Check dimensions
    assert D.shape == (3, 3)
    
    # Check symmetry (mass matrix should be symmetric)
    assert np.allclose(D, D.T)
    
    # Check positive definiteness (all eigenvalues should be positive)
    eigenvalues = np.linalg.eigvals(D)
    assert np.all(eigenvalues > 0)
    
    print("✓ Mass matrix properties test passed")
    print(f"  Eigenvalues: {eigenvalues}")


def test_coriolis_centrifugal_dimensions():
    """Test that Coriolis and centrifugal matrices have correct dimensions."""
    q = np.array([0.1, 0.2, 0.3])
    L = 2.0
    m = 10.0
    g = 9.81
    I = 5.0
    
    B = compute_B(q, L, m, g, I)
    C = compute_C(q, L, m, g, I)
    
    # Check dimensions
    assert B.shape == (3, 3)
    assert C.shape == (3, 3)
    
    print("✓ Coriolis and centrifugal matrix dimensions test passed")


def test_gravity_vector():
    """Test gravity vector properties."""
    q = np.array([0, 0, 0])
    L = 2.0
    m = 10.0
    g = 9.81
    I = 5.0
    
    G = compute_G(q, L, m, g, I)
    
    # Check dimensions
    assert G.shape == (3,)
    
    # At zero configuration, first joint should have zero gravity torque
    # (vertical joint doesn't experience gravity torque)
    assert np.isclose(G[0], 0.0)
    
    print("✓ Gravity vector test passed")
    print(f"  G at q=[0,0,0]: {G}")


def test_dynamics_no_nan():
    """Test that dynamics matrices don't contain NaN values."""
    q = np.array([0.5, -0.3, 0.8])
    L = 2.0
    m = 10.0
    g = 9.81
    I = 5.0
    
    D = compute_D(q, L, m, g, I)
    B = compute_B(q, L, m, g, I)
    C = compute_C(q, L, m, g, I)
    G = compute_G(q, L, m, g, I)
    
    # Check for NaN values
    assert not np.any(np.isnan(D))
    assert not np.any(np.isnan(B))
    assert not np.any(np.isnan(C))
    assert not np.any(np.isnan(G))
    
    print("✓ No NaN values in dynamics matrices test passed")


if __name__ == "__main__":
    test_mass_matrix_properties()
    test_coriolis_centrifugal_dimensions()
    test_gravity_vector()
    test_dynamics_no_nan()
    print("\n" + "="*50)
    print("All dynamics tests passed!")
    print("="*50)

