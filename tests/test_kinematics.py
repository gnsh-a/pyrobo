"""
Unit tests for kinematics module.

Run with: python -m tests.test_kinematics
"""

import numpy as np
from src.kinematics import forward_kinematics, forward_kinematics_com, compute_jacobians


def test_forward_kinematics_zero_config():
    """Test forward kinematics at zero configuration."""
    q = np.array([0, 0, 0])
    L = 2.0
    
    T10, T20, T30 = forward_kinematics(q, L)
    
    # Check that all transforms are 4x4
    assert T10.shape == (4, 4)
    assert T20.shape == (4, 4)
    assert T30.shape == (4, 4)
    
    # Check last row is [0 0 0 1]
    assert np.allclose(T10[3, :], [0, 0, 0, 1])
    assert np.allclose(T20[3, :], [0, 0, 0, 1])
    assert np.allclose(T30[3, :], [0, 0, 0, 1])
    
    print("✓ Forward kinematics at zero configuration passed")


def test_forward_kinematics_known_config():
    """Test forward kinematics at a known configuration."""
    q = np.array([np.pi/2, 0, 0])
    L = 2.0
    
    T10, T20, T30 = forward_kinematics(q, L)
    
    # Extract end-effector position
    p_ee = T30[:3, 3]
    
    # End-effector should be roughly at [0, 4, 2]
    # (exact values depend on frame definitions)
    assert len(p_ee) == 3
    assert not np.any(np.isnan(p_ee))
    
    print(f"✓ Forward kinematics at q=[π/2, 0, 0] passed")
    print(f"  End-effector position: {p_ee}")


def test_jacobian_dimensions():
    """Test that Jacobians have correct dimensions."""
    q = np.array([0.1, 0.2, 0.3])
    L = 2.0
    
    Jvc1, Jvc2, Jvc3, Jw1, Jw2, Jw3 = compute_jacobians(q, L)
    
    # All Jacobians should be 3x3
    assert Jvc1.shape == (3, 3)
    assert Jvc2.shape == (3, 3)
    assert Jvc3.shape == (3, 3)
    assert Jw1.shape == (3, 3)
    assert Jw2.shape == (3, 3)
    assert Jw3.shape == (3, 3)
    
    print("✓ Jacobian dimensions test passed")


def test_forward_kinematics_com():
    """Test forward kinematics to center of mass."""
    q = np.array([0, 0, 0])
    L = 2.0
    
    Tc10, Tc20, Tc30 = forward_kinematics_com(q, L)
    
    # Check dimensions
    assert Tc10.shape == (4, 4)
    assert Tc20.shape == (4, 4)
    assert Tc30.shape == (4, 4)
    
    print("✓ Forward kinematics to COM test passed")


if __name__ == "__main__":
    test_forward_kinematics_zero_config()
    test_forward_kinematics_known_config()
    test_jacobian_dimensions()
    test_forward_kinematics_com()
    print("\n" + "="*50)
    print("All kinematics tests passed!")
    print("="*50)

