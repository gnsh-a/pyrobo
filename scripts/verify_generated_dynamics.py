"""
Quick verification script to check that generated dynamics code works correctly.

Compares generated code against the backed-up original code to ensure
they produce identical results.
"""

import numpy as np
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Import generated functions
from src.dynamics import compute_D, compute_B, compute_C, compute_G

# Import backup functions (manually)
sys.path.insert(0, str(Path(__file__).parent.parent / "src" / "dynamics" / "backup"))


def load_backup_functions():
    """Load the original backed-up functions."""
    import importlib.util
    
    backup_dir = Path(__file__).parent.parent / "src" / "dynamics" / "backup"
    
    # Load mass_matrix backup
    spec = importlib.util.spec_from_file_location("mass_matrix_backup", 
                                                   backup_dir / "mass_matrix.py.backup")
    mass_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mass_module)
    
    # Load coriolis_matrix backup
    spec = importlib.util.spec_from_file_location("coriolis_matrix_backup",
                                                   backup_dir / "coriolis_matrix.py.backup")
    coriolis_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(coriolis_module)
    
    # Load centrifugal_matrix backup
    spec = importlib.util.spec_from_file_location("centrifugal_matrix_backup",
                                                   backup_dir / "centrifugal_matrix.py.backup")
    centrifugal_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(centrifugal_module)
    
    # Load gravity_vector backup
    spec = importlib.util.spec_from_file_location("gravity_vector_backup",
                                                   backup_dir / "gravity_vector.py.backup")
    gravity_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gravity_module)
    
    return (mass_module.compute_D, 
            coriolis_module.compute_B,
            centrifugal_module.compute_C,
            gravity_module.compute_G)


def test_configuration(q, L, m, g, I, label="Test"):
    """Test a single configuration."""
    
    # New generated functions
    D_new = compute_D(q, L, m, g, I)
    B_new = compute_B(q, L, m, g, I)
    C_new = compute_C(q, L, m, g, I)
    G_new = compute_G(q, L, m, g, I)
    
    # Old backup functions
    compute_D_old, compute_B_old, compute_C_old, compute_G_old = load_backup_functions()
    D_old = compute_D_old(q, L, m, g, I)
    B_old = compute_B_old(q, L, m, g, I)
    C_old = compute_C_old(q, L, m, g, I)
    G_old = compute_G_old(q, L, m, g, I)
    
    # Compare
    D_diff = np.max(np.abs(D_new - D_old))
    B_diff = np.max(np.abs(B_new - B_old))
    C_diff = np.max(np.abs(C_new - C_old))
    G_diff = np.max(np.abs(G_new - G_old))
    
    print(f"\n{label}:")
    print(f"  q = {q}")
    print(f"  Max difference in D matrix: {D_diff:.2e}")
    print(f"  Max difference in B matrix: {B_diff:.2e}")
    print(f"  Max difference in C matrix: {C_diff:.2e}")
    print(f"  Max difference in G vector: {G_diff:.2e}")
    
    # Check if differences are acceptable (within floating point tolerance)
    tolerance = 1e-12
    passed = all([
        D_diff < tolerance,
        B_diff < tolerance,
        C_diff < tolerance,
        G_diff < tolerance
    ])
    
    if passed:
        print(f"  ✓ PASSED (all differences < {tolerance})")
    else:
        print(f"  ✗ FAILED (some differences >= {tolerance})")
    
    return passed


def main():
    """Run verification tests."""
    print("="*60)
    print("VERIFICATION: Generated vs Original Dynamics")
    print("="*60)
    
    # Test parameters
    L = 2.0
    m = 10.0
    g = 9.81
    I = 5.0
    
    all_passed = True
    
    # Test 1: Zero configuration
    passed = test_configuration(
        q=np.array([0.0, 0.0, 0.0]),
        L=L, m=m, g=g, I=I,
        label="Test 1: Zero configuration"
    )
    all_passed = all_passed and passed
    
    # Test 2: Random configuration 1
    passed = test_configuration(
        q=np.array([0.5, 1.2, -0.8]),
        L=L, m=m, g=g, I=I,
        label="Test 2: Random configuration 1"
    )
    all_passed = all_passed and passed
    
    # Test 3: Random configuration 2
    passed = test_configuration(
        q=np.array([-1.5, 0.3, 2.1]),
        L=L, m=m, g=g, I=I,
        label="Test 3: Random configuration 2"
    )
    all_passed = all_passed and passed
    
    # Test 4: Near-singular configuration
    passed = test_configuration(
        q=np.array([np.pi/4, np.pi/2, -np.pi/3]),
        L=L, m=m, g=g, I=I,
        label="Test 4: Near-singular configuration"
    )
    all_passed = all_passed and passed
    
    # Summary
    print("\n" + "="*60)
    if all_passed:
        print("✓ ALL TESTS PASSED!")
        print("Generated code produces identical results to original.")
    else:
        print("✗ SOME TESTS FAILED!")
        print("Generated code differs from original.")
    print("="*60)
    
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())

