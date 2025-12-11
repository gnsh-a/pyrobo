#!/usr/bin/env python
"""
Generate dynamics functions from symbolic derivation.

This script derives the dynamics equations for the 3-DOF RRR manipulator
from first principles using Lagrangian mechanics, then generates optimized
Python code.

Usage:
    python scripts/generate_dynamics.py

The script will:
1. Symbolically derive D, B, C, G matrices using SymPy
2. Apply optimizations (common subexpression elimination)
3. Generate Python code for each matrix/vector
4. Write to src/dynamics/ directory

"""

import os
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from scripts.symbolic_derivation import derive_all_dynamics
from scripts.code_generator import (
    generate_d_matrix_code,
    generate_b_matrix_code, 
    generate_c_matrix_code,
    generate_g_vector_code,
    write_generated_code
)


def main():
    """Main generation function."""
    print()
    print("╔" + "="*58 + "╗")
    print("║" + " "*58 + "║")
    print("║" + "  SYMBOLIC DYNAMICS CODE GENERATOR".center(58) + "║")
    print("║" + "  3-DOF RRR Manipulator".center(58) + "║")
    print("║" + " "*58 + "║")
    print("╚" + "="*58 + "╝")
    print()
    print("This will regenerate the dynamics functions from first principles.")
    print("The process takes 30-90 seconds depending on your machine.")
    print()
    
    # Step 1: Derive dynamics symbolically
    print("STEP 1: Symbolic Derivation")
    print("-" * 60)
    dynamics = derive_all_dynamics()
    print()
    
    # Step 2: Generate optimized code
    print("STEP 2: Code Generation")
    print("-" * 60)
    
    D = dynamics['D']
    B = dynamics['B']
    C = dynamics['C']
    G = dynamics['G']
    q = dynamics['q']
    
    # Generate code for each matrix/vector
    d_code = generate_d_matrix_code(D, q)
    print()
    
    b_code = generate_b_matrix_code(B, q)
    print()
    
    c_code = generate_c_matrix_code(C, q)
    print()
    
    g_code = generate_g_vector_code(G, q)
    print()
    
    # Step 3: Write to files
    print("STEP 3: Writing Generated Code")
    print("-" * 60)
    
    # Determine output directory
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    output_dir = project_root / "src" / "dynamics"
    
    # Create backup directory
    backup_dir = output_dir / "backup"
    backup_dir.mkdir(exist_ok=True)
    
    # Backup existing files
    files_to_generate = [
        ('mass_matrix.py', d_code),
        ('coriolis_matrix.py', b_code),
        ('centrifugal_matrix.py', c_code),
        ('gravity_vector.py', g_code)
    ]
    
    print("Creating backups of existing files...")
    for filename, _ in files_to_generate:
        src_file = output_dir / filename
        if src_file.exists():
            backup_file = backup_dir / f"{filename}.backup"
            import shutil
            shutil.copy2(src_file, backup_file)
            print(f"  Backed up {filename} → backup/{filename}.backup")
    print()
    
    # Write new files
    print("Writing generated files...")
    for filename, code in files_to_generate:
        output_path = output_dir / filename
        write_generated_code(code, output_path)
    print()
    
    # Step 4: Summary
    print("="*60)
    print("GENERATION COMPLETE!")
    print("="*60)
    print()
    print("Generated files:")
    for filename, _ in files_to_generate:
        print(f"  ✓ src/dynamics/{filename}")
    print()
    print("Backups saved to:")
    print(f"  src/dynamics/backup/")
    print()
    print("Next steps:")
    print("  1. Run tests to verify correctness:")
    print("     python -m pytest tests/test_dynamics.py -v")
    print()
    print("  2. Run an example to see it in action:")
    print("     python -m examples.example_3_dynamics")
    print()
    print("The generated functions should produce identical results to")
    print("the previous hardcoded versions (within floating-point precision).")
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nGeneration interrupted by user.")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

