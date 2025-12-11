# Dynamics Code Generation Guide (Essential Info)

## What

The dynamics Python code in `src/dynamics/` is **auto-generated** from symbolic math using SymPy.  
You do **not** need to hand-edit these files.

## How

1. **Edit robot model or derivation** in `scripts/symbolic_derivation.py`
2. **Generate new code**:
   ```bash
   pip install sympy>=1.12      # If not installed
   python scripts/generate_dynamics.py
   ```
   - Makes backups of old code
   - Overwrites with new, optimized `compute_D`, `compute_B`, `compute_C`, `compute_G` in `src/dynamics/`

3. **Run or test** as usual; the generated code is immediately usable and fast.

## Key Points

- **Generated Functions**: 
    - `mass_matrix.py`:     `compute_D(q, L, m, g, I)`
    - `coriolis_matrix.py`: `compute_B(q, L, m, g, I)`
    - `centrifugal_matrix.py`: `compute_C(q, L, m, g, I)`
    - `gravity_vector.py`:  `compute_G(q, L, m, g, I)`
- **Generation takes ~1 min** (symbolic simplification is slow, code runs fast after).
- **Optimization**: Common subexpressions are factored out for speed.
- **NumPy-native**: uses `np.sin`/`np.cos` for vector compatibility.

## Troubleshooting

- _"NameError: name 'math' is not defined"_: Regenerate; should use `np` functions.
- _Results mismatch_: Check DH parameters and inertia definitions in `symbolic_derivation.py`.

_Last updated: December 10, 2025_
