# Dynamics Code Generation Guide

## What

`src/pyrobo/dynamics/_generated.py` is auto-generated from symbolic math.
Do not hand-edit it.

## How

1. Edit the robot model / derivation in `scripts/symbolic_derivation.py`.
2. Regenerate:
   ```bash
   uv run python scripts/generate_dynamics.py
   ```
   This overwrites `src/pyrobo/dynamics/_generated.py` with fresh
   `compute_D`, `compute_B`, `compute_C`, `compute_G` definitions.
3. Verify via the normal test suite:
   ```bash
   uv run python tests/test_dynamics.py
   ```
4. Review the diff with `git diff src/pyrobo/dynamics/_generated.py` — git
   handles versioning, so no backup copies are kept.

## Pipeline

1. `symbolic_derivation.py` derives D, B, C, G symbolically via Lagrangian
   mechanics (SymPy). This is the pedagogical core — kept separate.
2. `generate_dynamics.py` applies `sympy.cse` for common subexpression
   elimination, uses `sympy.printing.numpy.NumPyPrinter` to emit NumPy
   code, and writes the single `_generated.py` module.

Generation takes ~1 minute (symbolic simplification is slow; the generated
code itself is fast).
