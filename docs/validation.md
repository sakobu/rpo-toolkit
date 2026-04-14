# Validation

This document covers how `rpo-core` and `rpo-nyx` are validated against full-physics propagation and published reference data. For the headline accuracy numbers and the observed-error table, see the **Validated Accuracy** section of the [README](../README.md#validated-accuracy).

## Reproducing the headline numbers

The bundled validation example reproduces the observed errors in the README table with one command:

```bash
cargo run -p rpo-cli -- validate --input examples/validate.json              # J2 STM
cargo run -p rpo-cli -- validate --input examples/validate.json --auto-drag  # J2 + DMF drag
```

First use downloads ~50 MB of ANISE kernels (DE440s, PCK) and caches them under the default `anise` path. Subsequent runs hit the cache.

## Test-suite gates

The regression tests enforce conservative pass/fail gates with ~10× margin over typically-observed errors. The gates live alongside the code that validates them:

| Constant                             | Value     | Location                                      |
| ------------------------------------ | --------- | --------------------------------------------- |
| `FULL_PHYSICS_SINGLE_LEG_POS_TOL_KM` | 500 m     | `rpo-nyx/src/validation/trajectory/pipeline.rs` |
| `DRAG_STM_VS_NYX_POS_TOL_KM`         | 1 km      | `rpo-nyx/src/validation/trajectory/pipeline.rs` |
| `FULL_PHYSICS_MULTI_LEG_POS_TOL_KM`  | 3 km      | `rpo-nyx/src/validation/trajectory/pipeline.rs` |
| `SUN_DIRECTION_VALIDATION_TOL_RAD`   | 3.5e-4 rad (≈ 0.02°) | `rpo-core/src/constants.rs`          |
| `ECLIPSE_TIMING_VALIDATION_TOL_S`    | 120 s     | `rpo-core/src/constants.rs`                   |

These are intentionally looser than the observed errors (see the README table) so the regression suite stays stable under small numerical perturbations while still catching genuine regressions. When an observed error approaches a gate, it is a signal to investigate and either tighten the test or fix the underlying method — not to loosen the gate.

## Per-component ROE validation against Koenig Table 4 Case 1

The J2 STM is independently validated against the per-component error bounds in Koenig et al. (2017), Table 4 Case 1. Each quasi-nonsingular ROE component (`δa`, `δλ`, `δex`, `δey`, `δix`, `δiy`) is bounded within ~10× of the published errors:

- `KOENIG_T4C1_DA_BOUND_M = 385 m`
- `KOENIG_T4C1_DIX_BOUND_M = 9 m`
- (full set in `rpo-nyx/tests/regression_tests.rs`)

The regression test `koenig_table4_j2_stm_accuracy_case1` lives in `rpo-nyx/tests/regression_tests.rs` and runs as part of the ignored-by-default full-physics suite (needs ANISE kernels).

## Eclipse validation

Eclipse geometry is validated along two axes:

1. **Sun/Moon direction**: Meeus-series positions are cross-checked against ANISE DE440s ephemerides. Gate: `SUN_DIRECTION_VALIDATION_TOL_RAD` (≈ 0.02°). Observed: ~0.005°.
2. **Entry/exit timing**: shadow transition epochs are cross-checked against a full-physics propagation with conical shadow geometry. Gate: `ECLIPSE_TIMING_VALIDATION_TOL_S = 120 s`. Observed: ~32 s mean, 83 s max.

Both gates live in `rpo-core/src/constants.rs`.

## References

- **Koenig, Guffanti, D'Amico** — "New State Transition Matrices for Spacecraft Relative Motion in Perturbed Orbits" ([PDF](references/Koenig_Guffanti_Damico.pdf)), JGCD 2017. J2/drag STMs, Tables 2–4 for validation targets.
- **D'Amico** — "Autonomous Formation Flying in Low Earth Orbit" ([PDF](references/Damico_PhD.pdf)), PhD thesis, TU Delft 2010. QNS ROE definitions, e/i separation, Sec. 2.1–2.2 worked examples.
