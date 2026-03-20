# Contributing to RPO Toolkit

## Quick Start

```bash
cargo build                                     # build workspace
cargo test                                      # run all tests
cargo clippy --workspace -- -D warnings         # lint (must pass)
cargo doc --workspace --no-deps                 # generate docs
```

## Coding Standards

### General

- **Clippy pedantic** enforced (`#![warn(clippy::pedantic)]`); `#![warn(missing_docs)]` on all public items
- **f64 everywhere** — never `f32`, never compare floats with `==`, use named epsilon constants
- Angles in **radians**, distances in **km**, velocities in **km/s**
- Single-char variable names (`a`, `e`, `i`, `u`, `n`, `r`, `v`, `h`) are acceptable for standard orbital mechanics quantities

### Naming

Unit suffixes are **mandatory** on public fields with physical units:

| Suffix  | Meaning    | Suffix | Meaning                 |
| ------- | ---------- | ------ | ----------------------- |
| `_km`   | kilometers | `_s`   | seconds                 |
| `_km_s` | km/s       | `_rad` | radians                 |
| `_km2`  | km²        | `_kg`  | kilograms               |
| `_m2`   | m²         | `_deg` | degrees (avoid in math) |

Frame tags are **mandatory** on state-like geometric vectors (`position_eci_km`, `dv_ric_km_s`, `dcm_eci_to_ric`). Be precise with terminology: mean vs osculating, chief vs deputy, ECI vs RIC.

### Tolerances

All tolerances must be named constants with documented justification — no anonymous `1e-8` literals. Categorize as geometry, convergence, singularity, or test tolerances.

### Error Handling

- `Result<T, E>` for all failure paths — no `unwrap`/`expect` in library code
- Structured error enums with diagnostic fields, not strings
- No `assert!` for input validation — return `Err(...)` instead

### Design Patterns

- Struct-based public APIs — no raw scalar argument lists at API boundaries
- Enums over boolean flags for algorithm regimes
- Separate classification from execution in solvers
- Explicit `for` loops in iterative algorithms (no recursion), no heap allocation in solver loops

### Traceability

Astrodynamics code must cite source equations (Koenig, D'Amico). Cross-check against reference PDFs in `docs/` before modifying formulas.

### Testing

- Cover nominal cases, edge conditions, degenerate geometries, and regression against published data (cite exact table/figure)
- Include invariant tests: roundtrip transforms, STM identity at dt=0, covariance symmetry, DCM orthonormality, energy/momentum conservation
- Test tolerances must be documented with justification

## Workspace Layout

| Crate      | Purpose                                                                  |
| ---------- | ------------------------------------------------------------------------ |
| `rpo-core` | Library: orbital mechanics, planning, propagation, mission orchestration |
| `rpo-cli`  | Binary: CLI tool exercising the pipeline                                 |
| `rpo-api`  | Binary: WebSocket server wrapping the pipeline                           |

The dependency graph is `rpo-core` <- `rpo-cli` and `rpo-core` <- `rpo-api`. The two binaries never depend on each other.

Shared orchestration lives in `rpo-core/src/pipeline/`. Both the CLI and API call `execute_mission()`, `compute_transfer()`, and `replan_mission()` from this module.

## How to Add a New CLI Command

1. **Define the command in `rpo-cli/src/cli.rs`.**
   Add a variant to the `Command` enum with clap attributes. Porcelain commands (human-readable output) take `--json`; plumbing commands always output JSON.

2. **Create a handler in `rpo-cli/src/commands/`.**
   Add a new file (e.g., `commands/mycommand.rs`). The handler function signature is `pub fn run(...) -> Result<(), CliError>`. Re-export it from `commands/mod.rs`.

3. **Wire it into `main.rs`.**
   Add a match arm in the `match cli.command { ... }` block that calls your handler.

4. **Add output formatting (porcelain only).**
   If your command has human-readable output, add a formatter in `rpo-cli/src/output/` and re-export from `output/mod.rs`.

## How to Add a New API Handler

1. **Add the message variants in `rpo-api/src/protocol.rs`.**
   Add a variant to `ClientMessage` (with `request_id` and any fields) and a corresponding variant to `ServerMessage`.

2. **Create a handler in `rpo-api/src/handlers/`.**
   Add a new file (e.g., `handlers/myhandler.rs`). Handlers are pure functions that take references and return `Result<T, ApiError>`. Re-export from `handlers/mod.rs`.

3. **Wire it into `rpo-api/src/ws.rs`.**
   Add a match arm in `handle_text_message()`. Inline handlers call the function directly and send the response. Background handlers use `tokio::task::spawn_blocking` with progress/result channels.

4. **Add an integration test.**
   Add a test to `rpo-api/tests/integration.rs`. Tests connect via WebSocket, send a `ClientMessage`, and assert on the `ServerMessage` response.

## How to Add a New Error Type

Error types flow from `rpo-core` through both the CLI and API.

1. **Define the error in `rpo-core`.**
   Add a variant to the relevant error enum (e.g., `MissionError`, `PropagationError`). Use structured fields, not strings — see the Coding Standards section above.

2. **Add a `From` impl in `rpo-api/src/error.rs`.**
   Map the new error to an `ApiError` variant. Implement `extract_code_and_detail()` to produce a machine-readable `ErrorCode` and optional `detail` JSON.

3. **Add a `From` impl in `rpo-cli/src/error.rs`.**
   Map the new error to a `CliError` variant (usually via `PipelineError`).

4. **If it is a new top-level error enum**, also add:
   - A variant to `PipelineError` in `rpo-core/src/pipeline/errors.rs`
   - `From` impls in `pipeline/errors.rs`, `rpo-api/src/error.rs`, and `rpo-cli/src/error.rs`

## PR Checklist

Before submitting, verify:

- [ ] `cargo build` succeeds
- [ ] `cargo test` passes (all existing + new tests)
- [ ] `cargo clippy --workspace -- -D warnings` is clean
- [ ] `cargo doc --workspace --no-deps` generates without warnings
- [ ] All public items have doc comments (`#![warn(missing_docs)]`)
- [ ] Errors use structured enums with diagnostic fields, not strings
- [ ] Astrodynamics changes cite the source equation (Koenig, D'Amico, or other reference)
- [ ] Numerical tolerances are named constants with documented justification
- [ ] Unit suffixes on public fields with physical units (`_km`, `_rad`, `_km_s`, etc.)
- [ ] Frame tags on state-like geometric vectors (`_eci_`, `_ric_`)
- [ ] Tests cover nominal cases, edge conditions, and regression against published data
