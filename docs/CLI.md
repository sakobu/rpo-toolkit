# RPO CLI Reference

The `rpo-cli` crate provides batch execution and shell-composable commands for mission planning, validation, and Monte Carlo analysis. It wraps the same `rpo-core` pipeline used by `rpo-api`.

## Commands

**Porcelain** commands produce human-readable output by default (use `--json` for machine output). **Plumbing** commands always produce JSON for scripting and pipeline composition.

### Porcelain

#### mission

End-to-end analytical mission: classification, Lambert transfer, perch handoff, waypoint targeting, safety, covariance, eclipse.

| Flag                      | Type | Default  | Description                                                                      |
| ------------------------- | ---- | -------- | -------------------------------------------------------------------------------- |
| `-i, --input`             | path | required | JSON input file (PipelineInput)                                                  |
| `--json`                  | bool | false    | Write JSON report to `reports/json/mission.json` (conflicts with `--markdown`)   |
| `--markdown`              | bool | false    | Write markdown report to `reports/markdown/mission.md` (conflicts with `--json`) |
| `--cola-threshold`        | f64  | none     | Target miss distance for collision avoidance (km). Enables COLA analysis         |
| `--cola-budget`           | f64  | 0.01     | Maximum delta-v budget for COLA (km/s). Requires `--cola-threshold`              |
| `--auto-enrich`           | bool | false    | Enable formation design enrichment with 0.1 km separation threshold              |
| `--auto-enrich-threshold` | f64  | none     | Custom min separation (km) for formation design. Implies `--auto-enrich`         |

Free-drift (abort-case) analysis and closest-approach refinement (Brent's method on range rate) are included automatically when `config.safety` is present in the input.

**COLA auto-compute:** When `config.safety` is present and any POCA violates `min_distance_3d_km`, collision avoidance maneuvers are automatically computed using the safety threshold as the target distance and a default budget of 10 m/s. The `--cola-threshold` and `--cola-budget` flags override this auto-derived config. Post-avoidance multi-leg verification detects secondary conjunctions (where an avoidance maneuver on one leg creates a new violation on a downstream leg). Legs where COLA fails (budget exceeded, degenerate geometry) are reported as skipped.

**Formation design:** When `--auto-enrich` is set, the perch ROE is enriched with safe e/i vectors (D'Amico Eq. 2.22) before waypoint targeting. Per-waypoint enrichment advisory and per-leg transit e/i separation profiles are computed and displayed alongside the standard mission output.

```bash
cargo run -p rpo-cli -- mission --input examples/mission.json
cargo run -p rpo-cli -- mission --input examples/mission.json --json
cargo run -p rpo-cli -- mission --input examples/mission.json --markdown
cargo run -p rpo-cli -- mission --input examples/mission.json --cola-threshold 0.3 --cola-budget 0.01
cargo run -p rpo-cli -- mission --input examples/mission.json --auto-enrich
cargo run -p rpo-cli -- mission --input examples/mission.json --auto-enrich-threshold 0.15 --json
```

#### validate

Mission planning + nyx high-fidelity validation. Compares analytical trajectories against full-physics propagation (J2, drag, SRP with eclipses, Sun/Moon third-body). Requires network on first run to download ANISE ephemeris kernels (~50 MB, cached).

| Flag                      | Type | Default  | Description                                                                       |
| ------------------------- | ---- | -------- | --------------------------------------------------------------------------------- |
| `-i, --input`             | path | required | JSON input file (PipelineInput with spacecraft configs)                           |
| `--json`                  | bool | false    | Write JSON report to `reports/json/validate.json` (conflicts with `--markdown`)   |
| `--markdown`              | bool | false    | Write markdown report to `reports/markdown/validate.md` (conflicts with `--json`) |
| `--samples-per-leg`       | u32  | 50       | Comparison points per leg                                                         |
| `--auto-drag`             | bool | false    | Auto-derive differential drag rates from spacecraft properties via nyx            |
| `--cola-threshold`        | f64  | none     | Target miss distance for collision avoidance (km). Enables COLA analysis          |
| `--cola-budget`           | f64  | 0.01     | Maximum delta-v budget for COLA (km/s). Requires `--cola-threshold`               |
| `--auto-enrich`           | bool | false    | Enable formation design enrichment with 0.1 km separation threshold               |
| `--auto-enrich-threshold` | f64  | none     | Custom min separation (km) for formation design. Implies `--auto-enrich`          |

```bash
cargo run -p rpo-cli -- validate --input examples/validate.json
cargo run -p rpo-cli -- validate --input examples/validate.json --auto-drag
cargo run -p rpo-cli -- validate --input examples/validate.json --json
cargo run -p rpo-cli -- validate --input examples/validate.json --auto-drag --markdown
cargo run -p rpo-cli -- validate --input examples/validate.json --cola-threshold 0.3 --cola-budget 0.01
cargo run -p rpo-cli -- validate --input examples/validate.json --auto-enrich
cargo run -p rpo-cli -- validate --input examples/validate.json --auto-enrich-threshold 0.15 --json
```

#### mc

Full-physics Monte Carlo ensemble analysis. Requires `monte_carlo` config in the input JSON. Uses rayon for parallel sample execution.

| Flag                      | Type | Default  | Description                                                                 |
| ------------------------- | ---- | -------- | --------------------------------------------------------------------------- |
| `-i, --input`             | path | required | JSON input file (PipelineInput with spacecraft + MC configs)                |
| `--json`                  | bool | false    | Write JSON report to `reports/json/mc.json` (conflicts with `--markdown`)   |
| `--markdown`              | bool | false    | Write markdown report to `reports/markdown/mc.md` (conflicts with `--json`) |
| `--auto-drag`             | bool | false    | Auto-derive differential drag rates before MC                               |
| `--cola-threshold`        | f64  | none     | Target miss distance for collision avoidance (km). Enables COLA analysis    |
| `--cola-budget`           | f64  | 0.01     | Maximum delta-v budget for COLA (km/s). Requires `--cola-threshold`         |
| `--auto-enrich`           | bool | false    | Enable formation design enrichment with 0.1 km separation threshold         |
| `--auto-enrich-threshold` | f64  | none     | Custom min separation (km) for formation design. Implies `--auto-enrich`    |

```bash
cargo run -p rpo-cli -- mc --input examples/mc.json --auto-drag
cargo run -p rpo-cli -- mc --input examples/mc.json --json
cargo run -p rpo-cli -- mc --input examples/mc.json --auto-drag --markdown
cargo run -p rpo-cli -- mc --input examples/mc.json --cola-threshold 0.3 --cola-budget 0.01
cargo run -p rpo-cli -- mc --input examples/mc.json --auto-drag --auto-enrich
cargo run -p rpo-cli -- mc --input examples/mc.json --auto-enrich-threshold 0.15 --json
```

### Plumbing

All plumbing commands take `-i, --input <path>` and produce JSON to stdout.

#### classify

Classify chief/deputy separation as proximity or far-field.

Input: `{ chief: StateVector, deputy: StateVector, proximity?: ProximityConfig }`

```bash
cargo run -p rpo-cli -- classify -i examples/classify.json
```

#### transfer

Solve Lambert transfer between two ECI states. TOF is derived from the epoch difference.

Input: `{ departure: StateVector, arrival: StateVector, config?: LambertConfig }`

```bash
cargo run -p rpo-cli -- transfer -i examples/transfer.json
```

#### convert

Convert ECI state to Keplerian elements.

Input: bare `StateVector { epoch, position_eci_km, velocity_eci_km_s }`

```bash
cargo run -p rpo-cli -- convert -i examples/convert.json --to keplerian
```

#### propagate

Propagate ROE state via J2 or J2+drag STM.

Input: `{ roe: ROE, chief_mean: KeplerianElements, epoch: string, dt_s: number, propagator?: "j2" }`

```bash
cargo run -p rpo-cli -- propagate -i examples/propagate.json
```

#### roe

Compute quasi-nonsingular ROE between chief and deputy states.

Input: `{ chief: StateVector, deputy: StateVector }`

```bash
cargo run -p rpo-cli -- roe -i examples/roe.json
```

#### safety

Analyze passive safety metrics (e/i separation, 3D keep-out) for a planned mission.

Input: full `PipelineInput` (same as `mission` command).

```bash
cargo run -p rpo-cli -- safety -i examples/mission.json
```

#### eclipse

Compute eclipse intervals along a planned mission trajectory.

Input: full `PipelineInput` (same as `mission` command).

```bash
cargo run -p rpo-cli -- eclipse -i examples/mission.json
```

## Input Format

All porcelain commands accept a `PipelineInput` JSON file. This is the same type used by the WebSocket API (`MissionDefinition`). See `docs/schema/pipeline-input.schema.json` for the full JSON Schema.

Example files in `examples/`:

| File             | Used by                        |
| ---------------- | ------------------------------ |
| `mission.json`   | `mission`, `safety`, `eclipse` |
| `validate.json`  | `validate`                     |
| `mc.json`        | `mc`                           |
| `classify.json`  | `classify`                     |
| `transfer.json`  | `transfer`                     |
| `convert.json`   | `convert`                      |
| `propagate.json` | `propagate`                    |
| `roe.json`       | `roe`                          |

### Spacecraft Presets

The `chief_config` and `deputy_config` fields accept a named preset or custom properties:

```json
"chief_config": "servicer_500kg"
```

```json
"chief_config": "cubesat_6u"
```

```json
"chief_config": {
  "custom": {
    "dry_mass_kg": 500.0,
    "drag_area_m2": 1.0,
    "coeff_drag": 2.2,
    "srp_area_m2": 1.0,
    "coeff_reflectivity": 1.8
  }
}
```

| Preset           | Mass   | Drag Area | Cd  | SRP Area | Cr  |
| ---------------- | ------ | --------- | --- | -------- | --- |
| `cubesat_6u`     | 12 kg  | 0.06 m2   | 2.2 | 0.06 m2  | 1.5 |
| `servicer_500kg` | 500 kg | 1.0 m2    | 2.2 | 1.0 m2   | 1.5 |

These configs are required for `validate`, `mc`, and `--auto-drag`. If omitted, `validate` defaults to `servicer_500kg`; `mc` and drag extraction return an error.

## Shell Completions

```bash
cargo run -p rpo-cli -- completions bash > /usr/local/etc/bash_completion.d/rpo-cli
cargo run -p rpo-cli -- completions zsh > ~/.zfunc/_rpo-cli
cargo run -p rpo-cli -- completions fish > ~/.config/fish/completions/rpo-cli.fish
```

## Global Options

| Flag      | Values                    | Default | Description       |
| --------- | ------------------------- | ------- | ----------------- |
| `--color` | `auto`, `always`, `never` | `auto`  | Color output mode |
