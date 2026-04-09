# RPO WebSocket API

The `rpo-api` crate provides a stateless WebSocket server for the RPO mission planner. It handles the 4 operations that require nyx-space: Lambert transfer, drag extraction, full-physics validation, and Monte Carlo ensemble. All analytical operations (classify, waypoint planning, targeting, safety, formation design, covariance, eclipse, POCA, COLA, free-drift) run in the browser via the WASM engine (`rpo-wasm`).

## Architecture

```
Browser (rpo-wasm)                     Server (rpo-api)
─────────────────                      ────────────────
classify_separation()                  compute_transfer()  ← Lambert solver (nyx)
plan_waypoint_mission()                extract_drag()      ← nyx full-physics sim
compute_safety_analysis()              validate()          ← nyx full-physics validation
compute_mission_covariance()           run_mc()            ← nyx Monte Carlo ensemble
compute_transfer_eclipse()
compute_mission_eclipse()
assess_cola()
compute_avoidance()
compute_free_drift_analysis()
compute_poca_analysis()
suggest_enrichment()
apply_perch_enrichment()
accept_waypoint_enrichment()
enrich_waypoint()
execute_mission_from_transfer()
replan_from_transfer()
get_mission_state_at_time()
resample_leg_trajectory()
```

The browser holds all mission state. Each server message is self-contained -- the client sends complete inputs with every request and the server holds no persistent state between messages.

## Connection

| Endpoint      | Description                   |
| ------------- | ----------------------------- |
| `GET /ws`     | WebSocket upgrade             |
| `GET /health` | Health check (returns `"ok"`) |

Default bind address: `127.0.0.1:3001`

Environment variables:

- `RPO_BIND` -- bind address (default: `127.0.0.1`)
- `RPO_PORT` -- port (default: `3001`)

This is a single-user desktop tool. One WebSocket connection at a time, one background job at a time.

## Protocol

All messages are JSON text frames with a `"type"` discriminator tag (serde internally-tagged enum, `snake_case`).

- Non-text WebSocket frames receive an `error` response with `code: "invalid_input"`.
- The server is **stateless**: every client message carries all inputs needed for the operation.
- Client-assigned `request_id` (u64) correlates requests with responses.

## Client Messages

### compute_transfer

Classify chief/deputy separation and solve Lambert transfer (far-field) or compute perch states (proximity). Synchronous (~100ms for far-field, microseconds for proximity).

| Field            | Type                 | Required | Description                                           |
| ---------------- | -------------------- | -------- | ----------------------------------------------------- |
| `type`           | `"compute_transfer"` | yes      | Message discriminator                                 |
| `request_id`     | `u64`                | yes      | Client-assigned correlation ID                        |
| `chief`          | `StateVector`        | yes      | Chief ECI state vector                                |
| `deputy`         | `StateVector`        | yes      | Deputy ECI state vector (same epoch as chief)         |
| `perch`          | `PerchGeometry`      | yes      | Perch geometry for Lambert arrival                    |
| `proximity`      | `ProximityConfig`    | yes      | Far-field vs. proximity classification thresholds     |
| `lambert_tof_s`  | `f64`                | yes      | Lambert time-of-flight (seconds)                      |
| `lambert_config` | `LambertConfig`      | yes      | Lambert solver configuration (direction, revolutions) |

Response: `transfer_result`

### extract_drag

Extract differential drag rates via nyx full-physics propagation. Background job (~3 seconds). Short-circuits to zero-rate `DragConfig` when spacecraft configs are identical (no differential drag).

| Field           | Type               | Required | Description                           |
| --------------- | ------------------ | -------- | ------------------------------------- |
| `type`          | `"extract_drag"`   | yes      | Message discriminator                 |
| `request_id`    | `u64`              | yes      | Client-assigned correlation ID        |
| `chief`         | `StateVector`      | yes      | Chief ECI state vector                |
| `deputy`        | `StateVector`      | yes      | Deputy ECI state vector               |
| `chief_config`  | `SpacecraftConfig` | yes      | Chief spacecraft physical properties  |
| `deputy_config` | `SpacecraftConfig` | yes      | Deputy spacecraft physical properties |

Response: `drag_result`

### validate

Full-physics nyx validation of an analytical mission plan. Background job (seconds to minutes). Streams `progress` messages during execution. Supports optional COLA avoidance burns injected into the nyx propagation.

| Field                     | Type                  | Required | Default | Description                                                      |
| ------------------------- | --------------------- | -------- | ------- | ---------------------------------------------------------------- |
| `type`                    | `"validate"`          | yes      |         | Message discriminator                                            |
| `request_id`              | `u64`                 | yes      |         | Client-assigned correlation ID                                   |
| `mission`                 | `WaypointMission`     | yes      |         | Analytical mission to validate                                   |
| `chief`                   | `StateVector`         | yes      |         | Chief ECI state at mission start                                 |
| `deputy`                  | `StateVector`         | yes      |         | Deputy ECI state at mission start                                |
| `chief_config`            | `SpacecraftConfig`    | yes      |         | Chief spacecraft physical properties                             |
| `deputy_config`           | `SpacecraftConfig`    | yes      |         | Deputy spacecraft physical properties                            |
| `samples_per_leg`         | `u32`                 | yes      |         | Intermediate comparison samples per leg                          |
| `cola_burns`              | `ColaBurnInput[]`     | no       | `[]`    | Optional COLA avoidance burns to inject during validation        |
| `analytical_cola`         | `AvoidanceManeuver[]` | no       | `[]`    | Analytical COLA avoidance maneuvers for effectiveness comparison |
| `cola_target_distance_km` | `f64 \| null`         | no       | `null`  | Target COLA separation threshold (km) from ColaConfig            |

`ColaBurnInput` fields:

| Field         | Type        | Description                                    |
| ------------- | ----------- | ---------------------------------------------- |
| `leg_index`   | `usize`     | Index of the mission leg this burn applies to  |
| `elapsed_s`   | `f64`       | Time from leg departure to COLA burn (seconds) |
| `dv_ric_km_s` | `[R, I, C]` | Delta-v in RIC frame (km/s)                    |

The `validation_result` response includes a `cola_effectiveness` array (empty when no COLA burns were injected) with these fields per entry:

| Field                           | Type           | Description                                                      |
| ------------------------------- | -------------- | ---------------------------------------------------------------- |
| `leg_index`                     | `usize`        | Mission leg index                                                |
| `analytical_post_cola_poca_km`  | `f64 \| null`  | Analytical post-avoidance POCA (km), from `AvoidanceManeuver`    |
| `nyx_post_cola_min_distance_km` | `f64`          | Minimum chief-deputy distance in post-COLA segment from nyx (km) |
| `target_distance_km`            | `f64 \| null`  | Target separation threshold (km), from `ColaConfig`              |
| `threshold_met`                 | `bool \| null` | Whether nyx min distance meets the target (`null` if no target)  |

Response: `progress` (0 or more), then `validation_result`

### run_mc

Monte Carlo ensemble with nyx full-physics propagation. Background job (minutes to hours). Streams `progress` messages during execution.

| Field               | Type                      | Required | Default | Description                                                             |
| ------------------- | ------------------------- | -------- | ------- | ----------------------------------------------------------------------- |
| `type`              | `"run_mc"`                | yes      |         | Message discriminator                                                   |
| `request_id`        | `u64`                     | yes      |         | Client-assigned correlation ID                                          |
| `mission`           | `WaypointMission`         | yes      |         | Nominal mission plan (reference for dispersions)                        |
| `chief`             | `StateVector`             | yes      |         | Chief ECI state at mission start                                        |
| `deputy`            | `StateVector`             | yes      |         | Deputy ECI state at mission start                                       |
| `chief_config`      | `SpacecraftConfig`        | yes      |         | Chief spacecraft physical properties                                    |
| `deputy_config`     | `SpacecraftConfig`        | yes      |         | Deputy spacecraft physical properties                                   |
| `mission_config`    | `MissionConfig`           | yes      |         | Mission targeting configuration (for closed-loop re-targeting)          |
| `propagator`        | `PropagatorChoice`        | yes      |         | `"j2"` or `{ "j2_drag": { "drag": { ... } } }` (externally-tagged enum) |
| `drag_config`       | `DragConfig`              | no       | `null`  | Drag config override (replaces drag embedded in `j2_drag` propagator)   |
| `monte_carlo`       | `MonteCarloConfig`        | yes      |         | MC configuration (samples, dispersions, mode, seed, trajectory_steps)   |
| `covariance_report` | `MissionCovarianceReport` | no       | `null`  | Optional covariance predictions for cross-check                         |

Response: `progress` (0 or more), then `monte_carlo_result`

### cancel

Cancel the active background job. If no job is active, the server still acknowledges with `cancelled`.

| Field        | Type       | Required | Description                    |
| ------------ | ---------- | -------- | ------------------------------ |
| `type`       | `"cancel"` | yes      | Message discriminator          |
| `request_id` | `u64`      | yes      | Client-assigned correlation ID |

Response: `cancelled`

## Server Messages

### transfer_result

Lambert transfer result (classification + perch states). Response to `compute_transfer`.

| Field        | Type                | Description                                                                                        |
| ------------ | ------------------- | -------------------------------------------------------------------------------------------------- |
| `type`       | `"transfer_result"` | Message discriminator                                                                              |
| `request_id` | `u64`               | Correlation ID from the client request                                                             |
| `result`     | `TransferResult`    | Mission plan (phase, transfer, perch ROE, chief elements), perch states, arrival epoch, Lambert Δv |

`TransferResult` fields:

| Field             | Type          | Description                                                                  |
| ----------------- | ------------- | ---------------------------------------------------------------------------- |
| `plan`            | `MissionPlan` | Phase classification, Lambert transfer, perch ROE, chief elements at arrival |
| `perch_chief`     | `StateVector` | Chief ECI state at Lambert arrival (or original if proximity)                |
| `perch_deputy`    | `StateVector` | Deputy ECI state at perch (or original if proximity)                         |
| `arrival_epoch`   | `string`      | ISO 8601 epoch at Lambert arrival                                            |
| `lambert_dv_km_s` | `f64`         | Lambert Δv magnitude (0.0 if proximity)                                      |

### drag_result

Extracted differential drag configuration. Response to `extract_drag`.

| Field        | Type            | Description                            |
| ------------ | --------------- | -------------------------------------- |
| `type`       | `"drag_result"` | Message discriminator                  |
| `request_id` | `u64`           | Correlation ID from the client request |
| `drag`       | `DragConfig`    | DMF differential drag rates            |

### validation_result

Full-physics validation report. Response to `validate`.

| Field        | Type                  | Description                                       |
| ------------ | --------------------- | ------------------------------------------------- |
| `type`       | `"validation_result"` | Message discriminator                             |
| `request_id` | `u64`                 | Correlation ID from the client request            |
| `report`     | `ValidationReport`    | Per-leg comparison and aggregate error statistics |

### monte_carlo_result

Monte Carlo ensemble report. Response to `run_mc`.

| Field        | Type                   | Description                                |
| ------------ | ---------------------- | ------------------------------------------ |
| `type`       | `"monte_carlo_result"` | Message discriminator                      |
| `request_id` | `u64`                  | Correlation ID from the client request     |
| `report`     | `MonteCarloReport`     | Ensemble statistics and per-sample results |

### progress

Streamed during long-running operations (`validate`, `run_mc`). Fire-and-forget -- progress updates may be dropped if the internal buffer is full.

| Field        | Type                | Description                                                 |
| ------------ | ------------------- | ----------------------------------------------------------- |
| `type`       | `"progress"`        | Message discriminator                                       |
| `request_id` | `u64`               | Correlation ID of the operation in progress                 |
| `phase`      | `string`            | `"validate"` or `"mc"`                                      |
| `detail`     | `string` or omitted | Human-readable detail (e.g., `"Running nyx validation..."`) |
| `fraction`   | `number` or omitted | Completion fraction, 0.0 to 1.0                             |

### error

Error response for any operation.

| Field        | Type                | Description                                                              |
| ------------ | ------------------- | ------------------------------------------------------------------------ |
| `type`       | `"error"`           | Message discriminator                                                    |
| `request_id` | `u64` or omitted    | Correlation ID (omitted for connection-level errors like malformed JSON) |
| `code`       | `string`            | Machine-readable error code (see below)                                  |
| `message`    | `string`            | Human-readable error message                                             |
| `detail`     | `object` or omitted | Per-variant diagnostic fields (omitted when null)                        |

### cancelled

Confirmation that a background job was cancelled. Response to `cancel`.

| Field        | Type          | Description                               |
| ------------ | ------------- | ----------------------------------------- |
| `type`       | `"cancelled"` | Message discriminator                     |
| `request_id` | `u64`         | Correlation ID of the cancelled operation |

### heartbeat

Sent every 30 seconds while a background job is running. The counter resets to 0 when the job completes.

| Field  | Type          | Description                                   |
| ------ | ------------- | --------------------------------------------- |
| `type` | `"heartbeat"` | Message discriminator                         |
| `seq`  | `u64`         | Monotonic counter (increments each heartbeat) |

## Error Codes

All error codes are lowercase `snake_case` strings.

| Code                | Description                                               |
| ------------------- | --------------------------------------------------------- |
| `lambert_failure`   | Lambert solver failure (convergence, degenerate geometry) |
| `nyx_bridge_error`  | Nyx bridge error (almanac, dynamics, propagation)         |
| `validation_error`  | Full-physics validation error                             |
| `monte_carlo_error` | Monte Carlo execution error                               |
| `invalid_input`     | Malformed JSON, missing fields, bad values                |
| `cancelled`         | Operation was cancelled by the client                     |

The `detail` field carries per-error diagnostic data. Examples:

`lambert_failure` (convergence):

```json
{
  "reason": "convergence_failure",
  "details": "Izzo solver did not converge after 50 iterations"
}
```

`lambert_failure` (invalid input):

```json
{
  "reason": "invalid_input",
  "details": "Non-positive time of flight: -100.0 s"
}
```

`invalid_input` (malformed JSON):

```json
{
  "reason": "malformed_json",
  "detail": "missing field `chief` at line 1 column 42"
}
```

`invalid_input` (pipeline failure):

```json
{
  "reason": "pipeline_failure",
  "detail": "Classification failed: invalid orbit elements"
}
```

## Background Job Behavior

- Only one background job runs at a time (`extract_drag`, `validate`, `run_mc`).
- Sending a new background request automatically cancels the active job.
- Cancellation is cooperative: handlers check the cancel flag before starting heavy computation.
- `heartbeat` messages are sent every 30 seconds during background jobs to keep the connection alive through proxies.
- When the connection closes, any active background job is cancelled.
- `compute_transfer` is synchronous (not a background job) -- it replies immediately.

## Typical Flows

### Far-Field Mission

The browser handles classification, waypoint planning, and all analytical operations via WASM. The server is called only for nyx-dependent steps.

```
Browser (WASM)                              Server
──────────────                              ──────
classify_separation() → far_field
                                            compute_transfer → transfer_result (~100ms)
                                            extract_drag → drag_result (~3s)
plan_waypoint_mission()
compute_safety_analysis()
compute_mission_covariance()
compute_mission_eclipse()
  ... user iterates waypoints ...
                                            validate → progress* + validation_result
                                            run_mc → progress* + monte_carlo_result
```

### Proximity Mission

```
Browser (WASM)                              Server
──────────────                              ──────
classify_separation() → proximity
                                            extract_drag → drag_result (~3s)
plan_waypoint_mission()
compute_safety_analysis()
  ... user iterates waypoints ...
                                            validate → progress* + validation_result
                                            run_mc → progress* + monte_carlo_result
```

## Examples

### compute_transfer

Request:

```json
{
  "type": "compute_transfer",
  "request_id": 1,
  "chief": {
    "epoch": "2024-01-01T00:00:00 UTC",
    "position_eci_km": [5876.261, 3392.661, 0.0],
    "velocity_eci_km_s": [-2.380512, 4.123167, 6.006917]
  },
  "deputy": {
    "epoch": "2024-01-01T00:00:00 UTC",
    "position_eci_km": [6876.261, 2392.661, 500.0],
    "velocity_eci_km_s": [-2.180512, 4.323167, 5.806917]
  },
  "perch": { "v_bar": { "along_track_km": 2.0 } },
  "proximity": { "roe_threshold": 0.005 },
  "lambert_tof_s": 3600.0,
  "lambert_config": { "direction": "auto", "revolutions": 0 }
}
```

Response:

```json
{
  "type": "transfer_result",
  "request_id": 1,
  "result": {
    "plan": {
      "phase": { "far_field": { "...": "..." } },
      "transfer": { "...": "..." },
      "perch_roe": { "...": "..." },
      "chief_at_arrival": { "...": "..." }
    },
    "perch_chief": { "...": "..." },
    "perch_deputy": { "...": "..." },
    "arrival_epoch": "2024-01-01T01:00:00 UTC",
    "lambert_dv_km_s": 0.123
  }
}
```

### extract_drag

Request:

```json
{
  "type": "extract_drag",
  "request_id": 2,
  "chief": {
    "epoch": "2024-01-01T00:00:00 UTC",
    "position_eci_km": [5876.261, 3392.661, 0.0],
    "velocity_eci_km_s": [-2.380512, 4.123167, 6.006917]
  },
  "deputy": {
    "epoch": "2024-01-01T00:00:00 UTC",
    "position_eci_km": [5876.561, 3392.261, 0.3],
    "velocity_eci_km_s": [-2.380612, 4.123067, 6.006817]
  },
  "chief_config": {
    "dry_mass_kg": 500.0,
    "drag_area_m2": 1.0,
    "coeff_drag": 2.2,
    "srp_area_m2": 1.0,
    "coeff_reflectivity": 1.5
  },
  "deputy_config": {
    "dry_mass_kg": 12.0,
    "drag_area_m2": 0.06,
    "coeff_drag": 2.2,
    "srp_area_m2": 0.06,
    "coeff_reflectivity": 1.5
  }
}
```

Response:

```json
{
  "type": "drag_result",
  "request_id": 2,
  "drag": {
    "da_dot": -1.23e-10,
    "dex_dot": 4.56e-12,
    "dey_dot": -7.89e-13
  }
}
```

### validate (with progress)

Request:

```json
{
  "type": "validate",
  "request_id": 3,
  "mission": { "...": "WaypointMission" },
  "chief": { "...": "StateVector" },
  "deputy": { "...": "StateVector" },
  "chief_config": { "...": "SpacecraftConfig" },
  "deputy_config": { "...": "SpacecraftConfig" },
  "samples_per_leg": 50,
  "cola_burns": [],
  "analytical_cola": [],
  "cola_target_distance_km": null
}
```

Response stream:

```json
{"type": "progress", "request_id": 3, "phase": "validate", "detail": "Starting validation...", "fraction": 0.0}
{"type": "progress", "request_id": 3, "phase": "validate", "detail": "Running nyx validation...", "fraction": 0.1}
{"type": "heartbeat", "seq": 1}
{"type": "progress", "request_id": 3, "phase": "validate", "detail": "Validation complete", "fraction": 1.0}
{"type": "validation_result", "request_id": 3, "report": { "...": "ValidationReport" }}
```

### cancel

Request:

```json
{
  "type": "cancel",
  "request_id": 3
}
```

Response:

```json
{
  "type": "cancelled",
  "request_id": 3
}
```

### error

Malformed JSON (no `request_id` available):

```json
{
  "type": "error",
  "code": "invalid_input",
  "message": "Malformed JSON: missing field `type` at line 1 column 2",
  "detail": {
    "reason": "malformed_json",
    "detail": "missing field `type` at line 1 column 2"
  }
}
```

Lambert failure:

```json
{
  "type": "error",
  "request_id": 1,
  "code": "lambert_failure",
  "message": "Lambert solver error: Izzo solver did not converge",
  "detail": {
    "reason": "convergence_failure",
    "details": "Izzo solver did not converge after 50 iterations"
  }
}
```

### progress + heartbeat

During a long validation or MC run, the client may see interleaved progress and heartbeat messages:

```json
{"type": "progress", "request_id": 3, "phase": "validate", "detail": "Starting validation...", "fraction": 0.0}
{"type": "progress", "request_id": 3, "phase": "validate", "detail": "Running nyx validation...", "fraction": 0.1}
{"type": "heartbeat", "seq": 1}
{"type": "heartbeat", "seq": 2}
{"type": "progress", "request_id": 3, "phase": "validate", "detail": "Validation complete", "fraction": 1.0}
{"type": "validation_result", "request_id": 3, "report": { "...": "..." }}
```
