# RPO WebSocket API

The `rpo-api` crate provides a WebSocket server for the RPO mission planner. It wraps the same `rpo-core` pipeline used by `rpo-cli` behind a stateless, request/response WebSocket protocol.

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

All messages are JSON text frames with a `"type"` discriminator tag (serde internally-tagged enum).

- Non-text WebSocket frames receive an `Error` response with `code: "invalid_input"`.
- The server is stateless: the client sends the full `MissionDefinition` (alias for `PipelineInput`) with every request.
- Client-assigned `request_id` (u64) correlates requests with responses.

## Client Messages

### Classify

Classify chief/deputy separation as `Proximity` or `FarField`. Returns immediately.

| Field        | Type                | Required | Description                              |
| ------------ | ------------------- | -------- | ---------------------------------------- |
| `type`       | `"Classify"`        | yes      | Message discriminator                    |
| `request_id` | `u64`               | yes      | Client-assigned correlation ID           |
| `mission`    | `MissionDefinition` | yes      | Only `chief`, `deputy`, `proximity` used |

Response: `ClassifyResult`

### PlanMission

Full mission plan: classify, Lambert transfer, perch, waypoint targeting, safety, covariance, eclipse. Returns immediately.

| Field        | Type                | Required | Description                    |
| ------------ | ------------------- | -------- | ------------------------------ |
| `type`       | `"PlanMission"`     | yes      | Message discriminator          |
| `request_id` | `u64`               | yes      | Client-assigned correlation ID |
| `mission`    | `MissionDefinition` | yes      | Full mission definition        |

Response: `MissionResult`

### MoveWaypoint

Replan from a moved waypoint, keeping earlier legs cached. Returns immediately.

| Field            | Type                | Required | Description                                   |
| ---------------- | ------------------- | -------- | --------------------------------------------- |
| `type`           | `"MoveWaypoint"`    | yes      | Message discriminator                         |
| `request_id`     | `u64`               | yes      | Client-assigned correlation ID                |
| `modified_index` | `usize`             | yes      | Index of the waypoint that changed            |
| `mission`        | `MissionDefinition` | yes      | Full mission definition with updated waypoint |

Response: `MissionResult`

### UpdateConfig

Re-solve all waypoints with new config or propagator settings. Returns immediately.

| Field        | Type                | Required | Description                                 |
| ------------ | ------------------- | -------- | ------------------------------------------- |
| `type`       | `"UpdateConfig"`    | yes      | Message discriminator                       |
| `request_id` | `u64`               | yes      | Client-assigned correlation ID              |
| `mission`    | `MissionDefinition` | yes      | Full mission definition with updated config |

Response: `MissionResult`

### ExtractDrag

Extract differential drag rates via nyx full-physics propagation. Runs in background (~3 seconds). Requires `chief_config` and `deputy_config` in the mission definition.

| Field        | Type                | Required | Description                            |
| ------------ | ------------------- | -------- | -------------------------------------- |
| `type`       | `"ExtractDrag"`     | yes      | Message discriminator                  |
| `request_id` | `u64`               | yes      | Client-assigned correlation ID         |
| `mission`    | `MissionDefinition` | yes      | Needs `chief_config` + `deputy_config` |

Response: `DragResult`

### Validate

Per-leg nyx high-fidelity validation with progress streaming. Runs in background (seconds to minutes depending on leg count).

| Field             | Type                | Required | Default | Description                              |
| ----------------- | ------------------- | -------- | ------- | ---------------------------------------- |
| `type`            | `"Validate"`        | yes      |         | Message discriminator                    |
| `request_id`      | `u64`               | yes      |         | Client-assigned correlation ID           |
| `mission`         | `MissionDefinition` | yes      |         | Full mission definition                  |
| `samples_per_leg` | `u32` or `null`     | no       | `50`    | Number of comparison points per leg      |
| `auto_drag`       | `bool`              | no       | `false` | Auto-derive drag rates before validation |

Response: `Progress` (0 or more), then `ValidationResult`

### RunMC

Full-physics Monte Carlo ensemble analysis. Runs in background (minutes). Requires `monte_carlo` config in the mission definition.

| Field        | Type                | Required | Default | Description                      |
| ------------ | ------------------- | -------- | ------- | -------------------------------- |
| `type`       | `"RunMC"`           | yes      |         | Message discriminator            |
| `request_id` | `u64`               | yes      |         | Client-assigned correlation ID   |
| `mission`    | `MissionDefinition` | yes      |         | Full mission definition          |
| `auto_drag`  | `bool`              | no       | `false` | Auto-derive drag rates before MC |

Response: `Progress` (0 or more), then `MonteCarloResult`

### Cancel

Cancel a running background operation. If no background job is active, no response is sent.

| Field        | Type            | Required | Description                                       |
| ------------ | --------------- | -------- | ------------------------------------------------- |
| `type`       | `"Cancel"`      | yes      | Message discriminator                             |
| `request_id` | `u64` or `null` | no       | ID to cancel; cancels current job if null/omitted |

Response: `Cancelled` (if a job was active)

## Server Messages

### ClassifyResult

| Field        | Type               | Description                                                    |
| ------------ | ------------------ | -------------------------------------------------------------- |
| `type`       | `"ClassifyResult"` | Message discriminator                                          |
| `request_id` | `u64`              | Correlation ID from the client request                         |
| `phase`      | `MissionPhase`     | `Proximity { roe, chief_elements, ... }` or `FarField { ... }` |

### MissionResult

| Field        | Type              | Description                            |
| ------------ | ----------------- | -------------------------------------- |
| `type`       | `"MissionResult"` | Message discriminator                  |
| `request_id` | `u64`             | Correlation ID from the client request |
| `result`     | `PipelineOutput`  | Full mission result payload            |

### DragResult

| Field        | Type           | Description                            |
| ------------ | -------------- | -------------------------------------- |
| `type`       | `"DragResult"` | Message discriminator                  |
| `request_id` | `u64`          | Correlation ID from the client request |
| `drag`       | `DragConfig`   | Extracted differential drag rates      |

### Progress

Sent zero or more times during `Validate` and `RunMC` operations.

| Field        | Type               | Description                                          |
| ------------ | ------------------ | ---------------------------------------------------- |
| `type`       | `"Progress"`       | Message discriminator                                |
| `request_id` | `u64`              | Correlation ID of the operation in progress          |
| `phase`      | `string`           | `"validate"` or `"mc"`                               |
| `detail`     | `string` or `null` | Human-readable detail (e.g., `"Validating leg 3/5"`) |
| `fraction`   | `number` or `null` | Completion fraction, 0.0 to 1.0                      |

### ValidationResult

| Field        | Type                 | Description                            |
| ------------ | -------------------- | -------------------------------------- |
| `type`       | `"ValidationResult"` | Message discriminator                  |
| `request_id` | `u64`                | Correlation ID from the client request |
| `report`     | `ValidationReport`   | Full per-leg validation report         |

### MonteCarloResult

| Field        | Type                 | Description                            |
| ------------ | -------------------- | -------------------------------------- |
| `type`       | `"MonteCarloResult"` | Message discriminator                  |
| `request_id` | `u64`                | Correlation ID from the client request |
| `report`     | `MonteCarloReport`   | Full Monte Carlo ensemble report       |

### Cancelled

| Field        | Type          | Description                               |
| ------------ | ------------- | ----------------------------------------- |
| `type`       | `"Cancelled"` | Message discriminator                     |
| `request_id` | `u64`         | Correlation ID of the cancelled operation |

### Heartbeat

Sent every 30 seconds while a background job is running. The counter resets to 0 when the job completes.

| Field  | Type          | Description                                   |
| ------ | ------------- | --------------------------------------------- |
| `type` | `"Heartbeat"` | Message discriminator                         |
| `seq`  | `u64`         | Monotonic counter (increments each heartbeat) |

### Error

| Field        | Type                | Description                                       |
| ------------ | ------------------- | ------------------------------------------------- |
| `type`       | `"Error"`           | Message discriminator                             |
| `request_id` | `u64` or `null`     | Correlation ID (null if not tied to a request)    |
| `code`       | `string`            | Machine-readable error code (see below)           |
| `message`    | `string`            | Human-readable error message                      |
| `detail`     | `object` or omitted | Per-variant diagnostic fields (omitted when null) |

## Error Codes

All error codes are lowercase snake_case strings.

| Code                    | Description                                       |
| ----------------------- | ------------------------------------------------- |
| `targeting_convergence` | Newton-Raphson targeting solver did not converge  |
| `lambert_failure`       | Lambert solver failure                            |
| `propagation_error`     | Propagation error                                 |
| `validation_error`      | Nyx validation error                              |
| `monte_carlo_error`     | Monte Carlo error                                 |
| `nyx_bridge_error`      | Nyx bridge error (almanac, dynamics, propagation) |
| `covariance_error`      | Covariance propagation error                      |
| `invalid_input`         | Malformed JSON, missing fields, bad values        |
| `mission_error`         | General mission planning error                    |
| `cancelled`             | Operation was cancelled                           |

The `detail` field carries per-error diagnostic data. For example, a `targeting_convergence` error includes:

```json
{
  "final_error_km": 0.0015,
  "iterations": 100
}
```

An `invalid_input` error for a missing field includes:

```json
{
  "reason": "missing_field",
  "field": "chief_config",
  "context": "validate"
}
```

## Background Job Behavior

- Only one background job runs at a time (`ExtractDrag`, `Validate`, `RunMC`).
- Sending a new background request automatically cancels the active job.
- Cancellation is cooperative: the server checks the cancel flag between phases (legs, MC samples).
- `Heartbeat` messages are sent every 30 seconds during background jobs to keep the connection alive.
- When the connection closes, any active background job is cancelled.

## MissionDefinition

`MissionDefinition` is a type alias for `PipelineInput` from `rpo-core`. See `docs/schema/pipeline-input.schema.json` for the full JSON Schema.

## Examples

### Classify

Request:

```json
{
  "type": "Classify",
  "request_id": 1,
  "mission": {
    "chief": {
      "epoch": "2024-01-01T00:00:00 UTC",
      "position_eci_km": [5876.261, 3392.661, 0.0],
      "velocity_eci_km_s": [-2.380512, 4.123167, 6.006917]
    },
    "deputy": {
      "epoch": "2024-01-01T00:00:00 UTC",
      "position_eci_km": [5199.839421, 4281.648523, 1398.070066],
      "velocity_eci_km_s": [-3.993103, 2.970313, 5.76454]
    },
    "waypoints": []
  }
}
```

Response (far-field):

```json
{
  "type": "ClassifyResult",
  "request_id": 1,
  "phase": {
    "far_field": {
      "chief_elements": {
        "a_km": 6886.6,
        "e": 0.001,
        "i_rad": 1.7001,
        "raan_rad": 0.0,
        "aop_rad": 0.0,
        "mean_anomaly_rad": 0.5236
      },
      "deputy_elements": { "...": "..." },
      "separation_km": 1532.4,
      "delta_r_over_r": 0.2225
    }
  }
}
```

### PlanMission

Request:

```json
{
  "type": "PlanMission",
  "request_id": 2,
  "mission": {
    "chief": {
      "epoch": "2024-01-01T00:00:00 UTC",
      "position_eci_km": [5876.261, 3392.661, 0.0],
      "velocity_eci_km_s": [-2.380512, 4.123167, 6.006917]
    },
    "deputy": {
      "epoch": "2024-01-01T00:00:00 UTC",
      "position_eci_km": [5199.839421, 4281.648523, 1398.070066],
      "velocity_eci_km_s": [-3.993103, 2.970313, 5.76454]
    },
    "waypoints": [
      {
        "label": "Approach to 2 km",
        "position_ric_km": [0.5, 2.0, 0.5],
        "velocity_ric_km_s": [0.0, 0.0, 0.0],
        "tof_s": 4200.0
      },
      {
        "label": "Close to 0.5 km",
        "position_ric_km": [0.5, 0.5, 0.5],
        "tof_s": 4200.0
      }
    ],
    "config": {
      "safety": {
        "min_ei_separation_km": 0.1,
        "min_distance_3d_km": 0.05
      }
    }
  }
}
```

Response:

```json
{
  "type": "MissionResult",
  "request_id": 2,
  "result": {
    "phase": { "far_field": { "...": "..." } },
    "transfer": { "...": "..." },
    "perch_roe": { "da": 0.0, "dlambda": 0.000726, "...": "..." },
    "mission": {
      "legs": ["..."],
      "total_dv_km_s": 0.0034,
      "total_duration_s": 8400.0
    },
    "total_dv_km_s": 1.234,
    "total_duration_s": 12000.0
  }
}
```

### Cancel

Request:

```json
{
  "type": "Cancel",
  "request_id": null
}
```

Response (if a job was active):

```json
{
  "type": "Cancelled",
  "request_id": 3
}
```

### Error

```json
{
  "type": "Error",
  "request_id": 2,
  "code": "targeting_convergence",
  "message": "targeting did not converge after 100 iterations (error: 0.0015 km)",
  "detail": {
    "final_error_km": 0.0015,
    "iterations": 100
  }
}
```

### Progress + Heartbeat

During a validation run, the client may see interleaved Progress and Heartbeat messages:

```json
{"type": "Progress", "request_id": 3, "phase": "validate", "detail": "Validating leg 1/5", "fraction": 0.2}
{"type": "Progress", "request_id": 3, "phase": "validate", "detail": "Validating leg 2/5", "fraction": 0.4}
{"type": "Heartbeat", "seq": 1}
{"type": "Progress", "request_id": 3, "phase": "validate", "detail": "Validating leg 3/5", "fraction": 0.6}
```
