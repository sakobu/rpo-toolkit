# RPO WebSocket API

The `rpo-api` crate provides a WebSocket server for the RPO mission planner. It wraps the same `rpo-core` pipeline used by `rpo-cli` behind a **stateful, session-based** WebSocket protocol.

## Session Model

Each WebSocket connection has a server-side session. The client sets state incrementally (chief, deputy, waypoints, config) and the server computes results from the stored session. No need to resend the full mission definition with every request.

### 5-Tier State

```
Tier 0 - Base inputs:     chief, deputy
Tier 1 - Spacecraft:      chief_config, deputy_config (default: cubesat_6u)
Tier 2 - Transfer design: transfer, perch, lambert_tof_s, lambert_config, proximity, drag_config
Tier 3 - Mission plan:    waypoints, config, propagator, safety_requirements, baseline/enriched missions, selected_variant
Tier 4 - Overlays:        navigation_accuracy, maneuver_uncertainty, monte_carlo_config
```

### Invalidation Rules

Changing an upstream tier clears downstream computed results so that stale data is never served.

| Setter                                        | Clears                                            |
| --------------------------------------------- | ------------------------------------------------- |
| `set_states(chief, deputy)`                   | transfer, drag_config, missions                   |
| `set_spacecraft(chief_config, deputy_config)` | drag_config only                                  |
| `compute_transfer(perch, tof, config)`        | missions (via store_transfer)                     |
| `store_drag(drag)`                            | nothing (propagator toggle is explicit)           |
| `set_waypoints(waypoints)`                    | missions, selected_variant                        |
| `set_safety_requirements(requirements)`       | missions, selected_variant                        |
| `update_config(config)`                       | missions (if config/propagator/proximity changed) |
| `reset()`                                     | everything                                        |

Tier 4 overlays (`navigation_accuracy`, `maneuver_uncertainty`, `monte_carlo_config`) never trigger invalidation or replan.

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

All messages are JSON text frames with a `"type"` discriminator tag (serde internally-tagged enum, snake_case).

- Non-text WebSocket frames receive an `error` response with `code: "invalid_input"`.
- The server is **stateful**: the client mutates session state incrementally via setter messages and queries computed results via action messages.
- Client-assigned `request_id` (u64) correlates requests with responses.

## Client Messages

### set_states

Upload chief and deputy ECI states. Always the first message in a session. Invalidates transfer, drag_config, and mission.

| Field        | Type           | Required | Description                    |
| ------------ | -------------- | -------- | ------------------------------ |
| `type`       | `"set_states"` | yes      | Message discriminator          |
| `request_id` | `u64`          | yes      | Client-assigned correlation ID |
| `chief`      | `StateVector`  | yes      | Chief spacecraft ECI state     |
| `deputy`     | `StateVector`  | yes      | Deputy spacecraft ECI state    |

Response: `state_updated`

### set_spacecraft

Set spacecraft preset or custom configuration for drag extraction, validation, and MC. Invalidates drag_config only. Defaults to `cubesat_6u` if never called.

| Field           | Type               | Required | Description                             |
| --------------- | ------------------ | -------- | --------------------------------------- |
| `type`          | `"set_spacecraft"` | yes      | Message discriminator                   |
| `request_id`    | `u64`              | yes      | Client-assigned correlation ID          |
| `chief_config`  | `SpacecraftChoice` | no       | Chief spacecraft (None = keep current)  |
| `deputy_config` | `SpacecraftChoice` | no       | Deputy spacecraft (None = keep current) |

`SpacecraftChoice` is either a preset string (`"cubesat_6u"`, `"servicer_500kg"`) or a custom object `{ "custom": { "dry_mass_kg": ..., "drag_area_m2": ..., "coeff_drag": ..., "srp_area_m2": ..., "coeff_reflectivity": ... } }`.

Response: `state_updated`

### classify

Classify chief/deputy separation as `proximity` or `far_field`. Reads states from the session. Returns immediately.

| Field        | Type         | Required | Description                    |
| ------------ | ------------ | -------- | ------------------------------ |
| `type`       | `"classify"` | yes      | Message discriminator          |
| `request_id` | `u64`        | yes      | Client-assigned correlation ID |

Requires: `set_states` must have been called.

Response: `classify_result`

### compute_transfer

Compute Lambert transfer and store result in session. Optional fields override session defaults (perch, TOF, Lambert config). Far-field only. Returns immediately.

| Field            | Type                 | Required | Description                                   |
| ---------------- | -------------------- | -------- | --------------------------------------------- |
| `type`           | `"compute_transfer"` | yes      | Message discriminator                         |
| `request_id`     | `u64`                | yes      | Client-assigned correlation ID                |
| `perch`          | `PerchGeometry`      | no       | Override perch geometry (None = keep session) |
| `lambert_tof_s`  | `f64`                | no       | Override Lambert TOF (None = keep session)    |
| `lambert_config` | `LambertConfig`      | no       | Override Lambert config (None = keep session) |

Requires: `set_states` must have been called.

Response: `transfer_computed`

### extract_drag

Extract differential drag rates via nyx full-physics propagation. Background job (~3 seconds). Uses perch states if a transfer exists, otherwise original states. Short-circuits with zero-rate if spacecraft configs are identical.

| Field        | Type             | Required | Description                    |
| ------------ | ---------------- | -------- | ------------------------------ |
| `type`       | `"extract_drag"` | yes      | Message discriminator          |
| `request_id` | `u64`            | yes      | Client-assigned correlation ID |

Requires: `set_states` must have been called.

Response: `drag_result`

### set_waypoints

Set waypoints and plan/replan the mission. For far-field, requires a stored transfer result. For proximity, auto-computes the transfer internally. Returns immediately.

| Field          | Type              | Required | Default | Description                                             |
| -------------- | ----------------- | -------- | ------- | ------------------------------------------------------- |
| `type`         | `"set_waypoints"` | yes      |         | Message discriminator                                   |
| `request_id`   | `u64`             | yes      |         | Client-assigned correlation ID                          |
| `waypoints`    | `WaypointInput[]` | yes      |         | Waypoint targets in RIC frame                           |
| `changed_from` | `usize` or `null` | no       | `null`  | Index of first modified waypoint for incremental replan |

Response: `plan_result`

### update_config

Update solver configuration, propagator, proximity thresholds, and/or analysis overlays. Replans the mission if config, propagator, or proximity changed and a mission exists.

| Field                  | Type                  | Required | Description                                |
| ---------------------- | --------------------- | -------- | ------------------------------------------ |
| `type`                 | `"update_config"`     | yes      | Message discriminator                      |
| `request_id`           | `u64`                 | yes      | Client-assigned correlation ID             |
| `config`               | `MissionConfig`       | no       | Solver config (None = keep current)        |
| `propagator`           | `PropagatorToggle`    | no       | Propagator selection (None = keep current) |
| `proximity`            | `ProximityConfig`     | no       | Classification thresholds (None = keep)    |
| `navigation_accuracy`  | `NavigationAccuracy`  | no       | For covariance (None = keep)               |
| `maneuver_uncertainty` | `ManeuverUncertainty` | no       | For covariance (None = keep)               |

Response: `state_updated` (if only overlays changed) or `plan_result` (if replanned)

### get_trajectory

Fetch trajectory data for 3D visualization. On-demand, reads from stored mission.

| Field        | Type               | Required | Default | Description                                 |
| ------------ | ------------------ | -------- | ------- | ------------------------------------------- |
| `type`       | `"get_trajectory"` | yes      |         | Message discriminator                       |
| `request_id` | `u64`              | yes      |         | Client-assigned correlation ID              |
| `legs`       | `usize[]`          | no       | all     | Leg indices to include (None = all legs)    |
| `max_points` | `u32`              | no       | full    | Max points per leg (None = full resolution) |

Requires: `set_waypoints` must have been called (mission must exist).

Response: `trajectory_data`

### get_covariance

Fetch covariance propagation report. On-demand computation.

| Field        | Type               | Required | Description                    |
| ------------ | ------------------ | -------- | ------------------------------ |
| `type`       | `"get_covariance"` | yes      | Message discriminator          |
| `request_id` | `u64`              | yes      | Client-assigned correlation ID |

Requires: mission must exist and `navigation_accuracy` must be set.

Response: `covariance_data`

### get_eclipse

Fetch eclipse data for transfer and/or mission phases. On-demand.

| Field        | Type            | Required | Description                    |
| ------------ | --------------- | -------- | ------------------------------ |
| `type`       | `"get_eclipse"` | yes      | Message discriminator          |
| `request_id` | `u64`           | yes      | Client-assigned correlation ID |

Requires: `set_states` must have been called (transfer and/or mission should exist for meaningful data).

Response: `eclipse_data`

### get_free_drift_trajectory

Fetch free-drift (abort-case) trajectory and safety data. On-demand, computed from stored mission. For each requested leg, propagates from the pre-departure ROE (burn skipped) and returns trajectory points and safety summaries.

| Field        | Type                          | Required | Default | Description                                 |
| ------------ | ----------------------------- | -------- | ------- | ------------------------------------------- |
| `type`       | `"get_free_drift_trajectory"` | yes      |         | Message discriminator                       |
| `request_id` | `u64`                         | yes      |         | Client-assigned correlation ID              |
| `legs`       | `usize[]`                     | no       | all     | Leg indices to include (None = all legs)    |
| `max_points` | `u32`                         | no       | full    | Max points per leg (None = full resolution) |

Requires: `set_waypoints` must have been called (mission must exist).

Response: `free_drift_data`

### get_poca

Fetch Brent-refined closest-approach data. On-demand, computed from stored mission. For each leg, refines the grid-sampled minimum distance using Brent's method on range rate to find the exact time, distance, and RIC position of closest approach.

| Field        | Type         | Required | Default | Description                              |
| ------------ | ------------ | -------- | ------- | ---------------------------------------- |
| `type`       | `"get_poca"` | yes      |         | Message discriminator                    |
| `request_id` | `u64`        | yes      |         | Client-assigned correlation ID           |
| `legs`       | `usize[]`    | no       | all     | Leg indices to include (None = all legs) |

Requires: `set_waypoints` must have been called (mission must exist).

Response: `poca_data`

### run_cola

Compute collision avoidance maneuvers for POCA violations with multi-leg post-avoidance verification. On-demand, computed from stored mission. For each leg where the Brent-refined POCA distance is below the target, computes the minimum-fuel impulsive maneuver to push the closest approach beyond the threshold, then cascades the perturbation through all downstream legs to detect secondary conjunctions.

| Field        | Type         | Required | Description                     |
| ------------ | ------------ | -------- | ------------------------------- |
| `type`       | `"run_cola"` | yes      | Message discriminator           |
| `request_id` | `u64`        | yes      | Client-assigned correlation ID  |
| `config`     | `ColaConfig` | yes      | Target distance and fuel budget |

`ColaConfig` fields:

| Field                | Type  | Description                                        |
| -------------------- | ----- | -------------------------------------------------- |
| `target_distance_km` | `f64` | Desired minimum POCA distance after avoidance (km) |
| `max_dv_km_s`        | `f64` | Maximum available delta-v budget (km/s)            |

Requires: `set_waypoints` must have been called (mission must exist) and `config.safety` must be set.

Response: `cola_result`

### set_safety_requirements

Set or clear safety requirements for formation design enrichment. Invalidates the mission (enriched perch changes the departure state, requiring retargeting). When set, `set_waypoints` returns both a baseline plan (unenriched) and an enriched plan with safe e/i vectors. Use `select_plan` to choose which is active for downstream operations.

| Field          | Type                           | Required | Description                    |
| -------------- | ------------------------------ | -------- | ------------------------------ |
| `type`         | `"set_safety_requirements"`    | yes      | Message discriminator          |
| `request_id`   | `u64`                          | yes      | Client-assigned correlation ID |
| `requirements` | `SafetyRequirements` or `null` | yes      | Requirements, or null to clear |

`SafetyRequirements` fields:

| Field               | Type     | Required | Default      | Description                                   |
| ------------------- | -------- | -------- | ------------ | --------------------------------------------- |
| `min_separation_km` | `f64`    | yes      |              | Minimum R/C separation (km, D'Amico Eq. 2.22) |
| `alignment`         | `string` | no       | `"parallel"` | `"parallel"`, `"anti_parallel"`, or `"auto"`  |

Response: `state_updated`

### select_plan

Select which plan variant (baseline or enriched) is active for downstream operations (validate, MC, COLA, etc.). Default after `set_waypoints` is `baseline`.

| Field        | Type            | Required | Description                    |
| ------------ | --------------- | -------- | ------------------------------ |
| `type`       | `"select_plan"` | yes      | Message discriminator          |
| `request_id` | `u64`           | yes      | Client-assigned correlation ID |
| `variant`    | `string`        | yes      | `"baseline"` or `"enriched"`   |

Response: `plan_selected`

Returns an error if `"enriched"` is requested but no enriched plan is available (safety requirements not set or enrichment failed).

### get_formation_design

Compute formation design analysis for the current mission. On-demand, reads from stored mission, transfer, and safety requirements. Returns perch enrichment, per-waypoint enrichment (advisory), and per-leg transit e/i separation profiles.

| Field        | Type                     | Required | Description                    |
| ------------ | ------------------------ | -------- | ------------------------------ |
| `type`       | `"get_formation_design"` | yes      | Message discriminator          |
| `request_id` | `u64`                    | yes      | Client-assigned correlation ID |

Requires: mission, transfer, and `safety_requirements` must be set.

Response: `formation_design_result`

### get_safe_alternative

Compute a safe alternative ROE for a single waypoint. Stateless query -- computes on the fly from current session state. Shows what the waypoint's ROE would look like with safe e/i vectors.

| Field            | Type                     | Required | Description                    |
| ---------------- | ------------------------ | -------- | ------------------------------ |
| `type`           | `"get_safe_alternative"` | yes      | Message discriminator          |
| `request_id`     | `u64`                    | yes      | Client-assigned correlation ID |
| `waypoint_index` | `usize`                  | yes      | 0-based leg index to enrich    |

Requires: mission, transfer, and `safety_requirements` must be set.

Response: `safe_alternative_result`

### validate

Per-leg nyx high-fidelity validation with progress streaming. Background job (seconds to minutes depending on leg count).

| Field             | Type            | Required | Default | Description                              |
| ----------------- | --------------- | -------- | ------- | ---------------------------------------- |
| `type`            | `"validate"`    | yes      |         | Message discriminator                    |
| `request_id`      | `u64`           | yes      |         | Client-assigned correlation ID           |
| `samples_per_leg` | `u32` or `null` | no       | `50`    | Number of comparison points per leg      |
| `auto_drag`       | `bool`          | no       | `false` | Auto-derive drag rates before validation |

`auto_drag` is ephemeral: it extracts drag and replans temporarily without modifying session state.

Requires: mission must exist.

Response: `progress` (0 or more), then `validation_result`

### run_mc

Full-physics Monte Carlo ensemble analysis. Background job (minutes).

| Field         | Type               | Required | Default | Description                               |
| ------------- | ------------------ | -------- | ------- | ----------------------------------------- |
| `type`        | `"run_mc"`         | yes      |         | Message discriminator                     |
| `request_id`  | `u64`              | yes      |         | Client-assigned correlation ID            |
| `monte_carlo` | `MonteCarloConfig` | no       | `null`  | MC config override (None = use session's) |
| `auto_drag`   | `bool`             | no       | `false` | Auto-derive drag rates before MC          |

Requires: mission must exist and `monte_carlo` config must be provided (inline or via session).

Response: `progress` (0 or more), then `monte_carlo_result`

### get_session

Get a snapshot of the current session state for diagnostics.

| Field        | Type            | Required | Description                    |
| ------------ | --------------- | -------- | ------------------------------ |
| `type`       | `"get_session"` | yes      | Message discriminator          |
| `request_id` | `u64`           | yes      | Client-assigned correlation ID |

Response: `session_state`

### reset

Reset all session state to defaults.

| Field        | Type      | Required | Description                    |
| ------------ | --------- | -------- | ------------------------------ |
| `type`       | `"reset"` | yes      | Message discriminator          |
| `request_id` | `u64`     | yes      | Client-assigned correlation ID |

Response: `state_updated`

### cancel

Cancel a running background operation. If no background job is active, no response is sent.

| Field        | Type            | Required | Description                                       |
| ------------ | --------------- | -------- | ------------------------------------------------- |
| `type`       | `"cancel"`      | yes      | Message discriminator                             |
| `request_id` | `u64` or `null` | no       | ID to cancel; cancels current job if null/omitted |

Response: `cancelled` (if a job was active)

## Server Messages

### state_updated

Acknowledgement of a state-setting message. Reports what was set and what was invalidated.

| Field         | Type              | Description                            |
| ------------- | ----------------- | -------------------------------------- |
| `type`        | `"state_updated"` | Message discriminator                  |
| `request_id`  | `u64`             | Correlation ID from the client request |
| `updated`     | `string[]`        | Fields that were set                   |
| `invalidated` | `string[]`        | Fields that were invalidated (cleared) |

### classify_result

| Field        | Type                | Description                                                     |
| ------------ | ------------------- | --------------------------------------------------------------- |
| `type`       | `"classify_result"` | Message discriminator                                           |
| `request_id` | `u64`               | Correlation ID from the client request                          |
| `phase`      | `MissionPhase`      | `proximity { roe, chief_elements, ... }` or `far_field { ... }` |

### transfer_computed

| Field        | Type                  | Description                                                   |
| ------------ | --------------------- | ------------------------------------------------------------- |
| `type`       | `"transfer_computed"` | Message discriminator                                         |
| `request_id` | `u64`                 | Correlation ID from the client request                        |
| `result`     | `TransferResult`      | Classification, Lambert transfer, perch states, arrival epoch |

### plan_result

Returns both a baseline plan (unenriched perch) and an optional enriched plan (safe e/i vectors). When `safety_requirements` is not set, only `baseline` is present. When set and enrichment succeeds, both plans are returned with their respective formation design reports.

| Field                | Type                     | Description                                                     |
| -------------------- | ------------------------ | --------------------------------------------------------------- |
| `type`               | `"plan_result"`          | Message discriminator                                           |
| `request_id`         | `u64`                    | Correlation ID from the client request                          |
| `baseline`           | `LeanPlanResult`         | Baseline plan (unenriched geometric perch)                      |
| `enriched`           | `LeanPlanResult?`        | Enriched plan (safe e/i vectors, omitted when not available)    |
| `baseline_formation` | `FormationDesignReport?` | Baseline formation report (omitted when no safety_requirements) |
| `enriched_formation` | `FormationDesignReport?` | Enriched formation report (omitted when enrichment unavailable) |

`LeanPlanResult` fields:

| Field              | Type                  | Description                             |
| ------------------ | --------------------- | --------------------------------------- |
| `phase`            | `MissionPhase`        | Classification (proximity or far_field) |
| `transfer_summary` | `TransferSummary?`    | Lambert summary (None if proximity)     |
| `perch_roe`        | `QuasiNonsingularROE` | Idealized perch ROE target              |
| `legs`             | `LegSummary[]`        | Per-leg maneuver summaries              |
| `total_dv_km_s`    | `f64`                 | Total delta-v: Lambert + waypoint legs  |
| `total_duration_s` | `f64`                 | Total duration: Lambert + waypoint legs |
| `safety`           | `SafetyMetrics?`      | Safety metrics (if computed)            |

### drag_result

| Field        | Type            | Description                            |
| ------------ | --------------- | -------------------------------------- |
| `type`       | `"drag_result"` | Message discriminator                  |
| `request_id` | `u64`           | Correlation ID from the client request |
| `drag`       | `DragConfig`    | Extracted differential drag rates      |

### trajectory_data

On-demand trajectory points for 3D visualization.

| Field        | Type                | Description                            |
| ------------ | ------------------- | -------------------------------------- |
| `type`       | `"trajectory_data"` | Message discriminator                  |
| `request_id` | `u64`               | Correlation ID from the client request |
| `legs`       | `LegTrajectory[]`   | Per-leg trajectory points              |

Each `LegTrajectory` contains a `leg_index` and `points` array of `TrajectoryPoint` objects: `{ elapsed_s, position_ric_km: [R, I, C], velocity_ric_km_s: [R, I, C] }`.

### free_drift_data

Free-drift trajectory and safety data for abort-case analysis.

| Field        | Type                 | Description                            |
| ------------ | -------------------- | -------------------------------------- |
| `type`       | `"free_drift_data"`  | Message discriminator                  |
| `request_id` | `u64`                | Correlation ID from the client request |
| `legs`       | `LegTrajectory[]`    | Per-leg free-drift trajectory points   |
| `analyses`   | `FreeDriftSummary[]` | Per-leg free-drift safety summaries    |

Each `FreeDriftSummary` contains:

| Field                     | Type    | Description                                                                    |
| ------------------------- | ------- | ------------------------------------------------------------------------------ |
| `leg_index`               | `usize` | Leg index                                                                      |
| `min_distance_3d_km`      | `f64`   | Min 3D distance on free-drift arc (km)                                         |
| `min_rc_separation_km`    | `f64`   | Min R/C distance on free-drift arc (km)                                        |
| `min_ei_separation_km`    | `f64`   | Min e/i separation on free-drift arc (km)                                      |
| `bounded_motion_residual` | `f64`   | Bounded-motion residual (D'Amico Eq. 2.33). Zero = bounded; nonzero = drifting |

### poca_data

Brent-refined closest-approach data.

| Field        | Type          | Description                            |
| ------------ | ------------- | -------------------------------------- |
| `type`       | `"poca_data"` | Message discriminator                  |
| `request_id` | `u64`         | Correlation ID from the client request |
| `points`     | `PocaPoint[]` | Closest-approach points across legs    |

Each `PocaPoint` contains:

| Field               | Type       | Description                                          |
| ------------------- | ---------- | ---------------------------------------------------- |
| `leg_index`         | `usize`    | Leg index within the mission                         |
| `elapsed_s`         | `f64`      | Elapsed time from leg start (seconds)                |
| `distance_km`       | `f64`      | Refined minimum distance (km)                        |
| `position_ric_km`   | `[f64; 3]` | RIC position at closest approach (km): [R, I, C]     |
| `is_global_minimum` | `bool`     | True for the single closest approach across all legs |

Legs with no close approach (diverging geometry) produce no points for that leg index.

### cola_result

Collision avoidance maneuvers for POCA violations, with multi-leg post-avoidance verification.

| Field                    | Type                          | Description                                                   |
| ------------------------ | ----------------------------- | ------------------------------------------------------------- |
| `type`                   | `"cola_result"`               | Message discriminator                                         |
| `request_id`             | `u64`                         | Correlation ID from the client request                        |
| `maneuvers`              | `AvoidanceManeuverSummary[]`  | Avoidance maneuvers (empty if no violations)                  |
| `secondary_conjunctions` | `SecondaryViolationSummary[]` | Downstream violations created by avoidance (omitted if empty) |
| `skipped`                | `SkippedLegSummary[]`         | Legs where COLA failed (omitted if empty)                     |

Each `AvoidanceManeuverSummary` contains:

| Field                    | Type       | Description                                       |
| ------------------------ | ---------- | ------------------------------------------------- |
| `leg_index`              | `usize`    | Leg index where the violation was found           |
| `dv_ric_km_s`            | `[f64; 3]` | Avoidance delta-v in RIC frame `[R, I, C]` (km/s) |
| `maneuver_location_rad`  | `f64`      | Mean argument of latitude at optimal burn (rad)   |
| `post_avoidance_poca_km` | `f64`      | Verified post-avoidance POCA distance (km)        |
| `fuel_cost_km_s`         | `f64`      | Total delta-v magnitude (km/s)                    |
| `correction_type`        | `string`   | `"in_plane"`, `"cross_track"`, or `"combined"`    |

Each `SecondaryViolationSummary` contains:

| Field                | Type       | Description                                         |
| -------------------- | ---------- | --------------------------------------------------- |
| `original_leg_index` | `usize`    | Leg where the avoidance maneuver was applied        |
| `violated_leg_index` | `usize`    | Downstream leg where the new violation was detected |
| `distance_km`        | `f64`      | Closest approach distance on the violated leg (km)  |
| `elapsed_s`          | `f64`      | Elapsed time from downstream leg start (s)          |
| `position_ric_km`    | `[f64; 3]` | RIC position at closest approach `[R, I, C]` (km)   |

Each `SkippedLegSummary` contains:

| Field           | Type     | Description                                         |
| --------------- | -------- | --------------------------------------------------- |
| `leg_index`     | `usize`  | Leg with the unaddressed POCA violation             |
| `error_message` | `string` | Why COLA failed (e.g., budget exceeded, degenerate) |

### plan_selected

Confirmation that the active plan variant was changed (response to `select_plan`).

| Field        | Type              | Description                                           |
| ------------ | ----------------- | ----------------------------------------------------- |
| `type`       | `"plan_selected"` | Message discriminator                                 |
| `request_id` | `u64`             | Correlation ID from the client request                |
| `variant`    | `string`          | The now-active variant (`"baseline"` or `"enriched"`) |

### formation_design_result

Full formation design report for the currently selected plan variant (response to `get_formation_design`).

| Field        | Type                        | Description                            |
| ------------ | --------------------------- | -------------------------------------- |
| `type`       | `"formation_design_result"` | Message discriminator                  |
| `request_id` | `u64`                       | Correlation ID from the client request |
| `report`     | `FormationDesignReport`     | Full formation design report           |

`FormationDesignReport` fields:

| Field                          | Type                              | Description                                                |
| ------------------------------ | --------------------------------- | ---------------------------------------------------------- |
| `perch`                        | `PerchEnrichmentResult`           | Perch enrichment: `enriched` (with safe e/i) or `fallback` |
| `waypoints`                    | `(EnrichedWaypoint \| null)[]`    | Per-leg advisory enrichment (null if failed for that leg)  |
| `transit_safety`               | `(TransitSafetyReport \| null)[]` | Per-leg e/i separation profile (null if failed)            |
| `mission_min_ei_separation_km` | `f64?`                            | Mission-wide minimum e/i separation (omitted if no data)   |
| `drift_prediction`             | `DriftPrediction?`                | Predicted mid-transit e/i for leg-1 coast arc (omitted if outside compensation regime). `{ predicted_min_ei_km: f64, predicted_phase_angle_rad: f64 }` |

`TransitSafetyReport` fields:

| Field                   | Type         | Description                                                       |
| ----------------------- | ------------ | ----------------------------------------------------------------- |
| `min_ei_separation_km`  | `f64`        | Minimum e/i separation across the arc (km)                        |
| `min_elapsed_s`         | `f64`        | Elapsed time at minimum (seconds from leg departure)              |
| `min_phase_angle_rad`   | `f64`        | E/I phase angle at minimum (rad). 0 = parallel, pi/2 = orthogonal |
| `satisfies_requirement` | `bool`       | Whether the arc satisfies the safety requirement                  |
| `threshold_km`          | `f64`        | The threshold used for `satisfies_requirement` (km)               |
| `profile`               | `EiSample[]` | Per-sample e/i separation profile                                 |

`EiSample` fields:

| Field              | Type  | Description                               |
| ------------------ | ----- | ----------------------------------------- |
| `elapsed_s`        | `f64` | Elapsed time from leg departure (seconds) |
| `ei_separation_km` | `f64` | E/I separation at this point (km)         |
| `phase_angle_rad`  | `f64` | E/I phase angle at this point (rad)       |

`PerchEnrichmentResult` is tagged by `status`:

- `"enriched"`: `{ status: "enriched", roe, de_magnitude_km, di_magnitude_km, min_rc_separation_km, alignment }`
- `"fallback"`: `{ status: "fallback", unenriched_roe, reason: { type: "singular_geometry" | "separation_unachievable" | ... } }`

### safe_alternative_result

Safe alternative ROE for a single waypoint (response to `get_safe_alternative`).

| Field        | Type                        | Description                            |
| ------------ | --------------------------- | -------------------------------------- |
| `type`       | `"safe_alternative_result"` | Message discriminator                  |
| `request_id` | `u64`                       | Correlation ID from the client request |
| `enriched`   | `EnrichedWaypoint`          | Waypoint with safe e/i vectors         |

`EnrichedWaypoint` fields:

| Field                | Type                  | Description                                         |
| -------------------- | --------------------- | --------------------------------------------------- |
| `roe`                | `QuasiNonsingularROE` | Safety-enriched ROE target                          |
| `baseline_roe`       | `QuasiNonsingularROE` | Original (minimum-norm) ROE for comparison          |
| `position_ric_km`    | `Vector3`             | RIC position (identical to operator's input)        |
| `enriched_ei`        | `EiSeparation`        | E/I separation of enriched state (D'Amico Eq. 2.22) |
| `baseline_ei`        | `EiSeparation`        | E/I separation of unenriched state                  |
| `perturbation_norm`  | `f64`                 | Euclidean norm of the null-space perturbation       |
| `mode`               | `string`              | `"position_only"` or `"velocity_constrained"`       |
| `resolved_alignment` | `string`              | `"parallel"` or `"anti_parallel"`                   |

### covariance_data

On-demand covariance propagation report.

| Field        | Type                      | Description                            |
| ------------ | ------------------------- | -------------------------------------- |
| `type`       | `"covariance_data"`       | Message discriminator                  |
| `request_id` | `u64`                     | Correlation ID from the client request |
| `report`     | `MissionCovarianceReport` | Full covariance report                 |

### eclipse_data

Eclipse data for transfer and/or mission phases.

| Field        | Type                   | Description                                     |
| ------------ | ---------------------- | ----------------------------------------------- |
| `type`       | `"eclipse_data"`       | Message discriminator                           |
| `request_id` | `u64`                  | Correlation ID from the client request          |
| `transfer`   | `TransferEclipseData?` | Transfer-phase eclipse data (None if proximity) |
| `mission`    | `MissionEclipseData?`  | Mission-phase eclipse data (None if no mission) |

### session_state

Diagnostic snapshot of the current session.

| Field        | Type              | Description                            |
| ------------ | ----------------- | -------------------------------------- |
| `type`       | `"session_state"` | Message discriminator                  |
| `request_id` | `u64`             | Correlation ID from the client request |
| `summary`    | `SessionSummary`  | Session state flags and config values  |

`SessionSummary` fields:

| Field                      | Type               | Description                         |
| -------------------------- | ------------------ | ----------------------------------- |
| `has_chief`                | `bool`             | Whether chief state is set          |
| `has_deputy`               | `bool`             | Whether deputy state is set         |
| `has_transfer`             | `bool`             | Whether transfer is computed        |
| `has_mission`              | `bool`             | Whether mission is planned          |
| `has_drag_config`          | `bool`             | Whether drag config is available    |
| `has_navigation_accuracy`  | `bool`             | Whether nav accuracy is configured  |
| `has_maneuver_uncertainty` | `bool`             | Whether maneuver uncertainty is set |
| `has_monte_carlo_config`   | `bool`             | Whether MC config is set            |
| `has_safety_requirements`  | `bool`             | Whether safety requirements are set |
| `waypoint_count`           | `usize`            | Number of waypoints defined         |
| `chief_config`             | `SpacecraftChoice` | Current chief spacecraft            |
| `deputy_config`            | `SpacecraftChoice` | Current deputy spacecraft           |
| `propagator`               | `PropagatorChoice` | Current propagator                  |
| `lambert_tof_s`            | `f64`              | Current Lambert TOF (seconds)       |

### progress

Sent zero or more times during `validate` and `run_mc` operations.

| Field        | Type               | Description                                          |
| ------------ | ------------------ | ---------------------------------------------------- |
| `type`       | `"progress"`       | Message discriminator                                |
| `request_id` | `u64`              | Correlation ID of the operation in progress          |
| `phase`      | `string`           | `"validate"` or `"mc"`                               |
| `detail`     | `string` or `null` | Human-readable detail (e.g., `"Validating leg 3/5"`) |
| `fraction`   | `number` or `null` | Completion fraction, 0.0 to 1.0                      |

### validation_result

| Field        | Type                  | Description                            |
| ------------ | --------------------- | -------------------------------------- |
| `type`       | `"validation_result"` | Message discriminator                  |
| `request_id` | `u64`                 | Correlation ID from the client request |
| `report`     | `ValidationReport`    | Full per-leg validation report         |

### monte_carlo_result

| Field        | Type                   | Description                            |
| ------------ | ---------------------- | -------------------------------------- |
| `type`       | `"monte_carlo_result"` | Message discriminator                  |
| `request_id` | `u64`                  | Correlation ID from the client request |
| `report`     | `MonteCarloReport`     | Full Monte Carlo ensemble report       |

### cancelled

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

### error

| Field        | Type                | Description                                       |
| ------------ | ------------------- | ------------------------------------------------- |
| `type`       | `"error"`           | Message discriminator                             |
| `request_id` | `u64` or `null`     | Correlation ID (null if not tied to a request)    |
| `code`       | `string`            | Machine-readable error code (see below)           |
| `message`    | `string`            | Human-readable error message                      |
| `detail`     | `object` or omitted | Per-variant diagnostic fields (omitted when null) |

## PropagatorToggle

The `propagator` field in `update_config` uses a simplified toggle enum:

| Value       | Description                                            |
| ----------- | ------------------------------------------------------ |
| `"j2"`      | J2-only analytical propagation                         |
| `"j2_drag"` | J2 + differential drag (requires prior `extract_drag`) |

`"j2_drag"` resolves against the session's stored `DragConfig`. It does not carry embedded drag values. The server returns an error if no drag has been extracted.

## Error Codes

All error codes are lowercase snake_case strings.

| Code                     | Description                                                         |
| ------------------------ | ------------------------------------------------------------------- |
| `missing_session_state`  | Required session state not available (call setter first)            |
| `targeting_convergence`  | Newton-Raphson targeting solver did not converge                    |
| `lambert_failure`        | Lambert solver failure                                              |
| `propagation_error`      | Propagation error                                                   |
| `validation_error`       | Nyx validation error                                                |
| `monte_carlo_error`      | Monte Carlo error                                                   |
| `nyx_bridge_error`       | Nyx bridge error (almanac, dynamics, propagation)                   |
| `covariance_error`       | Covariance propagation error                                        |
| `formation_design_error` | Formation design error (singular geometry, unachievable separation) |
| `invalid_input`          | Malformed JSON, missing fields, bad values                          |
| `mission_error`          | General mission planning error                                      |
| `cancelled`              | Operation was cancelled                                             |

The `detail` field carries per-error diagnostic data. Examples:

`targeting_convergence`:

```json
{
  "final_error_km": 0.0015,
  "iterations": 100
}
```

`missing_session_state`:

```json
{
  "missing": "chief",
  "context": "set chief/deputy states via set_states before this operation"
}
```

`invalid_input` (missing field):

```json
{
  "reason": "missing_field",
  "field": "chief_config",
  "context": "validate"
}
```

## Background Job Behavior

- Only one background job runs at a time (`extract_drag`, `validate`, `run_mc`).
- Sending a new background request automatically cancels the active job.
- Cancellation is cooperative: the server checks the cancel flag between phases (legs, MC samples).
- `heartbeat` messages are sent every 30 seconds during background jobs to keep the connection alive.
- When the connection closes, any active background job is cancelled.
- `auto_drag` on `validate` and `run_mc` is ephemeral: it extracts drag and replans temporarily without modifying session state.

## Session Flows

### Far-Field

```
set_states -> state_updated
set_spacecraft -> state_updated                          (optional; defaults to cubesat_6u)
classify -> classify_result (far_field)
compute_transfer -> transfer_computed
extract_drag -> drag_result (~3s async)                  (skip if spacecraft configs identical)
set_safety_requirements -> state_updated                  (optional; enables dual-plan)
set_waypoints -> plan_result (baseline + enriched if safety_requirements set)
select_plan -> plan_selected                              (optional; default is baseline)
|- get_trajectory -> trajectory_data (~20KB, on-demand)              |
|- get_eclipse -> eclipse_data (on-demand)                           |
|- get_covariance -> covariance_data (~70KB, on-demand)              | parallel on-demand fetches
|- get_free_drift_trajectory -> free_drift_data (on-demand)          |
|- get_poca -> poca_data (on-demand)                                 |
|- run_cola -> cola_result (on-demand, requires safety config)       |
|- get_formation_design -> formation_design_result (on-demand)       |
|- get_safe_alternative -> safe_alternative_result (on-demand)       |
update_config(j2_drag) -> plan_result                    (when drag_result arrives)
validate -> progress* + validation_result                 (operates on selected plan)
run_mc -> progress* + monte_carlo_result                  (operates on selected plan)
```

### Proximity

```
set_states -> state_updated
|- classify -> classify_result (proximity)               |
|- extract_drag -> drag_result (~3s async)               | parallel (both read session states)
set_safety_requirements -> state_updated                  (optional; enables dual-plan)
set_waypoints -> plan_result (auto-computes transfer, baseline + enriched)
select_plan -> plan_selected                              (optional; default is baseline)
|- get_trajectory -> trajectory_data                             |
|- get_eclipse -> eclipse_data                                   |
|- get_covariance -> covariance_data                             | parallel on-demand fetches
|- get_free_drift_trajectory -> free_drift_data                  |
|- run_cola -> cola_result (on-demand, requires safety config)   |
|- get_formation_design -> formation_design_result (on-demand)   |
validate -> validation_result                             (operates on selected plan)
```

### Parallelism Notes

- **classify + extract_drag (proximity):** Both only read `chief`/`deputy` from the session. No mutation conflict. Fire both after `set_states` to overlap the ~3s drag simulation with the instant classify.
- **On-demand getters (get_trajectory, get_covariance, get_eclipse, get_free_drift_trajectory, get_poca, run_cola, get_formation_design, get_safe_alternative):** All read from the stored mission/transfer. No mutation. Fire whichever are needed in parallel after planning.
- **extract_drag (far-field):** Must wait for `compute_transfer` because it uses perch states (post-Lambert geometry).

## Examples

### classify

Request:

```json
{
  "type": "classify",
  "request_id": 1
}
```

Response (far-field):

```json
{
  "type": "classify_result",
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

### set_waypoints

Request:

```json
{
  "type": "set_waypoints",
  "request_id": 3,
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
  ]
}
```

Response:

```json
{
  "type": "plan_result",
  "request_id": 3,
  "result": {
    "phase": { "far_field": { "...": "..." } },
    "transfer_summary": {
      "total_dv_km_s": 1.234,
      "tof_s": 3600.0,
      "direction": "short_way",
      "arrival_epoch": "..."
    },
    "perch_roe": { "da": 0.0, "dlambda": 0.000726, "...": "..." },
    "legs": ["..."],
    "total_dv_km_s": 1.237,
    "total_duration_s": 12000.0
  }
}
```

### cancel

Request:

```json
{
  "type": "cancel",
  "request_id": null
}
```

Response (if a job was active):

```json
{
  "type": "cancelled",
  "request_id": 3
}
```

### error

```json
{
  "type": "error",
  "request_id": 2,
  "code": "targeting_convergence",
  "message": "targeting did not converge after 100 iterations (error: 0.0015 km)",
  "detail": {
    "final_error_km": 0.0015,
    "iterations": 100
  }
}
```

### progress + heartbeat

During a validation run, the client may see interleaved progress and heartbeat messages:

```json
{"type": "progress", "request_id": 3, "phase": "validate", "detail": "Validating leg 1/5", "fraction": 0.2}
{"type": "progress", "request_id": 3, "phase": "validate", "detail": "Validating leg 2/5", "fraction": 0.4}
{"type": "heartbeat", "seq": 1}
{"type": "progress", "request_id": 3, "phase": "validate", "detail": "Validating leg 3/5", "fraction": 0.6}
```
