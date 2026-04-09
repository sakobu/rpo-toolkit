# RPO WASM Engine

The `rpo-wasm` crate provides WebAssembly bindings for the RPO analytical engine. It exposes 18 functions to JavaScript, covering classification, waypoint mission planning, safety analysis, covariance propagation, eclipse computation, formation design enrichment, and state queries. All functions run entirely in the browser with no server round-trip. TypeScript definitions are auto-generated via `tsify-next`.

## Build

```bash
wasm-pack build rpo-wasm --target web
```

This produces:

```
rpo-wasm/pkg/
  rpo_wasm.js          # ES module glue
  rpo_wasm.d.ts        # TypeScript definitions (auto-generated)
  rpo_wasm_bg.wasm     # compiled WASM binary
  rpo_wasm_bg.wasm.d.ts
  package.json
```

Initialize the WASM module before calling any function:

```typescript
import init, {
  classify_separation,
  plan_waypoint_mission,
  // ... other functions
} from "./pkg/rpo_wasm.js";

await init();
```

## Architecture

```
Browser (rpo-wasm)                          Server (rpo-api)
-------------------------------             -------------------------
classify_separation()                       compute_transfer()  <- Lambert (nyx)
plan_waypoint_mission()                     extract_drag()      <- nyx full-physics
execute_mission_from_transfer()             validate()          <- nyx validation
replan_from_transfer()                      run_mc()            <- nyx Monte Carlo
compute_mission_covariance()
compute_free_drift_analysis()
compute_poca_analysis()
resample_leg_trajectory()
compute_safety_analysis()
assess_cola()
compute_avoidance()
compute_transfer_eclipse()
compute_mission_eclipse()
get_mission_state_at_time()
suggest_enrichment()
apply_perch_enrichment()
accept_waypoint_enrichment()
enrich_waypoint()
```

The browser holds all mission state. The server handles only the 4 operations requiring nyx-space (Lambert transfer, drag extraction, full-physics validation, Monte Carlo ensemble). Every other computation runs locally via WASM.

**Ownership model.** All parameters are passed by value across the WASM boundary. Inputs are consumed, and new objects are returned. There are no mutable references or shared state -- each call is a pure function from the JavaScript perspective. Functions that need to return both a result and a mutated input (e.g., `execute_mission_from_transfer`) wrap them in a combined output struct.

## Error Model

Fallible functions return `Result<T, WasmError>`. On error, `wasm-bindgen` throws a JavaScript exception whose value is a `WasmError` object.

### WasmError

```typescript
interface WasmError {
  code: WasmErrorCode; // Machine-readable error category
  message: string; // Human-readable description
  details?: string; // Optional source error chain
}
```

### WasmErrorCode

All codes are `snake_case` strings.

| Code               | Description                                                   |
| ------------------ | ------------------------------------------------------------- |
| `mission`          | Mission planning error (classification, targeting, waypoints) |
| `propagation`      | Propagation error (STM, Keplerian)                            |
| `covariance`       | Covariance propagation error                                  |
| `avoidance`        | Collision avoidance maneuver error                            |
| `missing_field`    | A required field is missing                                   |
| `empty_trajectory` | Trajectory data is empty                                      |
| `lambert`          | Lambert solver error                                          |
| `eclipse`          | Eclipse computation error                                     |
| `formation`        | Formation design error                                        |
| `deserialization`  | Input deserialization failed (invalid JSON shape or types)    |
| `internal`         | Catch-all for unexpected errors                               |

### Error handling in TypeScript

Fallible functions throw on error. Use try/catch:

```typescript
try {
  const result = classify_separation(chief, deputy, config);
} catch (e) {
  const err = e as WasmError;
  switch (err.code) {
    case "mission":
      console.error("Classification failed:", err.message);
      break;
    case "deserialization":
      console.error("Bad input shape:", err.message, err.details);
      break;
    default:
      console.error("Unexpected error:", err.code, err.message);
  }
}
```

**Infallible functions.** Three functions never throw: `compute_safety_analysis`, `apply_perch_enrichment`, and `suggest_enrichment`. Two functions (`compute_free_drift_analysis`, `compute_poca_analysis`) return `undefined` on failure instead of throwing, providing graceful degradation.

## Module Reference

### Planning

Classification and waypoint mission planning.

| Function                | Return            | Fallible |
| ----------------------- | ----------------- | -------- |
| `classify_separation`   | `MissionPhase`    | yes      |
| `plan_waypoint_mission` | `WaypointMission` | yes      |

```typescript
classify_separation(chief: StateVector, deputy: StateVector, config: ProximityConfig): MissionPhase
plan_waypoint_mission(departure: DepartureState, waypoints: WaypointInput[], config: MissionConfig, propagator: PropagatorChoice): WaypointMission
```

`classify_separation` determines whether two spacecraft are in far-field or proximity regime based on their ECI (Earth-Centered Inertial) state vectors and a separation threshold.

`plan_waypoint_mission` plans a multi-waypoint mission from a departure state. The `waypoints` parameter accepts a plain `WaypointInput[]` array, deserialized internally via `serde_wasm_bindgen` -- see [JsValue Parameters](#jsvalue-parameters).

```typescript
// Classify separation
const phase = classify_separation(chief, deputy, proximityConfig);

// Plan mission
const waypoints = [
  { position_ric_km: [0.0, -2.0, 0.0], tof_s: 5400.0 },
  { position_ric_km: [0.0, -0.5, 0.0], tof_s: 2700.0 },
];
const mission = plan_waypoint_mission(
  departure,
  waypoints,
  missionConfig,
  propagator,
);
```

### Mission

Mission execution and replanning from a pre-computed Lambert transfer.

| Function                        | Return          | Fallible |
| ------------------------------- | --------------- | -------- |
| `execute_mission_from_transfer` | `MissionResult` | yes      |
| `replan_from_transfer`          | `MissionResult` | yes      |

```typescript
execute_mission_from_transfer(transfer: TransferResult, input: PipelineInput): MissionResult
replan_from_transfer(transfer: TransferResult, input: PipelineInput, modified_index: number, cached_mission?: WaypointMission | null): MissionResult
```

`execute_mission_from_transfer` runs the full analytical pipeline from a server-provided Lambert transfer result.

`replan_from_transfer` re-executes the mission after modifying a waypoint. When a `cached_mission` is provided, converged legs before `modified_index` are preserved, avoiding redundant re-targeting.

```typescript
// Full execution
const { output, transfer: updatedTransfer } = execute_mission_from_transfer(
  transfer,
  pipelineInput,
);

// Incremental replan after editing waypoint 2
const replanResult = replan_from_transfer(
  transfer,
  pipelineInput,
  2, // modified_index
  cachedMission, // reuse converged legs before index 2
);
```

### Analysis

Covariance propagation, free-drift analysis, POCA (Point of Closest Approach) refinement, and trajectory resampling.

| Function                      | Return                    | Fallible |
| ----------------------------- | ------------------------- | -------- |
| `compute_mission_covariance`  | `MissionCovarianceReport` | yes      |
| `compute_free_drift_analysis` | `FreeDriftResult?`        | no       |
| `compute_poca_analysis`       | `PocaResult?`             | no       |
| `resample_leg_trajectory`     | `ResampledTrajectory`     | yes      |

```typescript
compute_mission_covariance(mission: WaypointMission, chief_at_arrival: KeplerianElements, navigation_accuracy: NavigationAccuracy, maneuver_uncertainty: ManeuverUncertainty | null | undefined, propagator: PropagatorChoice): MissionCovarianceReport
compute_free_drift_analysis(mission: WaypointMission, propagator: PropagatorChoice): FreeDriftResult | undefined
compute_poca_analysis(mission: WaypointMission, propagator: PropagatorChoice): PocaResult | undefined
resample_leg_trajectory(leg: ManeuverLeg, n_steps: number, propagator: PropagatorChoice): ResampledTrajectory
```

`compute_mission_covariance` propagates position/velocity uncertainty through the mission. When `maneuver_uncertainty` is `null`/`undefined`, perfect maneuver execution is assumed (no covariance injection at burn epochs).

`compute_free_drift_analysis` evaluates abort-case safety for all legs (what happens if maneuvers are not executed). Returns `undefined` if any leg computation fails.

`compute_poca_analysis` refines closest-approach distances for all legs using Brent's method. Returns `undefined` if any leg computation fails.

`resample_leg_trajectory` re-propagates a single leg at a different step count, useful for visualization at higher or lower resolution.

```typescript
// Covariance
const covReport = compute_mission_covariance(
  mission,
  chiefAtArrival,
  navAccuracy,
  maneuverUncertainty,
  propagator,
);

// Free-drift (returns undefined on failure)
const freeDrift = compute_free_drift_analysis(mission, propagator);
if (freeDrift !== undefined) {
  console.log("Free-drift analyses:", freeDrift.analyses);
}

// POCA (returns undefined on failure)
const poca = compute_poca_analysis(mission, propagator);
if (poca !== undefined) {
  console.log("POCA per leg:", poca.legs);
}

// Resample for visualization
try {
  const resampled = resample_leg_trajectory(leg, 200, propagator);
  console.log("Resampled states:", resampled.states.length);
} catch (e) {
  console.error("Resample failed:", e.message);
}
```

### Safety

Safety analysis, COLA (Collision Avoidance) assessment, and avoidance maneuver computation.

| Function                  | Return              | Fallible |
| ------------------------- | ------------------- | -------- |
| `compute_safety_analysis` | `SafetyAnalysis`    | no       |
| `assess_cola`             | `ColaAssessment`    | yes      |
| `compute_avoidance`       | `AvoidanceManeuver` | yes      |

```typescript
compute_safety_analysis(mission: WaypointMission, safety: SafetyConfig | null | undefined, cola: ColaConfig | null | undefined, propagator: PropagatorChoice): SafetyAnalysis
assess_cola(mission: WaypointMission, poca: ClosestApproach[][], propagator: PropagatorChoice, config: ColaConfig): ColaAssessment
compute_avoidance(poca: ClosestApproach, roe: QuasiNonsingularROE, chief_mean: KeplerianElements, departure_epoch: string, tof_s: number, propagator: PropagatorChoice, config: ColaConfig): AvoidanceManeuver
```

`compute_safety_analysis` is the all-in-one safety function: it computes free-drift analysis, POCA refinement, and optional COLA assessment in a single call. When `safety` is `null`/`undefined`, free-drift analysis, POCA refinement, and COLA assessment are all skipped (the returned `SafetyAnalysis` will have `undefined` for all fields). Pass a `SafetyConfig` to enable safety computation. When `cola` is `null`/`undefined`, avoidance maneuver computation is skipped.

`assess_cola` evaluates COLA threat level from pre-computed POCA data. The `poca` parameter accepts a plain `ClosestApproach[][]` array, deserialized internally via `serde_wasm_bindgen` -- see [JsValue Parameters](#jsvalue-parameters).

`compute_avoidance` solves a single avoidance maneuver for a POCA violation using inverse GVE (Gauss Variational Equations). The `departure_epoch` is an ISO 8601 string parsed internally. ROE (Relative Orbital Elements) describes the deputy's relative orbit geometry.

```typescript
// Full safety analysis
const safety = compute_safety_analysis(
  mission,
  safetyConfig,
  colaConfig,
  propagator,
);

// Standalone COLA assessment from pre-computed POCA data
try {
  const cola = assess_cola(mission, pocaData, propagator, colaConfig);
} catch (e) {
  console.error("COLA assessment failed:", e.message);
}

// Compute avoidance maneuver for a specific POCA violation
try {
  const maneuver = compute_avoidance(
    poca, // ClosestApproach to mitigate
    roe, // deputy ROE at leg departure
    chiefMean, // chief mean Keplerian elements
    "2024-01-01T01:00:00 UTC", // departure epoch (ISO 8601)
    5400.0, // leg time-of-flight (seconds)
    propagator,
    colaConfig,
  );
} catch (e) {
  console.error("Avoidance failed:", e.message);
}
```

### Eclipse

Eclipse computation along transfer arcs and mission legs.

| Function                   | Return                | Fallible |
| -------------------------- | --------------------- | -------- |
| `compute_transfer_eclipse` | `TransferEclipseData` | yes      |
| `compute_mission_eclipse`  | `MissionEclipseData`  | yes      |

```typescript
compute_transfer_eclipse(transfer: LambertTransfer, chief: StateVector, arc_steps: number): TransferEclipseData
compute_mission_eclipse(mission: WaypointMission): MissionEclipseData
```

`compute_transfer_eclipse` samples eclipse state (sunlit, penumbra, umbra) along a Lambert transfer arc.

`compute_mission_eclipse` computes per-leg eclipse data for a planned mission.

```typescript
// Transfer arc eclipse
try {
  const eclipseData = compute_transfer_eclipse(lambertTransfer, chief, 100);
} catch (e) {
  console.error("Eclipse computation failed:", e.message);
}

// Mission eclipse
try {
  const missionEclipse = compute_mission_eclipse(mission);
} catch (e) {
  console.error("Mission eclipse failed:", e.message);
}
```

### Query

Mission state queries at arbitrary elapsed times.

| Function                    | Return             | Fallible |
| --------------------------- | ------------------ | -------- |
| `get_mission_state_at_time` | `PropagatedState?` | yes      |

```typescript
get_mission_state_at_time(mission: WaypointMission, elapsed_s: number, propagator: PropagatorChoice): PropagatedState | undefined
```

`get_mission_state_at_time` returns the propagated deputy state at an arbitrary time within the mission. Returns `undefined` (not an error) if `elapsed_s` is outside the mission duration.

```typescript
try {
  const state = get_mission_state_at_time(mission, 3600.0, propagator);
  if (state !== undefined) {
    console.log("Position RIC:", state.position_ric_km);
  } else {
    console.log("Elapsed time outside mission duration");
  }
} catch (e) {
  console.error("Propagation failed:", e.message);
}
```

### Enrichment

Formation design enrichment: suggest safe-perch alternatives, apply them, and re-execute the mission with enriched waypoints. Enrichment projects waypoint ROE onto the nearest passively safe formation using e/i vector separation analysis (D'Amico Eq. 2.22).

| Function                     | Return                   | Fallible |
| ---------------------------- | ------------------------ | -------- |
| `suggest_enrichment`         | `EnrichmentSuggestion?`  | no       |
| `apply_perch_enrichment`     | `TransferResult`         | no       |
| `accept_waypoint_enrichment` | `EnrichmentAcceptResult` | yes      |
| `enrich_waypoint`            | `EnrichedWaypoint`       | yes      |

```typescript
suggest_enrichment(transfer: TransferResult, input: PipelineInput): EnrichmentSuggestion | undefined
apply_perch_enrichment(transfer: TransferResult, suggestion: EnrichmentSuggestion): TransferResult
accept_waypoint_enrichment(input: PipelineInput, transfer: TransferResult, waypoint_index: number, enriched_roe: QuasiNonsingularROE, chief_at_waypoint: KeplerianElements): EnrichmentAcceptResult
enrich_waypoint(position_ric_km: [number, number, number], velocity_ric_km_s: [number, number, number] | null, chief_mean: KeplerianElements, requirements: SafetyRequirements): EnrichedWaypoint
```

`suggest_enrichment` checks whether perch enrichment is available for a given transfer and pipeline input. Returns `undefined` if safety requirements are not configured or enrichment is not applicable.

`apply_perch_enrichment` mutates the transfer's perch ROE to the enriched safe-perch values and returns the updated transfer.

`accept_waypoint_enrichment` updates a waypoint's ROE target to enriched values, then re-runs the full mission pipeline. Returns both the new pipeline output and the mutated input/transfer.

`enrich_waypoint` computes a safe enriched waypoint for a single RIC (Radial-In-track-Cross-track) position, showing baseline vs enriched e/i vectors for a "safe alternative" card. The `position_ric_km` and `velocity_ric_km_s` parameters accept plain arrays, deserialized internally via `serde_wasm_bindgen` -- see [JsValue Parameters](#jsvalue-parameters).

```typescript
// Check for enrichment suggestion
const suggestion = suggest_enrichment(transfer, pipelineInput);
if (suggestion !== undefined) {
  // Apply perch enrichment
  const enrichedTransfer = apply_perch_enrichment(transfer, suggestion);
}

// Accept enrichment for a specific waypoint
try {
  const {
    output,
    input: updatedInput,
    transfer: updatedTransfer,
  } = accept_waypoint_enrichment(
    pipelineInput,
    transfer,
    1,
    enrichedRoe,
    chiefAtWaypoint,
  );
} catch (e) {
  console.error("Enrichment accept failed:", e.message);
}

// Per-waypoint enrichment preview
try {
  const enriched = enrich_waypoint(
    [0.0, -2.0, 0.0], // position_ric_km
    null, // velocity_ric_km_s (optional)
    chiefMean,
    safetyRequirements,
  );
} catch (e) {
  console.error("Waypoint enrichment failed:", e.message);
}
```

## Wrapper Types

Output-only structs that exist solely for WASM boundary compatibility. These wrap collections or combine multiple return values that cannot be passed directly across the WASM boundary. They are `Tsify` structs with `into_wasm_abi` only (never passed from JS to Rust).

| Type                     | Fields                                                                       | Description                                             |
| ------------------------ | ---------------------------------------------------------------------------- | ------------------------------------------------------- |
| `MissionResult`          | `output: PipelineOutput`, `transfer: TransferResult`                         | Combined pipeline output and mutated transfer           |
| `EnrichmentAcceptResult` | `output: PipelineOutput`, `input: PipelineInput`, `transfer: TransferResult` | Pipeline output, mutated input, and mutated transfer    |
| `FreeDriftResult`        | `analyses: FreeDriftAnalysis[]`                                              | Per-leg free-drift analysis results                     |
| `PocaResult`             | `legs: ClosestApproach[][]`                                                  | Per-leg POCA results (outer = legs, inner = approaches) |
| `ResampledTrajectory`    | `states: PropagatedState[]`                                                  | Resampled propagated states along a leg                 |
| `WasmError`              | `code: WasmErrorCode`, `message: string`, `details?: string`                 | Structured error with machine-readable code             |

For full field definitions, consult the generated `rpo_wasm.d.ts`.

## JsValue Parameters

Four parameters across three functions use `JsValue` instead of typed Tsify bindings. This is required when tsify cannot generate `FromWasmAbi` for bare `Vec<T>`, nested `Vec<Vec<T>>`, or fixed-size arrays.

| Function                | Parameter           | Expected JS Type                   | Reason                                                     |
| ----------------------- | ------------------- | ---------------------------------- | ---------------------------------------------------------- |
| `plan_waypoint_mission` | `waypoints`         | `WaypointInput[]`                  | tsify cannot generate `FromWasmAbi` for `Vec<T>`           |
| `assess_cola`           | `poca`              | `ClosestApproach[][]`              | tsify cannot generate `FromWasmAbi` for `Vec<Vec<T>>`      |
| `enrich_waypoint`       | `position_ric_km`   | `[number, number, number]`         | tsify cannot generate `FromWasmAbi` for `[f64; 3]`         |
| `enrich_waypoint`       | `velocity_ric_km_s` | `[number, number, number] \| null` | tsify cannot generate `FromWasmAbi` for `Option<[f64; 3]>` |

These parameters are deserialized internally via `serde_wasm_bindgen`. Pass plain JavaScript objects/arrays -- they are serialized automatically. On deserialization failure, a `WasmError` with code `deserialization` is thrown.

```typescript
// JsValue parameters accept plain JS values directly:
const mission = plan_waypoint_mission(
  departure,
  [{ position_ric_km: [0, -2, 0], tof_s: 5400 }], // plain array, not a typed object
  config,
  propagator,
);
```

## Typical Browser Flows

### Far-Field Mission

A far-field mission begins with a Lambert transfer computed on the server, then proceeds with all planning and analysis in the browser.

```
Browser (WASM)                              Server
--------------                              ------
classify_separation() -> far_field
                                            compute_transfer -> transfer_result
                                            extract_drag -> drag_result
execute_mission_from_transfer()
compute_safety_analysis()
compute_mission_covariance()
compute_mission_eclipse()
suggest_enrichment()
  ... user reviews enrichment ...
apply_perch_enrichment()
  ... user edits waypoints ...
replan_from_transfer()
compute_safety_analysis()
  ... user reviews safety ...
                                            validate -> progress* + validation_result
                                            run_mc -> progress* + monte_carlo_result
```

### Proximity Mission

A proximity mission skips the Lambert transfer (spacecraft are already co-orbital). The server is used only for drag extraction and final validation.

```
Browser (WASM)                              Server
--------------                              ------
classify_separation() -> proximity
                                            extract_drag -> drag_result
plan_waypoint_mission()
compute_safety_analysis()
compute_mission_covariance()
compute_mission_eclipse()
  ... user edits waypoints ...
replan_from_transfer()
enrich_waypoint()                           (per-waypoint safe alternative preview)
accept_waypoint_enrichment()
compute_safety_analysis()
  ... user reviews mission ...
                                            validate -> progress* + validation_result
                                            run_mc -> progress* + monte_carlo_result
```
